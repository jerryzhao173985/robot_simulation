#include "Terrain.h"
#include "PhysicsWorld.h"
#include <cmath>
#include <random>
#include <algorithm>
#include <iostream>

#include "PositionUtils.h"

#ifdef USE_OPENGL_FALLBACK
Terrain::Terrain(PhysicsWorld* physicsWorld, ref_ptr<Group> sceneGraph)
    : physicsWorld(physicsWorld), sceneGraph(sceneGraph) {
    
    terrainGroup = ref_ptr<Group>(new Group());
    if (sceneGraph && sceneGraph.ptr) {
        sceneGraph->addChild(terrainGroup.ptr);
    }
#else
Terrain::Terrain(PhysicsWorld* physicsWorld, vsg::ref_ptr<vsg::Group> sceneGraph)
    : physicsWorld(physicsWorld), sceneGraph(sceneGraph) {
    
    terrainGroup = vsg::Group::create();
    sceneGraph->addChild(terrainGroup);
#endif
    
    // Initialize default parameters
    currentType = FLAT;
    terrainGeom = nullptr;
    meshData = nullptr;
}

Terrain::~Terrain() {
    clear();
}

void Terrain::clear() {
    // Clean up physics
    if (terrainGeom) {
        dGeomDestroy(terrainGeom);
        terrainGeom = nullptr;
    }
    if (meshData) {
        dGeomTriMeshDataDestroy(meshData);
        meshData = nullptr;
    }
    
    // Clear visual
    terrainGroup->children.clear();
    vegetation.clear();
    
    // Clear data
    heightData.clear();
    normals.clear();
    vertices.clear();
    indices.clear();
    heightCache.clear();
    normalCache.clear();
}

void Terrain::generate(TerrainType type, const TerrainParams& params) {
    clear();
    
    currentType = type;
    this->params = params;
    
    // Generate height data
    generateHeightData();
    
    // Apply terrain-specific modifications
    switch (type) {
        case FLAT:
            generateFlat();
            break;
        case HILLS:
            generateHills();
            break;
        case MOUNTAINS:
            generateMountains();
            break;
        case ROUGH:
            generateRough();
            break;
        case STAIRS:
            generateStairs();
            break;
        case OBSTACLES:
            generateObstacles();
            break;
        case CUSTOM:
            // Custom terrain uses raw height data
            break;
    }
    
    // Apply erosion for more realistic terrain
    if (type == MOUNTAINS || type == HILLS) {
        applyErosion(5);
    }
    
    // Generate normals
    generateNormals();
    
    // Create mesh
    createMesh();
    
    // Create physics mesh
    createPhysicsMesh();
    
    // Create LODs
    if (lodLevels > 1) {
        createLODs();
    }
    
    // Place vegetation
    if (params.vegetationDensity > 0.0f) {
        placeVegetation();
    }
}

void Terrain::generateHeightData() {
    int size = params.resolution;
    heightData.resize(size * size);
    
    // Initialize with base height
    std::fill(heightData.begin(), heightData.end(), 0.0f);
}

void Terrain::generateFlat() {
    // Already flat from initialization
}

void Terrain::generateHills() {
    int size = params.resolution;
    
    for (int z = 0; z < size; ++z) {
        for (int x = 0; x < size; ++x) {
            float fx = (float)x / (size - 1) * params.frequency;
            float fz = (float)z / (size - 1) * params.frequency;
            
            float height = 0.0f;
            height += sin(fx * 2.0f) * cos(fz * 2.0f) * 0.5f;
            height += fractalNoise(fx, fz, 4, 0.5f) * 0.5f;
            
            heightData[z * size + x] = height * params.maxHeight;
        }
    }
}

void Terrain::generateMountains() {
    int size = params.resolution;
    
    for (int z = 0; z < size; ++z) {
        for (int x = 0; x < size; ++x) {
            float fx = (float)x / (size - 1) * params.frequency * 2.0f;
            float fz = (float)z / (size - 1) * params.frequency * 2.0f;
            
            float height = fractalNoise(fx, fz, params.octaves, params.persistence);
            
            // Ridge noise for mountain ridges
            float ridge = 1.0f - std::abs(noise(fx * 2.0f, fz * 2.0f));
            ridge = ridge * ridge;
            
            height = height * 0.7f + ridge * 0.3f;
            height = std::pow(std::abs(height), 1.5f) * ((height > 0) ? 1 : -1);
            
            heightData[z * size + x] = height * params.maxHeight;
        }
    }
}

void Terrain::generateRough() {
    int size = params.resolution;
    
    for (int z = 0; z < size; ++z) {
        for (int x = 0; x < size; ++x) {
            float fx = (float)x / (size - 1) * params.frequency * 4.0f;
            float fz = (float)z / (size - 1) * params.frequency * 4.0f;
            
            float height = 0.0f;
            
            // Multiple octaves of noise at different scales
            height += noise(fx, fz) * 1.0f;
            height += noise(fx * 3.1f, fz * 3.1f) * 0.5f;
            height += noise(fx * 7.7f, fz * 7.7f) * 0.25f;
            
            // Scale and ensure terrain is above Z=0
            height = height * params.maxHeight * params.roughness;
            // Shift terrain up so minimum height is 0
            height = std::max(0.0f, height + params.maxHeight * 0.5f);
            
            heightData[z * size + x] = height;
        }
    }
}

void Terrain::generateStairs() {
    int size = params.resolution;
    int numSteps = 10;
    float stepHeight = params.maxHeight / numSteps;
    
    for (int z = 0; z < size; ++z) {
        for (int x = 0; x < size; ++x) {
            float progress = (float)x / (size - 1);
            int step = (int)(progress * numSteps);
            
            float height = step * stepHeight;
            
            // Add some noise to make it less perfect
            float fx = (float)x / (size - 1) * 10.0f;
            float fz = (float)z / (size - 1) * 10.0f;
            height += noise(fx, fz) * stepHeight * 0.1f;
            
            heightData[z * size + x] = height;
        }
    }
}

void Terrain::generateObstacles() {
    int size = params.resolution;
    
    // Start with rough terrain
    generateRough();
    
    // Add random box obstacles
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    
    int numObstacles = 20;
    for (int i = 0; i < numObstacles; ++i) {
        int cx = (int)(dis(gen) * size);
        int cz = (int)(dis(gen) * size);
        int width = 5 + (int)(dis(gen) * 10);
        int depth = 5 + (int)(dis(gen) * 10);
        float height = params.maxHeight * 0.5f + dis(gen) * params.maxHeight * 0.5f;
        
        // Create box obstacle
        for (int z = std::max(0, cz - depth/2); z < std::min(size, cz + depth/2); ++z) {
            for (int x = std::max(0, cx - width/2); x < std::min(size, cx + width/2); ++x) {
                heightData[z * size + x] += height;
            }
        }
    }
}

float Terrain::noise(float x, float y) const {
    // Simple Perlin-like noise implementation
    int xi = (int)x;
    int yi = (int)y;
    float xf = x - xi;
    float yf = y - yi;
    
    // Hash function for pseudo-random values
    auto hash = [](int x, int y) -> float {
        int n = x + y * 57;
        n = (n << 13) ^ n;
        return (1.0f - ((n * (n * n * 15731 + 789221) + 1376312589) & 0x7fffffff) / 1073741824.0f);
    };
    
    // Interpolation
    float v00 = hash(xi, yi);
    float v10 = hash(xi + 1, yi);
    float v01 = hash(xi, yi + 1);
    float v11 = hash(xi + 1, yi + 1);
    
    // Smooth interpolation
    float sx = xf * xf * (3.0f - 2.0f * xf);
    float sy = yf * yf * (3.0f - 2.0f * yf);
    
    float v0 = v00 * (1.0f - sx) + v10 * sx;
    float v1 = v01 * (1.0f - sx) + v11 * sx;
    
    return v0 * (1.0f - sy) + v1 * sy;
}

float Terrain::fractalNoise(float x, float y, int octaves, float persistence) const {
    float value = 0.0f;
    float amplitude = 1.0f;
    float frequency = 1.0f;
    float maxValue = 0.0f;
    
    for (int i = 0; i < octaves; ++i) {
        value += noise(x * frequency, y * frequency) * amplitude;
        maxValue += amplitude;
        amplitude *= persistence;
        frequency *= params.lacunarity;
    }
    
    return value / maxValue;
}

void Terrain::generateNormals() {
    int size = params.resolution;
    normals.resize(size * size);
    
    float scale = params.size / (size - 1);
    
    for (int z = 0; z < size; ++z) {
        for (int x = 0; x < size; ++x) {
            // Get neighboring heights
            float hL = (x > 0) ? heightData[z * size + (x - 1)] : heightData[z * size + x];
            float hR = (x < size - 1) ? heightData[z * size + (x + 1)] : heightData[z * size + x];
            float hD = (z > 0) ? heightData[(z - 1) * size + x] : heightData[z * size + x];
            float hU = (z < size - 1) ? heightData[(z + 1) * size + x] : heightData[z * size + x];
            
            // Calculate normal using cross-product for accurate surface orientation
            // Tangent vectors in x and y directions
            vsg::vec3 dx(scale, 0.0f, hR - hL);
            vsg::vec3 dy(0.0f, scale, hU - hD);
            vsg_vec3 normal = normalize(cross(dx, dy));
            normals[z * size + x] = normal;
        }
    }
}

void Terrain::createMesh() {
    if (heightData.empty()) return;
    
    // Create physics mesh first
    createPhysicsMesh();
    
    // TODO: Update for current VSG API
    // Visual mesh creation will be added once we fix VSG geometry API
    /*
    // Create vertices and indices for rendering
    vertices.clear();
    indices.clear();
    
    // Generate vertices
    for (int z = 0; z < params.resolution; ++z) {
        for (int x = 0; x < params.resolution; ++x) {
            float worldX = (x / (float)(params.resolution - 1) - 0.5f) * params.size;
            float worldZ = (z / (float)(params.resolution - 1) - 0.5f) * params.size;
            float height = getHeightAt(worldX, worldZ);
            
            vertices.push_back(worldX);
            vertices.push_back(worldZ);
            vertices.push_back(height);
            
            // Texture coordinates
            vertices.push_back(x / (float)(params.resolution - 1));
            vertices.push_back(z / (float)(params.resolution - 1));
            
            // Normals (simplified - could be improved)
            vsg_vec3 normal = getNormalAt(worldX, worldZ);
            vertices.push_back(normal.x);
            vertices.push_back(normal.y);
            vertices.push_back(normal.z);
        }
    }
    
    // Generate indices
    for (int z = 0; z < params.resolution - 1; ++z) {
        for (int x = 0; x < params.resolution - 1; ++x) {
            uint32_t topLeft = z * params.resolution + x;
            uint32_t topRight = topLeft + 1;
            uint32_t bottomLeft = (z + 1) * params.resolution + x;
            uint32_t bottomRight = bottomLeft + 1;
            
            // First triangle
            indices.push_back(topLeft);
            indices.push_back(bottomLeft);
            indices.push_back(topRight);
            
            // Second triangle
            indices.push_back(topRight);
            indices.push_back(bottomLeft);
            indices.push_back(bottomRight);
        }
    }
    
    // Create VSG geometry
    auto vertexArray = vsg::vec3Array::create();
    auto normalArray = vsg::vec3Array::create();
    auto texCoordArray = vsg::vec2Array::create();
    auto colorArray = vsg::vec4Array::create();
    auto indexArray = vsg::ushortArray::create();
    
    // Fill arrays... (this would be the actual implementation)
    
    auto vid = vsg::VertexInputData::create();
    vid->arrays.push_back(vertexArray);
    vid->arrays.push_back(texCoordArray);
    vid->arrays.push_back(colorArray);
    vid->indices = indexArray;
    
    auto drawCommands = vsg::Commands::create();
    drawCommands->addChild(vsg::DrawIndexed::create(indexArray->size(), 1, 0, 0, 0));
    
    auto stateGroup = vsg::StateGroup::create();
    stateGroup->addChild(drawCommands);
    
    terrainTransform->addChild(stateGroup);
    */
}

void Terrain::createPhysicsMesh() {
    if (heightData.empty()) return;
    
    // Generate physics vertices and indices
    physicsVertices.clear();
    physicsIndices.clear();
    
    // Generate vertices for physics - ODE uses X,Y,Z coordinate system
    for (int z = 0; z < params.resolution; ++z) {
        for (int x = 0; x < params.resolution; ++x) {
            float worldX = (x / (float)(params.resolution - 1) - 0.5f) * params.size;
            float worldY = (z / (float)(params.resolution - 1) - 0.5f) * params.size;  // Y for forward/back
            float height = heightData[z * params.resolution + x];  // Z for up
            
            // ODE expects X,Y,Z order
            physicsVertices.push_back(worldX);
            physicsVertices.push_back(worldY);
            physicsVertices.push_back(height);
        }
    }
    
    // Generate indices for physics
    for (int z = 0; z < params.resolution - 1; ++z) {
        for (int x = 0; x < params.resolution - 1; ++x) {
            int topLeft = z * params.resolution + x;
            int topRight = topLeft + 1;
            int bottomLeft = (z + 1) * params.resolution + x;
            int bottomRight = bottomLeft + 1;
            
            // First triangle
            physicsIndices.push_back(topLeft);
            physicsIndices.push_back(bottomLeft);
            physicsIndices.push_back(topRight);
            
            // Second triangle
            physicsIndices.push_back(topRight);
            physicsIndices.push_back(bottomLeft);
            physicsIndices.push_back(bottomRight);
        }
    }
    
    // Create trimesh data
    meshData = dGeomTriMeshDataCreate();
    
    // Build trimesh from physics vertices and indices
    dGeomTriMeshDataBuildSingle(meshData,
        physicsVertices.data(), 3 * sizeof(float), physicsVertices.size() / 3,
        physicsIndices.data(), physicsIndices.size(), 3 * sizeof(int));
    
    // Create geometry
    terrainGeom = dCreateTriMesh(physicsWorld->getSpace(), meshData, 0, 0, 0);
    
    // Make sure terrain collides with everything
    dGeomSetCategoryBits(terrainGeom, ~0);
    dGeomSetCollideBits(terrainGeom, ~0);
    
    // Set friction
    // Note: ODE doesn't have per-geometry friction, it's set per contact
}

void Terrain::createLODs() {
    // Create multiple LOD levels
    // This is a simplified implementation
    // In a real system, you'd create progressively simpler meshes
}

void Terrain::placeVegetation() {
    if (params.vegetationDensity <= 0.0f) return;
    
    // TODO: Update for current VSG API
    // Vegetation placement will be added once we fix VSG geometry API
    
    /*
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);
    
    auto vegetationGroup = vsg::Group::create();
    
    for (int i = 0; i < params.resolution / 4; ++i) {
        float x = (dis(gen) - 0.5f) * params.size;
        float z = (dis(gen) - 0.5f) * params.size;
        float height = getHeightAt(x, z);
        
        if (height > params.maxHeight * 0.1f && height < params.maxHeight * 0.8f) {
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::translate(x, height, z) *
                               vsg::scale(0.1f, 0.3f + dis(gen) * 0.2f, 0.1f);
            
            if (dis(gen) > 0.7f) {
                // Tree
                auto cylinder = vsg::Cylinder::create();
                cylinder->radius = 0.02f;
                cylinder->height = 0.3f;
                
                vsg::GeometryInfo geomInfo;
                geomInfo.cylinder = cylinder;
                geomInfo.color = vsg::vec4(0.4f, 0.2f, 0.1f, 1.0f);
                
                auto builder = vsg::Builder::create();
                auto node = builder->createCylinder(geomInfo);
                transform->addChild(node);
                
                // Tree canopy
                auto trunkTransform = vsg::MatrixTransform::create();
                trunkTransform->matrix = vsg::translate(0.0f, 0.25f, 0.0f);
                
                auto trunk = vsg::Cylinder::create();
                trunk->radius = 0.02f;
                trunk->height = 0.3f;
                
                vsg::GeometryInfo trunkInfo;
                trunkInfo.cylinder = trunk;
                trunkInfo.color = vsg::vec4(0.4f, 0.2f, 0.1f, 1.0f);
                
                auto trunkNode = builder->createCylinder(trunkInfo);
                trunkTransform->addChild(trunkNode);
                
                // Canopy
                auto canopyTransform = vsg::MatrixTransform::create();
                canopyTransform->matrix = vsg::translate(0.0f, 0.4f, 0.0f);
                
                auto sphere = vsg::Sphere::create();
                sphere->radius = 0.1f;
                
                vsg::GeometryInfo canopyInfo;
                canopyInfo.sphere = sphere;
                canopyInfo.color = vsg::vec4(0.2f, 0.8f, 0.2f, 1.0f);
                
                auto canopyNode = builder->createSphere(canopyInfo);
                canopyTransform->addChild(canopyNode);
                
                transform->addChild(trunkTransform);
                transform->addChild(canopyTransform);
            } else {
                // Rock
                if (dis(gen) > 0.3f) {
                    auto box = vsg::Box::create();
                    box->min = vsg::vec3(-0.05f, 0.0f, -0.05f);
                    box->max = vsg::vec3(0.05f, 0.1f, 0.05f);
                    
                    vsg::GeometryInfo rockInfo;
                    rockInfo.box = box;
                    rockInfo.color = vsg::vec4(0.5f, 0.5f, 0.5f, 1.0f);
                    
                    auto builder = vsg::Builder::create();
                    auto rockNode = builder->createBox(rockInfo);
                    transform->addChild(rockNode);
                }
            }
            
            vegetationGroup->addChild(transform);
        }
    }
    
    if (terrainGroup) {
        terrainGroup->addChild(vegetationGroup);
    }
    */
}

void Terrain::applyErosion(int iterations) {
    int size = params.resolution;
    
    for (int iter = 0; iter < iterations; ++iter) {
        std::vector<float> newHeights = heightData;
        
        for (int z = 1; z < size - 1; ++z) {
            for (int x = 1; x < size - 1; ++x) {
                int idx = z * size + x;
                
                // Get neighbor heights
                float h = heightData[idx];
                float hL = heightData[idx - 1];
                float hR = heightData[idx + 1];
                float hU = heightData[idx - size];
                float hD = heightData[idx + size];
                
                // Simple erosion: average with neighbors
                float avg = (h + hL + hR + hU + hD) / 5.0f;
                float diff = h - avg;
                
                // Erode high points more
                if (diff > 0) {
                    newHeights[idx] = h - diff * 0.1f;
                }
            }
        }
        
        heightData = newHeights;
    }
}

float Terrain::getHeightAt(float x, float z) const {
    // Convert world coordinates to terrain coordinates
    int size = params.resolution;
    float scale = params.size / (size - 1);
    
    float tx = (x / scale) + size / 2.0f;
    float tz = (z / scale) + size / 2.0f;
    
    // Check bounds
    if (tx < 0 || tx >= size - 1 || tz < 0 || tz >= size - 1) {
        return 0.0f;
    }
    
    // Bilinear interpolation
    int x0 = (int)tx;
    int z0 = (int)tz;
    int x1 = x0 + 1;
    int z1 = z0 + 1;
    
    float fx = tx - x0;
    float fz = tz - z0;
    
    float h00 = heightData[z0 * size + x0];
    float h10 = heightData[z0 * size + x1];
    float h01 = heightData[z1 * size + x0];
    float h11 = heightData[z1 * size + x1];
    
    float h0 = h00 * (1.0f - fx) + h10 * fx;
    float h1 = h01 * (1.0f - fx) + h11 * fx;
    
    return h0 * (1.0f - fz) + h1 * fz;
}

vsg::vec3 Terrain::getNormalAt(float x, float z) const {
    // Similar to getHeightAt but for normals
    int size = params.resolution;
    float scale = params.size / (size - 1);
    
    float tx = (x / scale) + size / 2.0f;
    float tz = (z / scale) + size / 2.0f;
    
    if (tx < 0 || tx >= size - 1 || tz < 0 || tz >= size - 1) {
        return vsg::vec3(0.0f, 0.0f, 1.0f);
    }
    
    int x0 = (int)tx;
    int z0 = (int)tz;
    
    if (x0 >= 0 && x0 < size && z0 >= 0 && z0 < size) {
        return normals[z0 * size + x0];
    }
    
    return vsg::vec3(0.0f, 0.0f, 1.0f);
}

float Terrain::getSlopeAt(float x, float z) const {
    vsg::vec3 normal = getNormalAt(x, z);
    return 1.0f - normal.z;  // 0 = flat, 1 = vertical
}

void Terrain::deform(const vsg::vec3& center, float radius, float amount) {
    int size = params.resolution;
    float scale = params.size / (size - 1);
    
    for (int z = 0; z < size; ++z) {
        for (int x = 0; x < size; ++x) {
            float px = (x - size/2) * scale;
            float pz = (z - size/2) * scale;
            
            float dist = vsg::length(vsg::vec2(px - center.x, pz - center.z));
            if (dist < radius) {
                float factor = 1.0f - (dist / radius);
                factor = factor * factor;  // Smooth falloff
                heightData[z * size + x] += amount * factor;
            }
        }
    }
    
    // Regenerate normals and mesh
    generateNormals();
    createMesh();
    updatePhysicsMesh();
}

void Terrain::addRamp(const vsg::vec3& start, const vsg::vec3& end, float width) {
    int size = params.resolution;
    float scale = params.size / (size - 1);

    // Compute horizontal direction and perpendicular in X-Y plane
    vsg::vec2 horizStart(start.x, start.y);
    vsg::vec2 horizEnd(end.x, end.y);
    vsg::vec2 diff = horizEnd - horizStart;
    float length = vsg::length(diff);
    vsg::vec2 dir = (length > 0.0f) ? (diff / length) : vsg::vec2(0.0f, 0.0f);
    vsg::vec2 perp(-dir.y, dir.x);

    for (int iz = 0; iz < size; ++iz) {
        for (int ix = 0; ix < size; ++ix) {
            float px = (ix - size / 2) * scale;
            float py = (iz - size / 2) * scale;
            vsg::vec2 p2d(px, py);

            // Project point onto ramp line in horizontal plane
            vsg::vec2 toPoint = p2d - horizStart;
            float alongRamp = vsg::dot(toPoint, dir);
            float perpDist = vsg::dot(toPoint, perp);

            if (alongRamp >= 0 && alongRamp <= length && std::abs(perpDist) <= width / 2) {
                float t = alongRamp / length;
                float targetHeight = start.z * (1.0f - t) + end.z * t;

                // Smooth edges
                float edgeFactor = 1.0f - (std::abs(perpDist) / (width / 2));
                edgeFactor = edgeFactor * edgeFactor;

                heightData[iz * size + ix] =
                    heightData[iz * size + ix] * (1.0f - edgeFactor) +
                    targetHeight * edgeFactor;
            }
        }
    }

    // Update mesh
    generateNormals();
    createMesh();
    updatePhysicsMesh();
}

void Terrain::updatePhysicsMesh() {
    // Recreate physics mesh with updated data
    if (terrainGeom) {
        dGeomDestroy(terrainGeom);
    }
    if (meshData) {
        dGeomTriMeshDataDestroy(meshData);
    }
    
    createPhysicsMesh();
}

// setMaterial function is now inline in header

void Terrain::generate() {
    generateHeightData();
    
    // TODO: Update for current VSG API
    // Visual terrain generation will be added once we fix VSG geometry API
    
    /*
    generateNormals();
    createMesh();
    applyTextures();
    
    if (showWireframe) {
        enableWireframe();
    }
    */
    
    // For now, just create physics mesh
    createMesh();
}

#ifndef USE_OPENGL_FALLBACK
vsg::ref_ptr<vsg::Group> Terrain::getTerrainNode() {
    // TODO: Update for current VSG API
    // Return empty group for now until VSG geometry API is fixed
    return vsg::Group::create();
    
    /*
    if (!terrainGroup) {
        terrainGroup = vsg::Group::create();
        generate();
    }
    return terrainGroup;
    */
}
#else
ref_ptr<Group> Terrain::getTerrainNode() {
    // OpenGL fallback - terrain handled differently
    return ref_ptr<Group>(new Group());
}
#endif

void Terrain::applyTextures() {
    // TODO: Update for current VSG API
    // Texture application will be added once we fix VSG geometry API
    
    /*
    if (!terrainTransform) return;
    
    // Create textures based on height and slope
    auto descriptorSetLayout = vsg::DescriptorSetLayout::create(vsg::DescriptorSetLayoutBindings{
        {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr},
        {1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr},
        {2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr}
    });
    
    auto pipelineLayout = vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{descriptorSetLayout});
    
    // Apply to terrain
    auto stateGroup = vsg::StateGroup::create();
    stateGroup->stateCommands.push_back(vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, descriptorSet));
    
    terrainTransform->addChild(stateGroup);
    */
}

void Terrain::enableWireframe() {
    // TODO: Update for current VSG API
    // Wireframe mode will be added once we fix VSG geometry API
    
    /*
    if (!terrainTransform) return;
    
    auto rasterizationState = vsg::RasterizationState::create();
    rasterizationState->polygonMode = VK_POLYGON_MODE_LINE;
    
    auto stateGroup = vsg::StateGroup::create();
    stateGroup->stateCommands.push_back(rasterizationState);
    stateGroup->addChild(terrainTransform);
    
    showWireframe = true;
    */
}