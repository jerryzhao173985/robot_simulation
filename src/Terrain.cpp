#include "Terrain.h"
#include "PhysicsWorld.h"
#include <cmath>
#include <random>
#include <algorithm>
#include <iostream>

Terrain::Terrain(PhysicsWorld* physicsWorld, vsg::ref_ptr<vsg::Group> sceneGraph)
    : physicsWorld(physicsWorld), sceneGraph(sceneGraph) {
    
    terrainGroup = vsg::Group::create();
    sceneGraph->addChild(terrainGroup);
    
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
    if (grassEnabled || treesEnabled || rocksEnabled) {
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
            
            heightData[z * size + x] = height * params.maxHeight * params.roughness;
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
            
            // Calculate normal
            vsg::vec3 normal;
            normal.x = (hL - hR) / (2.0f * scale);
            normal.y = 2.0f;
            normal.z = (hD - hU) / (2.0f * scale);
            
            normals[z * size + x] = vsg::normalize(normal);
        }
    }
}

void Terrain::createMesh() {
    int size = params.resolution;
    float scale = params.size / (size - 1);
    
    // Generate vertices
    vertices.clear();
    vertices.reserve(size * size * 3);
    
    for (int z = 0; z < size; ++z) {
        for (int x = 0; x < size; ++x) {
            float px = (x - size/2) * scale;
            float py = heightData[z * size + x];
            float pz = (z - size/2) * scale;
            
            vertices.push_back(px);
            vertices.push_back(py);
            vertices.push_back(pz);
        }
    }
    
    // Generate indices
    indices.clear();
    indices.reserve((size - 1) * (size - 1) * 6);
    
    for (int z = 0; z < size - 1; ++z) {
        for (int x = 0; x < size - 1; ++x) {
            int idx = z * size + x;
            
            // First triangle
            indices.push_back(idx);
            indices.push_back(idx + size);
            indices.push_back(idx + 1);
            
            // Second triangle
            indices.push_back(idx + 1);
            indices.push_back(idx + size);
            indices.push_back(idx + size + 1);
        }
    }
    
    // Create visual mesh
    auto builder = vsg::Builder::create();
    
    // Create vertex arrays
    auto vertexArray = vsg::vec3Array::create(size * size);
    auto normalArray = vsg::vec3Array::create(size * size);
    auto texCoordArray = vsg::vec2Array::create(size * size);
    auto colorArray = vsg::vec4Array::create(size * size);
    
    for (int i = 0; i < size * size; ++i) {
        (*vertexArray)[i] = vsg::vec3(
            vertices[i * 3],
            vertices[i * 3 + 1],
            vertices[i * 3 + 2]
        );
        (*normalArray)[i] = normals[i];
        
        int x = i % size;
        int z = i / size;
        (*texCoordArray)[i] = vsg::vec2(
            (float)x / (size - 1),
            (float)z / (size - 1)
        );
        
        // Color based on height and slope
        float height = heightData[i];
        float slope = 1.0f - normals[i].y;
        
        vsg::vec4 color;
        if (slope > material.slopeThreshold) {
            color = material.rockColor;
        } else if (height < params.maxHeight * 0.3f) {
            color = material.sandColor;
        } else if (height < params.maxHeight * 0.7f) {
            color = material.grassColor;
        } else {
            color = material.rockColor;
        }
        
        (*colorArray)[i] = color;
    }
    
    // Create index array
    auto indexArray = vsg::uintArray::create(indices.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        (*indexArray)[i] = indices[i];
    }
    
    // Create draw command
    auto drawCommands = vsg::Commands::create();
    drawCommands->addChild(vsg::DrawIndexed::create(indices.size(), 1, 0, 0, 0));
    
    // Create vertex index draw
    auto vid = vsg::VertexIndexDraw::create();
    vid->arrays.push_back(vertexArray);
    vid->arrays.push_back(normalArray);
    vid->arrays.push_back(texCoordArray);
    vid->arrays.push_back(colorArray);
    vid->indices = indexArray;
    vid->indexCount = static_cast<uint32_t>(indices.size());
    vid->instanceCount = 1;
    
    terrainTransform = vsg::MatrixTransform::create();
    terrainTransform->addChild(vid);
    terrainGroup->addChild(terrainTransform);
}

void Terrain::createPhysicsMesh() {
    // Create trimesh data
    meshData = dGeomTriMeshDataCreate();
    
    // Build trimesh from vertices and indices
    dGeomTriMeshDataBuildSingle(meshData,
        vertices.data(), 3 * sizeof(float), vertices.size() / 3,
        indices.data(), indices.size(), 3 * sizeof(int));
    
    // Create geometry
    terrainGeom = dCreateTriMesh(physicsWorld->getSpace(), meshData, 0, 0, 0);
    
    // Set friction
    // Note: ODE doesn't have per-geometry friction, it's set per contact
}

void Terrain::createLODs() {
    // Create multiple LOD levels
    // This is a simplified implementation
    // In a real system, you'd create progressively simpler meshes
}

void Terrain::placeVegetation() {
    int size = params.resolution;
    float scale = params.size / (size - 1);
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    
    auto builder = vsg::Builder::create();
    
    // Place vegetation based on height and slope
    for (int z = 5; z < size - 5; z += 5) {
        for (int x = 5; x < size - 5; x += 5) {
            int idx = z * size + x;
            float height = heightData[idx];
            float slope = 1.0f - normals[idx].y;
            
            // Skip steep slopes
            if (slope > 0.5f) continue;
            
            float px = (x - size/2) * scale;
            float py = height;
            float pz = (z - size/2) * scale;
            
            // Place grass
            if (grassEnabled && height > params.maxHeight * 0.2f && height < params.maxHeight * 0.8f) {
                if (dis(gen) < 0.3f) {
                    auto grassTransform = vsg::MatrixTransform::create();
                    grassTransform->matrix = vsg::translate(px, py, pz) * 
                                           vsg::scale(0.1f, 0.3f + dis(gen) * 0.2f, 0.1f);
                    
                    // Simple grass blade
                    auto cylinder = vsg::Cylinder::create();
                    cylinder->radius = 0.5f;
                    cylinder->height = 1.0f;
                    
                    vsg::GeometryInfo geomInfo;
                    geomInfo.cylinder = cylinder;
                    geomInfo.color = vsg::vec4(0.2f, 0.8f, 0.2f, 1.0f);
                    
                    auto grass = builder->createCylinder(geomInfo);
                    grassTransform->addChild(grass);
                    vegetation.push_back(grassTransform);
                    terrainGroup->addChild(grassTransform);
                }
            }
            
            // Place trees
            if (treesEnabled && height > params.maxHeight * 0.3f && height < params.maxHeight * 0.7f) {
                if (dis(gen) < 0.05f) {
                    auto treeTransform = vsg::MatrixTransform::create();
                    float treeHeight = 3.0f + dis(gen) * 2.0f;
                    treeTransform->matrix = vsg::translate(px, py + treeHeight/2, pz);
                    
                    // Tree trunk
                    auto trunk = vsg::Cylinder::create();
                    trunk->radius = 0.2f;
                    trunk->height = treeHeight;
                    
                    vsg::GeometryInfo trunkInfo;
                    trunkInfo.cylinder = trunk;
                    trunkInfo.color = vsg::vec4(0.4f, 0.3f, 0.2f, 1.0f);
                    
                    auto trunkNode = builder->createCylinder(trunkInfo);
                    
                    // Tree canopy
                    auto canopyTransform = vsg::MatrixTransform::create();
                    canopyTransform->matrix = vsg::translate(0.0f, treeHeight/2 + 1.0f, 0.0f);
                    
                    auto sphere = vsg::Sphere::create();
                    sphere->radius = 1.5f;
                    
                    vsg::GeometryInfo canopyInfo;
                    canopyInfo.sphere = sphere;
                    canopyInfo.color = vsg::vec4(0.2f, 0.6f, 0.2f, 1.0f);
                    
                    auto canopyNode = builder->createSphere(canopyInfo);
                    canopyTransform->addChild(canopyNode);
                    
                    treeTransform->addChild(trunkNode);
                    treeTransform->addChild(canopyTransform);
                    
                    vegetation.push_back(treeTransform);
                    terrainGroup->addChild(treeTransform);
                }
            }
            
            // Place rocks
            if (rocksEnabled && slope > 0.3f) {
                if (dis(gen) < 0.1f) {
                    auto rockTransform = vsg::MatrixTransform::create();
                    float rockSize = 0.3f + dis(gen) * 0.5f;
                    rockTransform->matrix = vsg::translate(px, py + rockSize/2, pz) *
                                          vsg::scale(rockSize, rockSize * 0.7f, rockSize);
                    
                    auto box = vsg::Box::create();
                    box->min = vsg::vec3(-1.0f, -1.0f, -1.0f);
                    box->max = vsg::vec3(1.0f, 1.0f, 1.0f);
                    
                    vsg::GeometryInfo rockInfo;
                    rockInfo.box = box;
                    rockInfo.color = material.rockColor;
                    
                    auto rock = builder->createBox(rockInfo);
                    rockTransform->addChild(rock);
                    
                    vegetation.push_back(rockTransform);
                    terrainGroup->addChild(rockTransform);
                }
            }
        }
    }
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
        return vsg::vec3(0.0f, 1.0f, 0.0f);
    }
    
    int x0 = (int)tx;
    int z0 = (int)tz;
    
    if (x0 >= 0 && x0 < size && z0 >= 0 && z0 < size) {
        return normals[z0 * size + x0];
    }
    
    return vsg::vec3(0.0f, 1.0f, 0.0f);
}

float Terrain::getSlopeAt(float x, float z) const {
    vsg::vec3 normal = getNormalAt(x, z);
    return 1.0f - normal.y;  // 0 = flat, 1 = vertical
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
    
    vsg::vec3 dir = vsg::normalize(end - start);
    vsg::vec3 perp(-dir.z, 0, dir.x);
    float length = vsg::length(end - start);
    
    for (int z = 0; z < size; ++z) {
        for (int x = 0; x < size; ++x) {
            float px = (x - size/2) * scale;
            float pz = (z - size/2) * scale;
            vsg::vec3 p(px, 0, pz);
            
            // Project point onto ramp line
            vsg::vec3 toPoint = p - start;
            float alongRamp = vsg::dot(toPoint, dir);
            float perpDist = vsg::dot(toPoint, perp);
            
            if (alongRamp >= 0 && alongRamp <= length && std::abs(perpDist) <= width/2) {
                float t = alongRamp / length;
                float targetHeight = start.y * (1.0f - t) + end.y * t;
                
                // Smooth edges
                float edgeFactor = 1.0f - (std::abs(perpDist) / (width/2));
                edgeFactor = edgeFactor * edgeFactor;
                
                heightData[z * size + x] = heightData[z * size + x] * (1.0f - edgeFactor) + 
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

void Terrain::setMaterial(const TerrainMaterial& material) {
    this->material = material;
    // Would need to update the visual mesh colors
}