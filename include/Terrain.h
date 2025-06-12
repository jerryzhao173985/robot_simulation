#pragma once

#include <vsg/all.h>
#include <ode/ode.h>
#include <vector>
#include <memory>

class PhysicsWorld;

class Terrain {
public:
    enum TerrainType {
        FLAT,
        HILLS,
        MOUNTAINS,
        ROUGH,
        STAIRS,
        OBSTACLES,
        CUSTOM
    };

    struct TerrainParams {
        float size = 100.0f;
        int resolution = 256;
        float maxHeight = 10.0f;
        float roughness = 0.5f;
        float frequency = 0.1f;
        int octaves = 4;
        float persistence = 0.5f;
        float lacunarity = 2.0f;
    };

    struct TerrainMaterial {
        vsg::vec4 baseColor = vsg::vec4(0.4f, 0.5f, 0.3f, 1.0f);
        vsg::vec4 rockColor = vsg::vec4(0.5f, 0.5f, 0.5f, 1.0f);
        vsg::vec4 grassColor = vsg::vec4(0.2f, 0.6f, 0.2f, 1.0f);
        vsg::vec4 sandColor = vsg::vec4(0.9f, 0.8f, 0.6f, 1.0f);
        float metallic = 0.0f;
        float roughnessValue = 0.9f;
        float slopeThreshold = 0.7f;
    };

    Terrain(PhysicsWorld* physicsWorld, vsg::ref_ptr<vsg::Group> sceneGraph);
    ~Terrain();

    void generate(TerrainType type, const TerrainParams& params = TerrainParams());
    void generateFromHeightmap(const std::string& filename, float scale = 1.0f);
    void clear();
    
    // Terrain queries
    float getHeightAt(float x, float z) const;
    vsg::vec3 getNormalAt(float x, float z) const;
    float getSlopeAt(float x, float z) const;
    TerrainType getTypeAt(float x, float z) const;
    
    // Terrain modification
    void deform(const vsg::vec3& center, float radius, float amount);
    void smooth(const vsg::vec3& center, float radius, float strength);
    void addObstacle(const vsg::vec3& position, const vsg::vec3& size);
    void addRamp(const vsg::vec3& start, const vsg::vec3& end, float width);
    
    // Visual customization
    void setMaterial(const TerrainMaterial& material);
    void enableTexturing(bool enable) { texturingEnabled = enable; }
    void enableNormalMapping(bool enable) { normalMappingEnabled = enable; }
    void enableDetailTextures(bool enable) { detailTexturesEnabled = enable; }
    void setLODLevels(int levels) { lodLevels = levels; }
    
    // Physics
    void updatePhysicsMesh();
    void setFriction(float friction) { terrainFriction = friction; }
    void setRestitution(float restitution) { terrainRestitution = restitution; }
    
    // Environmental effects
    void enableGrass(bool enable) { grassEnabled = enable; }
    void enableTrees(bool enable) { treesEnabled = enable; }
    void enableRocks(bool enable) { rocksEnabled = enable; }
    void setWindStrength(float strength) { windStrength = strength; }
    void setWindDirection(const vsg::vec3& direction) { windDirection = direction; }

private:
    void generateFlat();
    void generateHills();
    void generateMountains();
    void generateRough();
    void generateStairs();
    void generateObstacles();
    
    float noise(float x, float y) const;
    float fractalNoise(float x, float y, int octaves, float persistence) const;
    void generateHeightData();
    void generateNormals();
    void createMesh();
    void createPhysicsMesh();
    void createLODs();
    void placeVegetation();
    void applyErosion(int iterations);
    
    // Core components
    PhysicsWorld* physicsWorld;
    vsg::ref_ptr<vsg::Group> sceneGraph;
    vsg::ref_ptr<vsg::Group> terrainGroup;
    vsg::ref_ptr<vsg::MatrixTransform> terrainTransform;
    
    // Terrain data
    TerrainType currentType;
    TerrainParams params;
    TerrainMaterial material;
    std::vector<float> heightData;
    std::vector<vsg::vec3> normals;
    std::vector<float> vertices;
    std::vector<uint32_t> indices;
    
    // Physics
    dGeomID terrainGeom;
    dTriMeshDataID meshData;
    float terrainFriction = 1.0f;
    float terrainRestitution = 0.1f;
    
    // Visual features
    bool texturingEnabled = true;
    bool normalMappingEnabled = true;
    bool detailTexturesEnabled = true;
    int lodLevels = 4;
    std::vector<vsg::ref_ptr<vsg::Node>> lodNodes;
    
    // Environmental features
    bool grassEnabled = true;
    bool treesEnabled = true;
    bool rocksEnabled = true;
    float windStrength = 0.5f;
    vsg::vec3 windDirection = vsg::vec3(1.0f, 0.0f, 0.0f);
    std::vector<vsg::ref_ptr<vsg::MatrixTransform>> vegetation;
    
    // Optimization
    mutable std::unordered_map<uint64_t, float> heightCache;
    mutable std::unordered_map<uint64_t, vsg::vec3> normalCache;
};