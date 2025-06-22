#pragma once

#include "FallbackTypes.h"
#include <unordered_map>

#ifdef USE_OPENGL_FALLBACK
    using vsg_vec3 = vec3;
    using vsg_vec4 = vec4;
    using vsg_quat = quat;
    class Node {};
#else
#include <vsg/all.h>
    using vsg_vec3 = vsg::vec3;
    using vsg_vec4 = vsg::vec4;
#endif

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
        int resolution;
        float size;
        float maxHeight;
        float lacunarity;
        int octaves;
        float persistence;
        float frequencyScale;
        float vegetationDensity;
        float roughness;          // Added missing field
        float frequency;          // Added missing field
        
        TerrainParams() : 
            resolution(256),
            size(100.0f),
            maxHeight(10.0f),
            lacunarity(2.0f),
            octaves(4),
            persistence(0.5f),
            frequencyScale(0.01f),
            vegetationDensity(0.1f),
            roughness(0.5f),
            frequency(0.1f) {}
    };

    struct TerrainMaterial {
        vsg_vec4 baseColor = vsg_vec4(0.4f, 0.5f, 0.3f, 1.0f);
        vsg_vec4 rockColor = vsg_vec4(0.5f, 0.5f, 0.5f, 1.0f);
        vsg_vec4 grassColor = vsg_vec4(0.2f, 0.6f, 0.2f, 1.0f);
        vsg_vec4 sandColor = vsg_vec4(0.9f, 0.8f, 0.6f, 1.0f);
        float metallic = 0.0f;
        float roughnessValue = 0.9f;
        float slopeThreshold = 0.7f;
    };

#ifdef USE_OPENGL_FALLBACK
    Terrain(PhysicsWorld* physicsWorld, ref_ptr<Group> sceneGraph);
#else
    Terrain(PhysicsWorld* physicsWorld, vsg::ref_ptr<vsg::Group> sceneGraph);
#endif
    ~Terrain();

    void generate(TerrainType type, const TerrainParams& params = TerrainParams());
    void generateFromHeightmap(const std::string& filename, float scale = 1.0f);
    void clear();
    
    // Terrain queries
    float getHeightAt(float x, float z) const;
    vsg_vec3 getNormalAt(float x, float z) const;
    float getSlopeAt(float x, float z) const;
    TerrainType getTypeAt(float x, float z) const;
    
    // Terrain modification
    void deform(const vsg_vec3& center, float radius, float amount);
    void smooth(const vsg_vec3& center, float radius, float strength);
    void addObstacle(const vsg_vec3& position, const vsg_vec3& size);
    void addRamp(const vsg_vec3& start, const vsg_vec3& end, float width);
    
    // Visual customization
    void setMaterial(const TerrainMaterial& mat) { material = mat; }
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
    void setWindDirection(const vsg_vec3& direction) { windDirection = direction; }

    // Terrain generation and access
    void generate();
    void generateHeightData();
    void generateNormals();
    void createMesh();
    void createPhysicsMesh();
    void applyTextures();
    void enableWireframe();
    
    // Scene access
#ifdef USE_OPENGL_FALLBACK
    ref_ptr<Group> getTerrainNode();
#else
    vsg::ref_ptr<vsg::Group> getTerrainNode();
#endif

private:
    void generateFlat();
    void generateHills();
    void generateMountains();
    void generateRough();
    void generateStairs();
    void generateObstacles();
    
    float noise(float x, float y) const;
    float fractalNoise(float x, float y, int octaves, float persistence) const;
    void createLODs();
    void placeVegetation();
    void applyErosion(int iterations);
    
    // Core components
    PhysicsWorld* physicsWorld;
#ifdef USE_OPENGL_FALLBACK
    ref_ptr<Group> sceneGraph;
    ref_ptr<Group> terrainGroup;
    ref_ptr<MatrixTransform> terrainTransform;
#else
    vsg::ref_ptr<vsg::Group> sceneGraph;
    vsg::ref_ptr<vsg::Group> terrainGroup;
    vsg::ref_ptr<vsg::MatrixTransform> terrainTransform;
#endif
    
    // Terrain data
    TerrainType currentType;
    TerrainParams params;
    TerrainMaterial material;
    std::vector<float> heightData;
    std::vector<vsg_vec3> normals;
    std::vector<float> vertices;
    std::vector<uint32_t> indices;
    
    // Physics-specific mesh data
    std::vector<float> physicsVertices;
    std::vector<int> physicsIndices;
    
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
#ifdef USE_OPENGL_FALLBACK
    std::vector<ref_ptr<Node>> lodNodes;
#else
    std::vector<vsg::ref_ptr<vsg::Node>> lodNodes;
#endif
    
    // Environmental features
    bool grassEnabled = true;
    bool treesEnabled = true;
    bool rocksEnabled = true;
    float windStrength = 0.5f;
    vsg_vec3 windDirection = vsg_vec3(1.0f, 0.0f, 0.0f);
#ifdef USE_OPENGL_FALLBACK
    std::vector<ref_ptr<MatrixTransform>> vegetation;
#else
    std::vector<vsg::ref_ptr<vsg::MatrixTransform>> vegetation;
#endif
    
    // Optimization
    mutable std::unordered_map<uint64_t, float> heightCache;
    mutable std::unordered_map<uint64_t, vsg_vec3> normalCache;
};