#pragma once

#include "../core/types.h"
#include <string>
#include <vector>
#include <memory>

namespace vss {

// 前向声明
class Tissue;

/**
 * @brief 网格文件格式
 */
enum class MeshFormat {
    OBJ,    // Wavefront OBJ
    STL,    // STL (Stereolithography)
    VTK,    // VTK Legacy
    MSH,    // Gmsh
    CUSTOM  // 自定义格式
};

/**
 * @brief 加载的网格数据
 */
struct MeshData {
    std::vector<Vector3d> vertices;
    std::vector<Vector3i> tetrahedra;   // 四面体单元
    std::vector<Vector3i> triangles;    // 表面三角形
    std::vector<Vector3d> vertex_normals;
    std::vector<Vector3d> vertex_colors;
    
    // 物理属性（可选）
    double youngs_modulus = 1000.0;     // 杨氏模量 (Pa)
    double poisson_ratio = 0.45;        // 泊松比
    double density = 1000.0;            // 密度 (kg/m³)
    
    void clear() {
        vertices.clear();
        tetrahedra.clear();
        triangles.clear();
        vertex_normals.clear();
        vertex_colors.clear();
    }
    
    bool isEmpty() const {
        return vertices.empty();
    }
};

/**
 * @brief 网格加载器
 * 
 * 支持多种网格格式的加载
 */
class MeshLoader {
public:
    MeshLoader() = default;
    ~MeshLoader() = default;

    /**
     * @brief 从文件加载网格
     * @param filepath 文件路径
     * @param format 文件格式（自动检测如果为CUSTOM）
     * @return 加载的网格数据
     */
    MeshData load(const std::string& filepath, MeshFormat format = MeshFormat::CUSTOM);

    /**
     * @brief 加载OBJ文件
     */
    MeshData loadOBJ(const std::string& filepath);

    /**
     * @brief 加载STL文件
     */
    MeshData loadSTL(const std::string& filepath);

    /**
     * @brief 加载VTK文件
     */
    MeshData loadVTK(const std::string& filepath);

    /**
     * @brief 加载Gmsh文件
     */
    MeshData loadMSH(const std::string& filepath);

    /**
     * @brief 保存网格到文件
     */
    bool save(const std::string& filepath, const MeshData& mesh, MeshFormat format);

    /**
     * @brief 保存为OBJ格式
     */
    bool saveOBJ(const std::string& filepath, const MeshData& mesh);

    /**
     * @brief 保存为VTK格式
     */
    bool saveVTK(const std::string& filepath, const MeshData& mesh);

    /**
     * @brief 获取最后错误信息
     */
    const std::string& getLastError() const { return last_error_; }

    /**
     * @brief 检测文件格式
     */
    static MeshFormat detectFormat(const std::string& filepath);

    /**
     * @brief 创建默认测试网格（四面体）
     */
    static MeshData createDefaultTetrahedron();

    /**
     * @brief 创建默认测试网格（立方体）
     */
    static MeshData createDefaultCube();

private:
    std::string last_error_;

    /**
     * @brief 计算表面法线
     */
    void computeNormals(MeshData& mesh);

    /**
     * @brief 从三角形生成四面体（简单方法）
     */
    void triangulateToTetrahedra(MeshData& mesh);
};

/**
 * @brief 状态序列化器
 * 
 * 保存和加载仿真状态
 */
class StateSerializer {
public:
    struct SimulationState {
        double time = 0.0;
        int step = 0;
        
        // 软组织状态
        std::vector<Vector3d> tissue_positions;
        std::vector<Vector3d> tissue_velocities;
        
        // 缝合线状态
        std::vector<Vector3d> suture_positions;
        std::vector<Vector3d> suture_velocities;
        
        // 器械状态
        Vector3d instrument_position = Vector3d::Zero();
        Vector3d instrument_velocity = Vector3d::Zero();
    };

    /**
     * @brief 保存状态到文件
     */
    bool save(const std::string& filepath, const SimulationState& state);

    /**
     * @brief 从文件加载状态
     */
    bool load(const std::string& filepath, SimulationState& state);

    /**
     * @brief 序列化到二进制
     */
    std::vector<uint8_t> serialize(const SimulationState& state);

    /**
     * @brief 从二进制反序列化
     */
    bool deserialize(const std::vector<uint8_t>& data, SimulationState& state);

    /**
     * @brief 获取最后错误信息
     */
    const std::string& getLastError() const { return last_error_; }

private:
    std::string last_error_;
};

} // namespace vss
