#include "io/mesh_loader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cctype>

namespace vss {

// ==================== MeshLoader ====================

MeshData MeshLoader::load(const std::string& filepath, MeshFormat format) {
    if (format == MeshFormat::CUSTOM) {
        format = detectFormat(filepath);
    }
    
    switch (format) {
        case MeshFormat::OBJ:
            return loadOBJ(filepath);
        case MeshFormat::STL:
            return loadSTL(filepath);
        case MeshFormat::VTK:
            return loadVTK(filepath);
        case MeshFormat::MSH:
            return loadMSH(filepath);
        default:
            last_error_ = "Unknown mesh format";
            return MeshData();
    }
}

MeshData MeshLoader::loadOBJ(const std::string& filepath) {
    MeshData mesh;
    std::ifstream file(filepath);
    
    if (!file.is_open()) {
        last_error_ = "Cannot open file: " + filepath;
        return mesh;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;
        
        if (prefix == "v") {
            // 顶点
            double x, y, z;
            iss >> x >> y >> z;
            mesh.vertices.emplace_back(x, y, z);
        } else if (prefix == "vn") {
            // 法线
            double x, y, z;
            iss >> x >> y >> z;
            mesh.vertex_normals.emplace_back(x, y, z);
        } else if (prefix == "f") {
            // 面（简化处理，假设只有顶点索引）
            std::string v1, v2, v3;
            iss >> v1 >> v2 >> v3;
            
            // 解析顶点索引（处理格式如 "1/1/1" 或 "1"）
            auto parseIndex = [](const std::string& s) -> int {
                size_t pos = s.find('/');
                std::string num = (pos == std::string::npos) ? s : s.substr(0, pos);
                return std::stoi(num) - 1;  // OBJ使用1-based索引
            };
            
            int i1 = parseIndex(v1);
            int i2 = parseIndex(v2);
            int i3 = parseIndex(v3);
            mesh.triangles.emplace_back(i1, i2, i3);
        }
    }
    
    file.close();
    
    // 如果没有法线，计算它们
    if (mesh.vertex_normals.empty()) {
        computeNormals(mesh);
    }
    
    return mesh;
}

MeshData MeshLoader::loadSTL(const std::string& filepath) {
    MeshData mesh;
    std::ifstream file(filepath, std::ios::binary);
    
    if (!file.is_open()) {
        last_error_ = "Cannot open file: " + filepath;
        return mesh;
    }
    
    // 检查是否是二进制STL
    char header[80];
    file.read(header, 80);
    
    uint32_t triangle_count;
    file.read(reinterpret_cast<char*>(&triangle_count), 4);
    
    size_t current_vertex = mesh.vertices.size();
    
    for (uint32_t i = 0; i < triangle_count; ++i) {
        // 读取法线
        float nx, ny, nz;
        file.read(reinterpret_cast<char*>(&nx), 4);
        file.read(reinterpret_cast<char*>(&ny), 4);
        file.read(reinterpret_cast<char*>(&nz), 4);
        
        // 读取三个顶点
        for (int j = 0; j < 3; ++j) {
            float x, y, z;
            file.read(reinterpret_cast<char*>(&x), 4);
            file.read(reinterpret_cast<char*>(&y), 4);
            file.read(reinterpret_cast<char*>(&z), 4);
            mesh.vertices.emplace_back(x, y, z);
            mesh.vertex_normals.emplace_back(nx, ny, nz);
        }
        
        // 添加三角形
        mesh.triangles.emplace_back(
            static_cast<int>(current_vertex),
            static_cast<int>(current_vertex + 1),
            static_cast<int>(current_vertex + 2)
        );
        current_vertex += 3;
        
        // 跳过属性字节
        uint16_t attribute;
        file.read(reinterpret_cast<char*>(&attribute), 2);
    }
    
    file.close();
    return mesh;
}

MeshData MeshLoader::loadVTK(const std::string& filepath) {
    MeshData mesh;
    std::ifstream file(filepath);
    
    if (!file.is_open()) {
        last_error_ = "Cannot open file: " + filepath;
        return mesh;
    }
    
    // VTK Legacy格式解析（简化版）
    std::string line;
    bool in_points = false;
    bool in_cells = false;
    int num_points = 0;
    int point_count = 0;
    
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string keyword;
        iss >> keyword;
        
        if (keyword == "POINTS") {
            iss >> num_points;
            in_points = true;
            in_cells = false;
        } else if (keyword == "CELLS" || keyword == "POLYGONS") {
            in_points = false;
            in_cells = true;
        } else if (in_points && point_count < num_points) {
            double x, y, z;
            iss >> x >> y >> z;
            mesh.vertices.emplace_back(x, y, z);
            point_count++;
        }
    }
    
    file.close();
    return mesh;
}

MeshData MeshLoader::loadMSH(const std::string& filepath) {
    // Gmsh格式加载（简化实现）
    last_error_ = "MSH format not fully implemented yet";
    return MeshData();
}

bool MeshLoader::save(const std::string& filepath, const MeshData& mesh, MeshFormat format) {
    if (format == MeshFormat::CUSTOM) {
        format = detectFormat(filepath);
    }
    
    switch (format) {
        case MeshFormat::OBJ:
            return saveOBJ(filepath, mesh);
        case MeshFormat::VTK:
            return saveVTK(filepath, mesh);
        default:
            last_error_ = "Save format not supported";
            return false;
    }
}

bool MeshLoader::saveOBJ(const std::string& filepath, const MeshData& mesh) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        last_error_ = "Cannot create file: " + filepath;
        return false;
    }
    
    file << "# Exported by Virtual Surgery Simulator\n";
    
    // 写入顶点
    for (const auto& v : mesh.vertices) {
        file << "v " << v.x() << " " << v.y() << " " << v.z() << "\n";
    }
    
    // 写入法线
    for (const auto& n : mesh.vertex_normals) {
        file << "vn " << n.x() << " " << n.y() << " " << n.z() << "\n";
    }
    
    // 写入面
    for (const auto& t : mesh.triangles) {
        // OBJ使用1-based索引
        file << "f " << (t[0] + 1) << " " << (t[1] + 1) << " " << (t[2] + 1) << "\n";
    }
    
    file.close();
    return true;
}

bool MeshLoader::saveVTK(const std::string& filepath, const MeshData& mesh) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        last_error_ = "Cannot create file: " + filepath;
        return false;
    }
    
    file << "# vtk DataFile Version 3.0\n";
    file << "Virtual Surgery Simulation Output\n";
    file << "ASCII\n";
    file << "DATASET POLYDATA\n";
    file << "POINTS " << mesh.vertices.size() << " double\n";
    
    // 写入顶点
    for (const auto& v : mesh.vertices) {
        file << v.x() << " " << v.y() << " " << v.z() << "\n";
    }
    
    // 写入多边形
    file << "POLYGONS " << mesh.triangles.size() << " " << (mesh.triangles.size() * 4) << "\n";
    for (const auto& t : mesh.triangles) {
        file << "3 " << t[0] << " " << t[1] << " " << t[2] << "\n";
    }
    
    file.close();
    return true;
}

MeshFormat MeshLoader::detectFormat(const std::string& filepath) {
    std::string ext = filepath.substr(filepath.find_last_of('.') + 1);
    
    // 转换为小写
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    
    if (ext == "obj") return MeshFormat::OBJ;
    if (ext == "stl") return MeshFormat::STL;
    if (ext == "vtk") return MeshFormat::VTK;
    if (ext == "msh") return MeshFormat::MSH;
    
    return MeshFormat::CUSTOM;
}

void MeshLoader::computeNormals(MeshData& mesh) {
    mesh.vertex_normals.assign(mesh.vertices.size(), Vector3d::Zero());
    std::vector<int> vertex_counts(mesh.vertices.size(), 0);
    
    // 计算每个面的法线并累加到顶点
    for (const auto& tri : mesh.triangles) {
        const Vector3d& v0 = mesh.vertices[tri[0]];
        const Vector3d& v1 = mesh.vertices[tri[1]];
        const Vector3d& v2 = mesh.vertices[tri[2]];
        
        Vector3d normal = (v1 - v0).cross(v2 - v0).normalized();
        
        for (int i = 0; i < 3; ++i) {
            mesh.vertex_normals[tri[i]] += normal;
            vertex_counts[tri[i]]++;
        }
    }
    
    // 平均法线
    for (size_t i = 0; i < mesh.vertex_normals.size(); ++i) {
        if (vertex_counts[i] > 0) {
            mesh.vertex_normals[i] /= vertex_counts[i];
            mesh.vertex_normals[i].normalize();
        }
    }
}

MeshData MeshLoader::createDefaultTetrahedron() {
    MeshData mesh;
    
    // 四面体顶点
    mesh.vertices = {
        Vector3d(0, 0, 0),
        Vector3d(1, 0, 0),
        Vector3d(0.5, 1, 0),
        Vector3d(0.5, 0.5, 1)
    };
    
    // 四面体单元（4个三角形面）
    mesh.triangles = {
        Vector3i(0, 1, 2),
        Vector3i(0, 1, 3),
        Vector3i(0, 2, 3),
        Vector3i(1, 2, 3)
    };
    
    // 一个四面体单元
    mesh.tetrahedra = {
        Vector3i(0, 1, 2)  // 简化表示
    };
    
    computeNormals(mesh);
    return mesh;
}

MeshData MeshLoader::createDefaultCube() {
    MeshData mesh;
    
    // 立方体顶点
    mesh.vertices = {
        Vector3d(0, 0, 0), Vector3d(1, 0, 0), Vector3d(1, 1, 0), Vector3d(0, 1, 0),
        Vector3d(0, 0, 1), Vector3d(1, 0, 1), Vector3d(1, 1, 1), Vector3d(0, 1, 1)
    };
    
    // 立方体面（12个三角形）
    mesh.triangles = {
        // 底面
        Vector3i(0, 1, 2), Vector3i(0, 2, 3),
        // 顶面
        Vector3i(4, 6, 5), Vector3i(4, 7, 6),
        // 前面
        Vector3i(0, 5, 1), Vector3i(0, 4, 5),
        // 后面
        Vector3i(2, 7, 3), Vector3i(2, 6, 7),
        // 左面
        Vector3i(0, 7, 4), Vector3i(0, 3, 7),
        // 右面
        Vector3i(1, 6, 2), Vector3i(1, 5, 6)
    };
    
    computeNormals(mesh);
    return mesh;
}

// ==================== StateSerializer ====================

bool StateSerializer::save(const std::string& filepath, const SimulationState& state) {
    std::ofstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        last_error_ = "Cannot create file: " + filepath;
        return false;
    }
    
    // 写入头部
    file.write(reinterpret_cast<const char*>(&state.time), sizeof(state.time));
    file.write(reinterpret_cast<const char*>(&state.step), sizeof(state.step));
    
    // 写入软组织状态
    size_t tissue_count = state.tissue_positions.size();
    file.write(reinterpret_cast<const char*>(&tissue_count), sizeof(tissue_count));
    for (const auto& pos : state.tissue_positions) {
        file.write(reinterpret_cast<const char*>(pos.data()), sizeof(double) * 3);
    }
    for (const auto& vel : state.tissue_velocities) {
        file.write(reinterpret_cast<const char*>(vel.data()), sizeof(double) * 3);
    }
    
    // 写入缝合线状态
    size_t suture_count = state.suture_positions.size();
    file.write(reinterpret_cast<const char*>(&suture_count), sizeof(suture_count));
    for (const auto& pos : state.suture_positions) {
        file.write(reinterpret_cast<const char*>(pos.data()), sizeof(double) * 3);
    }
    for (const auto& vel : state.suture_velocities) {
        file.write(reinterpret_cast<const char*>(vel.data()), sizeof(double) * 3);
    }
    
    // 写入器械状态
    file.write(reinterpret_cast<const char*>(state.instrument_position.data()), sizeof(double) * 3);
    file.write(reinterpret_cast<const char*>(state.instrument_velocity.data()), sizeof(double) * 3);
    
    file.close();
    return true;
}

bool StateSerializer::load(const std::string& filepath, SimulationState& state) {
    std::ifstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        last_error_ = "Cannot open file: " + filepath;
        return false;
    }
    
    // 读取头部
    file.read(reinterpret_cast<char*>(&state.time), sizeof(state.time));
    file.read(reinterpret_cast<char*>(&state.step), sizeof(state.step));
    
    // 读取软组织状态
    size_t tissue_count;
    file.read(reinterpret_cast<char*>(&tissue_count), sizeof(tissue_count));
    state.tissue_positions.resize(tissue_count);
    state.tissue_velocities.resize(tissue_count);
    for (auto& pos : state.tissue_positions) {
        file.read(reinterpret_cast<char*>(pos.data()), sizeof(double) * 3);
    }
    for (auto& vel : state.tissue_velocities) {
        file.read(reinterpret_cast<char*>(vel.data()), sizeof(double) * 3);
    }
    
    // 读取缝合线状态
    size_t suture_count;
    file.read(reinterpret_cast<char*>(&suture_count), sizeof(suture_count));
    state.suture_positions.resize(suture_count);
    state.suture_velocities.resize(suture_count);
    for (auto& pos : state.suture_positions) {
        file.read(reinterpret_cast<char*>(pos.data()), sizeof(double) * 3);
    }
    for (auto& vel : state.suture_velocities) {
        file.read(reinterpret_cast<char*>(vel.data()), sizeof(double) * 3);
    }
    
    // 读取器械状态
    file.read(reinterpret_cast<char*>(state.instrument_position.data()), sizeof(double) * 3);
    file.read(reinterpret_cast<char*>(state.instrument_velocity.data()), sizeof(double) * 3);
    
    file.close();
    return true;
}

std::vector<uint8_t> StateSerializer::serialize(const SimulationState& state) {
    std::vector<uint8_t> data;
    
    // 计算所需大小
    size_t size = sizeof(state.time) + sizeof(state.step);
    size += sizeof(size_t) + state.tissue_positions.size() * sizeof(Vector3d) * 2;
    size += sizeof(size_t) + state.suture_positions.size() * sizeof(Vector3d) * 2;
    size += sizeof(Vector3d) * 2;
    
    data.resize(size);
    size_t offset = 0;
    
    // 序列化数据
    std::memcpy(data.data() + offset, &state.time, sizeof(state.time));
    offset += sizeof(state.time);
    std::memcpy(data.data() + offset, &state.step, sizeof(state.step));
    offset += sizeof(state.step);
    
    // 软组织
    size_t tissue_count = state.tissue_positions.size();
    std::memcpy(data.data() + offset, &tissue_count, sizeof(tissue_count));
    offset += sizeof(tissue_count);
    
    for (const auto& pos : state.tissue_positions) {
        std::memcpy(data.data() + offset, pos.data(), sizeof(double) * 3);
        offset += sizeof(double) * 3;
    }
    for (const auto& vel : state.tissue_velocities) {
        std::memcpy(data.data() + offset, vel.data(), sizeof(double) * 3);
        offset += sizeof(double) * 3;
    }
    
    // 缝合线
    size_t suture_count = state.suture_positions.size();
    std::memcpy(data.data() + offset, &suture_count, sizeof(suture_count));
    offset += sizeof(suture_count);
    
    for (const auto& pos : state.suture_positions) {
        std::memcpy(data.data() + offset, pos.data(), sizeof(double) * 3);
        offset += sizeof(double) * 3;
    }
    for (const auto& vel : state.suture_velocities) {
        std::memcpy(data.data() + offset, vel.data(), sizeof(double) * 3);
        offset += sizeof(double) * 3;
    }
    
    // 器械
    std::memcpy(data.data() + offset, state.instrument_position.data(), sizeof(double) * 3);
    offset += sizeof(double) * 3;
    std::memcpy(data.data() + offset, state.instrument_velocity.data(), sizeof(double) * 3);
    
    return data;
}

bool StateSerializer::deserialize(const std::vector<uint8_t>& data, SimulationState& state) {
    if (data.size() < sizeof(state.time) + sizeof(state.step)) {
        last_error_ = "Data too small";
        return false;
    }
    
    size_t offset = 0;
    
    std::memcpy(&state.time, data.data() + offset, sizeof(state.time));
    offset += sizeof(state.time);
    std::memcpy(&state.step, data.data() + offset, sizeof(state.step));
    offset += sizeof(state.step);
    
    // 软组织
    size_t tissue_count;
    std::memcpy(&tissue_count, data.data() + offset, sizeof(tissue_count));
    offset += sizeof(tissue_count);
    
    state.tissue_positions.resize(tissue_count);
    state.tissue_velocities.resize(tissue_count);
    
    for (auto& pos : state.tissue_positions) {
        std::memcpy(pos.data(), data.data() + offset, sizeof(double) * 3);
        offset += sizeof(double) * 3;
    }
    for (auto& vel : state.tissue_velocities) {
        std::memcpy(vel.data(), data.data() + offset, sizeof(double) * 3);
        offset += sizeof(double) * 3;
    }
    
    // 缝合线
    size_t suture_count;
    std::memcpy(&suture_count, data.data() + offset, sizeof(suture_count));
    offset += sizeof(suture_count);
    
    state.suture_positions.resize(suture_count);
    state.suture_velocities.resize(suture_count);
    
    for (auto& pos : state.suture_positions) {
        std::memcpy(pos.data(), data.data() + offset, sizeof(double) * 3);
        offset += sizeof(double) * 3;
    }
    for (auto& vel : state.suture_velocities) {
        std::memcpy(vel.data(), data.data() + offset, sizeof(double) * 3);
        offset += sizeof(double) * 3;
    }
    
    // 器械
    std::memcpy(state.instrument_position.data(), data.data() + offset, sizeof(double) * 3);
    offset += sizeof(double) * 3;
    std::memcpy(state.instrument_velocity.data(), data.data() + offset, sizeof(double) * 3);
    
    return true;
}

} // namespace vss
