#include "rendering/renderer.h"
#include "physics/tissue.h"
#include "physics/suture.h"
#include "physics/instrument.h"
#include <iostream>
#include <iomanip>

namespace vss {

// ==================== Renderer ====================

Renderer::Renderer(const RenderConfig& config)
    : config_(config), render_mode_(config.default_mode) {
}

Renderer::~Renderer() {
    if (initialized_) {
        shutdown();
    }
}

bool Renderer::initialize() {
    std::cout << "[Renderer] Initializing..." << std::endl;
    std::cout << "  Window: " << config_.window_width << "x" << config_.window_height << std::endl;
    std::cout << "  VSync: " << (config_.enable_vsync ? "enabled" : "disabled") << std::endl;
    
    initialized_ = true;
    should_close_ = false;
    clearStats();
    
    return true;
}

void Renderer::shutdown() {
    std::cout << "[Renderer] Shutting down..." << std::endl;
    initialized_ = false;
}

void Renderer::beginFrame() {
    stats_.draw_calls = 0;
    // 帧时间计算由子类实现
}

void Renderer::endFrame() {
    // 交换缓冲区等操作由子类实现
}

void Renderer::clearStats() {
    stats_ = RenderStats{};
}

void Renderer::renderTissue(const Tissue* tissue) {
    if (!tissue) return;
    
    updateTissueMeshData(tissue);
    
    switch (render_mode_) {
        case RenderMode::WIREFRAME:
            drawWireframe(tissue_mesh_data_);
            break;
        case RenderMode::POINTS:
            drawPoints(tissue_mesh_data_);
            break;
        case RenderMode::SOLID:
        default:
            drawMesh(tissue_mesh_data_);
            break;
    }
}

void Renderer::renderSuture(const Suture* suture) {
    if (!suture) return;
    
    updateSutureMeshData(suture);
    
    // 缝合线通常用线框或点模式渲染
    switch (render_mode_) {
        case RenderMode::POINTS:
            drawPoints(suture_mesh_data_);
            break;
        case RenderMode::WIREFRAME:
        case RenderMode::SOLID:
        default:
            drawWireframe(suture_mesh_data_);
            break;
    }
}

void Renderer::renderInstrument(const Instrument* instrument) {
    if (!instrument) return;
    
    updateInstrumentMeshData(instrument);
    
    switch (render_mode_) {
        case RenderMode::WIREFRAME:
            drawWireframe(instrument_mesh_data_);
            break;
        case RenderMode::POINTS:
            drawPoints(instrument_mesh_data_);
            break;
        case RenderMode::SOLID:
        default:
            drawMesh(instrument_mesh_data_);
            break;
    }
}

void Renderer::renderContactPoint(const Vector3d& position, double force) {
    // 基础实现：只增加统计
    stats_.draw_calls++;
    (void)position;
    (void)force;
}

void Renderer::updateTissueMeshData(const Tissue* tissue) {
    tissue_mesh_data_.clear();
    
    // 这里应该从Tissue对象获取网格数据
    // 简化实现：创建默认的测试数据
    // 实际实现需要访问Tissue的内部节点和单元数据
    
    // 示例：创建一个简单的平面网格
    const int resolution = 10;
    for (int i = 0; i <= resolution; ++i) {
        for (int j = 0; j <= resolution; ++j) {
            double x = (i - resolution / 2.0) * 0.1;
            double z = (j - resolution / 2.0) * 0.1;
            tissue_mesh_data_.vertices.emplace_back(x, 0.0, z);
            tissue_mesh_data_.normals.emplace_back(0.0, 1.0, 0.0);
            tissue_mesh_data_.colors.emplace_back(0.8, 0.6, 0.5);  // 皮肤色
        }
    }
    
    // 创建三角形
    for (int i = 0; i < resolution; ++i) {
        for (int j = 0; j < resolution; ++j) {
            int idx = i * (resolution + 1) + j;
            
            // 第一个三角形
            tissue_mesh_data_.triangles.emplace_back(idx, idx + 1, idx + resolution + 1);
            
            // 第二个三角形
            tissue_mesh_data_.triangles.emplace_back(idx + 1, idx + resolution + 2, idx + resolution + 1);
        }
    }
}

void Renderer::updateSutureMeshData(const Suture* suture) {
    suture_mesh_data_.clear();
    
    // 从Suture获取粒子位置并创建线段
    // 简化实现
    
    // 示例：创建一条简单的线段
    for (int i = 0; i < 10; ++i) {
        double t = i / 9.0;
        suture_mesh_data_.vertices.emplace_back(t * 0.5, 0.1, 0.0);
        suture_mesh_data_.colors.emplace_back(0.9, 0.9, 0.2);  // 黄色
    }
    
    // 创建线段（用三角形表示，实际应该用线渲染）
    for (int i = 0; i < 9; ++i) {
        suture_mesh_data_.triangles.emplace_back(i, i + 1, i);
    }
}

void Renderer::updateInstrumentMeshData(const Instrument* instrument) {
    instrument_mesh_data_.clear();
    
    // 创建设备的简单几何表示
    // 简化实现：创建一个手柄形状
    
    // 手柄主体
    for (int i = 0; i < 8; ++i) {
        double angle = i * M_PI / 4.0;
        double x = 0.05 * std::cos(angle);
        double z = 0.05 * std::sin(angle);
        instrument_mesh_data_.vertices.emplace_back(x, 0.0, z);
        instrument_mesh_data_.vertices.emplace_back(x, 0.3, z);
        instrument_mesh_data_.colors.emplace_back(0.7, 0.7, 0.7);  // 银色
        instrument_mesh_data_.colors.emplace_back(0.7, 0.7, 0.7);
    }
    
    // 简单的三角形连接
    for (int i = 0; i < 8; ++i) {
        int next = (i + 1) % 8;
        instrument_mesh_data_.triangles.emplace_back(i * 2, next * 2, i * 2 + 1);
        instrument_mesh_data_.triangles.emplace_back(next * 2, next * 2 + 1, i * 2 + 1);
    }
}

void Renderer::drawMesh(const MeshRenderData& mesh_data) {
    stats_.draw_calls++;
    stats_.triangle_count += static_cast<int>(mesh_data.triangles.size());
    stats_.vertex_count += static_cast<int>(mesh_data.vertices.size());
    
    // 实际的OpenGL/Vulkan绘制代码由子类实现
    (void)mesh_data;
}

void Renderer::drawWireframe(const MeshRenderData& mesh_data) {
    stats_.draw_calls++;
    stats_.triangle_count += static_cast<int>(mesh_data.triangles.size());
    stats_.vertex_count += static_cast<int>(mesh_data.vertices.size());
    
    (void)mesh_data;
}

void Renderer::drawPoints(const MeshRenderData& mesh_data) {
    stats_.draw_calls++;
    stats_.vertex_count += static_cast<int>(mesh_data.vertices.size());
    
    (void)mesh_data;
}

// ==================== ConsoleRenderer ====================

ConsoleRenderer::ConsoleRenderer(const RenderConfig& config)
    : Renderer(config) {
}

bool ConsoleRenderer::initialize() {
    std::cout << "[ConsoleRenderer] Console rendering mode enabled" << std::endl;
    initialized_ = true;
    frame_count_ = 0;
    return true;
}

void ConsoleRenderer::shutdown() {
    std::cout << "[ConsoleRenderer] Total frames rendered: " << frame_count_ << std::endl;
    initialized_ = false;
}

void ConsoleRenderer::beginFrame() {
    Renderer::beginFrame();
    frame_count_++;
    
    if (frame_count_ % 60 == 0) {
        std::cout << "[Frame " << frame_count_ << "] Rendering..." << std::endl;
    }
}

void ConsoleRenderer::endFrame() {
    // 控制台渲染器不需要交换缓冲区
}

void ConsoleRenderer::renderTissue(const Tissue* tissue) {
    if (!tissue) return;
    
    stats_.draw_calls++;
    
    if (frame_count_ % 60 == 1) {
        std::cout << "  - Tissue: " << tissue->getNodeCount() << " nodes" << std::endl;
    }
}

void ConsoleRenderer::renderSuture(const Suture* suture) {
    if (!suture) return;
    
    stats_.draw_calls++;
    
    if (frame_count_ % 60 == 1) {
        std::cout << "  - Suture: " << suture->getParticleCount() << " particles" << std::endl;
    }
}

void ConsoleRenderer::renderInstrument(const Instrument* instrument) {
    if (!instrument) return;
    
    stats_.draw_calls++;
    
    if (frame_count_ % 60 == 1) {
        auto pos = instrument->getPosition();
        std::cout << "  - Instrument: pos(" 
                  << std::fixed << std::setprecision(3)
                  << pos.x() << ", " << pos.y() << ", " << pos.z() << ")" << std::endl;
    }
}

void ConsoleRenderer::renderContactPoint(const Vector3d& position, double force) {
    stats_.draw_calls++;
    
    if (frame_count_ % 60 == 1 && force > 0.01) {
        std::cout << "  - Contact: force=" << std::fixed << std::setprecision(3) 
                  << force << "N" << std::endl;
    }
}

} // namespace vss
