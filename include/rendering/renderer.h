#pragma once

#include "../core/types.h"
#include <memory>
#include <vector>
#include <functional>

namespace vss {

// 前向声明
class Tissue;
class Suture;
class Instrument;

/**
 * @brief 渲染模式
 */
enum class RenderMode {
    WIREFRAME,      // 线框模式
    SOLID,          // 实体模式
    POINTS          // 点模式
};

/**
 * @brief 渲染对象类型
 */
enum class RenderObjectType {
    TISSUE,         // 软组织
    SUTURE,         // 缝合线
    INSTRUMENT,     // 器械
    CONTACT_POINT   // 接触点
};

/**
 * @brief 渲染配置
 */
struct RenderConfig {
    int window_width = 1280;
    int window_height = 720;
    std::string window_title = "Virtual Surgery Simulation";
    bool enable_vsync = true;
    bool enable_shadow = false;
    RenderMode default_mode = RenderMode::SOLID;
    Vector3d background_color = Vector3d(0.2, 0.2, 0.2);
};

/**
 * @brief 网格渲染数据
 */
struct MeshRenderData {
    std::vector<Vector3d> vertices;
    std::vector<Vector3i> triangles;
    std::vector<Vector3d> normals;
    std::vector<Vector3d> colors;
    
    void clear() {
        vertices.clear();
        triangles.clear();
        normals.clear();
        colors.clear();
    }
};

/**
 * @brief 渲染器基类
 * 
 * 负责场景渲染、网格显示、交互可视化
 * 支持OpenGL/Vulkan后端（当前为接口定义）
 */
class Renderer {
public:
    explicit Renderer(const RenderConfig& config = RenderConfig{});
    virtual ~Renderer();

    // 禁止拷贝，允许移动
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    Renderer(Renderer&&) = default;
    Renderer& operator=(Renderer&&) = default;

    /**
     * @brief 初始化渲染系统
     */
    virtual bool initialize();

    /**
     * @brief 关闭渲染系统
     */
    virtual void shutdown();

    /**
     * @brief 开始一帧渲染
     */
    virtual void beginFrame();

    /**
     * @brief 结束一帧渲染
     */
    virtual void endFrame();

    /**
     * @brief 渲染软组织
     */
    virtual void renderTissue(const Tissue* tissue);

    /**
     * @brief 渲染缝合线
     */
    virtual void renderSuture(const Suture* suture);

    /**
     * @brief 渲染器械
     */
    virtual void renderInstrument(const Instrument* instrument);

    /**
     * @brief 渲染接触点
     * @param position 接触位置
     * @param force 接触力大小（用于颜色映射）
     */
    virtual void renderContactPoint(const Vector3d& position, double force);

    /**
     * @brief 设置渲染模式
     */
    void setRenderMode(RenderMode mode) { render_mode_ = mode; }

    /**
     * @brief 获取当前渲染模式
     */
    RenderMode getRenderMode() const { return render_mode_; }

    /**
     * @brief 设置视图矩阵
     */
    void setViewMatrix(const Matrix4d& view) { view_matrix_ = view; }

    /**
     * @brief 设置投影矩阵
     */
    void setProjectionMatrix(const Matrix4d& proj) { projection_matrix_ = proj; }

    /**
     * @brief 检查窗口是否应该关闭
     */
    bool shouldClose() const { return should_close_; }

    /**
     * @brief 请求关闭窗口
     */
    void requestClose() { should_close_ = true; }

    /**
     * @brief 获取渲染统计
     */
    struct RenderStats {
        int draw_calls = 0;
        int triangle_count = 0;
        int vertex_count = 0;
        double frame_time_ms = 0.0;
    };
    const RenderStats& getStats() const { return stats_; }

    /**
     * @brief 清除渲染统计
     */
    void clearStats();

    /**
     * @brief 设置相机位置
     */
    void setCameraPosition(const Vector3d& pos) { camera_position_ = pos; }

    /**
     * @brief 设置相机目标点
     */
    void setCameraTarget(const Vector3d& target) { camera_target_ = target; }

protected:
    RenderConfig config_;
    RenderMode render_mode_ = RenderMode::SOLID;
    bool initialized_ = false;
    bool should_close_ = false;

    // 变换矩阵
    Matrix4d view_matrix_ = Matrix4d::Identity();
    Matrix4d projection_matrix_ = Matrix4d::Identity();

    // 相机参数
    Vector3d camera_position_ = Vector3d(0, 0, 5);
    Vector3d camera_target_ = Vector3d(0, 0, 0);

    // 渲染统计
    RenderStats stats_;

    // 网格数据缓存
    MeshRenderData tissue_mesh_data_;
    MeshRenderData suture_mesh_data_;
    MeshRenderData instrument_mesh_data_;

    /**
     * @brief 更新软组织网格数据
     */
    void updateTissueMeshData(const Tissue* tissue);

    /**
     * @brief 更新缝合线网格数据
     */
    void updateSutureMeshData(const Suture* suture);

    /**
     * @brief 更新器械网格数据
     */
    void updateInstrumentMeshData(const Instrument* instrument);

    /**
     * @brief 绘制网格
     */
    virtual void drawMesh(const MeshRenderData& mesh_data);

    /**
     * @brief 绘制线框
     */
    virtual void drawWireframe(const MeshRenderData& mesh_data);

    /**
     * @brief 绘制点
     */
    virtual void drawPoints(const MeshRenderData& mesh_data);
};

/**
 * @brief 简单控制台渲染器（用于无GUI测试）
 */
class ConsoleRenderer : public Renderer {
public:
    explicit ConsoleRenderer(const RenderConfig& config = RenderConfig{});
    
    bool initialize() override;
    void shutdown() override;
    void beginFrame() override;
    void endFrame() override;
    void renderTissue(const Tissue* tissue) override;
    void renderSuture(const Suture* suture) override;
    void renderInstrument(const Instrument* instrument) override;
    void renderContactPoint(const Vector3d& position, double force) override;

private:
    int frame_count_ = 0;
};

} // namespace suture
