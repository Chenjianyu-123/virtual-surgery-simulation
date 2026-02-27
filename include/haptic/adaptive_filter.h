#pragma once

#include "../core/types.h"
#include <deque>
#include <functional>

namespace vss {

/**
 * @brief 滤波器状态枚举
 */
enum class FilterState {
    NORMAL,     // 正常状态 - 低截止频率
    EVENT       // 事件状态 - 高截止频率
};

/**
 * @brief 自适应滤波器
 * 
 * 根据力信号变化自动调整截止频率：
 * - 正常状态：低截止频率（5-10Hz），平滑力信号
 * - 事件状态：高截止频率（50-100Hz），快速响应穿刺等事件
 * 
 * 状态切换基于力变化率检测
 */
class AdaptiveFilter {
public:
    struct Config {
        double normal_cutoff = 10.0;      // 正常状态截止频率 (Hz)
        double event_cutoff = 100.0;      // 事件状态截止频率 (Hz)
        double event_threshold = 0.5;     // 事件检测阈值 (N/ms)
        double event_decay_time = 0.1;    // 事件状态衰减时间 (s)
        double sampling_rate = 1000.0;    // 采样率 (Hz)
    };

    AdaptiveFilter(const Config& config = Config{});
    ~AdaptiveFilter() = default;

    // 禁止拷贝，允许移动
    AdaptiveFilter(const AdaptiveFilter&) = delete;
    AdaptiveFilter& operator=(const AdaptiveFilter&) = delete;
    AdaptiveFilter(AdaptiveFilter&&) = default;
    AdaptiveFilter& operator=(AdaptiveFilter&&) = default;

    /**
     * @brief 重置滤波器状态
     */
    void reset();

    /**
     * @brief 处理单轴力信号
     * @param force 输入力值
     * @return 滤波后的力值
     */
    double filter(double force);

    /**
     * @brief 处理三维力向量
     * @param force 输入力向量
     * @return 滤波后的力向量
     */
    Vector3d filter(const Vector3d& force);

    /**
     * @brief 获取当前状态
     */
    FilterState getState() const { return state_; }

    /**
     * @brief 获取当前截止频率
     */
    double getCurrentCutoff() const { return current_cutoff_; }

    /**
     * @brief 手动触发事件状态
     */
    void triggerEvent();

    /**
     * @brief 设置配置参数
     */
    void setConfig(const Config& config);

    /**
     * @brief 获取配置参数
     */
    const Config& getConfig() const { return config_; }

private:
    Config config_;
    
    // 状态
    FilterState state_ = FilterState::NORMAL;
    double current_cutoff_ = 10.0;
    
    // 一阶低通滤波器状态
    double prev_output_ = 0.0;
    double alpha_ = 0.0;  // 滤波系数
    
    // 事件检测
    double prev_force_ = 0.0;
    double event_timer_ = 0.0;
    
    // 三通道滤波器状态（用于向量输入）
    Vector3d prev_output_vec_ = Vector3d::Zero();
    
    /**
     * @brief 更新滤波系数
     */
    void updateAlpha();
    
    /**
     * @brief 检测事件（力突变）
     */
    bool detectEvent(double force, double dt);
    
    /**
     * @brief 更新状态机
     */
    void updateState(double dt);
};

/**
 * @brief 简单低通滤波器（用于非自适应场景）
 */
class LowPassFilter {
public:
    explicit LowPassFilter(double cutoff_hz = 10.0, double sample_rate_hz = 1000.0);
    
    void reset();
    double filter(double input);
    void setCutoff(double cutoff_hz);
    
private:
    double alpha_;
    double prev_output_ = 0.0;
    double sample_rate_;
    double cutoff_;
};

} // namespace vss
