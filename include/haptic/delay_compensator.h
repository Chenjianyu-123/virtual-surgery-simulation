#pragma once

#include "../core/types.h"
#include <deque>
#include <vector>

namespace vss {

/**
 * @brief 延迟补偿器
 * 
 * 使用自回归(AR)模型预测力信号，补偿力反馈延迟
 * 
 * 算法原理：
 * - 建立AR模型：F(t) = a1*F(t-1) + a2*F(t-2) + ... + an*F(t-n) + e(t)
 * - 在线更新AR系数
 * - 基于历史数据预测未来力值
 * - 补偿渲染和通信延迟
 */
class DelayCompensator {
public:
    struct Config {
        int ar_order = 4;                 // AR模型阶数
        int window_size = 50;             // 滑动窗口大小
        double learning_rate = 0.01;      // 在线学习率
        double prediction_horizon = 0.005; // 预测时间范围 (s)
        double sampling_rate = 1000.0;    // 采样率 (Hz)
    };

    explicit DelayCompensator(const Config& config = Config{});
    ~DelayCompensator() = default;

    // 禁止拷贝，允许移动
    DelayCompensator(const DelayCompensator&) = delete;
    DelayCompensator& operator=(const DelayCompensator&) = delete;
    DelayCompensator(DelayCompensator&&) = default;
    DelayCompensator& operator=(DelayCompensator&&) = default;

    /**
     * @brief 重置补偿器
     */
    void reset();

    /**
     * @brief 更新力测量值并获取预测值
     * @param measured_force 当前测量的力值
     * @return 预测的未来力值（已补偿延迟）
     */
    double update(double measured_force);

    /**
     * @brief 更新三维力向量
     */
    Vector3d update(const Vector3d& measured_force);

    /**
     * @brief 获取当前AR系数
     */
    const std::vector<double>& getARCoefficients() const { return ar_coeffs_; }

    /**
     * @brief 获取预测误差（用于监控）
     */
    double getPredictionError() const { return prediction_error_; }

    /**
     * @brief 设置配置参数
     */
    void setConfig(const Config& config);

    /**
     * @brief 获取配置参数
     */
    const Config& getConfig() const { return config_; }

    /**
     * @brief 检查是否已收集足够数据进行预测
     */
    bool isReady() const { return history_.size() >= static_cast<size_t>(config_.ar_order); }

private:
    Config config_;
    
    // AR模型系数
    std::vector<double> ar_coeffs_;
    
    // 历史数据滑动窗口
    std::deque<double> history_;
    
    // 预测误差
    double prediction_error_ = 0.0;
    
    // 上一次的预测值
    double last_prediction_ = 0.0;
    
    // 三通道历史（用于向量输入）
    std::deque<Vector3d> history_vec_;
    std::vector<Vector3d> ar_coeffs_vec_;
    Vector3d last_prediction_vec_ = Vector3d::Zero();

    /**
     * @brief 使用当前AR系数预测未来值
     */
    double predict() const;
    
    /**
     * @brief 更新AR系数（在线学习）
     * @param actual 实际测量值
     * @param predicted 之前的预测值
     */
    void updateARCoefficients(double actual, double predicted);
    
    /**
     * @brief 向历史窗口添加新数据
     */
    void addToHistory(double value);
    
    // 向量版本
    Vector3d predictVector() const;
    void updateARCoefficientsVector(const Vector3d& actual, const Vector3d& predicted);
    void addToHistoryVector(const Vector3d& value);
};

/**
 * @brief 简单线性预测器（用于对比测试）
 */
class LinearPredictor {
public:
    explicit LinearPredictor(int window_size = 10);
    
    double predict(double new_value);
    void reset();
    
private:
    std::deque<double> window_;
    int window_size_;
    double slope_ = 0.0;
    double intercept_ = 0.0;
    
    void fitLinearModel();
};

} // namespace vss
