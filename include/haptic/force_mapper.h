#pragma once

#include "../core/types.h"
#include <vector>
#include <map>

namespace vss {

/**
 * @brief 力映射器
 * 
 * 将物理仿真计算出的力映射到力反馈设备输出范围
 * 支持非线性映射和用户自定义校准
 * 
 * 映射公式：F_output = f(F_input)
 * - 默认使用分段线性映射
 * - 支持非线性映射（平方根、对数等）
 * - 支持用户校准数据
 */
class ForceMapper {
public:
    enum class MappingType {
        LINEAR,         // 线性映射
        SQUARE_ROOT,    // 平方根映射（增强小力感知）
        LOGARITHMIC,    // 对数映射（大范围力）
        PIECEWISE,      // 分段线性映射
        USER_DEFINED    // 用户自定义映射
    };

    struct CalibrationPoint {
        double input_force;   // 输入力值 (N)
        double output_force;  // 输出力值 (N)
        
        CalibrationPoint(double in = 0.0, double out = 0.0)
            : input_force(in), output_force(out) {}
    };

    struct Config {
        double input_min = 0.0;           // 输入力最小值 (N)
        double input_max = 10.0;          // 输入力最大值 (N)
        double output_min = 0.0;          // 输出力最小值 (N)
        double output_max = 5.0;          // 输出力最大值 (N)
        MappingType mapping_type = MappingType::PIECEWISE;
        double nonlinearity_factor = 0.5; // 非线性因子 (0-1)
    };

    explicit ForceMapper(const Config& config = Config{});
    ~ForceMapper() = default;

    // 禁止拷贝，允许移动
    ForceMapper(const ForceMapper&) = delete;
    ForceMapper& operator=(const ForceMapper&) = delete;
    ForceMapper(ForceMapper&&) = default;
    ForceMapper& operator=(ForceMapper&&) = default;

    /**
     * @brief 映射单轴力
     * @param input_force 输入力值
     * @return 映射后的输出力值
     */
    double map(double input_force) const;

    /**
     * @brief 映射三维力向量
     * @param input_force 输入力向量
     * @return 映射后的输出力向量
     */
    Vector3d map(const Vector3d& input_force) const;

    /**
     * @brief 添加校准点
     * @param input 输入力值
     * @param output 输出力值
     */
    void addCalibrationPoint(double input, double output);

    /**
     * @brief 添加校准点
     */
    void addCalibrationPoint(const CalibrationPoint& point);

    /**
     * @brief 清除所有校准点
     */
    void clearCalibrationPoints();

    /**
     * @brief 从校准点自动拟合映射曲线
     */
    void fitMappingCurve();

    /**
     * @brief 设置配置参数
     */
    void setConfig(const Config& config);

    /**
     * @brief 获取配置参数
     */
    const Config& getConfig() const { return config_; }

    /**
     * @brief 获取校准点列表
     */
    const std::vector<CalibrationPoint>& getCalibrationPoints() const { return calibration_points_; }

    /**
     * @brief 设置映射类型
     */
    void setMappingType(MappingType type);

    /**
     * @brief 保存校准数据到文件
     */
    bool saveCalibration(const std::string& filepath) const;

    /**
     * @brief 从文件加载校准数据
     */
    bool loadCalibration(const std::string& filepath);

private:
    Config config_;
    std::vector<CalibrationPoint> calibration_points_;

    // 分段线性插值系数缓存
    mutable std::map<double, double> piecewise_cache_;
    bool cache_dirty_ = true;

    /**
     * @brief 线性映射
     */
    double mapLinear(double input) const;

    /**
     * @brief 平方根映射（增强小力感知）
     */
    double mapSquareRoot(double input) const;

    /**
     * @brief 对数映射（大范围力）
     */
    double mapLogarithmic(double input) const;

    /**
     * @brief 分段线性映射（基于校准点）
     */
    double mapPiecewise(double input) const;

    /**
     * @brief 更新分段线性缓存
     */
    void updatePiecewiseCache() const;

    /**
     * @brief 线性插值
     */
    static double lerp(double x, double x0, double x1, double y0, double y1);

    /**
     * @brief 限制值在范围内
     */
    static double clamp(double value, double min, double max);
};

/**
 * @brief 力缩放器（简单线性缩放）
 */
class ForceScaler {
public:
    explicit ForceScaler(double scale_factor = 1.0);
    
    void setScaleFactor(double factor) { scale_factor_ = factor; }
    double getScaleFactor() const { return scale_factor_; }
    
    double scale(double force) const { return force * scale_factor_; }
    Vector3d scale(const Vector3d& force) const { return force * scale_factor_; }
    
private:
    double scale_factor_ = 1.0;
};

} // namespace vss
