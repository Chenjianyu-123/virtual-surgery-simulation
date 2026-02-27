#pragma once

#include "types.h"
#include <cmath>
#include <algorithm>

namespace vss {
namespace math {

// 线性插值
template<typename T>
inline T lerp(const T& a, const T& b, double t) {
    return a + (b - a) * t;
}

// 平滑步函数
double smoothstep(double edge0, double edge1, double x);

// 更平滑的步函数
double smootherstep(double edge0, double edge1, double x);

// 限制值在范围内
template<typename T>
inline T clamp(const T& value, const T& min, const T& max) {
    return std::max(min, std::min(value, max));
}

// 符号函数
template<typename T>
inline int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

// 四元数工具
namespace quaternion {
    // 从旋转轴和角度创建四元数
    Quaterniond fromAxisAngle(const Vector3d& axis, double angle);
    
    // 从欧拉角创建四元数 (roll, pitch, yaw)
    Quaterniond fromEulerAngles(double roll, double pitch, double yaw);
    
    // 四元数到欧拉角
    Vector3d toEulerAngles(const Quaterniond& q);
    
    // 球面线性插值
    Quaterniond slerp(const Quaterniond& q1, const Quaterniond& q2, double t);
    
    // 四元数微分 (用于角速度积分)
    Quaterniond derivative(const Quaterniond& q, const Vector3d& angularVelocity);
    
    // 从两个向量计算旋转四元数
    Quaterniond fromToRotation(const Vector3d& from, const Vector3d& to);
}

// 矩阵工具
namespace matrix {
    // 创建 skew-symmetric 矩阵
    Matrix3d skewSymmetric(const Vector3d& v);
    
    // 从 skew-symmetric 矩阵提取向量
    Vector3d fromSkewSymmetric(const Matrix3d& m);
    
    // 计算矩阵的 Frobenius 范数
    double frobeniusNorm(const Matrix3d& m);
    
    // 极分解: F = R * S
    void polarDecomposition(const Matrix3d& F, Matrix3d& R, Matrix3d& S);
    
    // SVD 分解
    void svd(const Matrix3d& A, Matrix3d& U, Vector3d& S, Matrix3d& V);
    
    // 计算惯性张量 (对于点集)
    Matrix3d computeInertiaTensor(const std::vector<Vector3d>& points, 
                                   const std::vector<double>& masses);
}

// 几何工具
namespace geometry {
    // 点到平面的距离
    double pointPlaneDistance(const Vector3d& point, const Plane& plane);
    
    // 点到平面的投影
    Vector3d projectPointToPlane(const Vector3d& point, const Plane& plane);
    
    // 点到线段的距离
    double pointSegmentDistance(const Vector3d& point, const LineSegment& segment);
    
    // 点到三角形的距离
    double pointTriangleDistance(const Vector3d& point, const Triangle& triangle);
    
    // 线段与三角形相交
    bool segmentTriangleIntersect(const LineSegment& segment, const Triangle& triangle,
                                   Vector3d& intersection);
    
    // 计算三角形面积
    double triangleArea(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2);
    
    // 计算四面体体积
    double tetrahedronVolume(const Vector3d& v0, const Vector3d& v1, 
                              const Vector3d& v2, const Vector3d& v3);
    
    // 计算重心坐标
    std::array<double, 3> barycentricCoordinates(const Vector3d& p,
                                                  const Vector3d& a,
                                                  const Vector3d& b,
                                                  const Vector3d& c);
    
    // 计算四面体重心坐标
    std::array<double, 4> tetrahedralCoordinates(const Vector3d& p,
                                                  const Vector3d& v0,
                                                  const Vector3d& v1,
                                                  const Vector3d& v2,
                                                  const Vector3d& v3);
}

// 数值工具
namespace numeric {
    // 求解二次方程 ax^2 + bx + c = 0
    // 返回实数解的个数
    int solveQuadratic(double a, double b, double c, double& x1, double& x2);
    
    // 求解三次方程 ax^3 + bx^2 + cx + d = 0
    // 返回实数解的个数
    int solveCubic(double a, double b, double c, double d, 
                   double roots[3]);
    
    // 判断数值是否接近零
    inline bool isZero(double val, double epsilon = 1e-10) {
        return std::abs(val) < epsilon;
    }
    
    // 判断两个数值是否近似相等
    inline bool approxEqual(double a, double b, double epsilon = 1e-10) {
        return std::abs(a - b) < epsilon;
    }
    
    // 判断向量是否近似相等
    inline bool approxEqual(const Vector3d& a, const Vector3d& b, double epsilon = 1e-10) {
        return (a - b).norm() < epsilon;
    }
}

// 随机数生成
namespace random {
    // 初始化随机数生成器
    void seed(unsigned int s);
    
    // 生成 [0, 1) 范围内的随机数
    double uniform();
    
    // 生成 [min, max) 范围内的随机数
    double uniform(double min, double max);
    
    // 生成正态分布随机数
    double normal(double mean = 0.0, double stddev = 1.0);
    
    // 生成单位球面上的随机点
    Vector3d uniformSphere();
    
    // 生成单位半球面上的随机点 (给定法向)
    Vector3d uniformHemisphere(const Vector3d& normal);
}

// 统计工具
namespace statistics {
    // 计算均值
    double mean(const std::vector<double>& data);
    
    // 计算标准差
    double stddev(const std::vector<double>& data);
    
    // 计算方差
    double variance(const std::vector<double>& data);
    
    // 计算协方差
    double covariance(const std::vector<double>& x, const std::vector<double>& y);
    
    // 线性回归: y = a * x + b
    void linearRegression(const std::vector<double>& x, const std::vector<double>& y,
                          double& a, double& b);
    
    // 计算相关系数
    double correlation(const std::vector<double>& x, const std::vector<double>& y);
}

} // namespace math
} // namespace vss
