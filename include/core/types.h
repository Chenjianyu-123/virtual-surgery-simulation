#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <array>
#include <vector>
#include <memory>
#include <cstdint>

namespace vss {

using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using Vector3i = Eigen::Vector3i;
using Vector4d = Eigen::Vector4d;
using VectorXd = Eigen::VectorXd;

using Matrix2d = Eigen::Matrix2d;
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using MatrixXd = Eigen::MatrixXd;

using Quaterniond = Eigen::Quaterniond;
using AngleAxisd = Eigen::AngleAxisd;

using SparseMatrixd = Eigen::SparseMatrix<double>;
using Triplet = Eigen::Triplet<double>;

// 前向声明
class Tissue;
class Suture;
class Instrument;
struct ContactPair;

// AABB包围盒
struct AABB {
    Vector3d min;
    Vector3d max;
    
    AABB() : min(Vector3d::Zero()), max(Vector3d::Zero()) {}
    AABB(const Vector3d& minPoint, const Vector3d& maxPoint) 
        : min(minPoint), max(maxPoint) {}
    
    Vector3d center() const { return (min + max) * 0.5; }
    Vector3d extent() const { return max - min; }
    
    bool contains(const Vector3d& point) const {
        return (point.array() >= min.array()).all() && 
               (point.array() <= max.array()).all();
    }
    
    bool intersects(const AABB& other) const {
        return (min.array() <= other.max.array()).all() && 
               (max.array() >= other.min.array()).all();
    }
    
    void expand(const Vector3d& point) {
        min = min.cwiseMin(point);
        max = max.cwiseMax(point);
    }
    
    void expand(const AABB& other) {
        min = min.cwiseMin(other.min);
        max = max.cwiseMax(other.max);
    }
    
    double volume() const {
        Vector3d ext = extent();
        return ext.x() * ext.y() * ext.z();
    }
};

// 颜色结构
struct Color {
    float r, g, b, a;
    
    Color() : r(1.0f), g(1.0f), b(1.0f), a(1.0f) {}
    Color(float red, float green, float blue, float alpha = 1.0f)
        : r(red), g(green), b(blue), a(alpha) {}
    
    static Color Red() { return Color(1.0f, 0.0f, 0.0f); }
    static Color Green() { return Color(0.0f, 1.0f, 0.0f); }
    static Color Blue() { return Color(0.0f, 0.0f, 1.0f); }
    static Color White() { return Color(1.0f, 1.0f, 1.0f); }
    static Color Black() { return Color(0.0f, 0.0f, 0.0f); }
    static Color Gray(float v) { return Color(v, v, v); }
};

// 材质属性
struct Material {
    double youngsModulus;      // 杨氏模量 (Pa)
    double poissonRatio;       // 泊松比
    double density;            // 密度 (kg/m^3)
    double damping;            // 阻尼系数
    
    Material() 
        : youngsModulus(1e5), poissonRatio(0.45), 
          density(1000.0), damping(0.01) {}
    
    Material(double E, double nu, double rho, double damp)
        : youngsModulus(E), poissonRatio(nu), 
          density(rho), damping(damp) {}
    
    // 计算Lamé参数
    double lambda() const {
        return youngsModulus * poissonRatio / 
               ((1.0 + poissonRatio) * (1.0 - 2.0 * poissonRatio));
    }
    
    double mu() const {
        return youngsModulus / (2.0 * (1.0 + poissonRatio));
    }
    
    // 体积模量
    double bulkModulus() const {
        return youngsModulus / (3.0 * (1.0 - 2.0 * poissonRatio));
    }
};

// 仿真参数
struct SimulationParams {
    double timeStep = 0.001;           // 时间步长 (s)
    int maxIterations = 10;            // 最大迭代次数
    double tolerance = 1e-6;           // 收敛容差
    double contactStiffness = 1e6;     // 接触刚度
    double frictionCoefficient = 0.3;  // 摩擦系数
    bool useGPU = true;                // 是否使用GPU加速
};

// 渲染参数
struct RenderParams {
    bool wireframe = false;
    bool showNormals = false;
    bool showContacts = false;
    Color backgroundColor = Color(0.2f, 0.2f, 0.2f);
    float pointSize = 3.0f;
    float lineWidth = 1.0f;
};

// 力反馈参数
struct HapticParams {
    double normalCutoffFreq = 10.0;    // 正常状态截止频率 (Hz)
    double eventCutoffFreq = 50.0;     // 事件状态截止频率 (Hz)
    double forceScale = 1.0;           // 力缩放因子
    double maxForce = 10.0;            // 最大输出力 (N)
    double predictionHorizon = 0.001;  // 预测时间 (s)
};

// 时间统计
struct TimingStats {
    double totalTime = 0.0;
    double collisionTime = 0.0;
    double physicsTime = 0.0;
    double hapticTime = 0.0;
    double renderTime = 0.0;
    int frameCount = 0;
    
    double averageFps() const {
        return frameCount > 0 ? frameCount / totalTime : 0.0;
    }
    
    void reset() {
        totalTime = collisionTime = physicsTime = hapticTime = renderTime = 0.0;
        frameCount = 0;
    }
};

// 仿射变换
struct Transform {
    Matrix3d rotation;
    Vector3d translation;
    
    Transform() : rotation(Matrix3d::Identity()), translation(Vector3d::Zero()) {}
    
    explicit Transform(const Quaterniond& q, const Vector3d& t = Vector3d::Zero())
        : rotation(q.toRotationMatrix()), translation(t) {}
    
    Vector3d apply(const Vector3d& point) const {
        return rotation * point + translation;
    }
    
    Vector3d applyInverse(const Vector3d& point) const {
        return rotation.transpose() * (point - translation);
    }
    
    Transform inverse() const {
        Transform result;
        result.rotation = rotation.transpose();
        result.translation = -result.rotation * translation;
        return result;
    }
    
    Transform operator*(const Transform& other) const {
        Transform result;
        result.rotation = rotation * other.rotation;
        result.translation = rotation * other.translation + translation;
        return result;
    }
};

// 射线
struct Ray {
    Vector3d origin;
    Vector3d direction;
    
    Ray() : origin(Vector3d::Zero()), direction(Vector3d::UnitZ()) {}
    Ray(const Vector3d& o, const Vector3d& d) : origin(o), direction(d.normalized()) {}
    
    Vector3d at(double t) const {
        return origin + t * direction;
    }
};

// 平面
struct Plane {
    Vector3d normal;
    double distance;
    
    Plane() : normal(Vector3d::UnitZ()), distance(0.0) {}
    Plane(const Vector3d& n, double d) : normal(n.normalized()), distance(d) {}
    Plane(const Vector3d& n, const Vector3d& point) 
        : normal(n.normalized()), distance(-n.dot(point)) {}
    
    double distanceTo(const Vector3d& point) const {
        return normal.dot(point) + distance;
    }
    
    Vector3d project(const Vector3d& point) const {
        return point - distanceTo(point) * normal;
    }
};

// 线段
struct LineSegment {
    Vector3d start;
    Vector3d end;
    
    LineSegment() : start(Vector3d::Zero()), end(Vector3d::Zero()) {}
    LineSegment(const Vector3d& s, const Vector3d& e) : start(s), end(e) {}
    
    Vector3d direction() const {
        return end - start;
    }
    
    double length() const {
        return direction().norm();
    }
    
    Vector3d at(double t) const {
        return start + t * direction();
    }
    
    double closestPointParameter(const Vector3d& point) const {
        Vector3d dir = direction();
        double lenSq = dir.squaredNorm();
        if (lenSq < 1e-10) return 0.0;
        return std::clamp((point - start).dot(dir) / lenSq, 0.0, 1.0);
    }
    
    Vector3d closestPoint(const Vector3d& point) const {
        return at(closestPointParameter(point));
    }
};

// 三角形
struct Triangle {
    std::array<Vector3d, 3> vertices;
    
    Triangle() {
        vertices[0] = vertices[1] = vertices[2] = Vector3d::Zero();
    }
    
    Triangle(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2) {
        vertices[0] = v0;
        vertices[1] = v1;
        vertices[2] = v2;
    }
    
    Vector3d normal() const {
        return (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]).normalized();
    }
    
    double area() const {
        return 0.5 * (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]).norm();
    }
    
    Vector3d centroid() const {
        return (vertices[0] + vertices[1] + vertices[2]) / 3.0;
    }
    
    Plane plane() const {
        return Plane(normal(), vertices[0]);
    }
    
    // 计算重心坐标
    std::array<double, 3> barycentricCoords(const Vector3d& point) const {
        Vector3d v0 = vertices[1] - vertices[0];
        Vector3d v1 = vertices[2] - vertices[0];
        Vector3d v2 = point - vertices[0];
        double d00 = v0.dot(v0);
        double d01 = v0.dot(v1);
        double d11 = v1.dot(v1);
        double d20 = v2.dot(v0);
        double d21 = v2.dot(v1);
        double denom = d00 * d11 - d01 * d01;
        std::array<double, 3> coords;
        coords[1] = (d11 * d20 - d01 * d21) / denom;
        coords[2] = (d00 * d21 - d01 * d20) / denom;
        coords[0] = 1.0 - coords[1] - coords[2];
        return coords;
    }
    
    // 最近点
    Vector3d closestPoint(const Vector3d& point) const {
        auto coords = barycentricCoords(point);
        Vector3d result = Vector3d::Zero();
        for (int i = 0; i < 3; ++i) {
            result += coords[i] * vertices[i];
        }
        return result;
    }
    
    // 射线相交测试
    bool intersectRay(const Ray& ray, double& t, double& u, double& v) const {
        Vector3d edge1 = vertices[1] - vertices[0];
        Vector3d edge2 = vertices[2] - vertices[0];
        Vector3d h = ray.direction.cross(edge2);
        double a = edge1.dot(h);
        if (std::abs(a) < 1e-10) return false;
        double f = 1.0 / a;
        Vector3d s = ray.origin - vertices[0];
        u = f * s.dot(h);
        if (u < 0.0 || u > 1.0) return false;
        Vector3d q = s.cross(edge1);
        v = f * ray.direction.dot(q);
        if (v < 0.0 || u + v > 1.0) return false;
        t = f * edge2.dot(q);
        return t >= 0.0;
    }
};

// 四面体
struct Tetrahedron {
    std::array<Vector3d, 4> vertices;
    
    Tetrahedron() {
        vertices[0] = vertices[1] = vertices[2] = vertices[3] = Vector3d::Zero();
    }
    
    Tetrahedron(const Vector3d& v0, const Vector3d& v1, 
                const Vector3d& v2, const Vector3d& v3) {
        vertices[0] = v0;
        vertices[1] = v1;
        vertices[2] = v2;
        vertices[3] = v3;
    }
    
    double volume() const {
        return std::abs((vertices[1] - vertices[0]).dot(
            (vertices[2] - vertices[0]).cross(vertices[3] - vertices[0]))) / 6.0;
    }
    
    Vector3d centroid() const {
        return (vertices[0] + vertices[1] + vertices[2] + vertices[3]) / 4.0;
    }
    
    // 计算重心坐标
    std::array<double, 4> barycentricCoords(const Vector3d& point) const {
        Matrix3d T;
        T.col(0) = vertices[1] - vertices[0];
        T.col(1) = vertices[2] - vertices[0];
        T.col(2) = vertices[3] - vertices[0];
        Vector3d diff = point - vertices[0];
        Vector3d coords_vec = T.inverse() * diff;
        std::array<double, 4> result;
        result[1] = coords_vec.x();
        result[2] = coords_vec.y();
        result[3] = coords_vec.z();
        result[0] = 1.0 - result[1] - result[2] - result[3];
        return result;
    }
    
    // 包含测试
    bool contains(const Vector3d& point) const {
        auto coords = barycentricCoords(point);
        for (double c : coords) {
            if (c < -1e-10 || c > 1.0 + 1e-10) return false;
        }
        return true;
    }
    
    // 射线相交测试 (Möller-Trumbore)
    bool intersectRay(const Ray& ray, double& t, std::array<double, 4>& coords) const {
        Vector3d edge1 = vertices[1] - vertices[0];
        Vector3d edge2 = vertices[2] - vertices[0];
        Vector3d h = ray.direction.cross(edge2);
        double a = edge1.dot(h);
        if (std::abs(a) < 1e-10) return false;
        double f = 1.0 / a;
        Vector3d s = ray.origin - vertices[0];
        coords[1] = f * s.dot(h);
        if (coords[1] < 0.0 || coords[1] > 1.0) return false;
        Vector3d q = s.cross(edge1);
        coords[2] = f * ray.direction.dot(q);
        if (coords[2] < 0.0 || coords[1] + coords[2] > 1.0) return false;
        coords[3] = f * edge2.dot(q);
        if (coords[3] < 0.0 || coords[1] + coords[2] + coords[3] > 1.0) return false;
        coords[0] = 1.0 - coords[1] - coords[2] - coords[3];
        t = f * edge2.dot(q);
        return t >= 0.0;
    }
    
    // 获取AABB
    AABB boundingBox() const {
        AABB box;
        box.min = box.max = vertices[0];
        for (int i = 1; i < 4; ++i) {
            box.min = box.min.cwiseMin(vertices[i]);
            box.max = box.max.cwiseMax(vertices[i]);
        }
        return box;
    }
};

} // namespace vss
