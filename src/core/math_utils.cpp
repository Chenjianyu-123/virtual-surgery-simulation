#include "math_utils.h"
#include <random>
#include <cmath>

namespace vss {
namespace math {

// 平滑步函数
double smoothstep(double edge0, double edge1, double x) {
    x = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

// 更平滑的步函数
double smootherstep(double edge0, double edge1, double x) {
    x = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return x * x * x * (x * (x * 6.0 - 15.0) + 10.0);
}

// 四元数工具
namespace quaternion {
    Quaterniond fromAxisAngle(const Vector3d& axis, double angle) {
        Vector3d normalizedAxis = axis.normalized();
        double halfAngle = angle * 0.5;
        double sinHalfAngle = std::sin(halfAngle);
        return Quaterniond(
            std::cos(halfAngle),
            normalizedAxis.x() * sinHalfAngle,
            normalizedAxis.y() * sinHalfAngle,
            normalizedAxis.z() * sinHalfAngle
        );
    }
    
    Quaterniond fromEulerAngles(double roll, double pitch, double yaw) {
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);
        
        return Quaterniond(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        );
    }
    
    Vector3d toEulerAngles(const Quaterniond& q) {
        Vector3d angles;
        
        // roll (x-axis rotation)
        double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
        double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
        angles.x() = std::atan2(sinr_cosp, cosr_cosp);
        
        // pitch (y-axis rotation)
        double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
        if (std::abs(sinp) >= 1.0)
            angles.y() = std::copysign(M_PI / 2.0, sinp);
        else
            angles.y() = std::asin(sinp);
        
        // yaw (z-axis rotation)
        double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
        double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
        angles.z() = std::atan2(siny_cosp, cosy_cosp);
        
        return angles;
    }
    
    Quaterniond slerp(const Quaterniond& q1, const Quaterniond& q2, double t) {
        Quaterniond q2Adj = q2;
        double dot = q1.dot(q2);
        
        // 如果点积为负，取反一个四元数以走短路径
        if (dot < 0.0) {
            dot = -dot;
            q2Adj.coeffs() = -q2Adj.coeffs();
        }
        
        const double DOT_THRESHOLD = 0.9995;
        if (dot > DOT_THRESHOLD) {
            // 如果四元数非常接近，使用线性插值
            Quaterniond result = Quaterniond(
                q1.coeffs() + t * (q2Adj.coeffs() - q1.coeffs())
            );
            result.normalize();
            return result;
        }
        
        double theta_0 = std::acos(dot);
        double theta = theta_0 * t;
        double sin_theta = std::sin(theta);
        double sin_theta_0 = std::sin(theta_0);
        
        double s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
        double s1 = sin_theta / sin_theta_0;
        
        return Quaterniond(q1.coeffs() * s0 + q2Adj.coeffs() * s1);
    }
    
    Quaterniond derivative(const Quaterniond& q, const Vector3d& angularVelocity) {
        Quaterniond omega(0, angularVelocity.x(), angularVelocity.y(), angularVelocity.z());
        Quaterniond result = omega * q;
        result.coeffs() *= 0.5;
        return result;
    }
    
    Quaterniond fromToRotation(const Vector3d& from, const Vector3d& to) {
        Vector3d f = from.normalized();
        Vector3d t = to.normalized();
        
        double dot = f.dot(t);
        
        if (dot >= 1.0 - 1e-6) {
            return Quaterniond::Identity();
        }
        
        if (dot <= -1.0 + 1e-6) {
            // 向量相反，需要找到一个垂直轴
            Vector3d axis;
            if (std::abs(f.x()) < std::abs(f.y()) && std::abs(f.x()) < std::abs(f.z())) {
                axis = Vector3d(1, 0, 0).cross(f).normalized();
            } else if (std::abs(f.y()) < std::abs(f.z())) {
                axis = Vector3d(0, 1, 0).cross(f).normalized();
            } else {
                axis = Vector3d(0, 0, 1).cross(f).normalized();
            }
            return fromAxisAngle(axis, M_PI);
        }
        
        Vector3d axis = f.cross(t);
        double w = std::sqrt((1.0 + dot) * 2.0);
        double invW = 1.0 / w;
        
        return Quaterniond(w * 0.5, axis.x() * invW, axis.y() * invW, axis.z() * invW).normalized();
    }
}

// 矩阵工具
namespace matrix {
    Matrix3d skewSymmetric(const Vector3d& v) {
        Matrix3d m;
        m << 0, -v.z(), v.y(),
             v.z(), 0, -v.x(),
             -v.y(), v.x(), 0;
        return m;
    }
    
    Vector3d fromSkewSymmetric(const Matrix3d& m) {
        return Vector3d(m(2, 1), m(0, 2), m(1, 0));
    }
    
    double frobeniusNorm(const Matrix3d& m) {
        return std::sqrt(m.cwiseAbs2().sum());
    }
    
    void polarDecomposition(const Matrix3d& F, Matrix3d& R, Matrix3d& S) {
        // 使用SVD进行极分解: F = U * Sigma * V^T = (U * V^T) * (V * Sigma * V^T) = R * S
        Eigen::JacobiSVD<Matrix3d> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Matrix3d U = svd.matrixU();
        Matrix3d V = svd.matrixV();
        
        // 确保旋转矩阵的行列式为1
        Matrix3d D = Matrix3d::Identity();
        D(2, 2) = (U * V.transpose()).determinant();
        
        R = U * D * V.transpose();
        S = V * D * svd.singularValues().asDiagonal() * V.transpose();
    }
    
    void svd(const Matrix3d& A, Matrix3d& U, Vector3d& S, Matrix3d& V) {
        Eigen::JacobiSVD<Matrix3d> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
        U = svd.matrixU();
        S = svd.singularValues();
        V = svd.matrixV();
    }
    
    Matrix3d computeInertiaTensor(const std::vector<Vector3d>& points, 
                                   const std::vector<double>& masses) {
        Matrix3d inertia = Matrix3d::Zero();
        
        if (points.size() != masses.size() || points.empty()) {
            return inertia;
        }
        
        // 计算质心
        Vector3d centerOfMass = Vector3d::Zero();
        double totalMass = 0.0;
        for (size_t i = 0; i < points.size(); ++i) {
            centerOfMass += masses[i] * points[i];
            totalMass += masses[i];
        }
        centerOfMass /= totalMass;
        
        // 计算惯性张量
        for (size_t i = 0; i < points.size(); ++i) {
            Vector3d r = points[i] - centerOfMass;
            double m = masses[i];
            
            inertia(0, 0) += m * (r.y() * r.y() + r.z() * r.z());
            inertia(1, 1) += m * (r.x() * r.x() + r.z() * r.z());
            inertia(2, 2) += m * (r.x() * r.x() + r.y() * r.y());
            inertia(0, 1) -= m * r.x() * r.y();
            inertia(0, 2) -= m * r.x() * r.z();
            inertia(1, 2) -= m * r.y() * r.z();
        }
        
        inertia(1, 0) = inertia(0, 1);
        inertia(2, 0) = inertia(0, 2);
        inertia(2, 1) = inertia(1, 2);
        
        return inertia;
    }
}

// 几何工具
namespace geometry {
    double pointPlaneDistance(const Vector3d& point, const Plane& plane) {
        return plane.distanceTo(point);
    }
    
    Vector3d projectPointToPlane(const Vector3d& point, const Plane& plane) {
        return plane.project(point);
    }
    
    double pointSegmentDistance(const Vector3d& point, const LineSegment& segment) {
        return (point - segment.closestPoint(point)).norm();
    }
    
    double pointTriangleDistance(const Vector3d& point, const Triangle& triangle) {
        return (point - triangle.closestPoint(point)).norm();
    }
    
    bool segmentTriangleIntersect(const LineSegment& segment, const Triangle& triangle,
                                   Vector3d& intersection) {
        Vector3d edge1 = triangle.vertices[1] - triangle.vertices[0];
        Vector3d edge2 = triangle.vertices[2] - triangle.vertices[0];
        Vector3d h = segment.direction().cross(edge2);
        double a = edge1.dot(h);
        
        if (std::abs(a) < 1e-10) {
            return false;
        }
        
        double f = 1.0 / a;
        Vector3d s = segment.start - triangle.vertices[0];
        double u = f * s.dot(h);
        
        if (u < 0.0 || u > 1.0) {
            return false;
        }
        
        Vector3d q = s.cross(edge1);
        double v = f * segment.direction().dot(q);
        
        if (v < 0.0 || u + v > 1.0) {
            return false;
        }
        
        double t = f * edge2.dot(q);
        
        if (t >= 0.0 && t <= 1.0) {
            intersection = segment.at(t);
            return true;
        }
        
        return false;
    }
    
    double triangleArea(const Vector3d& v0, const Vector3d& v1, const Vector3d& v2) {
        return 0.5 * (v1 - v0).cross(v2 - v0).norm();
    }
    
    double tetrahedronVolume(const Vector3d& v0, const Vector3d& v1, 
                              const Vector3d& v2, const Vector3d& v3) {
        return std::abs((v1 - v0).dot((v2 - v0).cross(v3 - v0))) / 6.0;
    }
    
    std::array<double, 3> barycentricCoordinates(const Vector3d& p,
                                                  const Vector3d& a,
                                                  const Vector3d& b,
                                                  const Vector3d& c) {
        Vector3d v0 = b - a;
        Vector3d v1 = c - a;
        Vector3d v2 = p - a;
        
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
    
    std::array<double, 4> tetrahedralCoordinates(const Vector3d& p,
                                                  const Vector3d& v0,
                                                  const Vector3d& v1,
                                                  const Vector3d& v2,
                                                  const Vector3d& v3) {
        Matrix3d T;
        T.col(0) = v1 - v0;
        T.col(1) = v2 - v0;
        T.col(2) = v3 - v0;
        
        Vector3d diff = p - v0;
        Vector3d coords = T.inverse() * diff;
        
        std::array<double, 4> result;
        result[1] = coords.x();
        result[2] = coords.y();
        result[3] = coords.z();
        result[0] = 1.0 - result[1] - result[2] - result[3];
        
        return result;
    }
}

// 数值工具
namespace numeric {
    int solveQuadratic(double a, double b, double c, double& x1, double& x2) {
        double discriminant = b * b - 4.0 * a * c;
        
        if (discriminant < 0.0) {
            return 0;
        }
        
        if (discriminant == 0.0) {
            x1 = x2 = -b / (2.0 * a);
            return 1;
        }
        
        double sqrtDisc = std::sqrt(discriminant);
        x1 = (-b + sqrtDisc) / (2.0 * a);
        x2 = (-b - sqrtDisc) / (2.0 * a);
        return 2;
    }
    
    int solveCubic(double a, double b, double c, double d, double roots[3]) {
        // 归一化: x^3 + px + q = 0
        double invA = 1.0 / a;
        double p = (3.0 * a * c - b * b) * invA * invA / 3.0;
        double q = (2.0 * b * b * b - 9.0 * a * b * c + 27.0 * a * a * d) * invA * invA * invA / 27.0;
        
        double discriminant = q * q / 4.0 + p * p * p / 27.0;
        
        double offset = -b * invA / 3.0;
        
        if (discriminant > 0.0) {
            // 一个实根
            double sqrtDisc = std::sqrt(discriminant);
            double u = std::cbrt(-q / 2.0 + sqrtDisc);
            double v = std::cbrt(-q / 2.0 - sqrtDisc);
            roots[0] = u + v + offset;
            return 1;
        } else if (discriminant == 0.0) {
            // 三个实根，其中两个相等
            double u = std::cbrt(-q / 2.0);
            roots[0] = 2.0 * u + offset;
            roots[1] = -u + offset;
            roots[2] = roots[1];
            return 2;
        } else {
            // 三个不同的实根
            double rho = std::sqrt(-p * p * p / 27.0);
            double theta = std::acos(-q / (2.0 * rho));
            double rhoCbrt = std::cbrt(rho);
            double twoRhoCbrt = 2.0 * rhoCbrt;
            
            roots[0] = twoRhoCbrt * std::cos(theta / 3.0) + offset;
            roots[1] = twoRhoCbrt * std::cos((theta + 2.0 * M_PI) / 3.0) + offset;
            roots[2] = twoRhoCbrt * std::cos((theta + 4.0 * M_PI) / 3.0) + offset;
            return 3;
        }
    }
}

// 随机数生成
namespace random {
    static std::mt19937 generator;
    static std::uniform_real_distribution<double> uniformDist(0.0, 1.0);
    static std::normal_distribution<double> normalDist(0.0, 1.0);
    
    void seed(unsigned int s) {
        generator.seed(s);
    }
    
    double uniform() {
        return uniformDist(generator);
    }
    
    double uniform(double min, double max) {
        return min + uniform() * (max - min);
    }
    
    double normal(double mean, double stddev) {
        return mean + normalDist(generator) * stddev;
    }
    
    Vector3d uniformSphere() {
        double u = uniform();
        double v = uniform();
        
        double theta = 2.0 * M_PI * u;
        double phi = std::acos(2.0 * v - 1.0);
        
        double sinPhi = std::sin(phi);
        return Vector3d(
            sinPhi * std::cos(theta),
            sinPhi * std::sin(theta),
            std::cos(phi)
        );
    }
    
    Vector3d uniformHemisphere(const Vector3d& normal) {
        Vector3d v = uniformSphere();
        if (v.dot(normal) < 0.0) {
            v = -v;
        }
        return v;
    }
}

// 统计工具
namespace statistics {
    double mean(const std::vector<double>& data) {
        if (data.empty()) return 0.0;
        double sum = 0.0;
        for (double x : data) sum += x;
        return sum / data.size();
    }
    
    double variance(const std::vector<double>& data) {
        if (data.size() < 2) return 0.0;
        double m = mean(data);
        double sum = 0.0;
        for (double x : data) sum += (x - m) * (x - m);
        return sum / (data.size() - 1);
    }
    
    double stddev(const std::vector<double>& data) {
        return std::sqrt(variance(data));
    }
    
    double covariance(const std::vector<double>& x, const std::vector<double>& y) {
        if (x.size() != y.size() || x.size() < 2) return 0.0;
        
        double meanX = mean(x);
        double meanY = mean(y);
        double sum = 0.0;
        
        for (size_t i = 0; i < x.size(); ++i) {
            sum += (x[i] - meanX) * (y[i] - meanY);
        }
        
        return sum / (x.size() - 1);
    }
    
    void linearRegression(const std::vector<double>& x, const std::vector<double>& y,
                          double& a, double& b) {
        if (x.size() != y.size() || x.size() < 2) {
            a = b = 0.0;
            return;
        }
        
        double meanX = mean(x);
        double meanY = mean(y);
        double covXY = covariance(x, y);
        double varX = variance(x);
        
        if (std::abs(varX) < 1e-10) {
            a = 0.0;
            b = meanY;
            return;
        }
        
        a = covXY / varX;
        b = meanY - a * meanX;
    }
    
    double correlation(const std::vector<double>& x, const std::vector<double>& y) {
        double covXY = covariance(x, y);
        double stdX = stddev(x);
        double stdY = stddev(y);
        
        if (stdX < 1e-10 || stdY < 1e-10) return 0.0;
        
        return covXY / (stdX * stdY);
    }
}

} // namespace math
} // namespace vss
