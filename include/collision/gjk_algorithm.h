#pragma once

#include "types.h"
#include <array>

namespace vss {

// GJK (Gilbert-Johnson-Keerthi) 算法
// 用于计算两个凸包之间的最短距离

struct GJKResult {
    bool hasCollision;          // 是否碰撞
    Vector3d closestPointA;     // 物体A上的最近点
    Vector3d closestPointB;     // 物体B上的最近点
    Vector3d normal;            // 从A指向B的归一化方向
    double distance;            // 距离 (碰撞时为负的穿透深度)
    int iterations;             // 迭代次数
};

class GJKAlgorithm {
public:
    // 计算两个凸包的最短距离
    static GJKResult computeDistance(
        const std::vector<Vector3d>& shapeA,
        const std::vector<Vector3d>& shapeB,
        int maxIterations = 32,
        double tolerance = 1e-6
    );
    
    // 检测两个凸包是否相交
    static bool testIntersection(
        const std::vector<Vector3d>& shapeA,
        const std::vector<Vector3d>& shapeB,
        int maxIterations = 32
    );
    
    // 计算支撑点
    static Vector3d support(
        const std::vector<Vector3d>& shape,
        const Vector3d& direction
    );
    
    // 计算Minkowski差的支持点
    static Vector3d minkowskiSupport(
        const std::vector<Vector3d>& shapeA,
        const std::vector<Vector3d>& shapeB,
        const Vector3d& direction
    );
};

// EPA (Expanding Polytope Algorithm)
// 用于计算碰撞时的穿透深度和法向

struct EPAResult {
    bool success;               // 是否成功
    Vector3d normal;            // 碰撞法向 (指向物体A)
    double penetrationDepth;    // 穿透深度
    Vector3d contactPoint;      // 接触点
    int iterations;             // 迭代次数
};

class EPAAlgorithm {
public:
    // 计算穿透深度
    static EPAResult computePenetration(
        const std::vector<Vector3d>& shapeA,
        const std::vector<Vector3d>& shapeB,
        const std::array<Vector3d, 4>& simplex,  // GJK返回的单纯形
        int maxIterations = 32,
        double tolerance = 1e-6
    );
};

// 实用函数
namespace gjk_utils {
    // 计算点到三角形的最近点
    Vector3d closestPointOnTriangle(
        const Vector3d& p,
        const Vector3d& a,
        const Vector3d& b,
        const Vector3d& c
    );
    
    // 计算点到线段的最近点
    Vector3d closestPointOnSegment(
        const Vector3d& p,
        const Vector3d& a,
        const Vector3d& b
    );
    
    // 判断点是否在三角形内
    bool pointInTriangle(
        const Vector3d& p,
        const Vector3d& a,
        const Vector3d& b,
        const Vector3d& c
    );
    
    // 计算三角形法向
    Vector3d triangleNormal(
        const Vector3d& a,
        const Vector3d& b,
        const Vector3d& c
    );
}

} // namespace vss
