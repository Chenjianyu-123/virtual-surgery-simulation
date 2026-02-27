#pragma once

#include "types.h"
#include <vector>
#include <memory>

namespace vss {

// AABB树节点
struct AABBNode {
    AABB bbox;                  // 包围盒
    int left = -1;              // 左子节点索引 (-1表示叶子)
    int right = -1;             // 右子节点索引
    int objectIndex = -1;       // 物体索引 (仅叶子节点有效)
    int parent = -1;            // 父节点索引
    bool isLeaf = false;        // 是否为叶子节点
    
    AABBNode() = default;
    explicit AABBNode(const AABB& box) : bbox(box) {}
};

// AABB树
class AABBTree {
public:
    AABBTree();
    ~AABBTree();
    
    // 从物体列表构建树
    void build(const std::vector<AABB>& objectBboxes);
    
    // 增量式插入物体
    int insertObject(const AABB& bbox);
    
    // 移除物体
    void removeObject(int objectIndex);
    
    // 更新物体包围盒
    void updateObject(int objectIndex, const AABB& newBBox);
    
    // 查询与给定包围盒重叠的所有物体
    std::vector<int> queryOverlaps(const AABB& bbox) const;
    
    // 查询所有可能碰撞的物体对
    std::vector<std::pair<int, int>> queryAllOverlaps() const;
    
    // 射线检测
    bool raycast(const Ray& ray, int& hitObject, double& hitDistance) const;
    
    // 获取树的统计信息
    int getNodeCount() const { return static_cast<int>(nodes_.size()); }
    int getObjectCount() const { return static_cast<int>(objectIndices_.size()); }
    int getTreeDepth() const;
    
    // 清空树
    void clear();
    
    // 验证树的完整性
    bool validate() const;
    
private:
    std::vector<AABBNode> nodes_;
    std::vector<int> objectIndices_;    // 物体索引到节点索引的映射
    int root_ = -1;
    
    // 内部辅助函数
    int buildRecursive(const std::vector<int>& objectIndices, 
                       const std::vector<AABB>& objectBboxes);
    void queryOverlapsRecursive(int nodeIdx, const AABB& bbox, 
                                std::vector<int>& results) const;
    void queryAllOverlapsRecursive(int nodeA, int nodeB,
                                   std::vector<std::pair<int, int>>& pairs) const;
    bool raycastRecursive(int nodeIdx, const Ray& ray, 
                          int& hitObject, double& hitDistance) const;
    
    int allocateNode();
    void deallocateNode(int nodeIdx);
    void rotateTree(int nodeIdx);
    void fixUpwards(int nodeIdx);
    int findBestSibling(const AABB& bbox) const;
    double computeCost(int nodeIdx) const;
    
    // 计算两个AABB的并集
    static AABB merge(const AABB& a, const AABB& b);
    
    // 计算AABB的表面积 (用于SAH)
    static double surfaceArea(const AABB& bbox);
};

// 动态AABB树 (支持快速更新)
class DynamicAABBTree {
public:
    DynamicAABBTree();
    ~DynamicAABBTree();
    
    // 创建代理
    int createProxy(const AABB& bbox, void* userData = nullptr);
    
    // 销毁代理
    void destroyProxy(int proxyId);
    
    // 移动代理
    void moveProxy(int proxyId, const AABB& newBBox);
    
    // 查询重叠
    std::vector<int> queryOverlaps(int proxyId) const;
    std::vector<std::pair<int, int>> queryAllOverlaps() const;
    
    // 获取用户数据
    void* getUserData(int proxyId) const;
    
    // 获取包围盒
    AABB getAABB(int proxyId) const;
    
private:
    struct Proxy {
        AABB bbox;
        void* userData = nullptr;
        int nodeId = -1;
        bool isValid = false;
    };
    
    std::vector<Proxy> proxies_;
    AABBTree tree_;
    std::vector<int> freeProxies_;
};

} // namespace vss
