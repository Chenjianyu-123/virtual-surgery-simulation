#include "aabb_tree.h"
#include <algorithm>
#include <stack>
#include <queue>

namespace vss {

// AABBTree 实现

AABBTree::AABBTree() = default;
AABBTree::~AABBTree() = default;

void AABBTree::build(const std::vector<AABB>& objectBboxes) {
    clear();
    
    if (objectBboxes.empty()) {
        return;
    }
    
    // 创建物体索引列表
    std::vector<int> indices(objectBboxes.size());
    for (size_t i = 0; i < indices.size(); ++i) {
        indices[i] = static_cast<int>(i);
    }
    
    // 递归构建
    root_ = buildRecursive(indices, objectBboxes);
    
    // 更新物体索引映射
    objectIndices_.resize(objectBboxes.size(), -1);
    for (size_t i = 0; i < nodes_.size(); ++i) {
        if (nodes_[i].isLeaf && nodes_[i].objectIndex >= 0) {
            objectIndices_[nodes_[i].objectIndex] = static_cast<int>(i);
        }
    }
}

int AABBTree::buildRecursive(const std::vector<int>& objectIndices,
                              const std::vector<AABB>& objectBboxes) {
    if (objectIndices.empty()) {
        return -1;
    }
    
    // 创建节点
    int nodeIdx = allocateNode();
    AABBNode& node = nodes_[nodeIdx];
    
    // 如果是单个物体，创建叶子节点
    if (objectIndices.size() == 1) {
        node.isLeaf = true;
        node.objectIndex = objectIndices[0];
        node.bbox = objectBboxes[objectIndices[0]];
        return nodeIdx;
    }
    
    // 计算所有物体的包围盒
    AABB totalBBox = objectBboxes[objectIndices[0]];
    for (size_t i = 1; i < objectIndices.size(); ++i) {
        totalBBox.expand(objectBboxes[objectIndices[i]]);
    }
    node.bbox = totalBBox;
    
    // 选择分割轴 (最长轴)
    Vector3d extent = totalBBox.extent();
    int axis = 0;
    if (extent.y() > extent.x()) axis = 1;
    if (extent.z() > extent[axis]) axis = 2;
    
    // 按中心点排序
    std::vector<int> sortedIndices = objectIndices;
    std::sort(sortedIndices.begin(), sortedIndices.end(),
        [&objectBboxes, axis](int a, int b) {
            return objectBboxes[a].center()[axis] < objectBboxes[b].center()[axis];
        });
    
    // 分割列表
    size_t mid = sortedIndices.size() / 2;
    std::vector<int> leftIndices(sortedIndices.begin(), sortedIndices.begin() + mid);
    std::vector<int> rightIndices(sortedIndices.begin() + mid, sortedIndices.end());
    
    // 递归构建子树
    node.left = buildRecursive(leftIndices, objectBboxes);
    node.right = buildRecursive(rightIndices, objectBboxes);
    
    // 设置父节点
    if (node.left >= 0) nodes_[node.left].parent = nodeIdx;
    if (node.right >= 0) nodes_[node.right].parent = nodeIdx;
    
    return nodeIdx;
}

int AABBTree::insertObject(const AABB& bbox) {
    int objectIndex = static_cast<int>(objectIndices_.size());
    objectIndices_.push_back(-1);
    
    int nodeIdx = allocateNode();
    AABBNode& node = nodes_[nodeIdx];
    node.bbox = bbox;
    node.isLeaf = true;
    node.objectIndex = objectIndex;
    
    if (root_ == -1) {
        root_ = nodeIdx;
    } else {
        // 找到最佳兄弟节点
        int sibling = findBestSibling(bbox);
        
        // 创建新的父节点
        int parentIdx = allocateNode();
        AABBNode& parent = nodes_[parentIdx];
        parent.bbox = merge(nodes_[sibling].bbox, bbox);
        parent.parent = nodes_[sibling].parent;
        
        // 更新原父节点的子节点引用
        if (parent.parent >= 0) {
            AABBNode& grandparent = nodes_[parent.parent];
            if (grandparent.left == sibling) {
                grandparent.left = parentIdx;
            } else {
                grandparent.right = parentIdx;
            }
        } else {
            root_ = parentIdx;
        }
        
        // 设置新父节点的子节点
        parent.left = sibling;
        parent.right = nodeIdx;
        nodes_[sibling].parent = parentIdx;
        node.parent = parentIdx;
        
        // 向上修正包围盒
        fixUpwards(parentIdx);
    }
    
    objectIndices_[objectIndex] = nodeIdx;
    return objectIndex;
}

void AABBTree::removeObject(int objectIndex) {
    if (objectIndex < 0 || objectIndex >= static_cast<int>(objectIndices_.size())) {
        return;
    }
    
    int nodeIdx = objectIndices_[objectIndex];
    if (nodeIdx < 0) {
        return;
    }
    
    AABBNode& node = nodes_[nodeIdx];
    int parentIdx = node.parent;
    
    if (parentIdx < 0) {
        // 这是根节点
        root_ = -1;
    } else {
        AABBNode& parent = nodes_[parentIdx];
        int siblingIdx = (parent.left == nodeIdx) ? parent.right : parent.left;
        
        int grandparentIdx = parent.parent;
        if (grandparentIdx >= 0) {
            AABBNode& grandparent = nodes_[grandparentIdx];
            if (grandparent.left == parentIdx) {
                grandparent.left = siblingIdx;
            } else {
                grandparent.right = siblingIdx;
            }
            nodes_[siblingIdx].parent = grandparentIdx;
            fixUpwards(grandparentIdx);
        } else {
            root_ = siblingIdx;
            nodes_[siblingIdx].parent = -1;
        }
        
        deallocateNode(parentIdx);
    }
    
    deallocateNode(nodeIdx);
    objectIndices_[objectIndex] = -1;
}

void AABBTree::updateObject(int objectIndex, const AABB& newBBox) {
    if (objectIndex < 0 || objectIndex >= static_cast<int>(objectIndices_.size())) {
        return;
    }
    
    int nodeIdx = objectIndices_[objectIndex];
    if (nodeIdx < 0) {
        return;
    }
    
    nodes_[nodeIdx].bbox = newBBox;
    fixUpwards(nodes_[nodeIdx].parent);
}

std::vector<int> AABBTree::queryOverlaps(const AABB& bbox) const {
    std::vector<int> results;
    if (root_ >= 0) {
        queryOverlapsRecursive(root_, bbox, results);
    }
    return results;
}

void AABBTree::queryOverlapsRecursive(int nodeIdx, const AABB& bbox, 
                                       std::vector<int>& results) const {
    if (nodeIdx < 0 || nodeIdx >= static_cast<int>(nodes_.size())) {
        return;
    }
    
    const AABBNode& node = nodes_[nodeIdx];
    
    if (!node.bbox.intersects(bbox)) {
        return;
    }
    
    if (node.isLeaf) {
        results.push_back(node.objectIndex);
    } else {
        queryOverlapsRecursive(node.left, bbox, results);
        queryOverlapsRecursive(node.right, bbox, results);
    }
}

std::vector<std::pair<int, int>> AABBTree::queryAllOverlaps() const {
    std::vector<std::pair<int, int>> pairs;
    if (root_ >= 0) {
        queryAllOverlapsRecursive(root_, root_, pairs);
    }
    return pairs;
}

void AABBTree::queryAllOverlapsRecursive(int nodeA, int nodeB,
                                          std::vector<std::pair<int, int>>& pairs) const {
    if (nodeA < 0 || nodeB < 0) {
        return;
    }
    
    const AABBNode& A = nodes_[nodeA];
    const AABBNode& B = nodes_[nodeB];
    
    if (!A.bbox.intersects(B.bbox)) {
        return;
    }
    
    // 避免重复检测
    if (nodeA == nodeB) {
        // 自检测
        if (!A.isLeaf) {
            queryAllOverlapsRecursive(A.left, A.left, pairs);
            queryAllOverlapsRecursive(A.right, A.right, pairs);
            queryAllOverlapsRecursive(A.left, A.right, pairs);
        }
    } else if (A.isLeaf && B.isLeaf) {
        // 两个叶子节点
        if (A.objectIndex < B.objectIndex) {
            pairs.emplace_back(A.objectIndex, B.objectIndex);
        }
    } else if (A.isLeaf) {
        // A是叶子，B是内部节点
        queryAllOverlapsRecursive(nodeA, B.left, pairs);
        queryAllOverlapsRecursive(nodeA, B.right, pairs);
    } else if (B.isLeaf) {
        // B是叶子，A是内部节点
        queryAllOverlapsRecursive(A.left, nodeB, pairs);
        queryAllOverlapsRecursive(A.right, nodeB, pairs);
    } else {
        // 都是内部节点
        queryAllOverlapsRecursive(A.left, B.left, pairs);
        queryAllOverlapsRecursive(A.left, B.right, pairs);
        queryAllOverlapsRecursive(A.right, B.left, pairs);
        queryAllOverlapsRecursive(A.right, B.right, pairs);
    }
}

bool AABBTree::raycast(const Ray& ray, int& hitObject, double& hitDistance) const {
    hitObject = -1;
    hitDistance = std::numeric_limits<double>::max();
    
    if (root_ < 0) {
        return false;
    }
    
    return raycastRecursive(root_, ray, hitObject, hitDistance);
}

bool AABBTree::raycastRecursive(int nodeIdx, const Ray& ray,
                                 int& hitObject, double& hitDistance) const {
    if (nodeIdx < 0) {
        return false;
    }
    
    const AABBNode& node = nodes_[nodeIdx];
    
    // 射线与AABB相交测试
    double tmin = 0.0, tmax = hitDistance;
    
    for (int i = 0; i < 3; ++i) {
        if (std::abs(ray.direction[i]) < 1e-10) {
            if (ray.origin[i] < node.bbox.min[i] || ray.origin[i] > node.bbox.max[i]) {
                return false;
            }
        } else {
            double t1 = (node.bbox.min[i] - ray.origin[i]) / ray.direction[i];
            double t2 = (node.bbox.max[i] - ray.origin[i]) / ray.direction[i];
            
            if (t1 > t2) std::swap(t1, t2);
            
            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);
            
            if (tmin > tmax) {
                return false;
            }
        }
    }
    
    if (tmin > hitDistance) {
        return false;
    }
    
    if (node.isLeaf) {
        hitObject = node.objectIndex;
        hitDistance = tmin;
        return true;
    }
    
    // 递归检测子节点
    bool hit = false;
    if (raycastRecursive(node.left, ray, hitObject, hitDistance)) {
        hit = true;
    }
    if (raycastRecursive(node.right, ray, hitObject, hitDistance)) {
        hit = true;
    }
    
    return hit;
}

int AABBTree::getTreeDepth() const {
    if (root_ < 0) return 0;
    
    std::queue<std::pair<int, int>> q;  // (nodeIdx, depth)
    q.push({root_, 1});
    
    int maxDepth = 0;
    while (!q.empty()) {
        auto [nodeIdx, depth] = q.front();
        q.pop();
        
        maxDepth = std::max(maxDepth, depth);
        
        const AABBNode& node = nodes_[nodeIdx];
        if (!node.isLeaf) {
            if (node.left >= 0) q.push({node.left, depth + 1});
            if (node.right >= 0) q.push({node.right, depth + 1});
        }
    }
    
    return maxDepth;
}

void AABBTree::clear() {
    nodes_.clear();
    objectIndices_.clear();
    root_ = -1;
}

bool AABBTree::validate() const {
    if (root_ < 0) return true;
    
    // 检查所有节点的有效性
    for (size_t i = 0; i < nodes_.size(); ++i) {
        const AABBNode& node = nodes_[i];
        
        if (node.isLeaf) {
            if (node.objectIndex < 0) return false;
        } else {
            if (node.left < 0 || node.right < 0) return false;
            if (nodes_[node.left].parent != static_cast<int>(i)) return false;
            if (nodes_[node.right].parent != static_cast<int>(i)) return false;
        }
    }
    
    return true;
}

int AABBTree::allocateNode() {
    nodes_.emplace_back();
    return static_cast<int>(nodes_.size()) - 1;
}

void AABBTree::deallocateNode(int nodeIdx) {
    // 简单实现：标记为无效
    if (nodeIdx >= 0 && nodeIdx < static_cast<int>(nodes_.size())) {
        nodes_[nodeIdx].objectIndex = -1;
        nodes_[nodeIdx].isLeaf = false;
    }
}

void AABBTree::fixUpwards(int nodeIdx) {
    while (nodeIdx >= 0) {
        AABBNode& node = nodes_[nodeIdx];
        
        if (!node.isLeaf) {
            node.bbox = merge(nodes_[node.left].bbox, nodes_[node.right].bbox);
        }
        
        nodeIdx = node.parent;
    }
}

int AABBTree::findBestSibling(const AABB& bbox) const {
    if (root_ < 0) return -1;
    
    int current = root_;
    
    while (!nodes_[current].isLeaf) {
        const AABBNode& node = nodes_[current];
        
        double leftCost = surfaceArea(merge(bbox, nodes_[node.left].bbox));
        double rightCost = surfaceArea(merge(bbox, nodes_[node.right].bbox));
        
        current = (leftCost < rightCost) ? node.left : node.right;
    }
    
    return current;
}

double AABBTree::computeCost(int nodeIdx) const {
    if (nodeIdx < 0) return 0.0;
    return surfaceArea(nodes_[nodeIdx].bbox);
}

AABB AABBTree::merge(const AABB& a, const AABB& b) {
    AABB result = a;
    result.expand(b);
    return result;
}

double AABBTree::surfaceArea(const AABB& bbox) {
    Vector3d ext = bbox.extent();
    return 2.0 * (ext.x() * ext.y() + ext.y() * ext.z() + ext.z() * ext.x());
}

// DynamicAABBTree 实现

DynamicAABBTree::DynamicAABBTree() = default;
DynamicAABBTree::~DynamicAABBTree() = default;

int DynamicAABBTree::createProxy(const AABB& bbox, void* userData) {
    int proxyId;
    
    if (!freeProxies_.empty()) {
        proxyId = freeProxies_.back();
        freeProxies_.pop_back();
        proxies_[proxyId].bbox = bbox;
        proxies_[proxyId].userData = userData;
        proxies_[proxyId].isValid = true;
    } else {
        proxyId = static_cast<int>(proxies_.size());
        Proxy proxy;
        proxy.bbox = bbox;
        proxy.userData = userData;
        proxy.isValid = true;
        proxies_.push_back(proxy);
    }
    
    proxies_[proxyId].nodeId = tree_.insertObject(bbox);
    
    return proxyId;
}

void DynamicAABBTree::destroyProxy(int proxyId) {
    if (proxyId < 0 || proxyId >= static_cast<int>(proxies_.size())) {
        return;
    }
    
    if (!proxies_[proxyId].isValid) {
        return;
    }
    
    tree_.removeObject(proxies_[proxyId].nodeId);
    proxies_[proxyId].isValid = false;
    freeProxies_.push_back(proxyId);
}

void DynamicAABBTree::moveProxy(int proxyId, const AABB& newBBox) {
    if (proxyId < 0 || proxyId >= static_cast<int>(proxies_.size())) {
        return;
    }
    
    if (!proxies_[proxyId].isValid) {
        return;
    }
    
    proxies_[proxyId].bbox = newBBox;
    tree_.updateObject(proxies_[proxyId].nodeId, newBBox);
}

std::vector<int> DynamicAABBTree::queryOverlaps(int proxyId) const {
    std::vector<int> results;
    
    if (proxyId < 0 || proxyId >= static_cast<int>(proxies_.size())) {
        return results;
    }
    
    if (!proxies_[proxyId].isValid) {
        return results;
    }
    
    auto objectIndices = tree_.queryOverlaps(proxies_[proxyId].bbox);
    
    for (int objIdx : objectIndices) {
        if (objIdx != proxyId) {
            results.push_back(objIdx);
        }
    }
    
    return results;
}

std::vector<std::pair<int, int>> DynamicAABBTree::queryAllOverlaps() const {
    return tree_.queryAllOverlaps();
}

void* DynamicAABBTree::getUserData(int proxyId) const {
    if (proxyId < 0 || proxyId >= static_cast<int>(proxies_.size())) {
        return nullptr;
    }
    
    return proxies_[proxyId].userData;
}

AABB DynamicAABBTree::getAABB(int proxyId) const {
    if (proxyId < 0 || proxyId >= static_cast<int>(proxies_.size())) {
        return AABB();
    }
    
    return proxies_[proxyId].bbox;
}

} // namespace vss
