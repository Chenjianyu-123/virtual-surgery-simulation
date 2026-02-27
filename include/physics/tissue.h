#pragma once

#include "types.h"
#include "math_utils.h"
#include <vector>
#include <memory>

namespace vss {

// 材质模型枚举
enum class MaterialModel {
    LinearElastic,      // 线弹性
    NeoHookean,         // 新胡克超弹性
    StVenantKirchhoff   // St.Venant-Kirchhoff
};

// 软组织节点
struct TissueNode {
    int index;                      // 节点索引
    Vector3d position;              // 当前位置
    Vector3d restPosition;          // 静止位置
    Vector3d velocity;              // 速度
    Vector3d force;                 // 外力
    double mass;                    // 质量
    bool isFixed;                   // 是否固定
    
    TissueNode() : index(-1), mass(0.0), isFixed(false) {
        position = restPosition = velocity = force = Vector3d::Zero();
    }
};

// 四面体单元
struct TetrahedronElement {
    int index;                      // 单元索引
    std::array<int, 4> nodeIndices; // 节点索引
    Matrix3d Dm;                    // 参考构型矩阵 (X1-X0, X2-X0, X3-X0)
    Matrix3d DmInverse;             // Dm的逆矩阵
    double restVolume;              // 参考体积
    double currentVolume;           // 当前体积
    Material material;              // 材质属性
    MaterialModel materialModel;    // 材质模型
    
    TetrahedronElement() : index(-1), restVolume(0.0), currentVolume(0.0),
                           materialModel(MaterialModel::NeoHookean) {
        nodeIndices = {-1, -1, -1, -1};
        Dm = DmInverse = Matrix3d::Identity();
    }
    
    // 计算 deformation gradient F
    Matrix3d computeF(const std::vector<TissueNode>& nodes) const;
    
    // 计算 Green-Lagrange strain E
    Matrix3d computeE(const Matrix3d& F) const;
    
    // 计算第一 Piola-Kirchhoff stress P
    Matrix3d computeP(const Matrix3d& F) const;
    
    // 计算 Cauchy stress sigma
    Matrix3d computeCauchyStress(const Matrix3d& F) const;
    
    // 计算单元弹性力
    std::array<Vector3d, 4> computeForces(const std::vector<TissueNode>& nodes) const;
    
    // 计算单元刚度矩阵 (12x12)
    void computeStiffnessMatrix(const std::vector<TissueNode>& nodes, 
                                 std::vector<Triplet>& triplets) const;
};

// 软组织类
class Tissue {
public:
    Tissue();
    ~Tissue();
    
    // 禁用拷贝，允许移动
    Tissue(const Tissue&) = delete;
    Tissue& operator=(const Tissue&) = delete;
    Tissue(Tissue&&) = default;
    Tissue& operator=(Tissue&&) = default;
    
    // 节点管理
    int addNode(const Vector3d& position, double mass = 1.0, bool isFixed = false);
    void removeNode(int index);
    TissueNode& getNode(int index) { return nodes_[index]; }
    const TissueNode& getNode(int index) const { return nodes_[index]; }
    int getNodeCount() const { return static_cast<int>(nodes_.size()); }
    std::vector<TissueNode>& getNodes() { return nodes_; }
    const std::vector<TissueNode>& getNodes() const { return nodes_; }
    
    // 单元管理
    int addTetrahedron(int n0, int n1, int n2, int n3, 
                       const Material& material = Material(),
                       MaterialModel model = MaterialModel::NeoHookean);
    void removeTetrahedron(int index);
    TetrahedronElement& getTetrahedron(int index) { return elements_[index]; }
    const TetrahedronElement& getTetrahedron(int index) const { return elements_[index]; }
    int getTetrahedronCount() const { return static_cast<int>(elements_.size()); }
    std::vector<TetrahedronElement>& getTetrahedrons() { return elements_; }
    const std::vector<TetrahedronElement>& getTetrahedrons() const { return elements_; }
    
    // 批量添加四面体网格
    bool loadFromMesh(const std::vector<Vector3d>& vertices,
                      const std::vector<std::array<int, 4>>& tetrahedra,
                      const Material& material = Material(),
                      MaterialModel model = MaterialModel::NeoHookean);
    
    // 物理计算
    void computeMasses();                           // 根据体积和密度计算节点质量
    void computeForces();                           // 计算所有节点的弹性力
    void computeStiffnessMatrix(SparseMatrixd& K);  // 组装全局刚度矩阵
    void updateCurrentState();                      // 更新当前状态 (体积等)
    
    // 边界条件
    void fixNode(int index);
    void unfixNode(int index);
    void fixNodesByCondition(std::function<bool(const Vector3d&)> condition);
    
    // 材质设置
    void setMaterial(int elementIndex, const Material& material);
    void setMaterialModel(int elementIndex, MaterialModel model);
    void setGlobalMaterial(const Material& material);
    void setGlobalMaterialModel(MaterialModel model);
    
    // 获取系统信息
    double getTotalVolume() const;
    double getTotalMass() const;
    Vector3d getCenterOfMass() const;
    AABB getBoundingBox() const;
    
    // 重置到静止状态
    void resetToRestState();
    
    // 位移和速度向量操作
    void getPositionVector(VectorXd& x) const;
    void setPositionVector(const VectorXd& x);
    void getVelocityVector(VectorXd& v) const;
    void setVelocityVector(const VectorXd& v);
    void getForceVector(VectorXd& f) const;
    
    // 应用位移和速度更新
    void applyDisplacement(const VectorXd& dx);
    
private:
    std::vector<TissueNode> nodes_;
    std::vector<TetrahedronElement> elements_;
    
    // 内部辅助函数
    void updateElementVolumes();
    bool validateElement(int elementIndex) const;
};

// 材质模型实现
namespace material_models {
    // 线弹性模型
    Matrix3d linearElasticStress(const Matrix3d& F, const Material& material);
    
    // 新胡克模型
    Matrix3d neoHookeanStress(const Matrix3d& F, const Material& material);
    
    // St.Venant-Kirchhoff模型
    Matrix3d stVenantKirchhoffStress(const Matrix3d& F, const Material& material);
    
    // 计算材质模型的微分 (用于刚度矩阵)
    void computeStressDifferential(const Matrix3d& F, const Matrix3d& dF,
                                    const Material& material, MaterialModel model,
                                    Matrix3d& dP);
}

} // namespace vss
