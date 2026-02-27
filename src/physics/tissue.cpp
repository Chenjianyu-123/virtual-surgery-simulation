#include "tissue.h"
#include <algorithm>
#include <numeric>

namespace vss {

// TetrahedronElement 实现
Matrix3d TetrahedronElement::computeF(const std::vector<TissueNode>& nodes) const {
    // 计算当前构型矩阵 Ds = (x1-x0, x2-x0, x3-x0)
    Matrix3d Ds;
    const Vector3d& x0 = nodes[nodeIndices[0]].position;
    for (int i = 0; i < 3; ++i) {
        Ds.col(i) = nodes[nodeIndices[i + 1]].position - x0;
    }
    // F = Ds * Dm^{-1}
    return Ds * DmInverse;
}

Matrix3d TetrahedronElement::computeE(const Matrix3d& F) const {
    // Green-Lagrange strain: E = 0.5 * (F^T * F - I)
    return 0.5 * (F.transpose() * F - Matrix3d::Identity());
}

Matrix3d TetrahedronElement::computeP(const Matrix3d& F) const {
    switch (materialModel) {
        case MaterialModel::LinearElastic:
            return material_models::linearElasticStress(F, material);
        case MaterialModel::NeoHookean:
            return material_models::neoHookeanStress(F, material);
        case MaterialModel::StVenantKirchhoff:
            return material_models::stVenantKirchhoffStress(F, material);
        default:
            return material_models::neoHookeanStress(F, material);
    }
}

Matrix3d TetrahedronElement::computeCauchyStress(const Matrix3d& F) const {
    Matrix3d P = computeP(F);
    double J = F.determinant();
    if (std::abs(J) < 1e-10) {
        return Matrix3d::Zero();
    }
    // sigma = (1/J) * P * F^T
    return (1.0 / J) * P * F.transpose();
}

std::array<Vector3d, 4> TetrahedronElement::computeForces(const std::vector<TissueNode>& nodes) const {
    Matrix3d F = computeF(nodes);
    Matrix3d P = computeP(F);
    
    // H = -restVolume * P * Dm^{-T}
    Matrix3d H = -restVolume * P * DmInverse.transpose();
    
    std::array<Vector3d, 4> forces;
    forces[0] = -H.col(0) - H.col(1) - H.col(2);  // f0 = -(h1 + h2 + h3)
    forces[1] = H.col(0);                          // f1 = h1
    forces[2] = H.col(1);                          // f2 = h2
    forces[3] = H.col(2);                          // f3 = h3
    
    return forces;
}

void TetrahedronElement::computeStiffnessMatrix(const std::vector<TissueNode>& nodes,
                                                 std::vector<Triplet>& triplets) const {
    // 计算刚度矩阵需要应力微分，这里使用数值微分简化实现
    Matrix3d F = computeF(nodes);
    double h = 1e-8;
    
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Matrix3d dF = Matrix3d::Zero();
            dF(i, j) = 1.0;
            
            Matrix3d F_plus = F + h * dF;
            Matrix3d F_minus = F - h * dF;
            
            Matrix3d P_plus = computeP(F_plus);
            Matrix3d P_minus = computeP(F_minus);
            
            Matrix3d dP = (P_plus - P_minus) / (2.0 * h);
            
            // 组装到全局刚度矩阵
            // 这里简化处理，实际应该完整计算12x12的单元刚度矩阵
            // 并组装到全局矩阵
        }
    }
}

// Tissue 类实现
Tissue::Tissue() = default;
Tissue::~Tissue() = default;

int Tissue::addNode(const Vector3d& position, double mass, bool isFixed) {
    TissueNode node;
    node.index = static_cast<int>(nodes_.size());
    node.position = position;
    node.restPosition = position;
    node.velocity = Vector3d::Zero();
    node.force = Vector3d::Zero();
    node.mass = mass;
    node.isFixed = isFixed;
    
    nodes_.push_back(node);
    return node.index;
}

void Tissue::removeNode(int index) {
    if (index < 0 || index >= static_cast<int>(nodes_.size())) {
        return;
    }
    
    nodes_.erase(nodes_.begin() + index);
    
    // 更新剩余节点的索引
    for (int i = index; i < static_cast<int>(nodes_.size()); ++i) {
        nodes_[i].index = i;
    }
    
    // 更新单元中的节点索引
    for (auto& elem : elements_) {
        for (int& nodeIdx : elem.nodeIndices) {
            if (nodeIdx > index) {
                nodeIdx--;
            } else if (nodeIdx == index) {
                nodeIdx = -1;  // 标记为无效
            }
        }
    }
    
    // 移除包含无效节点的单元
    elements_.erase(
        std::remove_if(elements_.begin(), elements_.end(),
            [](const TetrahedronElement& elem) {
                return std::any_of(elem.nodeIndices.begin(), elem.nodeIndices.end(),
                    [](int idx) { return idx < 0; });
            }),
        elements_.end()
    );
    
    // 更新单元索引
    for (int i = 0; i < static_cast<int>(elements_.size()); ++i) {
        elements_[i].index = i;
    }
}

int Tissue::addTetrahedron(int n0, int n1, int n2, int n3,
                           const Material& material, MaterialModel model) {
    if (n0 < 0 || n0 >= static_cast<int>(nodes_.size()) ||
        n1 < 0 || n1 >= static_cast<int>(nodes_.size()) ||
        n2 < 0 || n2 >= static_cast<int>(nodes_.size()) ||
        n3 < 0 || n3 >= static_cast<int>(nodes_.size())) {
        return -1;
    }
    
    TetrahedronElement elem;
    elem.index = static_cast<int>(elements_.size());
    elem.nodeIndices = {n0, n1, n2, n3};
    elem.material = material;
    elem.materialModel = model;
    
    // 计算参考构型矩阵 Dm
    const Vector3d& X0 = nodes_[n0].restPosition;
    for (int i = 0; i < 3; ++i) {
        elem.Dm.col(i) = nodes_[elem.nodeIndices[i + 1]].restPosition - X0;
    }
    
    // 计算 Dm 的逆矩阵和参考体积
    elem.DmInverse = elem.Dm.inverse();
    elem.restVolume = std::abs(elem.Dm.determinant()) / 6.0;
    elem.currentVolume = elem.restVolume;
    
    if (elem.restVolume < 1e-10) {
        // 退化四面体
        return -1;
    }
    
    elements_.push_back(elem);
    return elem.index;
}

void Tissue::removeTetrahedron(int index) {
    if (index < 0 || index >= static_cast<int>(elements_.size())) {
        return;
    }
    
    elements_.erase(elements_.begin() + index);
    
    // 更新剩余单元的索引
    for (int i = index; i < static_cast<int>(elements_.size()); ++i) {
        elements_[i].index = i;
    }
}

bool Tissue::loadFromMesh(const std::vector<Vector3d>& vertices,
                          const std::vector<std::array<int, 4>>& tetrahedra,
                          const Material& material,
                          MaterialModel model) {
    // 清空现有数据
    nodes_.clear();
    elements_.clear();
    
    // 添加节点
    for (const auto& vertex : vertices) {
        addNode(vertex, 0.0, false);  // 质量稍后计算
    }
    
    // 添加四面体
    for (const auto& tet : tetrahedra) {
        addTetrahedron(tet[0], tet[1], tet[2], tet[3], material, model);
    }
    
    // 计算质量
    computeMasses();
    
    return !nodes_.empty() && !elements_.empty();
}

void Tissue::computeMasses() {
    // 初始化节点质量为0
    for (auto& node : nodes_) {
        node.mass = 0.0;
    }
    
    // 根据单元体积和密度分配质量到节点
    for (const auto& elem : elements_) {
        double elemMass = elem.restVolume * elem.material.density;
        double nodeMass = elemMass / 4.0;  // 平均分配到4个节点
        
        for (int nodeIdx : elem.nodeIndices) {
            nodes_[nodeIdx].mass += nodeMass;
        }
    }
}

void Tissue::computeForces() {
    // 重置节点力
    for (auto& node : nodes_) {
        node.force = Vector3d::Zero();
    }
    
    // 计算单元力并累加到节点
    for (const auto& elem : elements_) {
        auto forces = elem.computeForces(nodes_);
        
        for (int i = 0; i < 4; ++i) {
            nodes_[elem.nodeIndices[i]].force += forces[i];
        }
    }
}

void Tissue::computeStiffnessMatrix(SparseMatrixd& K) {
    int n = static_cast<int>(nodes_.size());
    K.resize(3 * n, 3 * n);
    
    std::vector<Triplet> triplets;
    
    for (const auto& elem : elements_) {
        elem.computeStiffnessMatrix(nodes_, triplets);
    }
    
    K.setFromTriplets(triplets.begin(), triplets.end());
}

void Tissue::updateCurrentState() {
    updateElementVolumes();
}

void Tissue::updateElementVolumes() {
    for (auto& elem : elements_) {
        Matrix3d Ds;
        const Vector3d& x0 = nodes_[elem.nodeIndices[0]].position;
        for (int i = 0; i < 3; ++i) {
            Ds.col(i) = nodes_[elem.nodeIndices[i + 1]].position - x0;
        }
        elem.currentVolume = std::abs(Ds.determinant()) / 6.0;
    }
}

void Tissue::fixNode(int index) {
    if (index >= 0 && index < static_cast<int>(nodes_.size())) {
        nodes_[index].isFixed = true;
        nodes_[index].velocity = Vector3d::Zero();
    }
}

void Tissue::unfixNode(int index) {
    if (index >= 0 && index < static_cast<int>(nodes_.size())) {
        nodes_[index].isFixed = false;
    }
}

void Tissue::fixNodesByCondition(std::function<bool(const Vector3d&)> condition) {
    for (auto& node : nodes_) {
        if (condition(node.position)) {
            node.isFixed = true;
            node.velocity = Vector3d::Zero();
        }
    }
}

void Tissue::setMaterial(int elementIndex, const Material& material) {
    if (elementIndex >= 0 && elementIndex < static_cast<int>(elements_.size())) {
        elements_[elementIndex].material = material;
    }
}

void Tissue::setMaterialModel(int elementIndex, MaterialModel model) {
    if (elementIndex >= 0 && elementIndex < static_cast<int>(elements_.size())) {
        elements_[elementIndex].materialModel = model;
    }
}

void Tissue::setGlobalMaterial(const Material& material) {
    for (auto& elem : elements_) {
        elem.material = material;
    }
}

void Tissue::setGlobalMaterialModel(MaterialModel model) {
    for (auto& elem : elements_) {
        elem.materialModel = model;
    }
}

double Tissue::getTotalVolume() const {
    double total = 0.0;
    for (const auto& elem : elements_) {
        total += elem.currentVolume;
    }
    return total;
}

double Tissue::getTotalMass() const {
    double total = 0.0;
    for (const auto& node : nodes_) {
        total += node.mass;
    }
    return total;
}

Vector3d Tissue::getCenterOfMass() const {
    Vector3d center = Vector3d::Zero();
    double totalMass = 0.0;
    
    for (const auto& node : nodes_) {
        center += node.mass * node.position;
        totalMass += node.mass;
    }
    
    if (totalMass > 0.0) {
        center /= totalMass;
    }
    
    return center;
}

AABB Tissue::getBoundingBox() const {
    if (nodes_.empty()) {
        return AABB();
    }
    
    AABB bbox;
    bbox.min = bbox.max = nodes_[0].position;
    
    for (const auto& node : nodes_) {
        bbox.expand(node.position);
    }
    
    return bbox;
}

void Tissue::resetToRestState() {
    for (auto& node : nodes_) {
        node.position = node.restPosition;
        node.velocity = Vector3d::Zero();
        node.force = Vector3d::Zero();
    }
    updateElementVolumes();
}

void Tissue::getPositionVector(VectorXd& x) const {
    int n = static_cast<int>(nodes_.size());
    x.resize(3 * n);
    
    for (int i = 0; i < n; ++i) {
        x.segment<3>(3 * i) = nodes_[i].position;
    }
}

void Tissue::setPositionVector(const VectorXd& x) {
    int n = static_cast<int>(nodes_.size());
    if (x.size() != 3 * n) return;
    
    for (int i = 0; i < n; ++i) {
        if (!nodes_[i].isFixed) {
            nodes_[i].position = x.segment<3>(3 * i);
        }
    }
    updateElementVolumes();
}

void Tissue::getVelocityVector(VectorXd& v) const {
    int n = static_cast<int>(nodes_.size());
    v.resize(3 * n);
    
    for (int i = 0; i < n; ++i) {
        v.segment<3>(3 * i) = nodes_[i].velocity;
    }
}

void Tissue::setVelocityVector(const VectorXd& v) {
    int n = static_cast<int>(nodes_.size());
    if (v.size() != 3 * n) return;
    
    for (int i = 0; i < n; ++i) {
        if (!nodes_[i].isFixed) {
            nodes_[i].velocity = v.segment<3>(3 * i);
        }
    }
}

void Tissue::getForceVector(VectorXd& f) const {
    int n = static_cast<int>(nodes_.size());
    f.resize(3 * n);
    
    for (int i = 0; i < n; ++i) {
        f.segment<3>(3 * i) = nodes_[i].force;
    }
}

void Tissue::applyDisplacement(const VectorXd& dx) {
    int n = static_cast<int>(nodes_.size());
    if (dx.size() != 3 * n) return;
    
    for (int i = 0; i < n; ++i) {
        if (!nodes_[i].isFixed) {
            nodes_[i].position += dx.segment<3>(3 * i);
        }
    }
    updateElementVolumes();
}

bool Tissue::validateElement(int elementIndex) const {
    if (elementIndex < 0 || elementIndex >= static_cast<int>(elements_.size())) {
        return false;
    }
    
    const auto& elem = elements_[elementIndex];
    for (int nodeIdx : elem.nodeIndices) {
        if (nodeIdx < 0 || nodeIdx >= static_cast<int>(nodes_.size())) {
            return false;
        }
    }
    
    return elem.restVolume > 1e-10;
}

// 材质模型实现
namespace material_models {

Matrix3d linearElasticStress(const Matrix3d& F, const Material& material) {
    // 小变形假设: P = lambda * tr(E) * I + 2 * mu * E
    // 其中 E = 0.5 * (F + F^T) - I (小应变)
    Matrix3d epsilon = 0.5 * (F + F.transpose()) - Matrix3d::Identity();
    double traceE = epsilon.trace();
    return material.lambda() * traceE * Matrix3d::Identity() + 2.0 * material.mu() * epsilon;
}

Matrix3d neoHookeanStress(const Matrix3d& F, const Material& material) {
    // 新胡克模型: P = mu * (F - F^{-T}) + lambda * ln(J) * F^{-T}
    double J = F.determinant();
    if (std::abs(J) < 1e-10) {
        return Matrix3d::Zero();
    }
    
    Matrix3d FInvT = F.inverse().transpose();
    return material.mu() * (F - FInvT) + material.lambda() * std::log(J) * FInvT;
}

Matrix3d stVenantKirchhoffStress(const Matrix3d& F, const Material& material) {
    // St.Venant-Kirchhoff: S = lambda * tr(E) * I + 2 * mu * E
    // P = F * S
    Matrix3d E = 0.5 * (F.transpose() * F - Matrix3d::Identity());
    double traceE = E.trace();
    Matrix3d S = material.lambda() * traceE * Matrix3d::Identity() + 2.0 * material.mu() * E;
    return F * S;
}

void computeStressDifferential(const Matrix3d& F, const Matrix3d& dF,
                                const Material& material, MaterialModel model,
                                Matrix3d& dP) {
    // 使用数值微分
    double h = 1e-8;
    
    Matrix3d P, P_plus, P_minus;
    
    // 计算当前P
    switch (model) {
        case MaterialModel::LinearElastic:
            P = linearElasticStress(F, material);
            break;
        case MaterialModel::NeoHookean:
            P = neoHookeanStress(F, material);
            break;
        case MaterialModel::StVenantKirchhoff:
            P = stVenantKirchhoffStress(F, material);
            break;
    }
    
    // 计算 F + h*dF 处的P
    Matrix3d F_plus = F + h * dF;
    switch (model) {
        case MaterialModel::LinearElastic:
            P_plus = linearElasticStress(F_plus, material);
            break;
        case MaterialModel::NeoHookean:
            P_plus = neoHookeanStress(F_plus, material);
            break;
        case MaterialModel::StVenantKirchhoff:
            P_plus = stVenantKirchhoffStress(F_plus, material);
            break;
    }
    
    // 计算 F - h*dF 处的P
    Matrix3d F_minus = F - h * dF;
    switch (model) {
        case MaterialModel::LinearElastic:
            P_minus = linearElasticStress(F_minus, material);
            break;
        case MaterialModel::NeoHookean:
            P_minus = neoHookeanStress(F_minus, material);
            break;
        case MaterialModel::StVenantKirchhoff:
            P_minus = stVenantKirchhoffStress(F_minus, material);
            break;
    }
    
    dP = (P_plus - P_minus) / (2.0 * h);
}

} // namespace material_models

} // namespace vss
