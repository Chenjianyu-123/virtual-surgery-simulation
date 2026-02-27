#include "fem_solver.h"
#include <chrono>
#include <iostream>

namespace vss {

// FEMSolver 实现

FEMSolver::FEMSolver() = default;
FEMSolver::~FEMSolver() = default;

bool FEMSolver::solve(Tissue& tissue, double dt) {
    if (dt <= 0) {
        return solveQuasiStatic(tissue);
    }
    return solveDynamic(tissue, dt);
}

bool FEMSolver::solveQuasiStatic(Tissue& tissue) {
    auto start = std::chrono::high_resolution_clock::now();
    
    int n = tissue.getNodeCount();
    
    // 组装刚度矩阵
    SparseMatrixd K;
    assembleStiffnessMatrix(tissue, K);
    
    // 组装外力向量
    VectorXd F_ext;
    assembleExternalForces(tissue, F_ext);
    
    // 组装内力向量
    VectorXd F_int;
    assembleInternalForces(tissue, F_int);
    
    // 残差向量 R = F_ext - F_int
    VectorXd R = F_ext - F_int;
    
    // 应用边界条件
    applyBoundaryConditions(K, R, tissue.getNodes());
    
    // 求解位移增量: K * du = R
    VectorXd du;
    bool converged = false;
    
    switch (config_.solverType) {
        case FEMSolverConfig::CG:
            du = solveCG(K, R, config_.maxIterations, config_.tolerance);
            break;
        case FEMSolverConfig::PCG:
            du = solvePCG(K, R, config_.maxIterations, config_.tolerance);
            break;
        case FEMSolverConfig::Direct:
            // 使用Eigen的直接求解器
            {
                Eigen::SimplicialLDLT<SparseMatrixd> solver;
                solver.compute(K);
                if (solver.info() == Eigen::Success) {
                    du = solver.solve(R);
                    converged = true;
                }
            }
            break;
    }
    
    // 应用位移
    tissue.applyDisplacement(du);
    
    auto end = std::chrono::high_resolution_clock::now();
    solveTime_ = std::chrono::duration<double>(end - start).count();
    
    return converged;
}

bool FEMSolver::solveDynamic(Tissue& tissue, double dt) {
    auto start = std::chrono::high_resolution_clock::now();
    
    int n = tissue.getNodeCount();
    
    // 获取当前状态
    VectorXd x, v;
    tissue.getPositionVector(x);
    tissue.getVelocityVector(v);
    
    // 组装质量矩阵
    SparseMatrixd M;
    assembleMassMatrix(tissue, M);
    
    // 组装刚度矩阵
    SparseMatrixd K;
    assembleStiffnessMatrix(tissue, K);
    
    // 组装外力
    VectorXd F_ext;
    assembleExternalForces(tissue, F_ext);
    
    // 隐式积分: (M + dt^2 * K) * dv = dt * (F_ext - F_int - dt * K * v)
    // 简化使用Newmark-beta方法
    double gamma = 0.5;
    double beta = 0.25;
    
    // 预测
    VectorXd x_pred = x + dt * v;
    
    // 修正
    SparseMatrixd A = M + beta * dt * dt * K;
    VectorXd b = F_ext * dt - K * (x_pred - x) * dt;
    
    // 应用边界条件
    applyBoundaryConditions(A, b, tissue.getNodes());
    
    // 求解
    VectorXd dv;
    switch (config_.solverType) {
        case FEMSolverConfig::CG:
            dv = solveCG(A, b, config_.maxIterations, config_.tolerance);
            break;
        case FEMSolverConfig::PCG:
            dv = solvePCG(A, b, config_.maxIterations, config_.tolerance);
            break;
        case FEMSolverConfig::Direct:
            {
                Eigen::SimplicialLDLT<SparseMatrixd> solver;
                solver.compute(A);
                dv = solver.solve(b);
            }
            break;
    }
    
    // 更新速度和位置
    v += dv;
    x = x_pred + beta * dt * dv;
    
    tissue.setPositionVector(x);
    tissue.setVelocityVector(v);
    
    auto end = std::chrono::high_resolution_clock::now();
    solveTime_ = std::chrono::duration<double>(end - start).count();
    
    return true;
}

void FEMSolver::assembleStiffnessMatrix(const Tissue& tissue, SparseMatrixd& K) {
    int n = tissue.getNodeCount();
    int ndof = 3 * n;
    
    K.resize(ndof, ndof);
    
    std::vector<Triplet> triplets;
    
    // 遍历所有单元组装刚度矩阵
    for (const auto& elem : tissue.getTetrahedrons()) {
        MatrixXd Ke(12, 12);
        computeElementStiffness(elem, tissue.getNodes(), Ke);
        
        // 组装到全局矩阵
        for (int i = 0; i < 4; ++i) {
            int nodeI = elem.nodeIndices[i];
            for (int j = 0; j < 4; ++j) {
                int nodeJ = elem.nodeIndices[j];
                for (int di = 0; di < 3; ++di) {
                    for (int dj = 0; dj < 3; ++dj) {
                        double val = Ke(3 * i + di, 3 * j + dj);
                        if (std::abs(val) > 1e-15) {
                            triplets.emplace_back(3 * nodeI + di, 3 * nodeJ + dj, val);
                        }
                    }
                }
            }
        }
    }
    
    K.setFromTriplets(triplets.begin(), triplets.end());
}

void FEMSolver::assembleMassMatrix(const Tissue& tissue, SparseMatrixd& M) {
    int n = tissue.getNodeCount();
    int ndof = 3 * n;
    
    M.resize(ndof, ndof);
    
    std::vector<Triplet> triplets;
    
    // 集中质量矩阵
    for (int i = 0; i < n; ++i) {
        double m = tissue.getNode(i).mass;
        for (int d = 0; d < 3; ++d) {
            triplets.emplace_back(3 * i + d, 3 * i + d, m);
        }
    }
    
    M.setFromTriplets(triplets.begin(), triplets.end());
}

void FEMSolver::assembleExternalForces(const Tissue& tissue, VectorXd& F) {
    int n = tissue.getNodeCount();
    F.resize(3 * n);
    F.setZero();
    
    // 重力
    Vector3d gravity(0, -9.81, 0);
    
    for (int i = 0; i < n; ++i) {
        const auto& node = tissue.getNode(i);
        Vector3d f_gravity = gravity * node.mass;
        F.segment<3>(3 * i) = f_gravity + node.force;
    }
}

void FEMSolver::assembleInternalForces(const Tissue& tissue, VectorXd& F) {
    int n = tissue.getNodeCount();
    F.resize(3 * n);
    F.setZero();
    
    for (const auto& elem : tissue.getTetrahedrons()) {
        auto forces = elem.computeForces(tissue.getNodes());
        
        for (int i = 0; i < 4; ++i) {
            int nodeIdx = elem.nodeIndices[i];
            F.segment<3>(3 * nodeIdx) += forces[i];
        }
    }
}

VectorXd FEMSolver::solveCG(const SparseMatrixd& A, const VectorXd& b,
                             int maxIter, double tol) {
    VectorXd x = VectorXd::Zero(b.size());
    
    VectorXd r = b - A * x;
    VectorXd p = r;
    
    double rsold = r.dot(r);
    double rs0 = rsold;
    
    for (int iter = 0; iter < maxIter; ++iter) {
        VectorXd Ap = A * p;
        double alpha = rsold / (p.dot(Ap) + 1e-15);
        
        x += alpha * p;
        r -= alpha * Ap;
        
        double rsnew = r.dot(r);
        
        lastResidual_ = std::sqrt(rsnew / rs0);
        lastIterations_ = iter + 1;
        
        if (lastResidual_ < tol) {
            break;
        }
        
        p = r + (rsnew / rsold) * p;
        rsold = rsnew;
    }
    
    return x;
}

VectorXd FEMSolver::solvePCG(const SparseMatrixd& A, const VectorXd& b,
                              int maxIter, double tol) {
    // 使用不完全Cholesky预处理
    if (config_.usePreconditioner) {
        preconditioner_.compute(A);
    }
    
    VectorXd x = VectorXd::Zero(b.size());
    
    VectorXd r = b - A * x;
    VectorXd z = config_.usePreconditioner ? preconditioner_.solve(r) : r;
    VectorXd p = z;
    
    double rzold = r.dot(z);
    double rz0 = rzold;
    
    for (int iter = 0; iter < maxIter; ++iter) {
        VectorXd Ap = A * p;
        double alpha = rzold / (p.dot(Ap) + 1e-15);
        
        x += alpha * p;
        r -= alpha * Ap;
        
        double rsnew = r.dot(r);
        lastResidual_ = std::sqrt(rsnew / rz0);
        lastIterations_ = iter + 1;
        
        if (lastResidual_ < tol) {
            break;
        }
        
        z = config_.usePreconditioner ? preconditioner_.solve(r) : r;
        double rznew = r.dot(z);
        
        p = z + (rznew / rzold) * p;
        rzold = rznew;
    }
    
    return x;
}

void FEMSolver::computeElementStiffness(const TetrahedronElement& elem,
                                         const std::vector<TissueNode>& nodes,
                                         MatrixXd& Ke) {
    Ke.resize(12, 12);
    Ke.setZero();
    
    // 使用数值微分计算单元刚度矩阵
    double h = 1e-8;
    
    // 获取当前构型
    std::array<Vector3d, 4> positions;
    for (int i = 0; i < 4; ++i) {
        positions[i] = nodes[elem.nodeIndices[i]].position;
    }
    
    // 计算参考力
    std::array<Vector3d, 4> f0;
    {
        TetrahedronElement tempElem = elem;
        auto forces = tempElem.computeForces(nodes);
        for (int i = 0; i < 4; ++i) f0[i] = forces[i];
    }
    
    // 数值微分
    for (int nodeIdx = 0; nodeIdx < 4; ++nodeIdx) {
        for (int dim = 0; dim < 3; ++dim) {
            // 正向扰动
            std::vector<TissueNode> nodesPlus = nodes;
            nodesPlus[elem.nodeIndices[nodeIdx]].position[dim] += h;
            
            TetrahedronElement elemPlus = elem;
            auto fPlus = elemPlus.computeForces(nodesPlus);
            
            // 计算刚度矩阵列
            for (int i = 0; i < 4; ++i) {
                Vector3d df = (fPlus[i] - f0[i]) / h;
                for (int j = 0; j < 3; ++j) {
                    Ke(3 * i + j, 3 * nodeIdx + dim) = -df[j];
                }
            }
        }
    }
}

void FEMSolver::applyBoundaryConditions(SparseMatrixd& K, VectorXd& F,
                                         const std::vector<TissueNode>& nodes) {
    // 对于固定节点，修改矩阵和向量
    for (size_t i = 0; i < nodes.size(); ++i) {
        if (nodes[i].isFixed) {
            // 将对应行和列设为单位矩阵
            for (int d = 0; d < 3; ++d) {
                int idx = 3 * i + d;
                
                // 清零行
                for (SparseMatrixd::InnerIterator it(K, idx); it; ++it) {
                    it.valueRef() = (it.col() == idx) ? 1.0 : 0.0;
                }
                
                // 清零列 (除了对角线)
                for (int row = 0; row < K.rows(); ++row) {
                    if (row != idx) {
                        K.coeffRef(row, idx) = 0.0;
                    }
                }
                
                F[idx] = 0.0;
            }
        }
    }
    
    K.makeCompressed();
}

// LinearSolver 实现

LinearSolver::LinearSolver(Type type) : type_(type) {}
LinearSolver::~LinearSolver() = default;

bool LinearSolver::analyze(const SparseMatrixd& A) {
    switch (type_) {
        case LU:
            lu_.analyzePattern(A);
            return lu_.info() == Eigen::Success;
        case QR:
            qr_.analyzePattern(A);
            return qr_.info() == Eigen::Success;
        default:
            return true;
    }
}

bool LinearSolver::factorize(const SparseMatrixd& A) {
    switch (type_) {
        case CG:
        case PCG_JACOBI:
            cg_.compute(A);
            return cg_.info() == Eigen::Success;
        case PCG_ICHOL:
            ichol_.compute(A);
            return ichol_.info() == Eigen::Success;
        case LU:
            lu_.factorize(A);
            return lu_.info() == Eigen::Success;
        case QR:
            qr_.factorize(A);
            return qr_.info() == Eigen::Success;
        default:
            return false;
    }
}

VectorXd LinearSolver::solve(const VectorXd& b) {
    switch (type_) {
        case CG:
            cg_.setMaxIterations(maxIterations_);
            cg_.setTolerance(tolerance_);
            return cg_.solve(b);
        case PCG_JACOBI:
            cg_.setMaxIterations(maxIterations_);
            cg_.setTolerance(tolerance_);
            return cg_.solveWithGuess(b, VectorXd::Zero(b.size()));
        case PCG_ICHOL:
            // 手动实现PCG
            return VectorXd::Zero(b.size());
        case LU:
            return lu_.solve(b);
        case QR:
            return qr_.solve(b);
        default:
            return VectorXd::Zero(b.size());
    }
}

VectorXd LinearSolver::solve(const SparseMatrixd& A, const VectorXd& b) {
    analyze(A);
    factorize(A);
    return solve(b);
}

} // namespace vss
