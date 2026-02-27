#pragma once

#include "types.h"
#include "tissue.h"
#include <vector>

namespace vss {

// FEM求解器配置
struct FEMSolverConfig {
    enum SolverType {
        CG,         // 共轭梯度法
        PCG,        // 预处理共轭梯度法
        Direct      // 直接求解
    };
    
    SolverType solverType = PCG;
    int maxIterations = 1000;
    double tolerance = 1e-6;
    bool usePreconditioner = true;
    bool useGPU = false;
};

// FEM求解器
class FEMSolver {
public:
    FEMSolver();
    ~FEMSolver();
    
    // 设置配置
    void setConfig(const FEMSolverConfig& config) { config_ = config; }
    const FEMSolverConfig& getConfig() const { return config_; }
    
    // 求解一个时间步
    // 返回是否收敛
    bool solve(Tissue& tissue, double dt);
    
    // 准静态求解 (忽略惯性)
    bool solveQuasiStatic(Tissue& tissue);
    
    // 动态求解 (考虑惯性)
    bool solveDynamic(Tissue& tissue, double dt);
    
    // 组装刚度矩阵
    void assembleStiffnessMatrix(const Tissue& tissue, SparseMatrixd& K);
    
    // 组装质量矩阵
    void assembleMassMatrix(const Tissue& tissue, SparseMatrixd& M);
    
    // 组装外力向量
    void assembleExternalForces(const Tissue& tissue, VectorXd& F);
    
    // 组装内力向量 (弹性力)
    void assembleInternalForces(const Tissue& tissue, VectorXd& F);
    
    // 共轭梯度法求解
    VectorXd solveCG(const SparseMatrixd& A, const VectorXd& b, 
                     int maxIter, double tol);
    
    // 预处理共轭梯度法
    VectorXd solvePCG(const SparseMatrixd& A, const VectorXd& b,
                      int maxIter, double tol);
    
    // 获取求解统计
    int getLastIterations() const { return lastIterations_; }
    double getLastResidual() const { return lastResidual_; }
    double getSolveTime() const { return solveTime_; }
    
private:
    FEMSolverConfig config_;
    
    // 统计信息
    int lastIterations_ = 0;
    double lastResidual_ = 0.0;
    double solveTime_ = 0.0;
    
    // 预处理矩阵 (用于PCG)
    Eigen::IncompleteCholesky<double> preconditioner_;
    
    // 内部辅助函数
    void computeElementStiffness(const TetrahedronElement& elem,
                                  const std::vector<TissueNode>& nodes,
                                  MatrixXd& Ke);
    
    void computeStressDifferential(const Matrix3d& F,
                                    const Matrix3d& dF,
                                    const Material& material,
                                    MaterialModel model,
                                    Matrix3d& dP);
    
    // 应用边界条件
    void applyBoundaryConditions(SparseMatrixd& K, VectorXd& F,
                                  const std::vector<TissueNode>& nodes);
    
    // 提取自由度的子矩阵
    void extractFreeSubMatrix(const SparseMatrixd& K,
                               const std::vector<int>& freeDofs,
                               SparseMatrixd& Kff);
    
    void extractFreeSubVector(const VectorXd& V,
                               const std::vector<int>& freeDofs,
                               VectorXd& Vf);
    
    void scatterFreeSubVector(const VectorXd& Vf,
                               const std::vector<int>& freeDofs,
                               VectorXd& V);
};

// 线性求解器包装
class LinearSolver {
public:
    enum Type {
        CG,
        PCG_ICHOL,      // 不完全Cholesky预处理
        PCG_JACOBI,     // Jacobi预处理
        LU,
        QR
    };
    
    LinearSolver(Type type = PCG_ICHOL);
    ~LinearSolver();
    
    // 分析模式 (预处理)
    bool analyze(const SparseMatrixd& A);
    
    // 分解/预处理
    bool factorize(const SparseMatrixd& A);
    
    // 求解
    VectorXd solve(const VectorXd& b);
    
    // 一次性求解
    VectorXd solve(const SparseMatrixd& A, const VectorXd& b);
    
    // 设置参数
    void setMaxIterations(int maxIter) { maxIterations_ = maxIter; }
    void setTolerance(double tol) { tolerance_ = tol; }
    
private:
    Type type_;
    int maxIterations_ = 1000;
    double tolerance_ = 1e-6;
    
    // 迭代求解器
    Eigen::ConjugateGradient<SparseMatrixd> cg_;
    Eigen::BiCGSTAB<SparseMatrixd> bicgstab_;
    
    // 直接求解器
    Eigen::SparseLU<SparseMatrixd> lu_;
    Eigen::SparseQR<SparseMatrixd, Eigen::COLAMDOrdering<int>> qr_;
    
    // 预处理
    Eigen::IncompleteCholesky<double> ichol_;
    Eigen::DiagonalPreconditioner<double> jacobi_;
};

} // namespace vss
