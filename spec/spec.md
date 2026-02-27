# 虚拟手术缝合多体耦合仿真系统 - 技术规范文档

## 1. 项目概述

### 1.1 项目名称
虚拟手术缝合多体耦合仿真系统 (Virtual Surgery Suture Multi-body Coupling Simulation System)

### 1.2 项目目标
实现组织-缝合线-器械的多体耦合仿真与实时交互控制，支持GPU并行加速和参数动态优化。

### 1.3 性能目标
- 每帧耗时 < 33 ms (30 fps)
- 力反馈延迟 < 5 ms
- 支持软组织单元数 > 10k，缝合线质点 > 100

---

## 2. 系统架构

### 2.1 模块划分

```
┌─────────────────────────────────────────────────────────────┐
│                    主循环控制模块                            │
└─────────────────────────────────────────────────────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        ▼                     ▼                     ▼
┌───────────────┐    ┌───────────────┐    ┌───────────────┐
│  场景管理模块  │    │  接触检测模块  │    │  动力学求解模块 │
└───────────────┘    └───────────────┘    └───────────────┘
        │                     │                     │
        ▼                     ▼                     ▼
┌───────────────┐    ┌───────────────┐    ┌───────────────┐
│  约束处理模块  │    │  力反馈模块    │    │ 可视化与交互模块│
└───────────────┘    └───────────────┘    └───────────────┘
        │                     │                     │
        └─────────────────────┼─────────────────────┘
                              ▼
              ┌───────────────────────────────┐
              │    参数标定与优化模块          │
              └───────────────────────────────┘
```

### 2.2 技术栈
- **编程语言**: C++17/20
- **并行计算**: CUDA (GPU加速)
- **图形渲染**: OpenGL/Vulkan
- **数学库**: Eigen (线性代数)
- **配置文件**: JSON

---

## 3. 核心数据结构规范

### 3.1 软组织 (Tissue)

```cpp
class Tissue {
public:
    // 节点数据
    struct Node {
        Vector3d position;      // 位置
        Vector3d velocity;      // 速度
        Vector3d force;         // 外力
        double mass;            // 质量
    };
    
    // 四面体单元
    struct Tetrahedron {
        array<int, 4> nodeIndices;  // 节点索引
        Matrix3d Dm;                // 参考构型矩阵
        double volume;              // 参考体积
        double youngsModulus;       // 杨氏模量
        double poissonRatio;        // 泊松比
    };
    
    vector<Node> nodes;
    vector<Tetrahedron> elements;
    SparseMatrix<double> K;     // 刚度矩阵
    VectorXd F_ext;             // 外力向量
    
    // 材质模型
    enum MaterialModel {
        LINEAR_ELASTIC,         // 线弹性
        NEO_HOOKEAN,            // 新胡克超弹性
        ST_VENANT_KIRCHHOFF     // St.Venant-Kirchhoff
    };
    MaterialModel material;
};
```

### 3.2 缝合线 (Suture)

```cpp
class Suture {
public:
    // 质点
    struct Particle {
        Vector3d position;      // 位置
        Vector3d velocity;      // 速度
        Vector3d force;         // 外力
        double mass;            // 质量
        double radius;          // 半径
        bool isFixed;           // 是否固定
    };
    
    // 约束基类
    class Constraint {
    public:
        virtual ~Constraint() = default;
        virtual void solve(double dt) = 0;
        virtual double evaluate() const = 0;
        
        double stiffness;       // 刚度
        double compliance;      // 柔度 (1/stiffness)
        double lambda;          // 拉格朗日乘子
    };
    
    // 距离约束
    class DistanceConstraint : public Constraint {
    public:
        int p1, p2;             // 质点索引
        double restLength;      // 静止长度
        void solve(double dt) override;
    };
    
    // 弯曲约束
    class BendingConstraint : public Constraint {
    public:
        int p1, p2, p3;         // 三个质点索引
        double restAngle;       // 静止角度
        void solve(double dt) override;
    };
    
    // 锚定约束
    class AnchorConstraint : public Constraint {
    public:
        int particle;           // 质点索引
        Vector3d anchorPos;     // 锚定位置
        void solve(double dt) override;
    };
    
    vector<Particle> particles;
    vector<shared_ptr<Constraint>> constraints;
    
    // XPBD参数
    int solverIterations = 10;
    double damping = 0.99;
};
```

### 3.3 器械 (Instrument)

```cpp
class Instrument {
public:
    // 刚体状态
    struct RigidBody {
        Vector3d position;          // 位置
        Quaterniond orientation;    // 姿态 (四元数)
        Vector3d linearVelocity;    // 线速度
        Vector3d angularVelocity;   // 角速度
        double mass;                // 质量
        Matrix3d inertia;           // 惯性张量
        Matrix3d invInertia;        // 逆惯性张量
    };
    
    // 夹持点
    struct GraspPoint {
        Vector3d localPos;          // 局部坐标
        Vector3d worldPos;          // 世界坐标
        bool isGrasping;            // 是否夹持
        int graspedParticle;        // 夹持的质点索引 (-1表示无)
    };
    
    RigidBody rigidBody;
    vector<GraspPoint> graspPoints;
    
    // 几何表示 (用于碰撞检测)
    vector<Vector3d> vertices;
    vector<array<int, 3>> faces;
    
    // 力反馈状态
    Vector3d feedbackForce;         // 反馈力
    Vector3d feedbackTorque;        // 反馈力矩
};
```

### 3.4 接触对 (ContactPair)

```cpp
struct ContactPair {
    enum ContactType {
        NEEDLE_TISSUE,          // 针-组织
        SUTURE_TISSUE,          // 线-组织
        INSTRUMENT_SUTURE,      // 器械-线
        INSTRUMENT_TISSUE,      // 器械-组织
        SUTURE_SELF             // 线自碰撞
    };
    
    ContactType type;
    
    // 接触几何信息
    Vector3d contactPoint;      // 接触点
    Vector3d normal;            // 法向量 (指向物体1)
    double penetration;         // 穿透深度 (>0表示穿透)
    
    // 物理属性
    double frictionCoeff;       // 摩擦系数
    double stiffnessEnhance;    // 刚度增强系数 (各向异性)
    
    // 接触力
    Vector3d normalForce;       // 法向力
    Vector3d frictionForce;     // 摩擦力
    
    // 参与接触的物体索引
    int object1Index;
    int object2Index;
    
    // 局部坐标 (用于映射力到节点/质点)
    array<double, 4> barycentricCoords;  // 四面体重心坐标
};
```

---

## 4. 算法流程规范

### 4.1 主循环流程

```
初始化场景()
设置时间步长 dt = 0.001 s

主循环 while running:
    
    // 1. 用户输入采集 (目标: < 1ms)
    force_user, torque_user = handle_device_input()
    
    // 2. 多体接触检测 (目标: < 5ms)
    candidate_pairs = broad_phase()           // AABB树并行遍历
    contact_list = []
    for each pair in candidate_pairs:
        contact_info = narrow_phase(pair)      // 定制精检算法
        if contact_info.penetration > 0:
            contact_info.force = compute_contact_force(contact_info)
            contact_list.append(contact_info)
    
    // 3. 动力学迭代耦合 (目标: < 20ms)
    for iter in 1..max_iter:
        
        // 3.1 软组织有限元求解
        assemble_external_forces(contact_list)
        solve_fem()                            // 共轭梯度法
        
        // 3.2 缝合线XPBD求解
        apply_external_forces(contact_list)
        solve_xpbd()
        
        // 3.3 器械刚体求解
        apply_forces(force_user, torque_user, contact_list)
        integrate_rigid_body(dt)
        
        // 3.4 多体约束投影
        project_constraints()
        
        // 3.5 收敛判断
        if converged:
            break
    
    // 4. 力反馈输出 (目标: < 2ms)
    total_force = compute_feedback_force(contact_list, instrument_state)
    filtered_force = adaptive_filter(total_force)
    mapped_force = nonlinear_map(filtered_force)
    send_to_device(mapped_force)
    
    // 5. 渲染更新 (目标: < 5ms)
    render_scene()
    
    // 6. 在线学习 (异步)
    if error_exceed_threshold:
        update_ar_model()
```

### 4.2 接触检测算法

#### 4.2.1 AABB树粗筛

```cpp
class AABBTree {
public:
    struct Node {
        AABB bbox;                  // 包围盒
        int left, right;            // 子节点索引 (-1表示叶子)
        int objectIndex;            // 物体索引 (仅叶子)
    };
    
    vector<Node> nodes;
    
    void build(const vector<AABB>& objectBboxes);
    vector<pair<int, int>> queryOverlaps();  // 返回重叠对
    
    // CUDA并行版本
    void buildCUDA(const vector<AABB>& objectBboxes);
    vector<pair<int, int>> queryOverlapsCUDA();
};
```

#### 4.2.2 针-组织接触检测 (Möller-Trumbore算法)

```cpp
bool intersectRayTetrahedron(
    const Vector3d& rayOrigin,
    const Vector3d& rayDir,
    const array<Vector3d, 4>& tetVertices,
    double& t,                          // 交点参数
    Vector3d& contactPoint,
    array<double, 4>& barycentricCoords
);
```

#### 4.2.3 线-组织接触检测

```cpp
ContactPair detectSutureTissueContact(
    const Suture::Particle& particle,
    const Tissue::Tetrahedron& tet,
    const vector<Tissue::Node>& nodes
) {
    // 计算质点到四面体各面的距离
    // 找到最近面
    // 计算重心坐标
    // 检测穿透并计算接触力
}
```

#### 4.2.4 器械-线接触检测 (GJK算法)

```cpp
class GJKAlgorithm {
public:
    // 计算两个凸包的最短距离
    static double computeDistance(
        const vector<Vector3d>& shapeA,
        const vector<Vector3d>& shapeB,
        Vector3d& closestPointA,
        Vector3d& closestPointB
    );
    
    // EPA (Expanding Polytope Algorithm) 用于穿透深度
    static double computePenetrationDepth(
        const vector<Vector3d>& shapeA,
        const vector<Vector3d>& shapeB,
        Vector3d& contactNormal,
        Vector3d& contactPoint
    );
};
```

### 4.3 动力学求解算法

#### 4.3.1 软组织有限元求解

```cpp
class FEMSolver {
public:
    // 组装刚度矩阵
    void assembleStiffnessMatrix(
        const Tissue& tissue,
        SparseMatrix<double>& K
    );
    
    // 组装外力向量
    void assembleExternalForces(
        const Tissue& tissue,
        const vector<ContactPair>& contacts,
        VectorXd& F
    );
    
    // 共轭梯度法求解
    VectorXd solveConjugateGradient(
        const SparseMatrix<double>& K,
        const VectorXd& F,
        double tolerance = 1e-6,
        int maxIterations = 1000
    );
    
    // 预处理共轭梯度 (使用不完全Cholesky)
    VectorXd solvePCG(
        const SparseMatrix<double>& K,
        const VectorXd& F,
        const IncompleteCholesky<double>& preconditioner
    );
};
```

#### 4.3.2 缝合线XPBD求解

```cpp
class XPBDSolver {
public:
    void solve(Suture& suture, double dt) {
        // 预测位置
        for (auto& p : suture.particles) {
            if (!p.isFixed) {
                p.velocity += dt * p.force / p.mass;
                p.velocity *= suture.damping;
                p.position += dt * p.velocity;
            }
        }
        
        // 约束投影迭代
        for (int iter = 0; iter < suture.solverIterations; iter++) {
            // 按约束组求解
            for (auto& constraint : suture.constraints) {
                constraint->solve(dt);
            }
        }
        
        // 更新速度
        for (auto& p : suture.particles) {
            if (!p.isFixed) {
                p.velocity = (p.position - p.prevPosition) / dt;
            }
        }
    }
};
```

#### 4.3.3 器械刚体求解

```cpp
class RigidBodySolver {
public:
    void integrate(Instrument::RigidBody& body, double dt) {
        // 半隐式欧拉积分
        body.linearVelocity += dt * body.force / body.mass;
        body.position += dt * body.linearVelocity;
        
        // 角速度积分
        body.angularVelocity += dt * body.invInertia * 
                               (body.torque - body.angularVelocity.cross(
                                body.inertia * body.angularVelocity));
        
        // 四元数更新
        Quaterniond omegaQuat(0, body.angularVelocity.x(), 
                                body.angularVelocity.y(), 
                                body.angularVelocity.z());
        body.orientation.coeffs() += dt * 0.5 * (omegaQuat * body.orientation).coeffs();
        body.orientation.normalize();
    }
};
```

### 4.4 约束投影算法

```cpp
class ConstraintProjector {
public:
    // 约束分组
    enum ConstraintGroup {
        CRITICAL,       // 关键组: 距离、自碰撞、夹持
        NON_CRITICAL    // 非关键组: 弯曲、扭转、锚定
    };
    
    void projectConstraints(
        Tissue& tissue,
        Suture& suture,
        Instrument& instrument,
        const vector<ContactPair>& contacts
    ) {
        // 1. 关键组迭代至收敛
        for (int i = 0; i < maxCriticalIterations; i++) {
            bool converged = projectCriticalConstraints();
            if (converged) break;
        }
        
        // 2. 非关键组投影
        projectNonCriticalConstraints();
        
        // 3. 局部修正关键组 (重复3次)
        for (int i = 0; i < 3; i++) {
            projectCriticalConstraints();
        }
    }
};
```

### 4.5 力反馈处理

#### 4.5.1 自适应滤波

```cpp
class AdaptiveFilter {
public:
    enum FilterState {
        NORMAL,         // 正常状态 (截止频率 10Hz)
        EVENT           // 事件状态 (截止频率 50Hz)
    };
    
    FilterState state = NORMAL;
    double cutoffFrequency = 10.0;
    
    Vector3d filter(const Vector3d& rawForce, double dt) {
        // 状态机检测事件
        detectEvent(rawForce);
        
        // 根据状态选择截止频率
        double targetFreq = (state == EVENT) ? 50.0 : 10.0;
        cutoffFrequency = lerp(cutoffFrequency, targetFreq, 0.1);
        
        // 一阶低通滤波
        double RC = 1.0 / (2.0 * M_PI * cutoffFrequency);
        double alpha = dt / (RC + dt);
        
        filteredForce = lerp(filteredForce, rawForce, alpha);
        return filteredForce;
    }
    
private:
    Vector3d filteredForce = Vector3d::Zero();
    
    void detectEvent(const Vector3d& force) {
        // 检测力突变 (如穿刺事件)
        double forceChange = (force - prevForce).norm();
        if (forceChange > eventThreshold) {
            state = EVENT;
            eventTimer = 0.1;  // 事件窗口 100ms
        }
        // ...
    }
};
```

#### 4.5.2 延迟补偿 (AR模型)

```cpp
class DelayCompensator {
public:
    // AR模型参数
    VectorXd arCoefficients;
    deque<Vector3d> history;
    int windowSize = 20;
    
    Vector3d predict(double predictionHorizon) {
        // 使用AR模型预测未来力
        Vector3d predictedForce = Vector3d::Zero();
        
        for (int i = 0; i < arCoefficients.size(); i++) {
            if (i < history.size()) {
                predictedForce += arCoefficients[i] * history[i];
            }
        }
        
        return predictedForce;
    }
    
    void updateModel(const Vector3d& actualForce) {
        history.push_front(actualForce);
        if (history.size() > windowSize) {
            history.pop_back();
        }
        
        // 滑动窗口+动量更新AR系数
        // ...
    }
};
```

---

## 5. 接口规范

### 5.1 场景管理接口

```cpp
class SceneManager {
public:
    // 加载软组织模型
    bool loadTissue(const string& meshFile, const MaterialConfig& config);
    
    // 创建缝合线
    bool createSuture(const vector<Vector3d>& initialPositions, 
                      const SutureConfig& config);
    
    // 加载器械模型
    bool loadInstrument(const string& meshFile, int instrumentId);
    
    // 加载材质参数
    bool loadMaterialConfig(const string& jsonFile);
    
    // 重置场景
    void reset();
    
    // 保存/加载场景状态
    bool saveState(const string& file);
    bool loadState(const string& file);
};
```

### 5.2 力反馈设备接口

```cpp
class HapticDevice {
public:
    // 初始化设备
    bool initialize(int deviceId);
    
    // 获取设备状态
    bool getPosition(Vector3d& pos);
    bool getOrientation(Quaterniond& ori);
    bool getButtonState(int buttonId);
    
    // 输出力反馈
    bool setForce(const Vector3d& force);
    bool setTorque(const Vector3d& torque);
    
    // 设备信息
    double getMaxForce();
    double getMaxStiffness();
    double getWorkspaceRadius();
};
```

### 5.3 参数配置接口

```cpp
class ParameterManager {
public:
    // 从JSON加载参数
    bool loadFromJSON(const string& file);
    
    // 热更新参数
    bool updateParameter(const string& name, double value);
    bool updateParameter(const string& name, const Vector3d& value);
    
    // 获取参数
    template<typename T>
    T getParameter(const string& name);
    
    // 保存当前参数
    bool saveToJSON(const string& file);
    
    // 参数结构
    struct SimulationParams {
        double timeStep;
        int maxIterations;
        double tolerance;
        // ...
    };
    
    struct MaterialParams {
        double youngsModulus;
        double poissonRatio;
        double density;
        // ...
    };
    
    struct HapticParams {
        double normalCutoffFreq;
        double eventCutoffFreq;
        double forceScale;
        // ...
    };
};
```

---

## 6. 文件结构规范

```
project/
├── include/
│   ├── core/
│   │   ├── types.h                 # 基础类型定义
│   │   ├── math_utils.h            # 数学工具
│   │   └── cuda_utils.h            # CUDA工具
│   ├── physics/
│   │   ├── tissue.h                # 软组织
│   │   ├── suture.h                # 缝合线
│   │   ├── instrument.h            # 器械
│   │   ├── contact.h               # 接触检测
│   │   ├── fem_solver.h            # FEM求解器
│   │   ├── xpbd_solver.h           # XPBD求解器
│   │   ├── rigid_body_solver.h     # 刚体求解器
│   │   └── constraint_projector.h  # 约束投影
│   ├── collision/
│   │   ├── aabb_tree.h             # AABB树
│   │   ├── gjk_algorithm.h         # GJK算法
│   │   └── narrow_phase.h          # 精细检测
│   ├── haptic/
│   │   ├── haptic_device.h         # 设备接口
│   │   ├── adaptive_filter.h       # 自适应滤波
│   │   ├── delay_compensator.h     # 延迟补偿
│   │   └── force_mapper.h          # 力映射
│   ├── rendering/
│   │   ├── renderer.h              # 渲染器
│   │   ├── shader.h                # 着色器
│   │   └── camera.h                # 相机
│   └── io/
│       ├── mesh_loader.h           # 网格加载
│       ├── config_loader.h         # 配置加载
│       └── state_serializer.h      # 状态序列化
├── src/
│   ├── core/
│   ├── physics/
│   ├── collision/
│   ├── haptic/
│   ├── rendering/
│   ├── io/
│   └── main.cpp                    # 主程序入口
├── cuda/
│   ├── collision_kernels.cu        # 碰撞检测核函数
│   ├── fem_kernels.cu              # FEM求解核函数
│   └── xpbd_kernels.cu             # XPBD求解核函数
├── config/
│   ├── default_params.json         # 默认参数
│   ├── tissue_materials.json       # 组织材质
│   └── haptic_settings.json        # 力反馈设置
├── shaders/
│   ├── vertex.glsl
│   └── fragment.glsl
├── tests/
│   ├── unit_tests/
│   └── integration_tests/
└── tools/
    └── calibration_tool/           # 参数标定工具
```

---

## 7. 测试规范

### 7.1 单元测试

| 模块 | 测试内容 | 通过标准 |
|------|----------|----------|
| AABB树 | 构建、查询、更新 | 100%包围盒正确检测 |
| GJK算法 | 距离计算、穿透深度 | 误差 < 1e-6 |
| FEM求解 | 单单元测试、patch测试 | 与解析解误差 < 1% |
| XPBD求解 | 约束收敛性 | 10次迭代后误差 < 0.1% |
| 滤波器 | 频率响应 | 符合设计截止频率 |

### 7.2 集成测试

| 场景 | 测试内容 | 通过标准 |
|------|----------|----------|
| 简单穿刺 | 针穿过单层组织 | 力曲线符合预期 |
| 缝合操作 | 完整缝合任务 | 30fps稳定运行 |
| 力反馈 | 设备通信 | 延迟 < 5ms |
| 大规模场景 | 10k+单元 | 帧率 > 30fps |

### 7.3 验证测试

- 与临床实验数据对比力-位移曲线
- 网格收敛性测试
- 时间步长敏感性分析

---

## 8. 依赖库

| 库名 | 版本 | 用途 |
|------|------|------|
| Eigen | 3.4+ | 线性代数 |
| CUDA | 11.0+ | GPU并行计算 |
| OpenGL | 4.5+ | 图形渲染 |
| GLFW | 3.3+ | 窗口管理 |
| GLEW | 2.1+ | OpenGL扩展 |
| jsoncpp | 1.9+ | JSON解析 |
| GoogleTest | 1.11+ | 单元测试 |

---

## 9. 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| 1.0 | 2026-02-16 | 初始版本 |
