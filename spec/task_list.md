# 虚拟手术缝合多体耦合仿真系统 - 任务列表

## 阶段一：基础框架搭建 (Week 1-2)

### 1.1 项目结构初始化
- [ ] 1.1.1 创建项目目录结构
  - [ ] 创建 `include/` 目录及子目录
  - [ ] 创建 `src/` 目录及子目录
  - [ ] 创建 `cuda/` 目录
  - [ ] 创建 `config/` 目录
  - [ ] 创建 `tests/` 目录
  - [ ] 创建 `tools/` 目录

- [ ] 1.1.2 配置构建系统
  - [ ] 创建 CMakeLists.txt
  - [ ] 配置 CUDA 编译选项
  - [ ] 配置第三方库依赖 (Eigen, GLFW, GLEW, jsoncpp)
  - [ ] 配置编译选项 (C++17/20)

### 1.2 核心类型定义
- [ ] 1.2.1 创建 `include/core/types.h`
  - [ ] 定义基础向量/矩阵类型别名
  - [ ] 定义 AABB 结构
  - [ ] 定义颜色/材质结构

- [ ] 1.2.2 创建 `include/core/math_utils.h`
  - [ ] 实现常用数学函数 (lerp, clamp)
  - [ ] 实现四元数工具函数
  - [ ] 实现矩阵分解工具

### 1.3 配置文件系统
- [ ] 1.3.1 创建 `config/default_params.json`
  - [ ] 定义仿真参数 (时间步长、迭代次数等)
  - [ ] 定义收敛容差

- [ ] 1.3.2 创建 `config/tissue_materials.json`
  - [ ] 定义默认组织材质参数
  - [ ] 定义缝合线材质参数

- [ ] 1.3.3 创建 `config/haptic_settings.json`
  - [ ] 定义力反馈设备参数
  - [ ] 定义滤波器参数

---

## 阶段二：物理核心模块 (Week 3-5)

### 2.1 软组织模块
- [ ] 2.1.1 实现 `include/physics/tissue.h` 和 `src/physics/tissue.cpp`
  - [ ] 定义 Tissue::Node 结构
  - [ ] 定义 Tissue::Tetrahedron 结构
  - [ ] 实现 Tissue 类构造函数/析构函数
  - [ ] 实现节点/单元管理接口
  - [ ] 实现材质属性设置

- [ ] 2.1.2 实现材质模型
  - [ ] 实现线弹性模型 (Linear Elastic)
  - [ ] 实现新胡克模型 (Neo-Hookean)
  - [ ] 实现 St.Venant-Kirchhoff 模型

### 2.2 缝合线模块
- [ ] 2.2.1 实现 `include/physics/suture.h` 和 `src/physics/suture.cpp`
  - [ ] 定义 Suture::Particle 结构
  - [ ] 实现 Suture 类基础接口
  - [ ] 实现质点管理 (添加/删除/查询)

- [ ] 2.2.2 实现约束基类
  - [ ] 实现 Constraint 基类
  - [ ] 实现 DistanceConstraint
  - [ ] 实现 BendingConstraint
  - [ ] 实现 AnchorConstraint

### 2.3 器械模块
- [ ] 2.3.1 实现 `include/physics/instrument.h` 和 `src/physics/instrument.cpp`
  - [ ] 定义 Instrument::RigidBody 结构
  - [ ] 定义 Instrument::GraspPoint 结构
  - [ ] 实现 Instrument 类基础接口
  - [ ] 实现刚体状态管理
  - [ ] 实现夹持点管理

### 2.4 接触检测模块
- [ ] 2.4.1 实现 `include/physics/contact.h`
  - [ ] 定义 ContactPair 结构
  - [ ] 定义 ContactType 枚举
  - [ ] 实现接触力计算函数

- [ ] 2.4.2 实现 AABB 树
  - [ ] 实现 `include/collision/aabb_tree.h`
  - [ ] 实现 AABB 树构建
  - [ ] 实现 AABB 树查询
  - [ ] 实现包围盒更新

- [ ] 2.4.3 实现精细检测算法
  - [ ] 实现 `include/collision/narrow_phase.h`
  - [ ] 实现 Möller-Trumbore 射线-四面体相交
  - [ ] 实现点-三角形距离计算
  - [ ] 实现 GJK 算法 (`include/collision/gjk_algorithm.h`)
  - [ ] 实现 EPA 算法

---

## 阶段三：求解器模块 (Week 6-8)

### 3.1 FEM 求解器
- [ ] 3.1.1 实现 `include/physics/fem_solver.h` 和 `src/physics/fem_solver.cpp`
  - [ ] 实现刚度矩阵组装
  - [ ] 实现外力向量组装
  - [ ] 实现共轭梯度法 (CG)
  - [ ] 实现预处理共轭梯度法 (PCG)
  - [ ] 实现不完全 Cholesky 预处理

- [ ] 3.1.2 FEM CUDA 加速
  - [ ] 实现 `cuda/fem_kernels.cu`
  - [ ] 实现单元应力计算核函数
  - [ ] 实现矩阵组装核函数

### 3.2 XPBD 求解器
- [ ] 3.2.1 实现 `include/physics/xpbd_solver.h` 和 `src/physics/xpbd_solver.cpp`
  - [ ] 实现位置预测
  - [ ] 实现约束投影迭代
  - [ ] 实现速度更新
  - [ ] 实现阻尼处理

- [ ] 3.2.2 XPBD CUDA 加速
  - [ ] 实现 `cuda/xpbd_kernels.cu`
  - [ ] 实现约束求解核函数
  - [ ] 实现并行约束分组

### 3.3 刚体求解器
- [ ] 3.3.1 实现 `include/physics/rigid_body_solver.h` 和 `src/physics/rigid_body_solver.cpp`
  - [ ] 实现半隐式欧拉积分
  - [ ] 实现四元数更新
  - [ ] 实现力和力矩应用

### 3.4 约束投影器
- [ ] 3.4.1 实现 `include/physics/constraint_projector.h` 和 `src/physics/constraint_projector.cpp`
  - [ ] 实现关键约束投影
  - [ ] 实现非关键约束投影
  - [ ] 实现多体约束耦合
  - [ ] 实现收敛检测

---

## 阶段四：力反馈模块 (Week 9-10)

### 4.1 设备接口
- [ ] 4.1.1 实现 `include/haptic/haptic_device.h`
  - [ ] 定义设备抽象接口
  - [ ] 实现设备初始化/关闭
  - [ ] 实现位置/姿态读取
  - [ ] 实现力/力矩输出

- [ ] 4.1.2 实现具体设备驱动
  - [ ] 实现 Phantom Omni 支持
  - [ ] 实现 Falcon 支持
  - [ ] 实现虚拟设备 (用于测试)

### 4.2 信号处理
- [ ] 4.2.1 实现 `include/haptic/adaptive_filter.h` 和 `src/haptic/adaptive_filter.cpp`
  - [ ] 实现状态机 (NORMAL/EVENT)
  - [ ] 实现事件检测
  - [ ] 实现自适应截止频率
  - [ ] 实现低通滤波

- [ ] 4.2.2 实现 `include/haptic/delay_compensator.h` 和 `src/haptic/delay_compensator.cpp`
  - [ ] 实现 AR 模型
  - [ ] 实现在线参数更新
  - [ ] 实现滑动窗口管理
  - [ ] 实现预测算法

- [ ] 4.2.3 实现 `include/haptic/force_mapper.h` 和 `src/haptic/force_mapper.cpp`
  - [ ] 实现非线性映射
  - [ ] 实现用户校准接口
  - [ ] 实现分段线性插值

---

## 阶段五：渲染与交互 (Week 11-12)

### 5.1 渲染系统
- [ ] 5.1.1 实现 `include/rendering/renderer.h` 和 `src/rendering/renderer.cpp`
  - [ ] 初始化 OpenGL/Vulkan
  - [ ] 实现场景渲染
  - [ ] 实现网格渲染
  - [ ] 实现线框/实体切换

- [ ] 5.1.2 实现着色器
  - [ ] 创建 `shaders/vertex.glsl`
  - [ ] 创建 `shaders/fragment.glsl`
  - [ ] 实现光照模型
  - [ ] 实现材质渲染

- [ ] 5.1.3 实现 `include/rendering/camera.h`
  - [ ] 实现相机控制
  - [ ] 实现视角切换
  - [ ] 实现缩放/平移/旋转

### 5.2 交互系统
- [ ] 5.2.1 实现输入处理
  - [ ] 键盘事件处理
  - [ ] 鼠标事件处理
  - [ ] 力反馈设备事件处理

- [ ] 5.2.2 实现场景管理
  - [ ] 实现 `include/io/scene_manager.h`
  - [ ] 实现场景加载/保存
  - [ ] 实现物体添加/删除
  - [ ] 实现参数实时调整

---

## 阶段六：IO与工具 (Week 13-14)

### 6.1 文件IO
- [ ] 6.1.1 实现 `include/io/mesh_loader.h` 和 `src/io/mesh_loader.cpp`
  - [ ] 实现 OBJ 格式加载
  - [ ] 实现 STL 格式加载
  - [ ] 实现 VTK 格式加载 (四面体网格)
  - [ ] 实现网格验证

- [ ] 6.1.2 实现 `include/io/config_loader.h` 和 `src/io/config_loader.cpp`
  - [ ] 实现 JSON 配置加载
  - [ ] 实现参数验证
  - [ ] 实现热更新支持

- [ ] 6.1.3 实现 `include/io/state_serializer.h` 和 `src/io/state_serializer.cpp`
  - [ ] 实现场景状态序列化
  - [ ] 实现二进制保存/加载
  - [ ] 实现版本兼容

### 6.2 参数标定工具
- [ ] 6.2.1 创建 `tools/calibration_tool/`
  - [ ] 实现离线数据拟合
  - [ ] 实现参数优化
  - [ ] 实现结果可视化
  - [ ] 实现参数导出

---

## 阶段七：测试与优化 (Week 15-16)

### 7.1 单元测试
- [ ] 7.1.1 创建 `tests/unit_tests/`
  - [ ] 测试 AABB 树
  - [ ] 测试 GJK/EPA 算法
  - [ ] 测试 FEM 求解器
  - [ ] 测试 XPBD 求解器
  - [ ] 测试约束投影
  - [ ] 测试滤波器

### 7.2 集成测试
- [ ] 7.2.1 创建 `tests/integration_tests/`
  - [ ] 测试简单穿刺场景
  - [ ] 测试缝合操作场景
  - [ ] 测试力反馈延迟
  - [ ] 测试大规模场景性能

### 7.3 性能优化
- [ ] 7.3.1 CUDA 优化
  - [ ] 优化内存访问模式
  - [ ] 优化线程块配置
  - [ ] 实现流并行

- [ ] 7.3.2 CPU 优化
  - [ ] 优化数据结构布局
  - [ ] 实现 SIMD 加速
  - [ ] 优化缓存利用

---

## 阶段八：集成与交付 (Week 17-18)

### 8.1 主程序
- [ ] 8.1.1 实现 `src/main.cpp`
  - [ ] 实现主循环
  - [ ] 实现模块初始化
  - [ ] 实现命令行参数解析
  - [ ] 实现异常处理

### 8.2 文档
- [ ] 8.2.1 完善文档
  - [ ] 编写 API 文档
  - [ ] 编写用户手册
  - [ ] 编写开发文档
  - [ ] 编写示例教程

### 8.3 打包交付
- [ ] 8.3.1 发布准备
  - [ ] 创建安装程序
  - [ ] 准备示例数据
  - [ ] 编写发布说明

---

## 任务统计

| 阶段 | 任务数 | 预计工期 |
|------|--------|----------|
| 阶段一：基础框架 | 10 | 2周 |
| 阶段二：物理核心 | 15 | 3周 |
| 阶段三：求解器 | 12 | 3周 |
| 阶段四：力反馈 | 8 | 2周 |
| 阶段五：渲染交互 | 8 | 2周 |
| 阶段六：IO与工具 | 7 | 2周 |
| 阶段七：测试优化 | 6 | 2周 |
| 阶段八：集成交付 | 5 | 2周 |
| **总计** | **71** | **18周** |

---

## 优先级标记

- **P0 (关键路径)**: 必须完成的任务，影响核心功能
- **P1 (高优先级)**: 重要任务，影响主要功能
- **P2 (中优先级)**: 一般任务，影响辅助功能
- **P3 (低优先级)**: 可选任务，增强功能

### P0 任务列表
1. 项目结构初始化
2. 核心类型定义
3. 软组织基础实现
4. 缝合线基础实现
5. 器械基础实现
6. AABB树实现
7. FEM求解器基础
8. XPBD求解器基础
9. 刚体求解器
10. 约束投影器
11. 主程序实现

### P1 任务列表
1. 配置文件系统
2. 精细检测算法
3. CUDA加速 (FEM/XPBD)
4. 力反馈设备接口
5. 渲染系统基础
6. 文件IO基础
7. 单元测试基础

### P2/P3 任务列表
1. 信号处理 (自适应滤波、延迟补偿)
2. 高级渲染效果
3. 参数标定工具
4. 集成测试
5. 性能优化
6. 完整文档
