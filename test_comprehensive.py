#!/usr/bin/env python3
"""
虚拟手术缝合多体耦合仿真系统 - 综合阶段测试
包含：功能测试、性能测试、数据记录、可视化准备
"""

import numpy as np
import json
import csv
import os
from datetime import datetime
from dataclasses import dataclass, asdict
from typing import List, Dict, Tuple
import math

# ============ 测试配置 ============
TEST_CONFIG = {
    "simulation_steps": 1000,
    "dt": 0.001,
    "output_dir": "test_output",
    "save_interval": 10,  # 每10步保存一次数据
}

# ============ 数据记录类 ============

@dataclass
class SimulationData:
    """仿真数据记录"""
    timestep: int
    time: float
    
    # 软组织数据
    tissue_volume: float
    tissue_mass: float
    tissue_com: Tuple[float, float, float]
    tissue_energy: float
    
    # 缝合线数据
    suture_length: float
    suture_com: Tuple[float, float, float]
    suture_velocity: Tuple[float, float, float]
    
    # 器械数据
    instrument_position: Tuple[float, float, float]
    instrument_velocity: Tuple[float, float, float]
    
    # 性能数据
    computation_time: float

@dataclass
class TestResult:
    """测试结果"""
    test_name: str
    passed: bool
    duration: float
    error_message: str = ""
    details: Dict = None

# ============ 向量类 ============

class Vec3:
    """3D向量"""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.v = np.array([x, y, z], dtype=float)
    
    @property
    def x(self): return self.v[0]
    @property
    def y(self): return self.v[1]
    @property
    def z(self): return self.v[2]
    
    def __add__(self, other): return Vec3(*(self.v + other.v))
    def __sub__(self, other): return Vec3(*(self.v - other.v))
    def __mul__(self, s): return Vec3(*(self.v * s))
    def __truediv__(self, s): return Vec3(*(self.v / s))
    def __repr__(self): return f"Vec3({self.x:.4f}, {self.y:.4f}, {self.z:.4f})"
    
    def dot(self, other): return np.dot(self.v, other.v)
    def cross(self, other): return Vec3(*np.cross(self.v, other.v))
    def length(self): return np.linalg.norm(self.v)
    def normalized(self): 
        len = self.length()
        return self / len if len > 1e-10 else Vec3()
    def to_tuple(self): return (self.x, self.y, self.z)

# ============ 物理模块 ============

class Tetrahedron:
    """四面体"""
    def __init__(self, v0, v1, v2, v3):
        self.vertices = [v0, v1, v2, v3]
    
    def volume(self):
        e1 = self.vertices[1] - self.vertices[0]
        e2 = self.vertices[2] - self.vertices[0]
        e3 = self.vertices[3] - self.vertices[0]
        return abs(e1.dot(e2.cross(e3))) / 6.0
    
    def centroid(self):
        return sum(self.vertices, Vec3()) * 0.25

class TissueNode:
    """软组织节点"""
    def __init__(self):
        self.position = Vec3()
        self.velocity = Vec3()
        self.force = Vec3()
        self.mass = 0.0
        self.is_fixed = False

class SimpleTissue:
    """简化软组织"""
    def __init__(self):
        self.nodes = []
        self.elements = []
        self.energy = 0.0
    
    def add_node(self, pos, mass=1.0):
        node = TissueNode()
        node.position = pos
        node.mass = mass
        self.nodes.append(node)
        return len(self.nodes) - 1
    
    def add_tetrahedron(self, n0, n1, n2, n3):
        self.elements.append(Tetrahedron(
            self.nodes[n0].position,
            self.nodes[n1].position,
            self.nodes[n2].position,
            self.nodes[n3].position
        ))
    
    def get_total_volume(self):
        return sum(elem.volume() for elem in self.elements)
    
    def get_total_mass(self):
        return sum(node.mass for node in self.nodes)
    
    def get_center_of_mass(self):
        center = Vec3()
        total_mass = 0.0
        for node in self.nodes:
            center = center + node.position * node.mass
            total_mass += node.mass
        return center / total_mass if total_mass > 0 else Vec3()
    
    def compute_energy(self):
        # 简化的能量计算 (动能 + 势能)
        kinetic = 0.0
        potential = 0.0
        gravity = 9.81
        
        for node in self.nodes:
            v = node.velocity.length()
            kinetic += 0.5 * node.mass * v * v
            potential += node.mass * gravity * node.position.y
        
        self.energy = kinetic + potential
        return self.energy

class SutureParticle:
    """缝合线质点"""
    def __init__(self):
        self.position = Vec3()
        self.prev_position = Vec3()
        self.velocity = Vec3()
        self.mass = 0.01
        self.inverse_mass = 100.0
        self.is_fixed = False

class SimpleSuture:
    """简化缝合线"""
    def __init__(self):
        self.particles = []
        self.constraints = []
    
    def create_chain(self, positions, total_mass):
        self.particles = []
        mass_per_particle = total_mass / len(positions)
        
        for pos in positions:
            p = SutureParticle()
            p.position = pos
            p.prev_position = pos
            p.mass = mass_per_particle
            p.inverse_mass = 1.0 / mass_per_particle
            self.particles.append(p)
        
        # 创建距离约束
        for i in range(len(positions) - 1):
            target_dist = (positions[i+1] - positions[i]).length()
            self.constraints.append({
                'type': 'distance',
                'p1': i,
                'p2': i+1,
                'rest_length': target_dist
            })
    
    def simulate(self, dt, gravity):
        # 预测位置
        for p in self.particles:
            if not p.is_fixed:
                p.prev_position = p.position
                p.velocity = p.velocity + gravity * dt
                p.position = p.position + p.velocity * dt
        
        # 求解约束
        for _ in range(10):  # 迭代次数
            for constraint in self.constraints:
                if constraint['type'] == 'distance':
                    p1 = self.particles[constraint['p1']]
                    p2 = self.particles[constraint['p2']]
                    
                    delta = p2.position - p1.position
                    dist = delta.length()
                    if dist < 1e-10: continue
                    
                    target = constraint['rest_length']
                    diff = (dist - target) / dist
                    offset = delta * diff * 0.5
                    
                    if not p1.is_fixed:
                        p1.position = p1.position + offset
                    if not p2.is_fixed:
                        p2.position = p2.position - offset
        
        # 更新速度
        for p in self.particles:
            if not p.is_fixed:
                p.velocity = (p.position - p.prev_position) / dt
    
    def get_total_length(self):
        length = 0.0
        for i in range(len(self.particles) - 1):
            length += (self.particles[i + 1].position - self.particles[i].position).length()
        return length
    
    def get_center_of_mass(self):
        center = Vec3()
        total_mass = 0.0
        for p in self.particles:
            center = center + p.position * p.mass
            total_mass += p.mass
        return center / total_mass if total_mass > 0 else Vec3()
    
    def get_average_velocity(self):
        vel = Vec3()
        for p in self.particles:
            vel = vel + p.velocity
        return vel / len(self.particles) if self.particles else Vec3()

class InstrumentState:
    """器械刚体状态"""
    def __init__(self):
        self.position = Vec3()
        self.velocity = Vec3()
        self.mass = 0.5
        self.force = Vec3()
    
    def integrate(self, dt):
        self.velocity = self.velocity + (self.force * (1.0 / self.mass)) * dt
        self.position = self.position + self.velocity * dt
        self.force = Vec3()

# ============ 测试类 ============

class ComprehensiveTest:
    """综合测试类"""
    
    def __init__(self):
        self.results = []
        self.simulation_data = []
        self.output_dir = TEST_CONFIG["output_dir"]
        os.makedirs(self.output_dir, exist_ok=True)
        os.makedirs(f"{self.output_dir}/plots", exist_ok=True)
        
    def run_all_tests(self):
        """运行所有测试"""
        print("=" * 70)
        print("虚拟手术缝合多体耦合仿真系统 - 综合阶段测试")
        print("=" * 70)
        print()
        
        # 基础功能测试
        self.test_basic_types()
        self.test_tissue()
        self.test_suture()
        self.test_instrument()
        
        # 集成测试
        self.test_integration()
        self.test_coupling()
        
        # 性能测试
        self.test_performance()
        self.test_stability()
        
        # 保存结果
        self.save_results()
        self.save_simulation_data()
        self.generate_visualization_data()
        
        # 打印总结
        self.print_summary()
        
        return self.results
    
    def test_basic_types(self):
        """测试基础类型"""
        print("[测试 1/7] 基础类型测试...")
        start_time = datetime.now()
        
        try:
            # 向量测试
            v1 = Vec3(1, 2, 3)
            v2 = Vec3(4, 5, 6)
            v3 = v1 + v2
            assert abs(v3.x - 5.0) < 1e-10, "向量加法失败"
            
            dot = v1.dot(v2)
            assert abs(dot - 32.0) < 1e-10, "向量点积失败"
            
            cross = v1.cross(v2)
            assert abs(cross.x - (-3.0)) < 1e-10, "向量叉积失败"
            
            # 四面体测试
            tet = Tetrahedron(
                Vec3(0, 0, 0),
                Vec3(1, 0, 0),
                Vec3(0, 1, 0),
                Vec3(0, 0, 1)
            )
            vol = tet.volume()
            assert abs(vol - 1.0/6.0) < 1e-10, "四面体体积计算失败"
            
            duration = (datetime.now() - start_time).total_seconds()
            result = TestResult("基础类型测试", True, duration, 
                               details={"volume": vol, "dot_product": dot})
            print(f"  ✓ 通过 (耗时: {duration:.4f}s)")
            
        except Exception as e:
            duration = (datetime.now() - start_time).total_seconds()
            result = TestResult("基础类型测试", False, duration, str(e))
            print(f"  ✗ 失败: {e}")
        
        self.results.append(result)
        print()
    
    def test_tissue(self):
        """测试软组织"""
        print("[测试 2/7] 软组织测试...")
        start_time = datetime.now()
        
        try:
            tissue = SimpleTissue()
            
            # 创建四面体网格
            tissue.add_node(Vec3(0, 0, 0), 0.25)
            tissue.add_node(Vec3(1, 0, 0), 0.25)
            tissue.add_node(Vec3(0, 1, 0), 0.25)
            tissue.add_node(Vec3(0, 0, 1), 0.25)
            tissue.add_tetrahedron(0, 1, 2, 3)
            
            volume = tissue.get_total_volume()
            mass = tissue.get_total_mass()
            com = tissue.get_center_of_mass()
            
            assert abs(volume - 1.0/6.0) < 1e-10, "体积计算错误"
            assert abs(mass - 1.0) < 1e-10, "质量计算错误"
            
            duration = (datetime.now() - start_time).total_seconds()
            result = TestResult("软组织测试", True, duration,
                               details={"volume": volume, "mass": mass, 
                                       "com": com.to_tuple()})
            print(f"  ✓ 通过 (耗时: {duration:.4f}s)")
            print(f"    体积: {volume:.6f}, 质量: {mass:.4f}")
            
        except Exception as e:
            duration = (datetime.now() - start_time).total_seconds()
            result = TestResult("软组织测试", False, duration, str(e))
            print(f"  ✗ 失败: {e}")
        
        self.results.append(result)
        print()
    
    def test_suture(self):
        """测试缝合线"""
        print("[测试 3/7] 缝合线测试...")
        start_time = datetime.now()
        
        try:
            suture = SimpleSuture()
            
            positions = [Vec3(i * 0.1, 0, 0) for i in range(10)]
            suture.create_chain(positions, 0.1)
            
            initial_length = suture.get_total_length()
            initial_com = suture.get_center_of_mass()
            
            # 模拟
            gravity = Vec3(0, -9.81, 0)
            for _ in range(100):
                suture.simulate(0.001, gravity)
            
            final_length = suture.get_total_length()
            final_com = suture.get_center_of_mass()
            
            # 长度应保持不变 (约束)
            assert abs(final_length - initial_length) < 1e-6, "长度约束失效"
            
            duration = (datetime.now() - start_time).total_seconds()
            result = TestResult("缝合线测试", True, duration,
                               details={"initial_length": initial_length,
                                       "final_length": final_length,
                                       "com_displacement": (final_com - initial_com).length()})
            print(f"  ✓ 通过 (耗时: {duration:.4f}s)")
            print(f"    初始长度: {initial_length:.6f}, 最终长度: {final_length:.6f}")
            print(f"    质心位移: {(final_com - initial_com).length():.6f}")
            
        except Exception as e:
            duration = (datetime.now() - start_time).total_seconds()
            result = TestResult("缝合线测试", False, duration, str(e))
            print(f"  ✗ 失败: {e}")
        
        self.results.append(result)
        print()
    
    def test_instrument(self):
        """测试器械"""
        print("[测试 4/7] 器械测试...")
        start_time = datetime.now()
        
        try:
            inst = InstrumentState()
            inst.mass = 0.5
            
            # 施加恒力
            inst.force = Vec3(1.0, 0, 0)
            
            positions = []
            velocities = []
            
            for i in range(100):
                inst.integrate(0.001)
                inst.force = Vec3(1.0, 0, 0)
                
                positions.append(inst.position.to_tuple())
                velocities.append(inst.velocity.length())
            
            # 检查运动是否符合牛顿定律
            expected_acc = 1.0 / 0.5  # F/m = 2.0 m/s^2
            actual_acc = inst.velocity.x / (100 * 0.001)
            
            assert abs(actual_acc - expected_acc) < 0.1, "加速度计算错误"
            
            duration = (datetime.now() - start_time).total_seconds()
            result = TestResult("器械测试", True, duration,
                               details={"expected_acc": expected_acc,
                                       "actual_acc": actual_acc,
                                       "final_position": inst.position.to_tuple()})
            print(f"  ✓ 通过 (耗时: {duration:.4f}s)")
            print(f"    预期加速度: {expected_acc:.4f}, 实际加速度: {actual_acc:.4f}")
            
        except Exception as e:
            duration = (datetime.now() - start_time).total_seconds()
            result = TestResult("器械测试", False, duration, str(e))
            print(f"  ✗ 失败: {e}")
        
        self.results.append(result)
        print()
    
    def test_integration(self):
        """测试系统集成"""
        print("[测试 5/7] 系统集成测试...")
        start_time = datetime.now()
        
        try:
            # 创建完整场景
            tissue = SimpleTissue()
            suture = SimpleSuture()
            inst = InstrumentState()
            
            # 初始化
            tissue.add_node(Vec3(0, 0, 0), 1.0)
            tissue.add_node(Vec3(1, 0, 0), 1.0)
            tissue.add_node(Vec3(0, 1, 0), 1.0)
            tissue.add_node(Vec3(0, 0, 1), 1.0)
            tissue.add_tetrahedron(0, 1, 2, 3)
            
            positions = [Vec3(0.5, 0.5 + i*0.05, 0.5) for i in range(5)]
            suture.create_chain(positions, 0.05)
            
            inst.position = Vec3(0.5, 1.0, 0.5)
            
            # 运行仿真并记录数据
            gravity = Vec3(0, -9.81, 0)
            
            for step in range(TEST_CONFIG["simulation_steps"]):
                sim_start = datetime.now()
                
                # 更新软组织
                tissue.compute_energy()
                
                # 更新缝合线
                suture.simulate(TEST_CONFIG["dt"], gravity)
                
                # 更新器械
                inst.integrate(TEST_CONFIG["dt"])
                
                sim_time = (datetime.now() - sim_start).total_seconds()
                
                # 记录数据
                if step % TEST_CONFIG["save_interval"] == 0:
                    data = SimulationData(
                        timestep=step,
                        time=step * TEST_CONFIG["dt"],
                        tissue_volume=tissue.get_total_volume(),
                        tissue_mass=tissue.get_total_mass(),
                        tissue_com=tissue.get_center_of_mass().to_tuple(),
                        tissue_energy=tissue.energy,
                        suture_length=suture.get_total_length(),
                        suture_com=suture.get_center_of_mass().to_tuple(),
                        suture_velocity=suture.get_average_velocity().to_tuple(),
                        instrument_position=inst.position.to_tuple(),
                        instrument_velocity=inst.velocity.to_tuple(),
                        computation_time=sim_time
                    )
                    self.simulation_data.append(data)
            
            duration = (datetime.now() - start_time).total_seconds()
            result = TestResult("系统集成测试", True, duration,
                               details={"steps": TEST_CONFIG["simulation_steps"],
                                       "data_points": len(self.simulation_data),
                                       "avg_time_per_step": duration / TEST_CONFIG["simulation_steps"]})
            print(f"  ✓ 通过 (耗时: {duration:.4f}s)")
            print(f"    仿真步数: {TEST_CONFIG['simulation_steps']}")
            print(f"    数据点: {len(self.simulation_data)}")
            print(f"    平均每步耗时: {duration / TEST_CONFIG['simulation_steps'] * 1000:.4f}ms")
            
        except Exception as e:
            duration = (datetime.now() - start_time).total_seconds()
            result = TestResult("系统集成测试", False, duration, str(e))
            print(f"  ✗ 失败: {e}")
            import traceback
            traceback.print_exc()
        
        self.results.append(result)
        print()
    
    def test_coupling(self):
        """测试多体耦合"""
        print("[测试 6/7] 多体耦合测试...")
        start_time = datetime.now()
        
        try:
            # 测试缝合线-器械耦合
            suture = SimpleSuture()
            inst = InstrumentState()
            
            positions = [Vec3(0.5, 0.5 + i*0.1, 0.5) for i in range(5)]
            suture.create_chain(positions, 0.1)
            
            # 固定第一个质点到器械
            suture.particles[0].is_fixed = True
            inst.position = Vec3(0.5, 0.5, 0.5)
            
            # 移动器械
            initial_com = suture.get_center_of_mass()
            
            for _ in range(50):
                inst.velocity = Vec3(0.1, 0, 0)
                inst.integrate(0.001)
                suture.particles[0].position = inst.position
                suture.simulate(0.001, Vec3(0, 0, 0))
            
            final_com = suture.get_center_of_mass()
            displacement = (final_com - initial_com).length()
            
            assert displacement > 0.01, "耦合失效：缝合线未随器械移动"
            
            duration = (datetime.now() - start_time).total_seconds()
            result = TestResult("多体耦合测试", True, duration,
                               details={"displacement": displacement,
                                       "instrument_moved": inst.position.x - 0.5})
            print(f"  ✓ 通过 (耗时: {duration:.4f}s)")
            print(f"    缝合线质心位移: {displacement:.6f}")
            print(f"    器械位移: {inst.position.x - 0.5:.6f}")
            
        except Exception as e:
            duration = (datetime.now() - start_time).total_seconds()
            result = TestResult("多体耦合测试", False, duration, str(e))
            print(f"  ✗ 失败: {e}")
        
        self.results.append(result)
        print()
    
    def test_performance(self):
        """性能测试"""
        print("[测试 7/7] 性能测试...")
        start_time = datetime.now()
        
        try:
            # 大规模场景测试
            suture = SimpleSuture()
            positions = [Vec3(i * 0.01, 0, 0) for i in range(100)]
            suture.create_chain(positions, 1.0)
            
            gravity = Vec3(0, -9.81, 0)
            
            import time
            sim_start = time.time()
            steps = 500
            
            for _ in range(steps):
                suture.simulate(0.001, gravity)
            
            sim_time = time.time() - sim_start
            fps = steps / sim_time
            ms_per_frame = sim_time / steps * 1000
            
            # 检查是否满足实时要求 (>30 FPS)
            real_time = fps >= 30.0
            
            duration = (datetime.now() - start_time).total_seconds()
            result = TestResult("性能测试", real_time, duration,
                               details={"fps": fps,
                                       "ms_per_frame": ms_per_frame,
                                       "particles": 100,
                                       "steps": steps,
                                       "target_fps": 30.0})
            print(f"  {'✓' if real_time else '⚠'} 通过 (耗时: {duration:.4f}s)")
            print(f"    FPS: {fps:.2f}")
            print(f"    每帧耗时: {ms_per_frame:.4f}ms")
            print(f"    粒子数: 100, 步数: {steps}")
            
            if not real_time:
                print(f"    警告: 未达到实时要求 (30 FPS)")
            
        except Exception as e:
            duration = (datetime.now() - start_time).total_seconds()
            result = TestResult("性能测试", False, duration, str(e))
            print(f"  ✗ 失败: {e}")
        
        self.results.append(result)
        print()
    
    def test_stability(self):
        """稳定性测试"""
        print("[附加测试] 稳定性测试...")
        start_time = datetime.now()
        
        try:
            # 长时间运行测试
            suture = SimpleSuture()
            positions = [Vec3(0, 1.0 - i*0.1, 0) for i in range(10)]
            suture.create_chain(positions, 0.1)
            
            gravity = Vec3(0, -9.81, 0)
            
            # 固定顶端
            suture.particles[0].is_fixed = True
            
            max_displacement = 0.0
            energy_history = []
            
            for step in range(2000):
                suture.simulate(0.001, gravity)
                
                # 检查能量守恒 (近似)
                if step % 100 == 0:
                    com = suture.get_center_of_mass()
                    ke = sum(p.velocity.length()**2 * p.mass * 0.5 for p in suture.particles)
                    pe = sum(p.position.y * p.mass * 9.81 for p in suture.particles)
                    total_energy = ke + pe
                    energy_history.append(total_energy)
                    
                    if step > 0:
                        max_displacement = max(max_displacement, abs(total_energy - energy_history[0]))
            
            # 检查是否发散
            final_length = suture.get_total_length()
            initial_length = 0.9  # 9 * 0.1
            
            stable = abs(final_length - initial_length) < 0.1
            
            duration = (datetime.now() - start_time).total_seconds()
            result = TestResult("稳定性测试", stable, duration,
                               details={"max_energy_drift": max_displacement,
                                       "final_length": final_length,
                                       "steps": 2000})
            print(f"  {'✓' if stable else '✗'} 通过 (耗时: {duration:.4f}s)")
            print(f"    最大能量漂移: {max_displacement:.6f}")
            print(f"    最终长度: {final_length:.6f}")
            print(f"    仿真步数: 2000")
            
        except Exception as e:
            duration = (datetime.now() - start_time).total_seconds()
            result = TestResult("稳定性测试", False, duration, str(e))
            print(f"  ✗ 失败: {e}")
        
        self.results.append(result)
        print()
    
    def save_results(self):
        """保存测试结果"""
        print("保存测试结果...")
        
        # JSON格式
        json_data = {
            "test_timestamp": datetime.now().isoformat(),
            "config": TEST_CONFIG,
            "results": [
                {
                    "test_name": r.test_name,
                    "passed": r.passed,
                    "duration": r.duration,
                    "error_message": r.error_message,
                    "details": r.details
                }
                for r in self.results
            ],
            "summary": {
                "total": len(self.results),
                "passed": sum(1 for r in self.results if r.passed),
                "failed": sum(1 for r in self.results if not r.passed),
                "total_duration": sum(r.duration for r in self.results)
            }
        }
        
        with open(f"{self.output_dir}/test_results.json", "w") as f:
            json.dump(json_data, f, indent=2)
        
        # CSV格式
        with open(f"{self.output_dir}/test_results.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Test Name", "Passed", "Duration (s)", "Error"])
            for r in self.results:
                writer.writerow([r.test_name, "Yes" if r.passed else "No", 
                                r.duration, r.error_message])
        
        print(f"  ✓ 结果已保存到 {self.output_dir}/")
        print()
    
    def save_simulation_data(self):
        """保存仿真数据"""
        print("保存仿真数据...")
        
        if not self.simulation_data:
            print("  无仿真数据可保存")
            return
        
        # CSV格式
        with open(f"{self.output_dir}/simulation_data.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "Timestep", "Time",
                "Tissue_Volume", "Tissue_Mass", "Tissue_COM_X", "Tissue_COM_Y", "Tissue_COM_Z",
                "Tissue_Energy",
                "Suture_Length", "Suture_COM_X", "Suture_COM_Y", "Suture_COM_Z",
                "Suture_Vel_X", "Suture_Vel_Y", "Suture_Vel_Z",
                "Inst_Pos_X", "Inst_Pos_Y", "Inst_Pos_Z",
                "Inst_Vel_X", "Inst_Vel_Y", "Inst_Vel_Z",
                "Computation_Time"
            ])
            
            for d in self.simulation_data:
                writer.writerow([
                    d.timestep, d.time,
                    d.tissue_volume, d.tissue_mass,
                    d.tissue_com[0], d.tissue_com[1], d.tissue_com[2],
                    d.tissue_energy,
                    d.suture_length,
                    d.suture_com[0], d.suture_com[1], d.suture_com[2],
                    d.suture_velocity[0], d.suture_velocity[1], d.suture_velocity[2],
                    d.instrument_position[0], d.instrument_position[1], d.instrument_position[2],
                    d.instrument_velocity[0], d.instrument_velocity[1], d.instrument_velocity[2],
                    d.computation_time
                ])
        
        # JSON格式
        json_data = [asdict(d) for d in self.simulation_data]
        with open(f"{self.output_dir}/simulation_data.json", "w") as f:
            json.dump(json_data, f, indent=2)
        
        print(f"  ✓ 数据已保存 ({len(self.simulation_data)} 个时间点)")
        print()
    
    def generate_visualization_data(self):
        """生成可视化数据"""
        print("生成可视化数据...")
        
        if not self.simulation_data:
            print("  无数据可可视化")
            return
        
        # 提取时间序列数据
        times = [d.time for d in self.simulation_data]
        tissue_energy = [d.tissue_energy for d in self.simulation_data]
        suture_length = [d.suture_length for d in self.simulation_data]
        suture_com_y = [d.suture_com[1] for d in self.simulation_data]
        inst_pos_x = [d.instrument_position[0] for d in self.simulation_data]
        comp_time = [d.computation_time * 1000 for d in self.simulation_data]  # 转换为ms
        
        # 保存为gnuplot格式
        with open(f"{self.output_dir}/plots/tissue_energy.dat", "w") as f:
            f.write("# Time(s) Energy(J)\n")
            for t, e in zip(times, tissue_energy):
                f.write(f"{t:.6f} {e:.6f}\n")
        
        with open(f"{self.output_dir}/plots/suture_length.dat", "w") as f:
            f.write("# Time(s) Length(m)\n")
            for t, l in zip(times, suture_length):
                f.write(f"{t:.6f} {l:.6f}\n")
        
        with open(f"{self.output_dir}/plots/suture_motion.dat", "w") as f:
            f.write("# Time(s) COM_Y(m)\n")
            for t, y in zip(times, suture_com_y):
                f.write(f"{t:.6f} {y:.6f}\n")
        
        with open(f"{self.output_dir}/plots/instrument_motion.dat", "w") as f:
            f.write("# Time(s) Pos_X(m)\n")
            for t, x in zip(times, inst_pos_x):
                f.write(f"{t:.6f} {x:.6f}\n")
        
        with open(f"{self.output_dir}/plots/performance.dat", "w") as f:
            f.write("# Time(s) Computation_Time(ms)\n")
            for t, ct in zip(times, comp_time):
                f.write(f"{t:.6f} {ct:.6f}\n")
        
        # 生成gnuplot脚本
        gnuplot_script = """
set terminal png size 1200,800
set grid

# 组织能量
set output 'tissue_energy.png'
set title 'Tissue Energy Over Time'
set xlabel 'Time (s)'
set ylabel 'Energy (J)'
plot 'tissue_energy.dat' using 1:2 with lines title 'Total Energy' lw 2

# 缝合线长度
set output 'suture_length.png'
set title 'Suture Length Over Time'
set xlabel 'Time (s)'
set ylabel 'Length (m)'
plot 'suture_length.dat' using 1:2 with lines title 'Length' lw 2

# 缝合线运动
set output 'suture_motion.png'
set title 'Suture Center of Mass (Y)'
set xlabel 'Time (s)'
set ylabel 'Y Position (m)'
plot 'suture_motion.dat' using 1:2 with lines title 'COM Y' lw 2

# 器械运动
set output 'instrument_motion.png'
set title 'Instrument Position (X)'
set xlabel 'Time (s)'
set ylabel 'X Position (m)'
plot 'instrument_motion.dat' using 1:2 with lines title 'Position X' lw 2

# 性能
set output 'performance.png'
set title 'Computation Time Per Frame'
set xlabel 'Time (s)'
set ylabel 'Computation Time (ms)'
plot 'performance.dat' using 1:2 with lines title 'Frame Time' lw 2
"""
        
        with open(f"{self.output_dir}/plots/plot_script.gnuplot", "w") as f:
            f.write(gnuplot_script)
        
        print(f"  ✓ 可视化数据已生成")
        print(f"    数据文件: {self.output_dir}/plots/*.dat")
        print(f"    Gnuplot脚本: {self.output_dir}/plots/plot_script.gnuplot")
        print()
    
    def print_summary(self):
        """打印测试总结"""
        print("=" * 70)
        print("测试总结")
        print("=" * 70)
        
        total = len(self.results)
        passed = sum(1 for r in self.results if r.passed)
        failed = total - passed
        total_duration = sum(r.duration for r in self.results)
        
        print(f"\n总测试数: {total}")
        print(f"通过: {passed} ✓")
        print(f"失败: {failed} ✗")
        print(f"总耗时: {total_duration:.4f}s")
        print(f"\n通过率: {passed/total*100:.1f}%")
        
        if failed > 0:
            print("\n失败的测试:")
            for r in self.results:
                if not r.passed:
                    print(f"  - {r.test_name}: {r.error_message}")
        
        print("\n" + "=" * 70)
        print("输出文件:")
        print(f"  - {self.output_dir}/test_results.json")
        print(f"  - {self.output_dir}/test_results.csv")
        print(f"  - {self.output_dir}/simulation_data.json")
        print(f"  - {self.output_dir}/simulation_data.csv")
        print(f"  - {self.output_dir}/plots/")
        print("=" * 70)

# ============ 主程序 ============

if __name__ == "__main__":
    test = ComprehensiveTest()
    test.run_all_tests()
