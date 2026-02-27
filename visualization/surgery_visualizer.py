#!/usr/bin/env python3
"""
虚拟手术缝合仿真可视化界面
支持3D显示、人机交互、实时数据监控
"""

import sys
import numpy as np
import math
from dataclasses import dataclass
from typing import List, Tuple, Optional
from datetime import datetime

# 尝试导入PyQt5，如果不存在则使用Tkinter
try:
    from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                                 QHBoxLayout, QPushButton, QLabel, QSlider, 
                                 QGroupBox, QGridLayout, QTextEdit, QTabWidget,
                                 QDoubleSpinBox, QCheckBox, QComboBox)
    from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread
    from PyQt5.QtGui import QFont, QColor, QPalette
    HAS_PYQT5 = True
except ImportError:
    HAS_PYQT5 = False
    print("PyQt5 not found, using Tkinter fallback")

# 尝试导入matplotlib
try:
    import matplotlib
    if HAS_PYQT5:
        matplotlib.use('Qt5Agg')
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.figure import Figure
    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("Matplotlib not found, visualization will be limited")


# ============ 核心仿真类（复用之前的改进版代码） ============

@dataclass
class Vec3:
    """3D向量"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    
    def __add__(self, other): 
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)
    def __sub__(self, other): 
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)
    def __mul__(self, s): 
        if isinstance(s, Vec3):
            return Vec3(self.x * s.x, self.y * s.y, self.z * s.z)
        return Vec3(self.x * s, self.y * s, self.z * s)
    def __truediv__(self, s): 
        return Vec3(self.x / s, self.y / s, self.z / s)
    def dot(self, other): 
        return self.x * other.x + self.y * other.y + self.z * other.z
    def length(self): 
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
    def length_sq(self): 
        return self.x * self.x + self.y * self.y + self.z * self.z
    def copy(self): 
        return Vec3(self.x, self.y, self.z)
    def to_tuple(self):
        return (self.x, self.y, self.z)


class SutureParticle:
    """缝合线质点"""
    __slots__ = ['position', 'prev_position', 'velocity', 'mass', 
                 'inverse_mass', 'is_fixed', 'external_force']
    
    def __init__(self):
        self.position = Vec3()
        self.prev_position = Vec3()
        self.velocity = Vec3()
        self.mass = 0.01
        self.inverse_mass = 100.0
        self.is_fixed = False
        self.external_force = Vec3()


class Suture:
    """缝合线"""
    def __init__(self):
        self.particles: List[SutureParticle] = []
        self.constraints = []
        self.solver_iterations = 10
        self.damping = 0.99
        self.gravity = Vec3(0, -0.5, 0)
        self.coupling_constraints = []
        
    def create_chain(self, positions: List[Vec3], total_mass: float):
        """创建链式结构"""
        self.particles = []
        mass_per_particle = total_mass / len(positions)
        
        for pos in positions:
            p = SutureParticle()
            p.position = pos.copy()
            p.prev_position = pos.copy()
            p.mass = mass_per_particle
            p.inverse_mass = 1.0 / mass_per_particle if mass_per_particle > 0 else 0.0
            self.particles.append(p)
        
        # 创建距离约束
        for i in range(len(positions) - 1):
            target_dist = (positions[i+1] - positions[i]).length()
            self.constraints.append({
                'type': 'distance',
                'p1': i,
                'p2': i+1,
                'rest_length': target_dist,
                'stiffness': 1.0
            })
    
    def simulate(self, dt: float):
        """XPBD模拟"""
        # 预测位置
        for p in self.particles:
            if not p.is_fixed:
                p.prev_position = p.position.copy()
                total_force = self.gravity * p.mass + p.external_force
                p.velocity.x += total_force.x * p.inverse_mass * dt
                p.velocity.y += total_force.y * p.inverse_mass * dt
                p.velocity.z += total_force.z * p.inverse_mass * dt
                
                p.velocity.x *= self.damping
                p.velocity.y *= self.damping
                p.velocity.z *= self.damping
                
                p.position.x += p.velocity.x * dt
                p.position.y += p.velocity.y * dt
                p.position.z += p.velocity.z * dt
                
                p.external_force = Vec3()
        
        # 求解约束
        for _ in range(self.solver_iterations):
            for constraint in self.constraints:
                if constraint['type'] == 'distance':
                    self._solve_distance_constraint(constraint)
            
            for constraint in self.coupling_constraints:
                self._solve_coupling_constraint(constraint)
        
        # 更新速度
        for p in self.particles:
            if not p.is_fixed:
                p.velocity.x = (p.position.x - p.prev_position.x) / dt
                p.velocity.y = (p.position.y - p.prev_position.y) / dt
                p.velocity.z = (p.position.z - p.prev_position.z) / dt
    
    def _solve_distance_constraint(self, constraint):
        """距离约束求解"""
        p1 = self.particles[constraint['p1']]
        p2 = self.particles[constraint['p2']]
        
        if p1.is_fixed and p2.is_fixed:
            return
        
        dx = p2.position.x - p1.position.x
        dy = p2.position.y - p1.position.y
        dz = p2.position.z - p1.position.z
        dist_sq = dx*dx + dy*dy + dz*dz
        
        if dist_sq < 1e-10:
            return
        
        dist = math.sqrt(dist_sq)
        rest_length = constraint['rest_length']
        
        diff = dist - rest_length
        w1 = p1.inverse_mass
        w2 = p2.inverse_mass
        w_sum = w1 + w2
        
        if w_sum < 1e-10:
            return
        
        k = 0.5
        correction = diff * k / w_sum
        
        if dist < 1e-10:
            return
        
        nx = dx / dist
        ny = dy / dist
        nz = dz / dist
        
        if not p1.is_fixed:
            p1.position.x += nx * correction * w1
            p1.position.y += ny * correction * w1
            p1.position.z += nz * correction * w1
        if not p2.is_fixed:
            p2.position.x -= nx * correction * w2
            p2.position.y -= ny * correction * w2
            p2.position.z -= nz * correction * w2
    
    def _solve_coupling_constraint(self, constraint):
        """耦合约束"""
        particle_idx = constraint['particle_idx']
        target_pos = constraint['target_pos']
        
        if particle_idx < 0 or particle_idx >= len(self.particles):
            return
        
        p = self.particles[particle_idx]
        p.position.x = target_pos.x
        p.position.y = target_pos.y
        p.position.z = target_pos.z
    
    def add_coupling_constraint(self, particle_idx: int, target_pos_func):
        """添加耦合约束"""
        constraint = {
            'particle_idx': particle_idx,
            'target_pos': target_pos_func(),
            'target_pos_func': target_pos_func
        }
        self.coupling_constraints.append(constraint)
        return len(self.coupling_constraints) - 1
    
    def update_coupling_constraint(self, constraint_idx: int, target_pos: Vec3):
        """更新耦合约束"""
        if 0 <= constraint_idx < len(self.coupling_constraints):
            self.coupling_constraints[constraint_idx]['target_pos'] = target_pos.copy()
    
    def get_positions(self) -> np.ndarray:
        """获取所有粒子位置作为numpy数组"""
        return np.array([[p.position.x, p.position.y, p.position.z] for p in self.particles])


class Instrument:
    """手术器械"""
    def __init__(self):
        self.position = Vec3()
        self.velocity = Vec3()
        self.mass = 0.5
        self.force = Vec3()
        self.grasp_points = []
        
    def add_grasp_point(self, local_pos: Vec3):
        """添加夹持点"""
        self.grasp_points.append({
            'local_pos': local_pos.copy(),
            'world_pos': Vec3(),
            'grasped_particle': -1,
            'is_grasping': False,
            'coupling_constraint_id': -1
        })
    
    def update_grasp_points(self):
        """更新夹持点世界坐标"""
        for gp in self.grasp_points:
            gp['world_pos'] = Vec3(
                self.position.x + gp['local_pos'].x,
                self.position.y + gp['local_pos'].y,
                self.position.z + gp['local_pos'].z
            )
    
    def integrate(self, dt: float):
        """积分"""
        ax = self.force.x / self.mass
        ay = self.force.y / self.mass
        az = self.force.z / self.mass
        
        self.velocity.x += ax * dt
        self.velocity.y += ay * dt
        self.velocity.z += az * dt
        
        self.velocity.x *= 0.99
        self.velocity.y *= 0.99
        self.velocity.z *= 0.99
        
        self.position.x += self.velocity.x * dt
        self.position.y += self.velocity.y * dt
        self.position.z += self.velocity.z * dt
        
        self.force = Vec3()
        self.update_grasp_points()
    
    def grasp_particle(self, suture: Suture, grasp_point_idx: int) -> bool:
        """夹持粒子"""
        if grasp_point_idx < 0 or grasp_point_idx >= len(self.grasp_points):
            return False
        
        gp = self.grasp_points[grasp_point_idx]
        
        # 查找最近的粒子
        min_dist = float('inf')
        closest_idx = -1
        
        for i, p in enumerate(suture.particles):
            dx = p.position.x - gp['world_pos'].x
            dy = p.position.y - gp['world_pos'].y
            dz = p.position.z - gp['world_pos'].z
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        if closest_idx >= 0 and min_dist < 0.05:
            gp['grasped_particle'] = closest_idx
            gp['is_grasping'] = True
            
            def get_target_pos():
                return gp['world_pos']
            
            constraint_id = suture.add_coupling_constraint(closest_idx, get_target_pos)
            gp['coupling_constraint_id'] = constraint_id
            
            return True
        
        return False
    
    def release_particle(self, suture: Suture, grasp_point_idx: int):
        """释放粒子"""
        if grasp_point_idx < 0 or grasp_point_idx >= len(self.grasp_points):
            return
        
        gp = self.grasp_points[grasp_point_idx]
        
        if gp['is_grasping'] and gp['coupling_constraint_id'] >= 0:
            if gp['grasped_particle'] >= 0:
                suture.particles[gp['grasped_particle']].velocity = self.velocity.copy()
        
        gp['grasped_particle'] = -1
        gp['is_grasping'] = False
        gp['coupling_constraint_id'] = -1
    
    def update_grasped_particles(self, suture: Suture):
        """更新被夹持的粒子"""
        for gp in self.grasp_points:
            if gp['is_grasping'] and gp['coupling_constraint_id'] >= 0:
                suture.update_coupling_constraint(
                    gp['coupling_constraint_id'],
                    gp['world_pos']
                )


# ============ PyQt5可视化界面 ============

if HAS_PYQT5 and HAS_MATPLOTLIB:
    
    class SimulationThread(QThread):
        """仿真线程"""
        update_signal = pyqtSignal(dict)
        
        def __init__(self, suture, instrument):
            super().__init__()
            self.suture = suture
            self.instrument = instrument
            self.running = False
            self.dt = 0.001
            self.step = 0
            
        def run(self):
            self.running = True
            while self.running:
                # 更新器械
                self.instrument.integrate(self.dt)
                self.instrument.update_grasped_particles(self.suture)
                
                # 更新缝合线
                self.suture.simulate(self.dt)
                
                # 发送更新信号
                data = {
                    'step': self.step,
                    'suture_positions': self.suture.get_positions(),
                    'instrument_pos': [
                        self.instrument.position.x,
                        self.instrument.position.y,
                        self.instrument.position.z
                    ]
                }
                self.update_signal.emit(data)
                
                self.step += 1
                self.msleep(1)  # 1ms延迟
        
        def stop(self):
            self.running = False
    
    
    class SurgeryVisualizer(QMainWindow):
        """主窗口"""
        
        def __init__(self):
            super().__init__()
            self.setWindowTitle("虚拟手术缝合仿真系统")
            self.setGeometry(100, 100, 1400, 900)
            
            # 创建仿真对象
            self.suture = Suture()
            self.instrument = Instrument()
            self.setup_simulation()
            
            # 创建UI
            self.init_ui()
            
            # 创建仿真线程
            self.sim_thread = SimulationThread(self.suture, self.instrument)
            self.sim_thread.update_signal.connect(self.update_visualization)
            
        def setup_simulation(self):
            """初始化仿真场景"""
            # 创建缝合线
            positions = [Vec3(0.5, 0.5 + i*0.08, 0.5) for i in range(8)]
            self.suture.create_chain(positions, 0.1)
            
            # 设置器械
            self.instrument.position = Vec3(0.5, 0.5, 0.5)
            self.instrument.add_grasp_point(Vec3(0, 0, 0))
            self.instrument.update_grasp_points()
            
        def init_ui(self):
            """初始化UI"""
            # 中央部件
            central_widget = QWidget()
            self.setCentralWidget(central_widget)
            
            # 主布局
            main_layout = QHBoxLayout(central_widget)
            
            # 左侧：3D可视化
            left_panel = QWidget()
            left_layout = QVBoxLayout(left_panel)
            
            # 3D图形
            self.fig = Figure(figsize=(8, 6), dpi=100)
            self.canvas = FigureCanvas(self.fig)
            self.ax = self.fig.add_subplot(111, projection='3d')
            self.ax.set_xlabel('X')
            self.ax.set_ylabel('Y')
            self.ax.set_zlabel('Z')
            self.ax.set_title('虚拟手术缝合仿真')
            
            left_layout.addWidget(self.canvas)
            
            # 控制按钮
            button_layout = QHBoxLayout()
            
            self.start_btn = QPushButton("开始仿真")
            self.start_btn.clicked.connect(self.start_simulation)
            button_layout.addWidget(self.start_btn)
            
            self.stop_btn = QPushButton("停止仿真")
            self.stop_btn.clicked.connect(self.stop_simulation)
            self.stop_btn.setEnabled(False)
            button_layout.addWidget(self.stop_btn)
            
            self.reset_btn = QPushButton("重置")
            self.reset_btn.clicked.connect(self.reset_simulation)
            button_layout.addWidget(self.reset_btn)
            
            left_layout.addLayout(button_layout)
            
            main_layout.addWidget(left_panel, stretch=2)
            
            # 右侧：控制面板
            right_panel = QWidget()
            right_layout = QVBoxLayout(right_panel)
            
            # 器械控制组
            instrument_group = QGroupBox("器械控制")
            instrument_layout = QGridLayout()
            
            # X位置
            instrument_layout.addWidget(QLabel("X位置:"), 0, 0)
            self.x_slider = QSlider(Qt.Horizontal)
            self.x_slider.setRange(0, 100)
            self.x_slider.setValue(50)
            self.x_slider.valueChanged.connect(self.update_instrument_position)
            instrument_layout.addWidget(self.x_slider, 0, 1)
            self.x_label = QLabel("0.50")
            instrument_layout.addWidget(self.x_label, 0, 2)
            
            # Y位置
            instrument_layout.addWidget(QLabel("Y位置:"), 1, 0)
            self.y_slider = QSlider(Qt.Horizontal)
            self.y_slider.setRange(0, 100)
            self.y_slider.setValue(50)
            self.y_slider.valueChanged.connect(self.update_instrument_position)
            instrument_layout.addWidget(self.y_slider, 1, 1)
            self.y_label = QLabel("0.50")
            instrument_layout.addWidget(self.y_label, 1, 2)
            
            # Z位置
            instrument_layout.addWidget(QLabel("Z位置:"), 2, 0)
            self.z_slider = QSlider(Qt.Horizontal)
            self.z_slider.setRange(0, 100)
            self.z_slider.setValue(50)
            self.z_slider.valueChanged.connect(self.update_instrument_position)
            instrument_layout.addWidget(self.z_slider, 2, 1)
            self.z_label = QLabel("0.50")
            instrument_layout.addWidget(self.z_label, 2, 2)
            
            # 夹持按钮
            self.grasp_btn = QPushButton("夹持缝合线")
            self.grasp_btn.setCheckable(True)
            self.grasp_btn.clicked.connect(self.toggle_grasp)
            instrument_layout.addWidget(self.grasp_btn, 3, 0, 1, 3)
            
            instrument_group.setLayout(instrument_layout)
            right_layout.addWidget(instrument_group)
            
            # 仿真参数组
            param_group = QGroupBox("仿真参数")
            param_layout = QGridLayout()
            
            param_layout.addWidget(QLabel("时间步长:"), 0, 0)
            self.dt_spin = QDoubleSpinBox()
            self.dt_spin.setRange(0.0001, 0.01)
            self.dt_spin.setValue(0.001)
            self.dt_spin.setDecimals(4)
            param_layout.addWidget(self.dt_spin, 0, 1)
            
            param_layout.addWidget(QLabel("重力:"), 1, 0)
            self.gravity_spin = QDoubleSpinBox()
            self.gravity_spin.setRange(-2.0, 0.0)
            self.gravity_spin.setValue(-0.5)
            self.gravity_spin.setDecimals(2)
            self.gravity_spin.valueChanged.connect(self.update_gravity)
            param_layout.addWidget(self.gravity_spin, 1, 1)
            
            param_layout.addWidget(QLabel("阻尼:"), 2, 0)
            self.damping_spin = QDoubleSpinBox()
            self.damping_spin.setRange(0.9, 1.0)
            self.damping_spin.setValue(0.99)
            self.damping_spin.setDecimals(3)
            self.damping_spin.valueChanged.connect(self.update_damping)
            param_layout.addWidget(self.damping_spin, 2, 1)
            
            param_group.setLayout(param_layout)
            right_layout.addWidget(param_group)
            
            # 状态显示组
            status_group = QGroupBox("状态信息")
            status_layout = QVBoxLayout()
            
            self.status_text = QTextEdit()
            self.status_text.setReadOnly(True)
            self.status_text.setMaximumHeight(150)
            status_layout.addWidget(self.status_text)
            
            status_group.setLayout(status_layout)
            right_layout.addWidget(status_group)
            
            # 数据监控组
            monitor_group = QGroupBox("实时监控")
            monitor_layout = QVBoxLayout()
            
            self.monitor_text = QTextEdit()
            self.monitor_text.setReadOnly(True)
            self.monitor_text.setMaximumHeight(200)
            monitor_layout.addWidget(self.monitor_text)
            
            monitor_group.setLayout(monitor_layout)
            right_layout.addWidget(monitor_group)
            
            right_layout.addStretch()
            
            main_layout.addWidget(right_panel, stretch=1)
            
            # 初始化3D显示
            self.update_3d_display()
            
        def update_instrument_position(self):
            """更新器械位置"""
            x = self.x_slider.value() / 100.0
            y = self.y_slider.value() / 100.0
            z = self.z_slider.value() / 100.0
            
            self.x_label.setText(f"{x:.2f}")
            self.y_label.setText(f"{y:.2f}")
            self.z_label.setText(f"{z:.2f}")
            
            self.instrument.position = Vec3(x, y, z)
            self.instrument.update_grasp_points()
            
        def toggle_grasp(self):
            """切换夹持状态"""
            if self.grasp_btn.isChecked():
                success = self.instrument.grasp_particle(self.suture, 0)
                if success:
                    self.grasp_btn.setText("释放缝合线")
                    self.status_text.append("[INFO] 成功夹持缝合线")
                else:
                    self.grasp_btn.setChecked(False)
                    self.status_text.append("[WARN] 夹持失败：距离太远")
            else:
                self.instrument.release_particle(self.suture, 0)
                self.grasp_btn.setText("夹持缝合线")
                self.status_text.append("[INFO] 已释放缝合线")
                
        def update_gravity(self):
            """更新重力"""
            self.suture.gravity = Vec3(0, self.gravity_spin.value(), 0)
            
        def update_damping(self):
            """更新阻尼"""
            self.suture.damping = self.damping_spin.value()
            
        def start_simulation(self):
            """开始仿真"""
            self.sim_thread.dt = self.dt_spin.value()
            self.sim_thread.start()
            
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
            self.status_text.append("[INFO] 仿真开始")
            
        def stop_simulation(self):
            """停止仿真"""
            self.sim_thread.stop()
            self.sim_thread.wait()
            
            self.start_btn.setEnabled(True)
            self.stop_btn.setEnabled(False)
            self.status_text.append("[INFO] 仿真停止")
            
        def reset_simulation(self):
            """重置仿真"""
            self.stop_simulation()
            self.setup_simulation()
            self.update_3d_display()
            self.status_text.append("[INFO] 仿真已重置")
            
        def update_visualization(self, data):
            """更新可视化"""
            self.update_3d_display()
            
            # 更新监控信息
            suture_pos = data['suture_positions']
            inst_pos = data['instrument_pos']
            
            monitor_info = f"""
步骤: {data['step']}
器械位置: ({inst_pos[0]:.3f}, {inst_pos[1]:.3f}, {inst_pos[2]:.3f})
缝合线质心: ({np.mean(suture_pos[:,0]):.3f}, {np.mean(suture_pos[:,1]):.3f}, {np.mean(suture_pos[:,2]):.3f})
缝合线长度: {self.calculate_suture_length():.4f} m
"""
            self.monitor_text.setText(monitor_info)
            
        def calculate_suture_length(self) -> float:
            """计算缝合线长度"""
            length = 0.0
            for i in range(len(self.suture.particles) - 1):
                p1 = self.suture.particles[i].position
                p2 = self.suture.particles[i+1].position
                dx = p2.x - p1.x
                dy = p2.y - p1.y
                dz = p2.z - p1.z
                length += math.sqrt(dx*dx + dy*dy + dz*dz)
            return length
            
        def update_3d_display(self):
            """更新3D显示"""
            self.ax.clear()
            
            # 绘制缝合线
            suture_pos = self.suture.get_positions()
            self.ax.plot(suture_pos[:, 0], suture_pos[:, 1], suture_pos[:, 2], 
                        'y-', linewidth=3, label='Suture')
            self.ax.scatter(suture_pos[:, 0], suture_pos[:, 1], suture_pos[:, 2], 
                          c='yellow', s=50)
            
            # 绘制器械
            inst_pos = self.instrument.position
            self.ax.scatter([inst_pos.x], [inst_pos.y], [inst_pos.z], 
                          c='red', s=200, marker='^', label='Instrument')
            
            # 绘制夹持点
            for gp in self.instrument.grasp_points:
                wp = gp['world_pos']
                color = 'green' if gp['is_grasping'] else 'blue'
                self.ax.scatter([wp.x], [wp.y], [wp.z], 
                              c=color, s=100, marker='o')
            
            # 设置坐标轴
            self.ax.set_xlim(-0.5, 1.5)
            self.ax.set_ylim(-0.5, 1.5)
            self.ax.set_zlim(-0.5, 1.5)
            self.ax.set_xlabel('X (m)')
            self.ax.set_ylabel('Y (m)')
            self.ax.set_zlabel('Z (m)')
            self.ax.set_title('虚拟手术缝合仿真')
            self.ax.legend()
            
            self.canvas.draw()
            
        def closeEvent(self, event):
            """关闭事件"""
            self.stop_simulation()
            event.accept()


def main():
    """主函数"""
    if not HAS_PYQT5:
        print("错误：需要安装PyQt5才能运行可视化界面")
        print("请运行: pip install PyQt5")
        return 1
    
    if not HAS_MATPLOTLIB:
        print("错误：需要安装matplotlib才能运行可视化界面")
        print("请运行: pip install matplotlib")
        return 1
    
    app = QApplication(sys.argv)
    
    # 设置样式
    app.setStyle('Fusion')
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(53, 53, 53))
    palette.setColor(QPalette.WindowText, Qt.white)
    palette.setColor(QPalette.Base, QColor(25, 25, 25))
    palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    palette.setColor(QPalette.ToolTipBase, Qt.white)
    palette.setColor(QPalette.ToolTipText, Qt.white)
    palette.setColor(QPalette.Text, Qt.white)
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, Qt.white)
    palette.setColor(QPalette.BrightText, Qt.red)
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, Qt.black)
    app.setPalette(palette)
    
    window = SurgeryVisualizer()
    window.show()
    
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
