# Virtual Surgery Simulation

A real-time virtual surgery simulation system with soft tissue and suture dynamics, featuring haptic feedback and 3D visualization.

## 🎯 Features

- **Real-time physics simulation** using XPBD (Extended Position-Based Dynamics)
- **3D visualization** with Three.js
- **Haptic feedback** simulation
- **Multi-language support** (English/Chinese)
- **Interactive controls** for parameter tuning
- **Real-time data visualization** (force, energy, velocity charts)
- **Cross-platform** (C++ core, web-based frontend)

## 🛠️ Technology Stack

### Core Engine
- **C++17** for physics simulation
- **Eigen3** for linear algebra
- **CMake** for build system

### Visualization
- **Three.js** for 3D rendering
- **HTML5/CSS3/JavaScript** for UI
- **Python** for alternative visualization

### Dependencies
- **Eigen3** (linear algebra library)
- **Three.js** (3D rendering)
- **PyQt5** (Python visualization)
- **Matplotlib** (data visualization)

## 📁 Project Structure

```
├── include/             # C++ header files
│   ├── collision/       # Collision detection
│   ├── core/            # Core utilities
│   ├── haptic/          # Haptic feedback
│   ├── io/              # Input/output
│   ├── physics/         # Physics solvers
│   └── rendering/       # Rendering interface
├── src/                 # C++ source files
├── visualization/       # Visualization files
│   ├── surgery_simulation.html  # Web-based 3D interface
│   └── surgery_visualizer.py    # PyQt5 interface
├── config/              # Configuration files
├── build/               # Build output
├── design.md            # Design documentation
└── README.md            # This file
```

## 🚀 Getting Started

### Prerequisites

- **C++17 compiler** (MinGW-w64, MSVC, or GCC)
- **CMake** (version 3.18+)
- **Python 3.8+** (for visualization)
- **Eigen3 library** (included in third_party)

### Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/Chenjianyu-123/virtual-surgery-simulation.git
   cd virtual-surgery-simulation
   ```

2. **Install Python dependencies**
   ```bash
   pip install PyQt5 matplotlib numpy
   ```

3. **Build the C++ core**
   ```bash
   mkdir build && cd build
   cmake .. -G "MinGW Makefiles"
   mingw32-make -j4
   ```

### Running the Simulation

#### Web-based 3D Visualization
```bash
cd visualization
python -m http.server 8080
```
Then open http://localhost:8080/surgery_simulation.html in your browser.

#### C++ Console Application
```bash
cd build
./surgery_sim.exe --test
```

#### Python Visualization
```bash
cd visualization
python surgery_visualizer.py
```

## 🎮 Usage

### Web Interface Controls
- **Mouse**: Drag to rotate view, scroll to zoom
- **WASD**: Move camera
- **Parameters**: Adjust physics parameters using sliders
- **Language**: Toggle between English and Chinese
- **Start/Stop**: Control simulation
- **Reset**: Reset simulation to initial state

### Command Line Options
```bash
./surgery_sim.exe [options]

Options:
  --test          Run test mode
  --headless      Run without rendering
  --steps N       Set maximum simulation steps
  --fps N         Set target FPS
  --dt VALUE      Set time step in seconds
  --help          Show this help message
```

## 📊 Key Components

### Physics Engine
- **XPBDSolver**: Extended position-based dynamics for sutures
- **FEMSolver**: Finite element method for soft tissue
- **RigidBodySolver**: Rigid body dynamics for instruments
- **ConstraintProjector**: Multi-body constraint handling

### Collision Detection
- **AABBTree**: Broad-phase collision detection
- **GJKAlgorithm**: Narrow-phase collision detection

### Haptic Feedback
- **AdaptiveFilter**: Event-based signal filtering
- **ForceMapper**: Non-linear force mapping
- **DelayCompensator**: AR model for delay compensation

### Visualization
- **Three.js**: Real-time 3D rendering
- **Data charts**: Force, energy, and velocity visualization
- **Multi-language support**: English and Chinese interfaces

## 🔧 Configuration

Configuration files are located in the `config/` directory:
- **default_params.json**: Default simulation parameters
- **haptic_settings.json**: Haptic feedback settings
- **tissue_materials.json**: Tissue material properties

## 📈 Performance

- **Simulation**: 60+ FPS on modern hardware
- **Force feedback delay**: < 5ms
- **Supported scale**: Soft tissue elements > 10k, suture particles > 100

## 🤝 Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🎓 Acknowledgments

- Based on the design document for virtual surgery simulation
- Uses Eigen3 for linear algebra
- Three.js for 3D visualization
- PyQt5 for Python-based visualization

## 📞 Contact

- Project Link: [https://github.com/Chenjianyu-123/virtual-surgery-simulation](https://github.com/Chenjianyu-123/virtual-surgery-simulation)

---

**Built with ❤️ for surgical simulation research**