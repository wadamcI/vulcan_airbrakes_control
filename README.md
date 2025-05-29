# Vulcan Rocket Simulator

**Vulcan Rocket Simulator** is a modular simulation environment that combines thrust-phase dynamics from [RocketPy](https://github.com/RocketPy-Team/RocketPy) with custom coast-phase control logic for airbrake deployment. The project is intended to support both automatic and manual control strategies and includes a graphical user interface (GUI) to streamline simulation setup, execution, and result visualization.

---

## 🚀 Project Vision

The simulator models a full suborbital rocket flight profile, divided into two main phases:

1. **Thrust Phase** (powered ascent):
   - Simulated using RocketPy based on solid motor `.eng` files and rocket geometry.
   - Outputs include velocity, altitude, acceleration, quaternions, and position data.

2. **Coast Phase** (post-burnout control):
   - Simulated using custom differential equations based on drag, gravity, and airbrake dynamics.
   - Airbrake control can be:
     - **Manual** (triggered at specific altitudes or times),
     - **Automatic** (via PID control logic).
   - Outputs are computed in discrete time, using either Euler or `solve_ivp` integration.

---

## 🧠 Key Features

- ✅ RocketPy simulation wrapper for thrust phase
- ✅ Coast-phase nonlinear dynamics using physics-based ODEs
- ✅ Airbrake modeling and control logic (manual or PID)
- ✅ GUI for input parameters, simulation control, and visualization
- ✅ Support for exporting:
  - CSVs of full trajectory and state data
  - KML files for map-based flight path display
  - Standard RocketPy and custom plots

---

## 📂 Planned Structure


```
vulcan_simulator/
├── data/                        # Input data: motors, drag curves, deployment profiles
│   ├── motors/
│   ├── drag_curves/
│   └── airbrake_profiles/
├── src/
│   ├── simulation/             # Thrust and coast-phase modeling
│   ├── control/                # PID controller & hardcoded deployment logic
│   ├── visualizations/         # Plotting and KML generation
│   └── gui/                    # GUI frontend
├── results/                    # Output data and plots
├── matlab/                     # Legacy MATLAB models and PID scripts
│   ├── coast_simulation.m
│   ├── pid_controller.m
│   ├── state_space_model.m
│   └── README.md
├── notebooks/                  # Debugging and development notebooks
├── tests/                      # Unit tests
├── run.py                      # Entry point script
├── requirements.txt / pyproject.toml
└── README.md
```
---

## 🧰 MATLAB Integration

During early development, coast-phase dynamics and control logic were implemented and validated using MATLAB. These scripts are preserved in `/matlab` for reference and legacy testing.

### Integration Methods:
- **Short-term**: Exchange data using `.csv` files between RocketPy and MATLAB.
- **Long-term**: Transition MATLAB models fully into Python using `scipy`, `numpy`, and `matplotlib`.

> You can also call MATLAB from Python using the `matlab.engine` API for more seamless integration, if needed.

---

## ✅ Roadmap / To Do

- [ ] Wrap RocketPy simulation and extract relevant state data
- [ ] Port MATLAB coast-phase and control logic to Python
- [ ] Implement hardcoded deployment logic (`deploy after 10s`, `at 700m`, etc.)
- [ ] Create GUI to configure, run, and visualize simulations
- [ ] Export full trajectory to CSV and KML
- [ ] Display RocketPy and coast-phase plots in GUI

---

## 📌 Status

> **📦 In Planning / Prototyping Phase**

Initial modules are being structured. MATLAB-to-Python porting is planned. GUI frontend will follow after integration of coast-phase and RocketPy outputs.

