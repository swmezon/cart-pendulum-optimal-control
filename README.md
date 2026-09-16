# Cart-Pendulum Optimal Control Benchmark

Nonlinear cart-pendulum model and reproducible simulation framework for optimal-control benchmarking. The current implementation provides the plant model, numerical integration, automated validation, and interactive visualization required for subsequent shooting and collocation methods.

## Current implementation

- Nonlinear cart-pendulum dynamics
- Fixed-step RK4 integration
- SciPy `solve_ivp` integration
- Equilibrium and energy-conservation tests
- Forward simulation
- Interactive animation

## System definition

The state vector is

```math
\mathbf{x}=\begin{bmatrix}p\\\theta\\v\\\omega\end{bmatrix}\in\mathbb{R}^{4}\qquad\text{(1)}
```

where \(p\) is cart position, \(\theta\) is pendulum angle, \(v\) is cart velocity, and \(\omega\) is pendulum angular velocity.

The angular convention is

```math
\theta=0\qquad\text{upright equilibrium}\qquad\text{(2)}
```

```math
\theta=\pi\qquad\text{hanging equilibrium}\qquad\text{(3)}
```

The control input is

```math
\mathbf{u}=\begin{bmatrix}F\end{bmatrix}\in\mathbb{R}\qquad\text{(4)}
```

with \(F\) denoting the horizontal cart force.

For cart mass \(M\), pendulum mass \(m\), pendulum length \(L\), and gravitational acceleration \(g\),

```math
D(\theta)=M+m\sin^{2}\theta\qquad\text{(5)}
```

The nonlinear accelerations are

```math
\ddot{p}=\frac{F+m\sin\theta\left(L\omega^{2}-g\cos\theta\right)}{M+m\sin^{2}\theta}\qquad\text{(6)}
```

```math
\ddot{\theta}=\frac{(M+m)g\sin\theta-F\cos\theta-mL\omega^{2}\sin\theta\cos\theta}{L\left(M+m\sin^{2}\theta\right)}\qquad\text{(7)}
```

Therefore,

```math
\dot{\mathbf{x}}=
\begin{bmatrix}
v\\
\omega\\
\ddot{p}\\
\ddot{\theta}
\end{bmatrix}
=
\mathbf{f}(\mathbf{x},\mathbf{u})
\qquad\text{(8)}
```

## Numerical integration

The fixed-step simulator uses fourth-order Runge-Kutta integration:

```math
\mathbf{k}_{1}=\mathbf{f}(t_k,\mathbf{x}_k,\mathbf{u}_k)\qquad\text{(9)}
```

```math
\mathbf{k}_{2}=\mathbf{f}\left(t_k+\frac{h}{2},\mathbf{x}_k+\frac{h}{2}\mathbf{k}_{1},\mathbf{u}_k\right)\qquad\text{(10)}
```

```math
\mathbf{k}_{3}=\mathbf{f}\left(t_k+\frac{h}{2},\mathbf{x}_k+\frac{h}{2}\mathbf{k}_{2},\mathbf{u}_k\right)\qquad\text{(11)}
```

```math
\mathbf{k}_{4}=\mathbf{f}\left(t_k+h,\mathbf{x}_k+h\mathbf{k}_{3},\mathbf{u}_k\right)\qquad\text{(12)}
```

```math
\mathbf{x}_{k+1}=\mathbf{x}_{k}+\frac{h}{6}\left(\mathbf{k}_{1}+2\mathbf{k}_{2}+2\mathbf{k}_{3}+\mathbf{k}_{4}\right)\qquad\text{(13)}
```

## Validation

Automated tests verify the equilibrium configurations and mechanical-energy conservation of the unforced ideal model.

```powershell
pytest -q
```

Expected:

```text
4 passed
```

## Run

Forward simulation:

```powershell
python scripts/run_simulation.py
```

Interactive animation:

```powershell
python scripts/run_animation.py
```

## Repository structure

```text
cart-pendulum-optimal-control/
├── README.md
├── requirements.txt
├── src/
│   ├── plant.py
│   ├── simulator.py
│   └── animation.py
├── scripts/
│   ├── run_simulation.py
│   └── run_animation.py
├── tests/
│   └── test_plant.py
└── outputs/
```

## Planned benchmarks

- LQR upright stabilization
- iLQR trajectory optimization
- Direct single shooting
- Direct multiple shooting
- Direct collocation
- Solver-performance comparison
