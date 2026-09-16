# Cart-Pendulum Optimal Control Benchmark

Nonlinear cart-pendulum model and reproducible simulation framework for optimal-control benchmarks. The current implementation establishes the plant, numerical integration, physics-based tests, and visualization layer used by subsequent shooting and collocation methods.

## Current implementation

- Nonlinear cart-pendulum dynamics
- Fixed-step RK4 integration
- SciPy `solve_ivp` integration
- Equilibrium and energy-conservation tests
- Forward simulation
- Interactive cart-pendulum animation

## System definition

The state is

```math
\mathbf{x}
=
\begin{bmatrix}
p & \theta & v & \omega
\end{bmatrix}^{\mathsf T}
\in \mathbb{R}^{4}.
\tag{1}
```

where \(p\) is cart position, \(\theta\) is pendulum angle, \(v\) is cart velocity, and \(\omega\) is pendulum angular velocity.

The coordinate convention is

```math
\theta = 0
\qquad
\text{upright equilibrium}.
\tag{2}
```

```math
\theta = \pi
\qquad
\text{hanging equilibrium}.
\tag{3}
```

The horizontal cart force is

```math
\mathbf{u}
=
\begin{bmatrix}
F
\end{bmatrix}
\in \mathbb{R}.
\tag{4}
```

For cart mass \(M\), pendulum mass \(m\), pendulum length \(L\), and gravity \(g\), define

```math
D(\theta)
=
M + m\sin^{2}\theta.
\tag{5}
```

The nonlinear cart acceleration is

```math
\ddot{p}
=
\frac{
F
+
m\sin\theta
\left(
L\omega^{2}
-
g\cos\theta
\right)
}{
M+m\sin^{2}\theta
}.
\tag{6}
```

The pendulum angular acceleration is

```math
\ddot{\theta}
=
\frac{
(M+m)g\sin\theta
-
F\cos\theta
-
mL\omega^{2}\sin\theta\cos\theta
}{
L\left(M+m\sin^{2}\theta\right)
}.
\tag{7}
```

The continuous-time model is

```math
\dot{\mathbf{x}}
=
\begin{bmatrix}
v \\
\omega \\
\ddot{p} \\
\ddot{\theta}
\end{bmatrix}
=
\mathbf{f}(\mathbf{x},\mathbf{u}).
\tag{8}
```

## Numerical integration

The benchmark includes classical fourth-order Runge-Kutta integration.

```math
\mathbf{k}_{1}
=
\mathbf{f}
\left(
t_k,\mathbf{x}_k,\mathbf{u}_k
\right).
\tag{9}
```

```math
\mathbf{k}_{2}
=
\mathbf{f}
\left(
t_k+\frac{h}{2},
\mathbf{x}_k+\frac{h}{2}\mathbf{k}_{1},
\mathbf{u}_k
\right).
\tag{10}
```

```math
\mathbf{k}_{3}
=
\mathbf{f}
\left(
t_k+\frac{h}{2},
\mathbf{x}_k+\frac{h}{2}\mathbf{k}_{2},
\mathbf{u}_k
\right).
\tag{11}
```

```math
\mathbf{k}_{4}
=
\mathbf{f}
\left(
t_k+h,
\mathbf{x}_k+h\mathbf{k}_{3},
\mathbf{u}_k
\right).
\tag{12}
```

```math
\mathbf{x}_{k+1}
=
\mathbf{x}_{k}
+
\frac{h}{6}
\left(
\mathbf{k}_{1}
+
2\mathbf{k}_{2}
+
2\mathbf{k}_{3}
+
\mathbf{k}_{4}
\right).
\tag{13}
```

## Validation

Automated tests verify both equilibrium configurations and near-conservation of mechanical energy for the unforced ideal model.

```powershell
pytest -q
```

Expected result:

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

## Roadmap

- LQR upright stabilization
- iLQR trajectory optimization
- Direct single shooting
- Direct multiple shooting
- Direct collocation
- Solver-performance comparison
