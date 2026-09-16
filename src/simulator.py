"""Deterministic forward simulation utilities for the cart-pendulum plant."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.integrate import solve_ivp

from .plant import CartPendulumParams, cart_pendulum_dynamics

FloatArray = NDArray[np.float64]
ControlLaw = Callable[[float, FloatArray], float]


@dataclass(frozen=True)
class SimulationResult:
    """Container for a sampled state and control trajectory."""

    time: FloatArray
    state: FloatArray
    control: FloatArray

    def __post_init__(self) -> None:
        if self.time.ndim != 1:
            raise ValueError("time must be one-dimensional.")
        if self.state.shape != (self.time.size, 4):
            raise ValueError("state must have shape (N, 4).")
        if self.control.shape != (self.time.size,):
            raise ValueError("control must have shape (N,).")


def _validate_time_grid(t0: float, tf: float, dt: float) -> FloatArray:
    if tf <= t0:
        raise ValueError("tf must be greater than t0.")
    if dt <= 0.0:
        raise ValueError("dt must be positive.")

    n_steps = int(np.ceil((tf - t0) / dt))
    return np.linspace(t0, tf, n_steps + 1, dtype=float)


def simulate_rk4(
    x0: ArrayLike,
    params: CartPendulumParams,
    control_law: ControlLaw,
    *,
    t0: float = 0.0,
    tf: float = 8.0,
    dt: float = 0.002,
) -> SimulationResult:
    """Simulate the nonlinear plant with fixed-step classical RK4.

    The control is treated as zero-order-held over each RK4 integration step.
    That convention is common in sampled-data control simulations and keeps
    the result deterministic and easy to reproduce.
    """

    time = _validate_time_grid(t0, tf, dt)
    state = np.empty((time.size, 4), dtype=float)
    control = np.empty(time.size, dtype=float)

    initial_state = np.asarray(x0, dtype=float).reshape(-1)
    if initial_state.size != 4:
        raise ValueError("x0 must contain exactly four state values.")
    state[0] = initial_state

    for k in range(time.size - 1):
        tk = time[k]
        xk = state[k]
        h = time[k + 1] - time[k]
        uk = float(control_law(tk, xk.copy()))
        control[k] = uk

        def f(t_eval: float, x_eval: FloatArray) -> FloatArray:
            return cart_pendulum_dynamics(t_eval, x_eval, uk, params)

        k1 = f(tk, xk)
        k2 = f(tk + 0.5 * h, xk + 0.5 * h * k1)
        k3 = f(tk + 0.5 * h, xk + 0.5 * h * k2)
        k4 = f(tk + h, xk + h * k3)

        state[k + 1] = xk + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    control[-1] = float(control_law(time[-1], state[-1].copy()))
    return SimulationResult(time=time, state=state, control=control)


def simulate_solve_ivp(
    x0: ArrayLike,
    params: CartPendulumParams,
    control_law: ControlLaw,
    *,
    t0: float = 0.0,
    tf: float = 8.0,
    dt: float = 0.002,
    rtol: float = 1e-9,
    atol: float = 1e-11,
) -> SimulationResult:
    """Simulate the nonlinear plant with SciPy's adaptive RK45 solver."""

    time = _validate_time_grid(t0, tf, dt)
    initial_state = np.asarray(x0, dtype=float).reshape(-1)
    if initial_state.size != 4:
        raise ValueError("x0 must contain exactly four state values.")

    def closed_loop_rhs(t_eval: float, x_eval: FloatArray) -> FloatArray:
        u_eval = float(control_law(t_eval, x_eval.copy()))
        return cart_pendulum_dynamics(t_eval, x_eval, u_eval, params)

    solution = solve_ivp(
        closed_loop_rhs,
        (t0, tf),
        initial_state,
        method="RK45",
        t_eval=time,
        rtol=rtol,
        atol=atol,
    )

    if not solution.success:
        raise RuntimeError(f"solve_ivp failed: {solution.message}")

    state = solution.y.T.astype(float, copy=False)
    control = np.array(
        [control_law(t, x.copy()) for t, x in zip(time, state)],
        dtype=float,
    )
    return SimulationResult(time=time, state=state, control=control)
