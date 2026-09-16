"""Core models and simulation utilities for the optimal-control benchmark."""

from .plant import CartPendulumParams, cart_pendulum_dynamics, total_energy
from .simulator import SimulationResult, simulate_rk4, simulate_solve_ivp

__all__ = [
    "CartPendulumParams",
    "SimulationResult",
    "cart_pendulum_dynamics",
    "total_energy",
    "simulate_rk4",
    "simulate_solve_ivp",
]
