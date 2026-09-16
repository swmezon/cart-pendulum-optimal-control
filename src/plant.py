"""Nonlinear cart-pendulum plant model.

Angle convention
----------------
theta = 0 rad  -> pendulum upright
theta = pi rad -> pendulum hanging downward

State order
-----------
x = [p, theta, v, omega]

where p is cart position, v is cart velocity, theta is pendulum angle,
and omega is pendulum angular velocity.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Union

import numpy as np
from numpy.typing import ArrayLike, NDArray

FloatArray = NDArray[np.float64]
ScalarLike = Union[float, int, np.floating]


@dataclass(frozen=True)
class CartPendulumParams:
    """Physical parameters for a point-mass cart-pendulum model.

    Attributes
    ----------
    cart_mass : float
        Cart mass M [kg].
    pendulum_mass : float
        Pendulum point mass m [kg].
    pendulum_length : float
        Distance L from pivot to pendulum point mass [m].
    gravity : float
        Gravitational acceleration g [m/s^2].
    """

    cart_mass: float = 1.0
    pendulum_mass: float = 0.2
    pendulum_length: float = 0.5
    gravity: float = 9.81

    def __post_init__(self) -> None:
        if self.cart_mass <= 0.0:
            raise ValueError("cart_mass must be positive.")
        if self.pendulum_mass <= 0.0:
            raise ValueError("pendulum_mass must be positive.")
        if self.pendulum_length <= 0.0:
            raise ValueError("pendulum_length must be positive.")
        if self.gravity <= 0.0:
            raise ValueError("gravity must be positive.")


def _as_scalar_force(u: ArrayLike | ScalarLike) -> float:
    """Convert a scalar-like control input into one floating-point force."""

    force = np.asarray(u, dtype=float).reshape(-1)
    if force.size != 1:
        raise ValueError("Control input u must contain exactly one force value.")
    return float(force[0])


def cart_pendulum_dynamics(
    t: float,
    x: ArrayLike,
    u: ArrayLike | ScalarLike,
    params: CartPendulumParams,
) -> FloatArray:
    """Evaluate the exact nonlinear cart-pendulum state derivative.

    Parameters
    ----------
    t : float
        Time [s]. Included for compatibility with numerical integrators.
    x : array-like, shape (4,)
        State [p, theta, v, omega].
    u : scalar or array-like with one element
        Horizontal cart force F [N]. Positive force acts in +p direction.
    params : CartPendulumParams
        Physical model parameters.

    Returns
    -------
    numpy.ndarray, shape (4,)
        State derivative [v, omega, p_ddot, theta_ddot].
    """

    del t  # Autonomous plant: dynamics do not explicitly depend on time.

    state = np.asarray(x, dtype=float).reshape(-1)
    if state.size != 4:
        raise ValueError("State x must contain exactly four values.")

    _, theta, velocity, omega = state
    force = _as_scalar_force(u)

    M = params.cart_mass
    m = params.pendulum_mass
    L = params.pendulum_length
    g = params.gravity

    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)
    denominator = M + m * sin_theta**2

    acceleration = (
        force
        + m * sin_theta * (L * omega**2 - g * cos_theta)
    ) / denominator

    angular_acceleration = (
        (M + m) * g * sin_theta
        - force * cos_theta
        - m * L * omega**2 * sin_theta * cos_theta
    ) / (L * denominator)

    return np.array(
        [velocity, omega, acceleration, angular_acceleration],
        dtype=float,
    )


def total_energy(x: ArrayLike, params: CartPendulumParams) -> float:
    """Return total mechanical energy of the unforced ideal plant [J].

    The zero of potential energy is arbitrary. With the chosen angle
    convention, theta = 0 is upright and therefore has maximum potential
    energy +m*g*L.
    """

    state = np.asarray(x, dtype=float).reshape(-1)
    if state.size != 4:
        raise ValueError("State x must contain exactly four values.")

    _, theta, velocity, omega = state

    M = params.cart_mass
    m = params.pendulum_mass
    L = params.pendulum_length
    g = params.gravity

    kinetic = (
        0.5 * (M + m) * velocity**2
        + m * L * velocity * omega * np.cos(theta)
        + 0.5 * m * L**2 * omega**2
    )
    potential = m * g * L * np.cos(theta)
    return float(kinetic + potential)
