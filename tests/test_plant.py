"""Physics-focused tests for the nonlinear cart-pendulum model."""

import numpy as np
import pytest

from src.plant import CartPendulumParams, cart_pendulum_dynamics, total_energy
from src.simulator import simulate_rk4


def test_upright_equilibrium_has_zero_derivative() -> None:
    """The exact upright state is an equilibrium when force is zero."""

    params = CartPendulumParams()
    x = np.array([0.25, 0.0, 0.0, 0.0])

    x_dot = cart_pendulum_dynamics(0.0, x, 0.0, params)

    np.testing.assert_allclose(x_dot, np.zeros(4), atol=1e-12)


def test_hanging_equilibrium_has_zero_derivative() -> None:
    """The exact downward state is also an equilibrium when force is zero."""

    params = CartPendulumParams()
    x = np.array([-0.40, np.pi, 0.0, 0.0])

    x_dot = cart_pendulum_dynamics(0.0, x, 0.0, params)

    np.testing.assert_allclose(x_dot, np.zeros(4), atol=1e-12)


def test_unforced_rk4_nearly_preserves_mechanical_energy() -> None:
    """An ideal unforced plant should conserve total mechanical energy."""

    params = CartPendulumParams(
        cart_mass=1.0,
        pendulum_mass=0.2,
        pendulum_length=0.5,
        gravity=9.81,
    )
    x0 = np.array([0.0, 0.8, 0.3, -0.4])

    result = simulate_rk4(
        x0,
        params,
        control_law=lambda _t, _x: 0.0,
        tf=2.0,
        dt=5e-4,
    )

    energy = np.array([total_energy(x, params) for x in result.state])
    max_energy_error = np.max(np.abs(energy - energy[0]))

    assert max_energy_error < 1e-8


def test_invalid_physical_parameters_are_rejected() -> None:
    """Nonphysical masses and lengths should fail immediately."""

    with pytest.raises(ValueError):
        CartPendulumParams(cart_mass=0.0)

    with pytest.raises(ValueError):
        CartPendulumParams(pendulum_length=-1.0)
