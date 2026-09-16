"""Launch the nonlinear cart-pendulum animation without Matplotlib."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]

if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))


from src.animation import animate_cart_pendulum
from src.plant import CartPendulumParams
from src.simulator import simulate_rk4


FORCE_LIMIT_N = 10.0


def bounded_excitation(
    t: float,
    _x: np.ndarray,
) -> float:
    """Small open-loop sinusoidal excitation for plant validation."""

    force = 0.5 * np.sin(
        2.0 * np.pi * 0.5 * t
    )

    return float(
        np.clip(
            force,
            -FORCE_LIMIT_N,
            FORCE_LIMIT_N,
        )
    )


def main() -> None:
    """Simulate the nonlinear plant and launch the animation."""

    params = CartPendulumParams(
        cart_mass=1.0,
        pendulum_mass=0.2,
        pendulum_length=0.5,
        gravity=9.81,
    )

    # theta = 0    -> upright
    # theta = pi   -> hanging downward
    #
    # Start 12 degrees away from the downward equilibrium.

    x0 = np.array(
        [
            0.0,
            np.pi + np.deg2rad(12.0),
            0.0,
            0.0,
        ],
        dtype=float,
    )

    print("Running nonlinear cart-pendulum simulation...")

    result = simulate_rk4(
        x0,
        params,
        bounded_excitation,
        t0=0.0,
        tf=8.0,
        dt=0.002,
    )

    print("Simulation complete.")
    print(f"Samples: {result.time.size}")
    print(f"Initial state: {result.state[0]}")
    print(f"Final state:   {result.state[-1]}")
    print()
    print("Opening cart-pendulum animation...")

    animate_cart_pendulum(
        result,
        params,
        playback_speed=1.0,
    )


if __name__ == "__main__":
    main()
