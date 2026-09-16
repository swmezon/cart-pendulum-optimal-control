"""Run the nonlinear cart-pendulum forward simulation."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


REPOSITORY_ROOT = Path(
    __file__
).resolve().parents[1]

if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(
        0,
        str(REPOSITORY_ROOT),
    )


from src.animation import animate_cart_pendulum
from src.plant import (
    CartPendulumParams,
    total_energy,
)
from src.simulator import (
    SimulationResult,
    simulate_rk4,
)


OUTPUT_DIRECTORY = (
    REPOSITORY_ROOT
    / "outputs"
)

FORCE_LIMIT_N = 10.0


def bounded_excitation(
    t: float,
    _x: np.ndarray,
) -> float:
    """Small deterministic force used for plant validation."""

    raw_force = (
        0.5
        * np.sin(
            2.0
            * np.pi
            * 0.5
            * t
        )
    )

    return float(
        np.clip(
            raw_force,
            -FORCE_LIMIT_N,
            FORCE_LIMIT_N,
        )
    )


def save_state_figure(
    result: SimulationResult,
) -> Path:
    """Save the four simulated state trajectories."""

    OUTPUT_DIRECTORY.mkdir(
        parents=True,
        exist_ok=True,
    )

    figure_path = (
        OUTPUT_DIRECTORY
        / "state_trajectories.png"
    )

    labels = (
        "Cart position p [m]",
        "Pendulum angle theta [rad]",
        "Cart velocity v [m/s]",
        "Angular velocity omega [rad/s]",
    )

    fig, axes = plt.subplots(
        4,
        1,
        figsize=(10, 10),
        sharex=True,
    )

    for index, axis in enumerate(axes):

        axis.plot(
            result.time,
            result.state[:, index],
            linewidth=1.6,
        )

        axis.set_ylabel(
            labels[index]
        )

        axis.grid(
            True,
            alpha=0.3,
        )

    axes[-1].set_xlabel(
        "Time [s]"
    )

    fig.suptitle(
        "Nonlinear Cart-Pendulum State Trajectories"
    )

    fig.tight_layout()

    fig.savefig(
        figure_path,
        dpi=180,
        bbox_inches="tight",
    )

    plt.close(fig)

    return figure_path


def save_control_figure(
    result: SimulationResult,
) -> Path:
    """Save the open-loop excitation trajectory."""

    OUTPUT_DIRECTORY.mkdir(
        parents=True,
        exist_ok=True,
    )

    figure_path = (
        OUTPUT_DIRECTORY
        / "control_effort.png"
    )

    fig, axis = plt.subplots(
        figsize=(10, 4.5)
    )

    axis.plot(
        result.time,
        result.control,
        linewidth=1.8,
    )

    axis.axhline(
        FORCE_LIMIT_N,
        linestyle="--",
        linewidth=1.0,
    )

    axis.axhline(
        -FORCE_LIMIT_N,
        linestyle="--",
        linewidth=1.0,
    )

    axis.set_xlabel(
        "Time [s]"
    )

    axis.set_ylabel(
        "Force F [N]"
    )

    axis.set_title(
        "Open-Loop Plant-Validation Excitation"
    )

    axis.grid(
        True,
        alpha=0.3,
    )

    fig.tight_layout()

    fig.savefig(
        figure_path,
        dpi=180,
        bbox_inches="tight",
    )

    plt.close(fig)

    return figure_path


def open_saved_plots(
    state_figure: Path,
    control_figure: Path,
) -> None:
    """Open saved PNG files with the default Windows image viewer."""

    if os.name == "nt":
        os.startfile(state_figure)
        os.startfile(control_figure)


def main(
    animate: bool = True,
    playback_speed: float = 1.0,
    open_plots: bool = False,
) -> None:
    """Execute the baseline nonlinear simulation."""

    params = CartPendulumParams(
        cart_mass=1.0,
        pendulum_mass=0.2,
        pendulum_length=0.5,
        gravity=9.81,
    )

    # theta = 0   -> upright
    # theta = pi  -> hanging downward
    #
    # Begin 12 degrees away from the stable downward equilibrium.

    x0 = np.array(
        [
            0.0,
            np.pi + np.deg2rad(12.0),
            0.0,
            0.0,
        ],
        dtype=float,
    )

    result = simulate_rk4(
        x0,
        params,
        bounded_excitation,
        t0=0.0,
        tf=8.0,
        dt=0.002,
    )

    state_figure = save_state_figure(
        result
    )

    control_figure = save_control_figure(
        result
    )

    initial_energy = total_energy(
        result.state[0],
        params,
    )

    final_energy = total_energy(
        result.state[-1],
        params,
    )

    print()
    print("Simulation complete.")
    print("--------------------")

    print(
        f"Samples: {result.time.size}"
    )

    print(
        f"Initial state: {result.state[0]}"
    )

    print(
        f"Final state:   {result.state[-1]}"
    )

    print(
        f"Initial mechanical energy: "
        f"{initial_energy:.6f} J"
    )

    print(
        f"Final mechanical energy:   "
        f"{final_energy:.6f} J"
    )

    print()
    print(
        "Energy is not conserved during this simulation "
        "because the external force performs work."
    )

    print()
    print(
        f"Saved: {state_figure}"
    )

    print(
        f"Saved: {control_figure}"
    )

    if open_plots:
        open_saved_plots(
            state_figure,
            control_figure,
        )

    if animate:
        print()
        print(
            "Opening cart-pendulum animation..."
        )

        animate_cart_pendulum(
            result,
            params,
            playback_speed=playback_speed,
        )


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description=__doc__
    )

    parser.add_argument(
        "--no-animation",
        action="store_true",
        help=(
            "Run the simulation without opening "
            "the animation window."
        ),
    )

    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help=(
            "Animation playback speed. "
            "Example: --speed 2"
        ),
    )

    parser.add_argument(
        "--open-plots",
        action="store_true",
        help=(
            "Open the saved PNG plots using "
            "the default Windows image viewer."
        ),
    )

    arguments = parser.parse_args()

    main(
        animate=not arguments.no_animation,
        playback_speed=arguments.speed,
        open_plots=arguments.open_plots,
    )
