"""Lightweight real-time cart-pendulum animation using Tkinter."""

from __future__ import annotations

import tkinter as tk
import numpy as np

from .plant import CartPendulumParams
from .simulator import SimulationResult


def animate_cart_pendulum(
    result: SimulationResult,
    params: CartPendulumParams,
    playback_speed: float = 1.0,
) -> None:
    """Display the simulated cart-pendulum trajectory in a Tkinter window."""

    if playback_speed <= 0.0:
        raise ValueError("playback_speed must be positive.")

    root = tk.Tk()
    root.title("Cart-Pendulum Simulation")
    root.geometry("1000x650")
    root.configure(bg="white")

    canvas = tk.Canvas(
        root,
        width=1000,
        height=520,
        bg="white",
        highlightthickness=0,
    )
    canvas.pack(fill="both", expand=True)

    control_frame = tk.Frame(root, bg="white")
    control_frame.pack(fill="x", pady=8)

    paused = {"value": False}
    frame_index = {"value": 0}

    # ------------------------------------------------------------
    # Animation geometry
    # ------------------------------------------------------------

    width = 1000
    height = 520

    track_y = 320
    track_left = 80
    track_right = 920

    cart_width = 110
    cart_height = 55

    pendulum_pixels = 150

    p_values = result.state[:, 0]

    p_min = float(np.min(p_values))
    p_max = float(np.max(p_values))

    if abs(p_max - p_min) < 1e-12:
        p_min -= 1.0
        p_max += 1.0

    margin = 0.15 * max(
        abs(p_min),
        abs(p_max),
        1.0,
    )

    p_min -= margin
    p_max += margin

    def position_to_pixel(p: float) -> float:
        ratio = (p - p_min) / (p_max - p_min)

        return (
            track_left
            + ratio * (track_right - track_left)
        )

    # ------------------------------------------------------------
    # Static graphics
    # ------------------------------------------------------------

    canvas.create_text(
        width / 2,
        35,
        text="Nonlinear Cart-Pendulum Forward Simulation",
        font=("Segoe UI", 20, "bold"),
    )

    canvas.create_text(
        width / 2,
        70,
        text="Open-loop plant validation",
        font=("Segoe UI", 11),
    )

    canvas.create_line(
        track_left,
        track_y + cart_height / 2 + 18,
        track_right,
        track_y + cart_height / 2 + 18,
        width=4,
    )

    canvas.create_text(
        track_left,
        track_y + 65,
        text=f"{p_min:.2f} m",
        font=("Segoe UI", 10),
    )

    canvas.create_text(
        track_right,
        track_y + 65,
        text=f"{p_max:.2f} m",
        font=("Segoe UI", 10),
    )

    # ------------------------------------------------------------
    # Dynamic graphics
    # ------------------------------------------------------------

    cart = canvas.create_rectangle(
        0,
        0,
        0,
        0,
        width=3,
    )

    left_wheel = canvas.create_oval(
        0,
        0,
        0,
        0,
        width=3,
    )

    right_wheel = canvas.create_oval(
        0,
        0,
        0,
        0,
        width=3,
    )

    pendulum_rod = canvas.create_line(
        0,
        0,
        0,
        0,
        width=6,
    )

    pendulum_mass = canvas.create_oval(
        0,
        0,
        0,
        0,
        width=3,
    )

    pivot = canvas.create_oval(
        0,
        0,
        0,
        0,
        width=2,
        fill="black",
    )

    force_arrow = canvas.create_line(
        0,
        0,
        0,
        0,
        width=4,
        arrow=tk.LAST,
    )

    info_text = canvas.create_text(
        50,
        115,
        anchor="nw",
        text="",
        font=("Consolas", 12),
    )

    angle_text = canvas.create_text(
        700,
        115,
        anchor="nw",
        text="",
        font=("Consolas", 12),
    )

    paused_text = canvas.create_text(
        width / 2,
        470,
        text="",
        font=("Segoe UI", 14, "bold"),
    )

    # ------------------------------------------------------------
    # Playback controls
    # ------------------------------------------------------------

    def toggle_pause() -> None:
        paused["value"] = not paused["value"]

        pause_button.configure(
            text=(
                "Resume"
                if paused["value"]
                else "Pause"
            )
        )

        canvas.itemconfigure(
            paused_text,
            text=(
                "PAUSED"
                if paused["value"]
                else ""
            ),
        )

    def restart() -> None:
        frame_index["value"] = 0
        paused["value"] = False
        pause_button.configure(text="Pause")

        canvas.itemconfigure(
            paused_text,
            text="",
        )

    pause_button = tk.Button(
        control_frame,
        text="Pause",
        width=12,
        command=toggle_pause,
    )
    pause_button.pack(side="left", padx=10)

    restart_button = tk.Button(
        control_frame,
        text="Restart",
        width=12,
        command=restart,
    )
    restart_button.pack(side="left", padx=10)

    close_button = tk.Button(
        control_frame,
        text="Close",
        width=12,
        command=root.destroy,
    )
    close_button.pack(side="right", padx=10)

    # ------------------------------------------------------------
    # Animation loop
    # ------------------------------------------------------------

    def update_frame() -> None:
        if not root.winfo_exists():
            return

        if not paused["value"]:

            k = frame_index["value"]

            if k >= result.time.size:
                k = 0
                frame_index["value"] = 0

            t = result.time[k]

            p = result.state[k, 0]
            theta = result.state[k, 1]
            velocity = result.state[k, 2]
            omega = result.state[k, 3]

            force = result.control[k]

            cart_x = position_to_pixel(p)

            cart_top = (
                track_y
                - cart_height / 2
            )

            cart_bottom = (
                track_y
                + cart_height / 2
            )

            canvas.coords(
                cart,
                cart_x - cart_width / 2,
                cart_top,
                cart_x + cart_width / 2,
                cart_bottom,
            )

            wheel_radius = 14

            wheel_y = (
                cart_bottom
                + wheel_radius
            )

            canvas.coords(
                left_wheel,
                cart_x
                - cart_width * 0.30
                - wheel_radius,
                wheel_y - wheel_radius,
                cart_x
                - cart_width * 0.30
                + wheel_radius,
                wheel_y + wheel_radius,
            )

            canvas.coords(
                right_wheel,
                cart_x
                + cart_width * 0.30
                - wheel_radius,
                wheel_y - wheel_radius,
                cart_x
                + cart_width * 0.30
                + wheel_radius,
                wheel_y + wheel_radius,
            )

            pivot_x = cart_x
            pivot_y = cart_top

            # theta = 0 points upward.
            bob_x = (
                pivot_x
                + pendulum_pixels
                * np.sin(theta)
            )

            bob_y = (
                pivot_y
                - pendulum_pixels
                * np.cos(theta)
            )

            canvas.coords(
                pendulum_rod,
                pivot_x,
                pivot_y,
                bob_x,
                bob_y,
            )

            bob_radius = 18

            canvas.coords(
                pendulum_mass,
                bob_x - bob_radius,
                bob_y - bob_radius,
                bob_x + bob_radius,
                bob_y + bob_radius,
            )

            pivot_radius = 7

            canvas.coords(
                pivot,
                pivot_x - pivot_radius,
                pivot_y - pivot_radius,
                pivot_x + pivot_radius,
                pivot_y + pivot_radius,
            )

            arrow_scale = 65.0

            arrow_length = (
                arrow_scale
                * force
                / 10.0
            )

            arrow_y = (
                cart_top
                - 35
            )

            canvas.coords(
                force_arrow,
                cart_x,
                arrow_y,
                cart_x + arrow_length,
                arrow_y,
            )

            canvas.itemconfigure(
                info_text,
                text=(
                    f"Time      : {t:6.2f} s\n"
                    f"Position  : {p:6.3f} m\n"
                    f"Velocity  : {velocity:6.3f} m/s\n"
                    f"Force     : {force:6.3f} N"
                ),
            )

            theta_deg = np.rad2deg(theta)

            downward_error_deg = np.rad2deg(
                theta - np.pi
            )

            canvas.itemconfigure(
                angle_text,
                text=(
                    f"Angle θ       : {theta_deg:7.2f} deg\n"
                    f"From downward : {downward_error_deg:7.2f} deg\n"
                    f"Angular rate  : {omega:7.3f} rad/s\n"
                    f"Pendulum L    : {params.pendulum_length:.3f} m"
                ),
            )

            frame_index["value"] += 1

        sample_dt = (
            result.time[1]
            - result.time[0]
        )

        # Do not attempt to render every numerical integration point.
        # Target approximately 60 visual frames per second.

        simulation_frame_step = max(
            1,
            int(
                (1.0 / 60.0)
                / sample_dt
                * playback_speed
            ),
        )

        frame_index["value"] += (
            simulation_frame_step - 1
        )

        delay_ms = 16

        root.after(
            delay_ms,
            update_frame,
        )

    update_frame()

    root.mainloop()
