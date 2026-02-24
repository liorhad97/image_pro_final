from scanner import Scanner
from test_angle_motors import move_by_angle_and_distance
from main import build_arg_parser, build_config

# sends angle and distance to the UI process over stdout
def _emit_ui_data(angle_deg, distance_cm) -> None:
    angle_value = float(angle_deg)
    dist_value = "None" if distance_cm is None else f"{float(distance_cm):.2f}"
    print(
        f"[UI_DATA] angle_deg={angle_value:.2f} distance_cm={dist_value}",
        flush=True,
    )


# runs the full pipeline: scan for target, then drive the robot to it
def integrated_system():
    print("[INFO] Starting the integrated system.")

    parser = build_arg_parser()
    args = parser.parse_args()
    config = build_config(args)
    scanner = Scanner(config)

    print("[INFO] Running scan to detect objects.")
    result = scanner.run()

    if result is not None:
        angle = getattr(result, "angle_deg", 90)
        distance_cm = getattr(result, "distance_cm", 100)

        _emit_ui_data(angle, distance_cm)
        print(f"[INFO] Moving the robot to angle {angle} and distance {distance_cm} cm.")
        if distance_cm is not None and distance_cm > 2.5:
            move_by_angle_and_distance(angle, distance_cm-2.5)
    else:
        print("[INFO] No object detected. System stopping.")

if __name__ == "__main__":
    try:
        integrated_system()
    except KeyboardInterrupt:
        print("[INFO] Stopping the system.")
    finally:
        print("[INFO] Cleaning up resources and exiting.")
