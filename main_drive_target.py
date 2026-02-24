import math
from scanner import Scanner
from test_angle_motors import move_by_angle_and_distance
from main import build_arg_parser, build_config
import hparams

def integrated_system():
    print("[INFO] Starting the integrated system.")

    # Set up scanner
    parser = build_arg_parser()
    args = parser.parse_args()
    config = build_config(args)
    scanner = Scanner(config)

    # Run scan
    print("[INFO] Running scan to detect objects.")
    result = scanner.run()

    if result is not None:
        angle = getattr(result, "angle_deg", 90)
        distance_cm = getattr(result, "distance_cm", 100)

        # The scanner may detect the object off-center in the frame.
        # Correct the angle by converting the pixel offset to degrees.
        if result.left_center is not None:
            frame_center_x = config.camera.width / 2.0
            offset_px = result.left_center[0] - frame_center_x
            angle_correction_deg = math.degrees(math.atan2(offset_px, hparams.STEREO_FX_PX))
            corrected_angle = max(0.0, min(180.0, angle + angle_correction_deg))
            print(f"[INFO] Pixel offset: {offset_px:.1f}px → angle correction: {angle_correction_deg:.2f}°")
            print(f"[INFO] Servo angle {angle}° → corrected angle {corrected_angle:.1f}°")
            angle = corrected_angle

        print(f"[INFO] Moving the robot to angle {angle:.1f} and distance {distance_cm} cm.")
        if distance_cm is not None and distance_cm > 2.5:
            move_by_angle_and_distance(angle, distance_cm - 2.5)
    else:
        print("[INFO] No object detected. System stopping.")

if __name__ == "__main__":
    try:
        integrated_system()
    except KeyboardInterrupt:
        print("[INFO] Stopping the system.")
    finally:
        print("[INFO] Cleaning up resources and exiting.")
