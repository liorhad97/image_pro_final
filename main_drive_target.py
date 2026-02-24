import math

from scanner import Scanner
from test_angle_motors import move_by_angle_and_distance
from main import build_arg_parser, build_config

def correct_angle_for_pixel_offset(servo_angle: float, center_x: float, frame_width: int, fx_px: float) -> float:
    """
    Adjust the servo angle to account for the object not being centred in the
    camera frame.

    When the scanner confirms a detection, the object's pixel centre may be
    offset from the frame centre.  That offset corresponds to a real angular
    error:

        correction_deg = atan(pixel_offset / fx_px)   [converted to degrees]

    The corrected angle is clamped to [0, 180].
    """
    pixel_offset = center_x - (frame_width / 2.0)
    correction_deg = math.degrees(math.atan2(pixel_offset, fx_px))
    corrected = servo_angle + correction_deg
    return max(0.0, min(180.0, corrected))


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
        servo_angle = getattr(result, "angle_deg", 90)
        distance_cm = getattr(result, "distance_cm", 100)
        left_center  = getattr(result, "left_center", None)

        # Correct for pixel offset so the robot aims at the object's true direction
        if left_center is not None:
            angle = correct_angle_for_pixel_offset(
                servo_angle,
                center_x=left_center[0],
                frame_width=config.camera.width,
                fx_px=config.stereo.fx_px,
            )
            print(f"[INFO] Servo angle: {servo_angle}°  |  pixel offset → corrected angle: {angle:.1f}°")
        else:
            angle = servo_angle
            print(f"[INFO] Servo angle: {angle}° (no pixel correction)")

        print(f"[INFO] Distance: {distance_cm} cm")
        print(f"[INFO] Moving the robot to angle {angle:.1f}° and distance {distance_cm} cm.")
        if distance_cm is not None and distance_cm > 2.5:  # check that the distance is positive before moving
            move_by_angle_and_distance(angle, distance_cm - 2.5)  # move to distance minus 2.5 cm to not get too close
    else:
        print("[INFO] No object detected. System stopping.")

if __name__ == "__main__":
    try:
        integrated_system()
    except KeyboardInterrupt:
        print("[INFO] Stopping the system.")
    finally:
        print("[INFO] Cleaning up resources and exiting.")
