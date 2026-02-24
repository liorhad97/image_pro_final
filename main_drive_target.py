from scanner import Scanner
from test_angle_motors import move_by_angle_and_distance
from main import build_arg_parser, build_config

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
        #print("[INFO] Object detected. Result:", result)
        print( getattr(result, "angle_deg", 90))
        print( getattr(result, "distance_cm", 100)) # Print the full result for testing
        # Use the result to move the robot
        angle = getattr(result, "angle_deg", 90)  # default angle 90 degrees
        distance_cm = getattr(result, "distance_cm", 100)  # default distance 100 cm

        print(f"[INFO] Moving the robot to angle {angle} and distance {distance_cm} cm.")
        if distance_cm is not None and distance_cm > 2.5:  # check that the distance is positive before moving
            move_by_angle_and_distance(angle, distance_cm-2.5)  # move to distance minus 2.5 cm to not get too close
    else:
        print("[INFO] No object detected. System stopping.")

if __name__ == "__main__":
    try:
        integrated_system()
    except KeyboardInterrupt:
        print("[INFO] Stopping the system.")
    finally:
        print("[INFO] Cleaning up resources and exiting.")
