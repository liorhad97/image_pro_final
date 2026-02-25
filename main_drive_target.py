import datetime
from pathlib import Path

import cv2

from scanner import Scanner
from test_angle_motors import move_by_angle_and_distance
from main import build_arg_parser, build_config


class VideoRecorder:
    """Collects annotated stereo frames and writes them to an MP4 at the end."""

    def __init__(self) -> None:
        self._fps: float = 30.0
        self._frames: list = []

    def add_frame(self, frame) -> None:
        self._frames.append(frame.copy())

    def save(self, path: str) -> None:
        if not self._frames:
            print("[VideoRecorder] No frames captured — skipping save.")
            return
        h, w = self._frames[0].shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(path, fourcc, self._fps, (w, h))
        for frame in self._frames:
            writer.write(frame)
        writer.release()
        print(f"[VideoRecorder] Saved {len(self._frames)} frames → {path}")


# sends angle and distance to the UI process over stdout
def _emit_ui_data(angle_deg, distance_cm) -> None:
    angle_value = float(angle_deg)
    dist_value = "None" if distance_cm is None else f"{float(distance_cm):.2f}"
    print(
        f"[UI_DATA] angle_deg={angle_value:.2f} distance_cm={dist_value}",
        flush=True,
    )


# runs the full pipeline: scan for target, then drive the robot to it
def integrated_system(recorder: VideoRecorder) -> None:
    print("[INFO] Starting the integrated system.")

    parser = build_arg_parser()
    args = parser.parse_args()
    config = build_config(args)
    recorder._fps = float(config.camera.fps)

    scanner = Scanner(config, frame_callback=recorder.add_frame)

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
    recorder = VideoRecorder()
    try:
        integrated_system(recorder)
    except KeyboardInterrupt:
        print("[INFO] Stopping the system.")
    finally:
        Path("outputs").mkdir(exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        video_path = f"outputs/session_{ts}.mp4"
        recorder.save(video_path)
        print("[INFO] Cleaning up resources and exiting.")
