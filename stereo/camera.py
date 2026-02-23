from __future__ import annotations

import time
from typing import Tuple

import cv2
import numpy as np
from picamera2 import Picamera2

import hparams as HP


class StereoCams:
    """
    Manages two Picamera2 instances (left + right) and provides
    synchronised BGR frame capture.

    Per Picamera2 docs, ``format="RGB888"`` returns a uint8 3-channel array
    that is already in BGR order suitable for OpenCV — no extra conversion
    needed.

    Parameters
    ----------
    cam0 : int
        Camera index for the left camera.
    cam1 : int
        Camera index for the right camera.
    size : tuple[int, int]
        Capture resolution as (width, height).
    fps : int
        Target frame rate.  Controls ``FrameDurationLimits``.
    warmup_frames : int
        Number of frames to discard during AE/AWB warmup.
    """

    def __init__(
        self,
        cam0: int = HP.CAMERA_CAM0_INDEX,
        cam1: int = HP.CAMERA_CAM1_INDEX,
        size: Tuple[int, int] = (HP.CAMERA_WIDTH, HP.CAMERA_HEIGHT),
        fps: int = HP.CAMERA_FPS,
        warmup_frames: int = HP.CAMERA_WARMUP_FRAMES,
    ) -> None:
        self._warmup_frames = warmup_frames

        frame_us = int(round(1_000_000 / fps))
        controls = {
            "FrameDurationLimits": (frame_us, frame_us),
            "AeEnable": True,
            "AwbEnable": True,
        }

        self._cam_left = self._init_camera(cam0, size, controls)
        self._cam_right = self._init_camera(cam1, size, controls)

    # ------------------------------------------------------------------ public
    def start(self) -> None:
        """Start both cameras and wait for AE/AWB to settle."""
        self._cam_left.start()
        self._cam_right.start()
        time.sleep(HP.CAMERA_STARTUP_SLEEP_S)

        for _ in range(self._warmup_frames):
            self._cam_left.capture_array("main")
            self._cam_right.capture_array("main")
            time.sleep(HP.CAMERA_WARMUP_SLEEP_S)

    def stop(self) -> None:
        """Stop both cameras gracefully."""
        self._cam_left.stop()
        self._cam_right.stop()

    def capture_bgr(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Capture one frame from each camera.

        Returns
        -------
        left_bgr, right_bgr : np.ndarray
            3-channel uint8 BGR arrays at the configured resolution.
        """
        left = self._cam_left.capture_array("main")
        right = self._cam_right.capture_array("main")
        return self._to_bgr(left), self._to_bgr(right)

    # ----------------------------------------------------------------- private
    @staticmethod
    def _init_camera(
        index: int,
        size: Tuple[int, int],
        controls: dict,
    ) -> Picamera2:
        cam = Picamera2(index)
        conf = cam.create_video_configuration(
            main={"size": size, "format": "RGB888"},
            controls=controls,
        )
        cam.configure(conf)
        return cam

    @staticmethod
    def _to_bgr(frame: np.ndarray) -> np.ndarray:
        """Drop alpha channel if present (safety guard for unexpected formats)."""
        if frame.ndim == 3 and frame.shape[2] == 4:
            return frame[:, :, :3]
        return frame
