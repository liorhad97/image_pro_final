from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import time
from typing import Tuple

import cv2
import numpy as np
from picamera2 import Picamera2

import hparams as HP


class StereoCams:
    """
    Manages two Picamera2 instances (left + right) and provides
    near-synchronised BGR frame capture.

    Frames are requested as ``RGB888`` from Picamera2 and converted to
    OpenCV BGR before returning.

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
        self._capture_pool = ThreadPoolExecutor(max_workers=2)

        frame_us = int(round(1_000_000 / fps))
        controls = {
            "FrameDurationLimits": (frame_us, frame_us),
            "AeEnable": True,
            "AwbEnable": True,
        }

        cam_left = None
        try:
            cam_left = self._init_camera(cam0, size, controls)
            cam_right = self._init_camera(cam1, size, controls)
        except Exception:
            if cam_left is not None:
                self._safe_close(cam_left)
            self._capture_pool.shutdown(wait=False, cancel_futures=True)
            raise

        self._cam_left = cam_left
        self._cam_right = cam_right

    # ------------------------------------------------------------------ public
    def start(self) -> None:
        """Start both cameras and wait for AE/AWB to settle."""
        self._cam_left.start()
        self._cam_right.start()
        time.sleep(HP.CAMERA_STARTUP_SLEEP_S)

        for _ in range(self._warmup_frames):
            self._capture_pair_raw()
            time.sleep(HP.CAMERA_WARMUP_SLEEP_S)

    def stop(self) -> None:
        """Stop and close both cameras gracefully."""
        try:
            self._cam_left.stop()
        except Exception:
            pass
        try:
            self._cam_right.stop()
        except Exception:
            pass
        self._safe_close(self._cam_left)
        self._safe_close(self._cam_right)
        self._capture_pool.shutdown(wait=True, cancel_futures=True)

    def capture_bgr(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Capture one frame from each camera.

        Returns
        -------
        left_bgr, right_bgr : np.ndarray
            3-channel uint8 BGR arrays at the configured resolution.
        """
        left, right = self._capture_pair_raw()
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
        """Convert camera output to BGR for OpenCV processing."""
        if frame.ndim == 3 and frame.shape[2] == 3:
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if frame.ndim == 3 and frame.shape[2] == 4:
            return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        return frame

    def _capture_pair_raw(self) -> Tuple[np.ndarray, np.ndarray]:
        left_future = self._capture_pool.submit(self._cam_left.capture_array, "main")
        right_future = self._capture_pool.submit(self._cam_right.capture_array, "main")
        left = left_future.result()
        right = right_future.result()
        return left, right

    @staticmethod
    def _safe_close(cam: Picamera2) -> None:
        try:
            cam.close()
        except Exception:
            pass
