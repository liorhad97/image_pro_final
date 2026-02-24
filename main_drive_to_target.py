#!/usr/bin/env python3
"""
Entry point: scan for a target, lock, then drive until the target looks close.

Workflow
--------
1. Sweep the camera servo until both cameras confirm the target.
2. Keep that lock for ``--lock-seconds`` (default: 1.0 s).
3. Drive toward the target while steering based on current servo angle.
4. Stop once the target bounding-box area reaches ``--bbox-stop-ratio``.

Usage:
    python main_drive_to_target.py --target Cube
    python main_drive_to_target.py               # any red object
"""
from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from typing import Optional

import hparams as HP
from config import AppConfig
from detection.models import DetResult
from detection.target_detector import TargetDetector
from main import build_arg_parser, build_config
from motor import MotorController
from servo.controller import ServoController
from stereo.camera import StereoCams
from utils.image_utils import bbox_margin_ok


@dataclass(frozen=True)
class DriveParams:
    lock_seconds: float
    bbox_stop_ratio: float
    drive_speed: float
    steer_gain: float
    lost_limit: int
    loop_sleep_s: float


@dataclass(frozen=True)
class LockedTarget:
    angle_deg: float
    target_cls: str


class DriveToTargetSystem:
    """
    Stateful scan/lock/drive pipeline.

    - Scan mode: sweep servo until the target is seen by both cameras.
    - Lock mode: hold the target for N seconds.
    - Drive mode: steer using servo angle and stop by bbox-size threshold.
    """

    def __init__(self, config: AppConfig, params: DriveParams) -> None:
        self._cfg = config
        self._params = params

        self._detector = TargetDetector(
            target=config.scan.target,
            min_area=config.detection.min_area,
            downscale_width=config.detection.downscale_width,
        )
        self._servo = ServoController(
            channel=config.servo.pwm_channel,
            chip=config.servo.pwm_chip,
            min_us=config.servo.min_us,
            max_us=config.servo.max_us,
        )
        self._cams = StereoCams(
            cam0=config.camera.cam0_index,
            cam1=config.camera.cam1_index,
            size=(config.camera.width, config.camera.height),
            fps=config.camera.fps,
        )
        self._motor = MotorController()

        self._frame_mid_x = config.camera.width / 2.0
        self._deadband_px = max(
            HP.TRACKER_DEADBAND_MIN_PX,
            config.camera.width * HP.TRACKER_DEADBAND_FRAC,
        )

        self._pan_sign = 1.0
        self._prev_error_px: Optional[float] = None
        self._prev_delta_deg = 0.0

    # ------------------------------------------------------------------ public
    def run(self) -> None:
        print(
            f"[Drive] Starting scan/drive pipeline | "
            f"target='{self._cfg.scan.target or 'Any'}' | "
            f"lock={self._params.lock_seconds:.1f}s | "
            f"bbox_stop={self._params.bbox_stop_ratio:.3f}"
        )

        self._servo.start(angle=90.0)
        self._cams.start()

        try:
            while True:
                locked = self._scan_and_lock_target()
                reached = self._drive_until_bbox_threshold(locked)
                if reached:
                    print("[Drive] Target threshold reached. Stopping.")
                    return
                print("[Drive] Re-scanning after target loss.")
        finally:
            self._cleanup()

    # ----------------------------------------------------------------- private
    def _scan_and_lock_target(self) -> LockedTarget:
        print("[Drive] Scanning for target...")

        while True:
            for ang in self._sweep_angles():
                self._servo.set_angle(float(ang))
                time.sleep(self._cfg.scan.settle_s)

                det_left, det_right = self._capture_detections()
                if not self._is_confirmed_pair(det_left, det_right):
                    continue

                target_cls = det_left.cls
                print(
                    f"[Drive] Candidate lock at servo={ang}° | class={target_cls} "
                    f"| validating {self._params.lock_seconds:.1f}s..."
                )
                lock_angle = self._lock_target(float(ang), target_cls)
                if lock_angle is not None:
                    print(
                        f"[Drive] Lock confirmed | class={target_cls} "
                        f"| servo={int(round(lock_angle))}°"
                    )
                    return LockedTarget(angle_deg=lock_angle, target_cls=target_cls)

    def _lock_target(self, initial_angle: float, target_cls: str) -> Optional[float]:
        self._reset_tracker_state()
        ang = initial_angle
        self._servo.set_angle(ang)

        start = time.monotonic()
        while (time.monotonic() - start) < self._params.lock_seconds:
            det_left, det_right = self._capture_detections()
            if not self._is_confirmed_pair(det_left, det_right, target_cls=target_cls):
                print("[Drive] Lock dropped before timeout; resuming scan.")
                return None

            ang = self._track_servo(ang, det_left, det_right)
            time.sleep(self._params.loop_sleep_s)

        return ang

    def _drive_until_bbox_threshold(self, locked: LockedTarget) -> bool:
        print(
            f"[Drive] Driving toward class={locked.target_cls} "
            f"(servo start={int(round(locked.angle_deg))}°)"
        )

        self._reset_tracker_state()
        ang = locked.angle_deg
        lost_count = 0

        while True:
            det_left, det_right = self._capture_detections()
            bbox_ratio = self._max_bbox_ratio(
                det_left,
                det_right,
                target_cls=locked.target_cls,
            )

            if bbox_ratio >= self._params.bbox_stop_ratio:
                self._motor.stop()
                print(
                    f"[Drive] Stop by bbox threshold | ratio={bbox_ratio:.3f} "
                    f">= {self._params.bbox_stop_ratio:.3f}"
                )
                return True

            has_center = (
                (det_left.found and det_left.cls == locked.target_cls and det_left.center is not None)
                or (det_right.found and det_right.cls == locked.target_cls and det_right.center is not None)
            )
            if not has_center:
                lost_count += 1
                self._motor.stop()
                if lost_count >= self._params.lost_limit:
                    print("[Drive] Target lost while driving.")
                    return False
            else:
                lost_count = 0
                ang = self._track_servo(
                    ang,
                    det_left,
                    det_right,
                    target_cls=locked.target_cls,
                )
                self._drive_with_servo_heading(ang)

            print(
                f"[Drive] servo={int(round(ang)):3d}° | "
                f"bbox={bbox_ratio:.3f}/{self._params.bbox_stop_ratio:.3f} | "
                f"lost={lost_count}/{self._params.lost_limit}"
            )
            time.sleep(self._params.loop_sleep_s)

    def _track_servo(
        self,
        ang: float,
        det_left: DetResult,
        det_right: DetResult,
        target_cls: Optional[str] = None,
    ) -> float:
        centers_x = []
        if (
            det_left.found
            and det_left.center is not None
            and (target_cls is None or det_left.cls == target_cls)
        ):
            centers_x.append(float(det_left.center[0]))
        if (
            det_right.found
            and det_right.center is not None
            and (target_cls is None or det_right.cls == target_cls)
        ):
            centers_x.append(float(det_right.center[0]))
        if not centers_x:
            return ang

        error_px = (sum(centers_x) / len(centers_x)) - self._frame_mid_x

        if (
            self._prev_error_px is not None
            and abs(self._prev_delta_deg) > 1e-3
            and abs(self._prev_error_px) > self._deadband_px
            and abs(error_px) > abs(self._prev_error_px) + (self._deadband_px * 0.25)
        ):
            self._pan_sign *= -1.0
            self._prev_delta_deg = 0.0

        delta_deg = 0.0
        if abs(error_px) > self._deadband_px:
            norm_error = error_px / self._frame_mid_x
            delta_deg = self._pan_sign * HP.TRACKER_KP_DEG * norm_error
            delta_deg = max(-HP.TRACKER_MAX_STEP_DEG, min(HP.TRACKER_MAX_STEP_DEG, delta_deg))
            new_ang = max(0.0, min(180.0, ang + delta_deg))
            if abs(new_ang - ang) > 1e-3:
                ang = new_ang
                self._servo.set_angle(ang)

        self._prev_error_px = error_px
        self._prev_delta_deg = delta_deg
        return ang

    def _drive_with_servo_heading(self, servo_angle: float) -> None:
        # Map servo heading directly to wheel steering by differential speed.
        steer_deg = max(-90.0, min(90.0, servo_angle - 90.0))
        steer_norm = steer_deg / 90.0

        delta = self._params.drive_speed * self._params.steer_gain * steer_norm
        speed_l = self._clamp_speed(self._params.drive_speed - delta)
        speed_r = self._clamp_speed(self._params.drive_speed + delta)
        self._motor.set_motors(speed_l, speed_r, direction="forward")

    def _capture_detections(self) -> tuple[DetResult, DetResult]:
        frame_left, frame_right = self._cams.capture_bgr()
        return self._detector.detect(frame_left), self._detector.detect(frame_right)

    def _is_confirmed_pair(
        self,
        det_left: DetResult,
        det_right: DetResult,
        target_cls: Optional[str] = None,
    ) -> bool:
        same_class = (
            det_left.found and det_right.found
            and det_left.cls != "None" and det_right.cls != "None"
            and det_left.cls == det_right.cls
        )
        if not same_class:
            return False
        if target_cls is not None and det_left.cls != target_cls:
            return False

        edge_ok_left = bbox_margin_ok(
            det_left,
            self._cfg.camera.width,
            self._cfg.camera.height,
            self._cfg.scan.edge_margin_px,
        )
        edge_ok_right = bbox_margin_ok(
            det_right,
            self._cfg.camera.width,
            self._cfg.camera.height,
            self._cfg.scan.edge_margin_px,
        )
        return edge_ok_left and edge_ok_right

    def _max_bbox_ratio(
        self,
        det_left: DetResult,
        det_right: DetResult,
        target_cls: Optional[str] = None,
    ) -> float:
        frame_area = float(self._cfg.camera.width * self._cfg.camera.height)
        if frame_area <= 0.0:
            return 0.0

        ratios = []
        for det in (det_left, det_right):
            if (
                det.found
                and det.bbox is not None
                and (target_cls is None or det.cls == target_cls)
            ):
                _, _, w, h = det.bbox
                ratios.append((w * h) / frame_area)
        return max(ratios) if ratios else 0.0

    def _sweep_angles(self) -> list[int]:
        step = self._cfg.scan.step_deg
        forward = list(range(0, 181, step))
        backward = list(range(180, -1, -step))
        return forward + backward[1:-1]

    def _reset_tracker_state(self) -> None:
        self._pan_sign = 1.0
        self._prev_error_px = None
        self._prev_delta_deg = 0.0

    @staticmethod
    def _clamp_speed(speed: float) -> float:
        return max(0.0, min(100.0, speed))

    def _cleanup(self) -> None:
        try:
            self._motor.stop()
        except Exception:
            pass
        try:
            self._motor.cleanup()
        except Exception:
            pass
        try:
            self._cams.stop()
        except Exception:
            pass
        try:
            self._servo.stop()
        except Exception:
            pass


def build_drive_parser() -> argparse.ArgumentParser:
    parser = build_arg_parser()
    g_drive = parser.add_argument_group("Drive-to-target")
    g_drive.add_argument(
        "--lock-seconds",
        type=float,
        default=1.0,
        help="How long the target must stay locked before driving starts",
    )
    g_drive.add_argument(
        "--bbox-stop-ratio",
        type=float,
        default=0.050,
        help="Stop driving when bbox area/frame area reaches this ratio (0..1)",
    )
    g_drive.add_argument(
        "--drive-speed",
        type=float,
        default=45.0,
        help="Base forward speed duty-cycle (0..100)",
    )
    g_drive.add_argument(
        "--steer-gain",
        type=float,
        default=0.70,
        help="Wheel steering gain from servo angle (0..1 typical)",
    )
    g_drive.add_argument(
        "--lost-limit",
        type=int,
        default=12,
        help="Consecutive no-center frames before re-scan",
    )
    g_drive.add_argument(
        "--loop-sleep",
        type=float,
        default=0.03,
        help="Control-loop sleep in seconds",
    )
    return parser


def _build_drive_params(args: argparse.Namespace, parser: argparse.ArgumentParser) -> DriveParams:
    if args.lock_seconds <= 0.0:
        parser.error("--lock-seconds must be > 0")
    if not (0.0 < args.bbox_stop_ratio <= 1.0):
        parser.error("--bbox-stop-ratio must be in (0, 1]")
    if not (0.0 < args.drive_speed <= 100.0):
        parser.error("--drive-speed must be in (0, 100]")
    if args.steer_gain < 0.0:
        parser.error("--steer-gain must be >= 0")
    if args.lost_limit < 1:
        parser.error("--lost-limit must be >= 1")
    if args.loop_sleep <= 0.0:
        parser.error("--loop-sleep must be > 0")

    return DriveParams(
        lock_seconds=args.lock_seconds,
        bbox_stop_ratio=args.bbox_stop_ratio,
        drive_speed=args.drive_speed,
        steer_gain=args.steer_gain,
        lost_limit=args.lost_limit,
        loop_sleep_s=args.loop_sleep,
    )


def integrated_system() -> None:
    """Full pipeline: scan → 1s lock → steer-drive until bbox threshold."""
    parser = build_drive_parser()
    args = parser.parse_args()
    config = build_config(args)
    params = _build_drive_params(args, parser)
    DriveToTargetSystem(config, params).run()


if __name__ == "__main__":
    try:
        integrated_system()
    except KeyboardInterrupt:
        print("[Drive] Stopping.")
    finally:
        print("[Drive] Cleaning up and exiting.")
