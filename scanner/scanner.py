from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import cv2

import hparams
from config import AppConfig
from detection.detector import RedObjectDetector
from detection.models import DetResult
from servo.controller import ServoController
from stereo.camera import StereoCams
from stereo.distance import StereoDistanceEstimator, StereoDistanceResult
from utils.image_utils import (
    bbox_margin_ok,
    hstack_resize,
    save_mask_png,
)


@dataclass
class ScanResult:
    """Structured result returned when both cameras confirm a detection."""

    angle_deg: int
    left_bbox: Optional[tuple]
    left_center: Optional[tuple]
    left_class: str
    right_bbox: Optional[tuple]
    right_center: Optional[tuple]
    right_class: str
    distance_m: Optional[float]
    distance_cm: Optional[float]
    stereo_debug: Dict[str, float] = field(default_factory=dict)
    stereo_error: Optional[str] = None


@dataclass
class TargetLockResult:
    """State captured after the lock-centering phase."""

    angle_deg: float
    det_left: DetResult
    det_right: DetResult
    dist_result: StereoDistanceResult
    quit_requested: bool = False


class Scanner:
    """
    Orchestrates a pan-tilt servo sweep while capturing stereo frames,
    running blue-object detection on each, and confirming a final target
    when both cameras agree.

    Acceptance criteria
    -------------------
    All four conditions must be met simultaneously:

    1. Both cameras detect a target in the current frame.
    2. Both cameras report the **same** object class.
    3. The detected class matches ``config.scan.target`` (or any class when
       ``target`` is ``None``).
    4. Both bounding boxes are at least ``edge_margin_px`` from the frame
       border (avoids partially cropped objects).

    Usage
    -----
    cfg = AppConfig()
    scanner = Scanner(cfg)
    result = scanner.run()   # blocks until found or interrupted
    """

    def __init__(self, config: AppConfig) -> None:
        self._cfg = config

        self._detector = RedObjectDetector(
            hsv_lo1=config.detection.hsv_lo,
            hsv_hi1=config.detection.hsv_hi,
            hsv_lo2=hparams.HSV_LO2,
            hsv_hi2=hparams.HSV_HI2,
            min_area=config.detection.min_area,
            downscale_width=config.detection.downscale_width,
        )
        self._distance_estimator = StereoDistanceEstimator(
            fx_px=config.stereo.fx_px,
            baseline_m=config.stereo.baseline_m,
            min_disparity_px=config.stereo.min_disparity_px,
            max_vertical_offset_px=config.stereo.max_vertical_offset_px,
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
        self._output_dirs = self._prepare_output_dirs(config.scan.outdir)

    # ------------------------------------------------------------------ public
    def run(self) -> Optional[ScanResult]:
        """
        Start the servo sweep and return the first confirmed :class:`ScanResult`,
        or ``None`` if the user exits early.
        """
        print(
            f"[Scanner] Starting — target='{self._cfg.scan.target or 'Any'}' | "
            f"resolution={self._cfg.camera.width}x{self._cfg.camera.height} | "
            f"baseline={self._cfg.stereo.baseline_m}m | fx={self._cfg.stereo.fx_px}px"
        )

        self._servo.start(angle=90.0)
        self._cams.start()

        event_idx = 0
        try:
            while True:
                for ang in self._sweep_angles():
                    self._servo.set_angle(ang)
                    time.sleep(self._cfg.scan.settle_s)

                    frame_left, frame_right = self._cams.capture_bgr()
                    det_left = self._detector.detect(frame_left)
                    det_right = self._detector.detect(frame_right)

                    match_left = self._matches_target(det_left)
                    match_right = self._matches_target(det_right)
                    same_class = self._same_valid_class(det_left, det_right)
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

                    dist_result: StereoDistanceResult = (
                        self._distance_estimator.estimate(det_left, det_right)
                        if same_class
                        else StereoDistanceResult(
                            distance_m=None,
                            distance_cm=None,
                            disparity_px=None,
                            dy_px=None,
                            error="no_same_class",
                        )
                    )

                    confirmed = (
                        match_left
                        and match_right
                        and same_class
                        and edge_ok_left
                        and edge_ok_right
                    )

                    self._log_frame(
                        ang,
                        det_left,
                        det_right,
                        match_left,
                        match_right,
                        edge_ok_left,
                        edge_ok_right,
                        same_class,
                        dist_result,
                    )

                    ann_left = self._detector.draw(frame_left, det_left, "L: ")
                    ann_right = self._detector.draw(frame_right, det_right, "R: ")
                    ts = int(time.time() * 1000)

                    if not self._cfg.scan.view:
                        self._save_per_camera(
                            event_idx,
                            ts,
                            ang,
                            ann_left,
                            ann_right,
                            det_left,
                            det_right,
                            match_left,
                            match_right,
                        )

                        if confirmed:
                            self._save_combined(
                                event_idx,
                                ts,
                                ang,
                                ann_left,
                                ann_right,
                                det_left,
                                dist_result,
                            )

                    if self._cfg.scan.view:
                        quit_requested = self._show_preview(
                            ann_left,
                            ann_right,
                            ang,
                            same_class,
                            edge_ok_left,
                            edge_ok_right,
                            dist_result,
                            det_left,
                        )
                        if quit_requested:
                            print("[Scanner] User pressed 'c' — stopping.")
                            return None

                    if match_left or match_right:
                        event_idx += 1

                    if confirmed:
                        locked = self._lock_target(
                            initial_angle=float(ang),
                            target_cls=det_left.cls,
                            initial_det_left=det_left,
                            initial_det_right=det_right,
                            initial_dist_result=dist_result,
                            iterations=hparams.TARGET_LOCK_ITERATIONS,
                        )
                        if locked.quit_requested:
                            print("[Scanner] User pressed 'c' during lock phase.")
                            return None

                        return self._build_result(
                            int(round(locked.angle_deg)),
                            locked.det_left,
                            locked.det_right,
                            locked.dist_result,
                        )

        finally:
            self._cleanup()

        return None

    # ----------------------------------------------------------------- private
    def _sweep_angles(self) -> List[int]:
        step = self._cfg.scan.step_deg
        forward = list(range(0, 181, step))
        backward = list(range(180, -1, -step))
        return forward + backward[1:-1]

    def _matches_target(self, det: DetResult) -> bool:
        if not det.is_valid:
            return False
        if self._cfg.scan.target is None:
            return True
        return det.cls == self._cfg.scan.target

    @staticmethod
    def _same_valid_class(det_left: DetResult, det_right: DetResult) -> bool:
        return (
            det_left.is_valid
            and det_right.is_valid
            and det_left.cls != "None"
            and det_right.cls != "None"
            and det_left.cls == det_right.cls
        )

    def _lock_target(
        self,
        initial_angle: float,
        target_cls: str,
        initial_det_left: DetResult,
        initial_det_right: DetResult,
        initial_dist_result: StereoDistanceResult,
        iterations: int,
    ) -> TargetLockResult:
        """
        Perform a bounded lock-centering phase around the first confirmed target.

        Runs a fixed number of small pan corrections (or fewer if centered early)
        and returns the final angle + best stereo frame captured during lock.
        """
        frame_mid_x = self._cfg.camera.width / 2.0
        deadband_px = max(
            hparams.TRACKER_DEADBAND_MIN_PX,
            self._cfg.camera.width * hparams.TRACKER_DEADBAND_FRAC,
        )

        kp_deg = hparams.TRACKER_KP_DEG
        max_step_deg = hparams.TRACKER_MAX_STEP_DEG
        lock_steps = max(0, int(iterations))

        ang = float(initial_angle)
        pan_sign = 1.0
        prev_error_px: Optional[float] = None

        print(
            f"[Lock] Centering class='{target_cls}' from pan={int(round(ang))}° "
            f"for up to {lock_steps} iterations."
        )

        best_det_left = initial_det_left
        best_det_right = initial_det_right
        best_dist_result = initial_dist_result

        for idx in range(lock_steps):
            frame_left, frame_right = self._cams.capture_bgr()
            det_left = self._detector.detect(frame_left)
            det_right = self._detector.detect(frame_right)

            same_class = self._same_valid_class(det_left, det_right)
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

            dist_result: StereoDistanceResult = (
                self._distance_estimator.estimate(det_left, det_right)
                if same_class
                else StereoDistanceResult(
                    distance_m=None,
                    distance_cm=None,
                    disparity_px=None,
                    dy_px=None,
                    error="no_same_class",
                )
            )

            match_left = det_left.is_valid and det_left.cls == target_cls
            match_right = det_right.is_valid and det_right.cls == target_cls

            centers_x = []
            if match_left and det_left.center is not None:
                centers_x.append(float(det_left.center[0]))
            if match_right and det_right.center is not None:
                centers_x.append(float(det_right.center[0]))

            if not centers_x:
                print("[Lock] Target lost during lock phase; using last confirmed angle.")
                break

            if match_left and match_right and same_class and edge_ok_left and edge_ok_right:
                best_det_left = det_left
                best_det_right = det_right
                best_dist_result = dist_result

            error_px = (sum(centers_x) / len(centers_x)) - frame_mid_x

            self._log_frame(
                int(round(ang)),
                det_left,
                det_right,
                match_left,
                match_right,
                edge_ok_left,
                edge_ok_right,
                same_class,
                dist_result,
            )

            ann_left = self._detector.draw(frame_left, det_left, "L: ")
            ann_right = self._detector.draw(frame_right, det_right, "R: ")

            if self._cfg.scan.view:
                quit_requested = self._show_preview(
                    ann_left,
                    ann_right,
                    int(round(ang)),
                    same_class,
                    edge_ok_left,
                    edge_ok_right,
                    dist_result,
                    det_left,
                )
                if quit_requested:
                    print("[Lock] User pressed 'c' — stopping.")
                    return TargetLockResult(
                        angle_deg=ang,
                        det_left=best_det_left,
                        det_right=best_det_right,
                        dist_result=best_dist_result,
                        quit_requested=True,
                    )

            if abs(error_px) <= deadband_px:
                print(
                    f"[Lock] Centered after {idx + 1}/{lock_steps} iterations "
                    f"(error={error_px:.1f}px, deadband={deadband_px:.1f}px)."
                )
                break

            if (
                prev_error_px is not None
                and abs(error_px) > abs(prev_error_px) + (deadband_px * 0.25)
            ):
                pan_sign *= -1.0
                print(
                    f"[Lock] Reversing pan direction guess (sign={pan_sign:+.0f})."
                )

            norm_error = error_px / frame_mid_x
            delta_deg = pan_sign * kp_deg * norm_error
            delta_deg = max(-max_step_deg, min(max_step_deg, delta_deg))

            new_ang = max(0.0, min(180.0, ang + delta_deg))
            if abs(new_ang - ang) > 1e-3:
                ang = new_ang
                self._servo.set_angle(ang)
                time.sleep(min(hparams.TRACKER_STEP_SLEEP_S, self._cfg.scan.settle_s))

            prev_error_px = error_px
            time.sleep(hparams.TRACKER_LOOP_SLEEP_S)

        return TargetLockResult(
            angle_deg=ang,
            det_left=best_det_left,
            det_right=best_det_right,
            dist_result=best_dist_result,
        )

    def _save_per_camera(
        self,
        event_idx: int, ts: int, ang: int,
        ann_left, ann_right,
        det_left: DetResult, det_right: DetResult,
        match_left: bool, match_right: bool,
    ) -> None:
        dirs = self._output_dirs
        prefix = f"ev_{event_idx:04d}_t{ts}_pan{ang:03d}"

        if match_left and det_left.mask_raw is not None:
            save_mask_png(
                dirs["masks_raw"] / f"{prefix}_L_RAW_{det_left.cls}.png",
                det_left.mask_raw,
            )
            save_mask_png(
                dirs["masks_clean"] / f"{prefix}_L_CLEAN_{det_left.cls}.png",
                det_left.mask_clean,
            )
            cv2.imwrite(
                str(dirs["left"] / f"{prefix}_L_{det_left.cls}.jpg"), ann_left
            )

        if match_right and det_right.mask_raw is not None:
            save_mask_png(
                dirs["masks_raw"] / f"{prefix}_R_RAW_{det_right.cls}.png",
                det_right.mask_raw,
            )
            save_mask_png(
                dirs["masks_clean"] / f"{prefix}_R_CLEAN_{det_right.cls}.png",
                det_right.mask_clean,
            )
            cv2.imwrite(
                str(dirs["right"] / f"{prefix}_R_{det_right.cls}.jpg"), ann_right
            )

    def _save_combined(
        self,
        event_idx: int, ts: int, ang: int,
        ann_left, ann_right,
        det_left: DetResult,
        dist_result: StereoDistanceResult,
    ) -> None:
        combo = hstack_resize(ann_left, ann_right, max_width=hparams.PREVIEW_MAX_WIDTH)
        status = (
            f"SAME class={det_left.cls} | "
            f"edge_margin={self._cfg.scan.edge_margin_px}px | "
            f"z={'%.3fm' % dist_result.distance_m if dist_result.is_valid else 'N/A'}"
        )
        combo = StereoDistanceEstimator.draw_overlay(
            combo,
            dist_result,
            title=f"Target: {self._cfg.scan.target or 'Any'}",
            extra_status=status,
        )
        prefix = f"ev_{event_idx:04d}_t{ts}_pan{ang:03d}"
        cv2.imwrite(
            str(self._output_dirs["both"] / f"{prefix}_BOTH_{det_left.cls}.jpg"),
            combo,
        )

    def _show_preview(
        self,
        ann_left, ann_right,
        ang: int,
        same_class: bool,
        edge_ok_left: bool, edge_ok_right: bool,
        dist_result: StereoDistanceResult,
        det_left: Optional[DetResult] = None,
    ) -> bool:
        """Render live preview. Returns ``True`` if user pressed 'c' to quit."""
        # Draw distance inside the bounding box on the left frame
        if (
            det_left is not None
            and det_left.is_valid
            and det_left.bbox is not None
            and dist_result.is_valid
        ):
            x, y, w, h = det_left.bbox
            dist_text = f"{dist_result.distance_cm:.1f} cm"
            (tw, th), _ = cv2.getTextSize(
                dist_text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2
            )
            tx = x + (w - tw) // 2
            ty = y + (h + th) // 2
            cv2.putText(
                ann_left, dist_text, (tx, ty),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2,
            )

        combo = hstack_resize(ann_left, ann_right, max_width=hparams.PREVIEW_MAX_WIDTH)
        extra = (
            f"SERVO={ang}°  "
            f"SAME={'Y' if same_class else 'N'}  "
            f"EDGE_L={'OK' if edge_ok_left else 'BAD'}  "
            f"EDGE_R={'OK' if edge_ok_right else 'BAD'}"
        )
        combo = StereoDistanceEstimator.draw_overlay(
            combo,
            dist_result,
            title=(
                f"Target: {self._cfg.scan.target or 'Any'} | SERVO={ang}°"
            ),
            extra_status=extra,
        )
        if hparams.SHOW_PREVIEW:
            cv2.imshow("Stereo Scan (L | R)", combo)
            key = cv2.waitKey(1) & 0xFF
            return key == ord("c")  # press 'c' to stop
        return False

    @staticmethod
    def _log_frame(
        ang: int,
        det_left: DetResult, det_right: DetResult,
        match_left: bool, match_right: bool,
        edge_ok_left: bool, edge_ok_right: bool,
        same_class: bool,
        dist_result: StereoDistanceResult,
    ) -> None:
        z_str = (
            f"{dist_result.distance_m:.3f}m"
            if dist_result.is_valid
            else (dist_result.error or "--")
        )
        print(
            f"servo={ang:3d}° | "
            f"L={det_left.cls:8s} {'OK' if match_left else '--'} "
            f"{'EDGE_OK' if edge_ok_left else 'EDGE_BAD'} | "
            f"R={det_right.cls:8s} {'OK' if match_right else '--'} "
            f"{'EDGE_OK' if edge_ok_right else 'EDGE_BAD'} | "
            f"SAME={'Y' if same_class else 'N'} | Z={z_str}"
        )

    @staticmethod
    def _build_result(
        ang: int,
        det_left: DetResult,
        det_right: DetResult,
        dist_result: StereoDistanceResult,
    ) -> ScanResult:
        print(
            "\n[Scanner] Target CONFIRMED in both cameras!\n"
            f"  Angle    : {ang}°\n"
            f"  Left     : class={det_left.cls}  bbox={det_left.bbox}  center={det_left.center}\n"
            f"  Right    : class={det_right.cls}  bbox={det_right.bbox}  center={det_right.center}\n"
            + (
                f"  Distance : {dist_result.distance_m:.3f} m "
                f"({dist_result.distance_cm:.1f} cm)"
                if dist_result.is_valid
                else f"  Distance : unavailable ({dist_result.error})"
            )
        )
        return ScanResult(
            angle_deg=ang,
            left_bbox=det_left.bbox,
            left_center=det_left.center,
            left_class=det_left.cls,
            right_bbox=det_right.bbox,
            right_center=det_right.center,
            right_class=det_right.cls,
            distance_m=dist_result.distance_m,
            distance_cm=dist_result.distance_cm,
            stereo_debug=dist_result.debug,
            stereo_error=dist_result.error,
        )

    def _cleanup(self) -> None:
        try:
            self._cams.stop()
        except Exception:
            pass
        try:
            self._servo.stop()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

    @staticmethod
    def _prepare_output_dirs(base: str) -> Dict[str, Path]:
        root = Path(base)
        dirs = {
            "left": root / "left",
            "right": root / "right",
            "both": root / "both",
            "masks_raw": root / "masks_raw",
            "masks_clean": root / "masks_clean",
        }
        for d in dirs.values():
            d.mkdir(parents=True, exist_ok=True)
        return dirs
