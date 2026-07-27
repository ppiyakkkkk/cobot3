#!/usr/bin/env python3
"""coverage_visualization_node.py에 안전한 TF fallback을 적용한다."""
from __future__ import annotations
import argparse
from pathlib import Path
import shutil
import sys

RELATIVE_PATH = Path(
    "src/forest_rescue_system/forest_rescue_system/visualization/"
    "coverage_visualization_node.py"
)
DEPTH_ANCHOR = '''        self.declare_parameter("maximum_depth_m", 30.0)
'''
DEPTH_REPLACEMENT = '''        self.declare_parameter("maximum_depth_m", 13.0)
'''
PARAMETER_ANCHOR = '''        self.declare_parameter("drone_04_color_rgb", [0.90, 0.25, 0.65])
        self.declare_parameter("warning_period_sec", 5.0)
'''
PARAMETER_REPLACEMENT = '''        self.declare_parameter("drone_04_color_rgb", [0.90, 0.25, 0.65])
        # exact Depth timestamp TF가 아직 도착하지 않았을 때 최신 TF를
        # 제한적으로 사용한다. 허용 시각차를 넘으면 프레임은 누적하지 않는다.
        self.declare_parameter("tf_latest_fallback_enabled", True)
        self.declare_parameter("tf_latest_fallback_max_age_sec", 0.15)
        self.declare_parameter("warning_period_sec", 5.0)
'''
TF_ANCHOR = '''        camera_frame = f"{drone_id}/camera_optical_frame"
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                self.map_frame,
                camera_frame,
                Time.from_msg(depth_stamp),
                timeout=Duration(seconds=0.0),
            )
        except TransformException as error:
            self.flashlight_state.pop(drone_id, None)
            self._warn_throttled(
                f"tf_{drone_id}",
                f"{drone_id} Camera→map exact TF 대기 중: {error}",
            )
            return False
'''
TF_REPLACEMENT = '''        camera_frame = f"{drone_id}/camera_optical_frame"
        exact_tf_error = None
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                self.map_frame,
                camera_frame,
                Time.from_msg(depth_stamp),
                timeout=Duration(seconds=0.0),
            )
        except TransformException as error:
            exact_tf_error = error
            fallback_enabled = bool(
                self.get_parameter("tf_latest_fallback_enabled").value
            )
            if not fallback_enabled:
                self.flashlight_state.pop(drone_id, None)
                self._warn_throttled(
                    f"tf_{drone_id}",
                    f"{drone_id} Camera→map exact TF 대기 중: {error}",
                )
                return False
            try:
                transform_stamped = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    camera_frame,
                    Time(),
                    timeout=Duration(seconds=0.0),
                )
            except TransformException as latest_error:
                self.flashlight_state.pop(drone_id, None)
                self._warn_throttled(
                    f"tf_{drone_id}",
                    f"{drone_id} Camera→map TF 없음: "
                    f"exact={error}; latest={latest_error}",
                )
                return False
            depth_stamp_ns = _stamp_to_nanoseconds(depth_stamp)
            tf_stamp_ns = _stamp_to_nanoseconds(transform_stamped.header.stamp)
            tf_time_error_sec = abs(depth_stamp_ns - tf_stamp_ns) / 1.0e9
            maximum_fallback_age_sec = max(
                0.0,
                float(
                    self.get_parameter(
                        "tf_latest_fallback_max_age_sec"
                    ).value
                ),
            )
            if tf_time_error_sec > maximum_fallback_age_sec:
                self.flashlight_state.pop(drone_id, None)
                self._warn_throttled(
                    f"tf_{drone_id}",
                    f"{drone_id} Camera→map 최신 TF 시각차 초과: "
                    f"차이={tf_time_error_sec:.3f}s/"
                    f"{maximum_fallback_age_sec:.3f}s, "
                    f"exact_error={exact_tf_error}",
                )
                return False
'''

def replace_once(text: str, old: str, new: str, label: str) -> str:
    if new in text and old not in text:
        print(f"[SKIP] 이미 적용됨: {label}")
        return text
    count = text.count(old)
    if count != 1:
        raise RuntimeError(
            f"{label}: 기존 블록이 정확히 1개여야 하지만 {count}개입니다."
        )
    print(f"[UPDATE] {label}")
    return text.replace(old, new, 1)

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", default="~/b3_cobot3_ws")
    args = parser.parse_args()
    workspace = Path(args.workspace).expanduser().resolve()
    target = workspace / RELATIVE_PATH
    if not target.is_file():
        raise FileNotFoundError(target)
    original = target.read_text(encoding="utf-8")
    updated = replace_once(original, DEPTH_ANCHOR, DEPTH_REPLACEMENT, "coverage maximum depth 13m")
    updated = replace_once(updated, PARAMETER_ANCHOR, PARAMETER_REPLACEMENT, "TF parameters")
    updated = replace_once(updated, TF_ANCHOR, TF_REPLACEMENT, "bounded TF fallback")
    backup = target.with_suffix(target.suffix + ".bak_tf_fallback")
    if not backup.exists():
        shutil.copy2(target, backup)
        print(f"[BACKUP] {backup}")
    target.write_text(updated, encoding="utf-8")
    print(f"[DONE] {target}")
    return 0

if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        raise SystemExit(1)
