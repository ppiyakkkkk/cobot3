#!/usr/bin/env python3
"""현재 drone_controller_node.py에 진행방향 Yaw 정렬 기능을 적용한다.

이 스크립트는 사용자의 최신 ROS 2 제어 파일을 통째로 오래된 버전으로
교체하지 않는다. 현재 파일에서 필요한 부분만 찾아 수정하고, 원본은
같은 폴더에 ``.before_direction_yaw.bak`` 확장자로 백업한다.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import shutil
import sys

PATCH_MARKER = "[DIRECTION_YAW_PATCH_V1]"


def fail(message: str) -> "NoReturn":
    raise RuntimeError(message)


def find_indented_block(text: str, header_pattern: str) -> tuple[int, int]:
    """indent 4의 함수 블록 시작과 다음 함수 시작 위치를 반환한다."""
    match = re.search(header_pattern, text, flags=re.MULTILINE)
    if not match:
        fail(f"함수 블록을 찾지 못했습니다: {header_pattern}")

    start = match.start()
    next_match = re.search(
        r"^    (?:async def|def)\s+",
        text[match.end():],
        flags=re.MULTILINE,
    )
    if next_match:
        end = match.end() + next_match.start()
    else:
        end = len(text)
    return start, end


def insert_parameters(text: str) -> str:
    if f"{PATCH_MARKER} parameters" in text:
        return text

    parameter_block = f'''        # {PATCH_MARKER} parameters
        # 수평 이동을 시작하기 전에 기체 Yaw를 목표 이동 방향으로 맞춘다.
        # 카메라가 body 전방에 고정되어 있으므로 이 값이 True이면
        # RGB/Depth 영상도 실제 비행 진행 방향을 바라보게 된다.
        self.declare_parameter("face_movement_direction", True)
        self.declare_parameter("yaw_alignment_min_distance_m", 0.50)
        self.declare_parameter("yaw_alignment_tolerance_deg", 8.0)
        self.declare_parameter("yaw_alignment_timeout_sec", 4.0)
'''

    pattern = re.compile(
        r'(?P<line>^[ \t]*self\.declare_parameter\("search_yaw_deg",\s*0\.0\)\s*$\n)',
        flags=re.MULTILINE,
    )
    match = pattern.search(text)
    if not match:
        fail('"search_yaw_deg" 파라미터 선언 위치를 찾지 못했습니다.')
    return text[:match.end()] + parameter_block + text[match.end():]


def insert_helpers(text: str) -> str:
    if f"{PATCH_MARKER} helpers" in text:
        return text

    helper_block = f"    # {PATCH_MARKER} helpers\n" + '''    @staticmethod
    def _normalize_yaw_deg(yaw_deg):
        """Yaw를 -180도 이상 180도 미만 범위로 정규화한다."""
        return (float(yaw_deg) + 180.0) % 360.0 - 180.0

    def _yaw_toward_target(
        self,
        target_north_m,
        target_east_m,
        fallback_yaw_deg=None,
    ):
        """현재 위치에서 목표 XY를 바라보는 PX4 NED Yaw를 계산한다.

        PX4 Local NED에서는 북쪽이 0도이고 동쪽이 +90도이므로
        ``atan2(delta_east, delta_north)``를 사용한다. 목표가 현재 XY와
        거의 같아 수평 방향을 정할 수 없으면 기존 Yaw를 유지한다.
        """
        fallback = (
            self.latest_yaw_deg
            if fallback_yaw_deg is None
            else float(fallback_yaw_deg)
        )
        if not bool(
            self.get_parameter("face_movement_direction").value
        ):
            return self._normalize_yaw_deg(fallback)

        delta_north = float(target_north_m) - self.latest_north_m
        delta_east = float(target_east_m) - self.latest_east_m
        distance = math.hypot(delta_north, delta_east)
        minimum_distance = max(
            0.05,
            float(
                self.get_parameter(
                    "yaw_alignment_min_distance_m"
                ).value
            ),
        )
        if distance < minimum_distance:
            return self._normalize_yaw_deg(fallback)

        return self._normalize_yaw_deg(
            math.degrees(math.atan2(delta_east, delta_north))
        )

    async def _align_yaw_to_target(
        self,
        target_north_m,
        target_east_m,
        fallback_yaw_deg=None,
    ):
        """현재 XY를 유지한 채 목표 이동 방향으로 먼저 회전한다.

        자동차처럼 이동 방향과 기체 전방을 맞추기 위한 단계다. 회전이
        제한시간 안에 완전히 끝나지 않아도 계산한 목표 Yaw로 이동 명령은
        계속 수행하며, 한 드론의 Yaw 지연이 전체 임무 오류가 되지 않게 한다.
        """
        target_yaw_deg = self._yaw_toward_target(
            target_north_m,
            target_east_m,
            fallback_yaw_deg,
        )
        if not bool(
            self.get_parameter("face_movement_direction").value
        ):
            return target_yaw_deg

        horizontal_distance = math.hypot(
            float(target_north_m) - self.latest_north_m,
            float(target_east_m) - self.latest_east_m,
        )
        minimum_distance = max(
            0.05,
            float(
                self.get_parameter(
                    "yaw_alignment_min_distance_m"
                ).value
            ),
        )
        if horizontal_distance < minimum_distance:
            return target_yaw_deg

        tolerance_deg = max(
            1.0,
            float(
                self.get_parameter(
                    "yaw_alignment_tolerance_deg"
                ).value
            ),
        )
        initial_error_deg = abs(
            self._normalize_yaw_deg(
                target_yaw_deg - self.latest_yaw_deg
            )
        )
        if initial_error_deg <= tolerance_deg:
            return target_yaw_deg

        self.get_logger().info(
            f"진행방향 Yaw 정렬: current={self.latest_yaw_deg:.1f}°, "
            f"target={target_yaw_deg:.1f}°, "
            f"error={initial_error_deg:.1f}°"
        )
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(
                self.latest_north_m,
                self.latest_east_m,
                self.latest_down_m,
                target_yaw_deg,
            )
        )

        deadline = time.monotonic() + max(
            0.5,
            float(
                self.get_parameter(
                    "yaw_alignment_timeout_sec"
                ).value
            ),
        )
        while time.monotonic() < deadline:
            # Yaw가 변하는 동안에도 LiDAR가 검사할 실제 목표 방향을
            # body 상대각으로 계속 갱신한다.
            self._publish_movement_direction(
                target_north_m,
                target_east_m,
            )
            yaw_error_deg = abs(
                self._normalize_yaw_deg(
                    target_yaw_deg - self.latest_yaw_deg
                )
            )
            if yaw_error_deg <= tolerance_deg:
                return target_yaw_deg
            await asyncio.sleep(0.1)

        self.get_logger().warning(
            "진행방향 Yaw 정렬 제한시간 초과: "
            f"target={target_yaw_deg:.1f}°, "
            f"current={self.latest_yaw_deg:.1f}°; 이동은 계속합니다."
        )
        return target_yaw_deg

'''

    marker = re.search(
        r"^    def _publish_movement_direction\(",
        text,
        flags=re.MULTILINE,
    )
    if not marker:
        fail("_publish_movement_direction() 삽입 위치를 찾지 못했습니다.")
    return text[:marker.start()] + helper_block + text[marker.start():]


def patch_go_to_setpoint(text: str) -> str:
    start, end = find_indented_block(
        text,
        r"^    async def _go_to_setpoint\(\n",
    )
    block = text[start:end]
    if f"{PATCH_MARKER} primary-target" in block:
        return text

    needle = "        commanded_down_m = down_m\n"
    if needle not in block:
        fail("_go_to_setpoint()의 commanded_down_m 초기화를 찾지 못했습니다.")
    alignment = f'''        commanded_down_m = down_m
        # {PATCH_MARKER} primary-target
        # 목표 XY 방향으로 먼저 회전한 뒤 같은 Yaw로 이동한다.
        yaw_deg = await self._align_yaw_to_target(
            north_m,
            east_m,
            yaw_deg,
        )
'''
    block = block.replace(needle, alignment, 1)

    # 수평 회피가 끝난 뒤에는 현재 우회점 위치에서 원래 Waypoint 방향을
    # 다시 계산해야 한다. 그렇지 않으면 우회 전의 Yaw가 남을 수 있다.
    post_avoidance_pattern = re.compile(
        r"(?P<prefix>^[ \t]*deadline \+= time\.monotonic\(\) - avoidance_started\n"
        r"[ \t]*self\._publish_movement_direction\(north_m, east_m\)\n)"
        r"(?P<setpoint>[ \t]*await self\.drone\.offboard\.set_position_ned\()",
        flags=re.MULTILINE,
    )
    match = post_avoidance_pattern.search(block)
    if not match:
        fail("회피 후 원래 Waypoint 재명령 위치를 찾지 못했습니다.")
    inserted = (
        match.group("prefix")
        + f"                # {PATCH_MARKER} resume-target\n"
        + "                yaw_deg = await self._align_yaw_to_target(\n"
        + "                    north_m,\n"
        + "                    east_m,\n"
        + "                    yaw_deg,\n"
        + "                )\n"
        + match.group("setpoint")
    )
    block = block[:match.start()] + inserted + block[match.end():]

    return text[:start] + block + text[end:]


def patch_horizontal_avoidance(text: str) -> str:
    start, end = find_indented_block(
        text,
        r"^    async def _perform_horizontal_avoidance\(\n",
    )
    block = text[start:end]
    if f"{PATCH_MARKER} detour-target" in block:
        return text

    pattern = re.compile(
        r"(?P<indent>^[ \t]*)await self\.drone\.offboard\.set_position_ned\(\n"
        r"(?P=indent)    PositionNedYaw\(\n"
        r"(?P=indent)        detour_north,\n"
        r"(?P=indent)        detour_east,\n"
        r"(?P=indent)        detour_down,\n"
        r"(?P=indent)        yaw_deg,\n"
        r"(?P=indent)    \)\n"
        r"(?P=indent)\)",
        flags=re.MULTILINE,
    )
    match = pattern.search(block)
    if not match:
        fail("수평 회피 detour PositionNedYaw 명령을 찾지 못했습니다.")

    indent = match.group("indent")
    replacement = f'''{indent}# {PATCH_MARKER} detour-target
{indent}# 임시 우회점으로 이동할 때도 기체와 카메라가 우회 진행방향을 본다.
{indent}detour_yaw_deg = await self._align_yaw_to_target(
{indent}    detour_north,
{indent}    detour_east,
{indent}    yaw_deg,
{indent})
{indent}self._publish_movement_direction(detour_north, detour_east)
{indent}await asyncio.sleep(0.25)
{indent}if self.obstacle_blocked:
{indent}    self.get_logger().warning(
{indent}        "Yaw 정렬 후 우회 방향이 차단되어 다른 경로를 재검사합니다."
{indent}    )
{indent}    continue
{indent}await self.drone.offboard.set_position_ned(
{indent}    PositionNedYaw(
{indent}        detour_north,
{indent}        detour_east,
{indent}        detour_down,
{indent}        detour_yaw_deg,
{indent}    )
{indent})'''
    block = block[:match.start()] + replacement + block[match.end():]
    return text[:start] + block + text[end:]


def patch_file(path: Path) -> None:
    if not path.is_file():
        fail(f"제어 파일을 찾지 못했습니다: {path}")

    original = path.read_text(encoding="utf-8")
    if PATCH_MARKER in original:
        print(f"[SKIP] 진행방향 Yaw 패치가 이미 적용되어 있습니다: {path}")
        return

    backup = path.with_suffix(path.suffix + ".before_direction_yaw.bak")
    shutil.copy2(path, backup)
    print(f"[BACKUP] {backup}")

    updated = original
    updated = insert_parameters(updated)
    updated = insert_helpers(updated)
    updated = patch_go_to_setpoint(updated)
    updated = patch_horizontal_avoidance(updated)

    # 파일을 덮어쓰기 전에 최소한의 Python 문법 검사를 수행한다.
    compile(updated, str(path), "exec")
    path.write_text(updated, encoding="utf-8")
    print(f"[OK] 진행방향 Yaw 패치 적용: {path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "controller_path",
        nargs="?",
        default=(
            "~/b3_cobot3_ws/src/forest_rescue_system/"
            "forest_rescue_system/drone_controller_node.py"
        ),
    )
    args = parser.parse_args()

    try:
        patch_file(Path(args.controller_path).expanduser().resolve())
    except Exception as error:  # noqa: BLE001 - 설치 스크립트의 오류를 명확히 출력
        print(f"[ERROR] {error}", file=sys.stderr)
        sys.exit(1)
