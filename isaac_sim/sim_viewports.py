#!/usr/bin/env python3
"""센서 Viewport 도킹과 메인 3인칭 추적 카메라 관리."""

import carb
import numpy as np
import omni.usd
from isaacsim.core.utils.viewports import set_camera_view
from pxr import Gf, Sdf, Usd, UsdGeom

from sim_config import (
    CAMERA_PRIM_PATHS,
    DRONE_CONFIGS,
    FOLLOW_CAMERA_BACK_DISTANCE_M,
    FOLLOW_CAMERA_DIRECTION_SMOOTHING,
    FOLLOW_CAMERA_HEIGHT_M,
    FOLLOW_CAMERA_LOOK_AHEAD_M,
    FOLLOW_CAMERA_MIN_MOVEMENT_M,
    FOLLOW_CAMERA_PRIM_PATH,
    FOLLOW_CAMERA_TARGET_HEIGHT_M,
    FOLLOW_DRONE_PRIM_PATH,
    FREE_CAMERA_PRIM_PATH,
    PERSON_CAMERA_BACK_DISTANCE_M,
    PERSON_CAMERA_HEIGHT_M,
    PERSON_CAMERA_SIDE_DISTANCE_M,
    RESCUER_FOLLOW_PRIM_PATH,
    VICTIM_FOLLOW_PRIM_PATH,
)


class ViewportManager:
    """오른쪽 N대 센서 화면과 왼쪽 추적 화면을 독립적으로 관리한다."""

    def __init__(self, simulation_app):
        self.simulation_app = simulation_app

        # 왼쪽 메인 Viewport용 추적 카메라 상태다.
        self._follow_viewport_api = None
        self._follow_camera_ready = False
        self._follow_previous_position = None
        self._follow_direction_xy = None
        self._view_mode = "drone"
        self._person_follow_prim_path = None
        self._person_follow_label = None

        # 실행 중인 드론만 추적 후보로 등록한다.
        self._follow_drone_prim_paths = [
            f"{prim_path}/body"
            for prim_path, _, _ in DRONE_CONFIGS
        ]
        if FOLLOW_DRONE_PRIM_PATH in self._follow_drone_prim_paths:
            self._follow_drone_index = self._follow_drone_prim_paths.index(
                FOLLOW_DRONE_PRIM_PATH
            )
        else:
            self._follow_drone_index = 0
        self._follow_drone_prim_path = (
            self._follow_drone_prim_paths[self._follow_drone_index]
            if self._follow_drone_prim_paths
            else FOLLOW_DRONE_PRIM_PATH
        )

        # 숫자키 0~4/8/9와 F키 입력 구독 상태다.
        self._input_interface = None
        self._keyboard = None
        self._keyboard_subscription = None

    def create_docked_camera_viewports(self):
        """ROS2CameraGraph의 실제 센서 Viewport를 드론 수에 맞춰 배치한다.

        배치 구조는 오른쪽 영역의 상단에 Camera 01, 하단을 좌우로
        나눠 Camera 02와 Camera 03을 두는 형태다. 별도 Viewport를
        만들지 않고 ROS RGB/Depth 발행에 사용되는 640x480 Viewport
        자체를 도킹하므로 RViz 영상과 같은 카메라·화각을 사용한다.
        """
        try:
            import omni.ui as ui
            from omni.kit.viewport.utility import (
                get_active_viewport_window,
                get_viewport_from_window_name,
            )

            main_window = get_active_viewport_window(
                window_name="Viewport"
            )
            if main_window is None:
                raise RuntimeError("기본 Viewport 창을 찾지 못했습니다.")

            main_title = getattr(main_window, "title", "Viewport")
            main_window_handle = ui.Workspace.get_window(main_title)
            if main_window_handle is None:
                main_window_handle = ui.Workspace.get_window("Viewport")
            if main_window_handle is None:
                raise RuntimeError(
                    "기본 Viewport WindowHandle을 찾지 못했습니다."
                )

            sensor_viewport_specs = [
                (f"/quadrotor_{index:02d}/Camera", camera_path)
                for index, camera_path in enumerate(
                    CAMERA_PRIM_PATHS,
                    start=1,
                )
            ]
            if not sensor_viewport_specs:
                raise RuntimeError("도킹할 센서 Viewport 설정이 없습니다.")

            # ROS2CameraGraph가 센서 Viewport와 WindowHandle을 모두
            # 생성할 때까지 기다린다. 새 Viewport는 만들지 않는다.
            sensor_viewports = {}
            sensor_window_handles = {}
            for _ in range(120):
                self.simulation_app.update()
                sensor_viewports = {
                    title: get_viewport_from_window_name(title)
                    for title, _ in sensor_viewport_specs
                }
                sensor_window_handles = {
                    title: ui.Workspace.get_window(title)
                    for title, _ in sensor_viewport_specs
                }
                if all(
                    viewport is not None
                    for viewport in sensor_viewports.values()
                ) and all(
                    handle is not None
                    for handle in sensor_window_handles.values()
                ):
                    break

            missing = [
                title
                for title, viewport in sensor_viewports.items()
                if viewport is None
            ]
            if missing:
                raise RuntimeError(
                    "ROS 센서 Viewport를 찾지 못했습니다: "
                    f"{missing}"
                )
            missing_handles = [
                title
                for title, handle in sensor_window_handles.items()
                if handle is None
            ]
            if missing_handles:
                raise RuntimeError(
                    "ROS 센서 WindowHandle을 찾지 못했습니다: "
                    f"{missing_handles}"
                )

            # Viewport 메뉴의 Camera Light와 같은 RTX 조명 모드를 켠다.
            carb.settings.get_settings().set_bool(
                "/rtx/useViewLightingMode",
                True,
            )

            # 창이 먼저 만들어지면 잠시 /OmniverseKit_Persp를 가리킬 수
            # 있다. 이 경우 실패시키지 않고 해당 ROS Render Product를
            # 실제 드론 카메라에 명시적으로 연결한다.
            for title, expected_camera_path in sensor_viewport_specs:
                viewport_api = sensor_viewports[title]
                actual_camera_path = str(viewport_api.camera_path)
                if actual_camera_path != expected_camera_path:
                    print(
                        f"[INFO] Sensor Viewport camera binding: {title}, "
                        f"from={actual_camera_path}, "
                        f"to={expected_camera_path}"
                    )
                    viewport_api.camera_path = expected_camera_path

                for _ in range(30):
                    self.simulation_app.update()
                    actual_camera_path = str(viewport_api.camera_path)
                    if actual_camera_path == expected_camera_path:
                        break
                if actual_camera_path != expected_camera_path:
                    raise RuntimeError(
                        f"{title} Camera 연결 실패: "
                        f"expected={expected_camera_path}, "
                        f"actual={actual_camera_path}"
                    )

                self._enable_viewport_camera_light(viewport_api, title)
                print(
                    f"[CHECK] Sensor Viewport {title}: "
                    f"camera={actual_camera_path}, "
                    f"resolution={viewport_api.resolution}"
                )

            sensor_windows = [
                sensor_window_handles[title]
                for title, _ in sensor_viewport_specs
            ]
            for window in sensor_windows:
                window.visible = True

            # 첫 센서 화면은 메인 화면 오른쪽에 배치한다.
            sensor_windows[0].dock_in(
                main_window_handle,
                ui.DockPosition.RIGHT,
                0.38,
            )
            self.simulation_app.update()

            # 2대 이상이면 두 번째 화면을 첫 화면 아래에 둔다.
            if len(sensor_windows) >= 2:
                sensor_windows[1].dock_in(
                    sensor_windows[0],
                    ui.DockPosition.BOTTOM,
                    0.50,
                )
                self.simulation_app.update()

            # 3대면 기존 V1처럼 하단 오른쪽에 세 번째 화면을 둔다.
            if len(sensor_windows) >= 3:
                sensor_windows[2].dock_in(
                    sensor_windows[1],
                    ui.DockPosition.RIGHT,
                    0.50,
                )
                self.simulation_app.update()

            # 4대면 상단 오른쪽에 네 번째 화면을 추가해 2x2로 만든다.
            if len(sensor_windows) >= 4:
                sensor_windows[3].dock_in(
                    sensor_windows[0],
                    ui.DockPosition.RIGHT,
                    0.50,
                )
                self.simulation_app.update()

            for _ in range(3):
                self.simulation_app.update()

            dock_states = {
                title: bool(sensor_window_handles[title].docked)
                for title, _ in sensor_viewport_specs
            }
            if not all(dock_states.values()):
                raise RuntimeError(
                    f"WindowHandle 도킹 상태 확인 실패: {dock_states}"
                )

            print(
                "[INFO] ROS sensor Viewports docked: "
                f"count={len(sensor_windows)}, titles="
                f"{[title for title, _ in sensor_viewport_specs]}"
            )
        except Exception as error:
            carb.log_error(
                "ROS 센서 Viewport 자동 배치 실패. "
                f"기본 시뮬레이션은 계속 실행합니다: {error}"
            )

    def setup_follow_viewport(self):
        """왼쪽 메인 Viewport를 특정 드론을 따라가는 카메라에 연결한다."""
        try:
            from omni.kit.viewport.utility import (
                get_viewport_from_window_name,
            )

            stage = omni.usd.get_context().get_stage()
            target_prim = stage.GetPrimAtPath(self._follow_drone_prim_path)
            if not target_prim.IsValid():
                raise RuntimeError(
                    "추적 대상 드론 Prim을 찾지 못했습니다: "
                    f"{self._follow_drone_prim_path}"
                )

            # 센서 카메라와 분리된 Viewport 전용 Camera Prim이다.
            follow_camera = UsdGeom.Camera.Define(
                stage,
                FOLLOW_CAMERA_PRIM_PATH,
            )
            follow_camera.GetFocalLengthAttr().Set(24.0)

            main_viewport = get_viewport_from_window_name("Viewport")
            if main_viewport is None:
                raise RuntimeError("왼쪽 메인 Viewport를 찾지 못했습니다.")

            # 오른쪽 센서 Viewport에는 영향을 주지 않고 메인 Viewport만 연결한다.
            main_viewport.camera_path = FOLLOW_CAMERA_PRIM_PATH
            self._follow_viewport_api = main_viewport
            self._follow_camera_ready = True

            # 첫 프레임부터 드론이 화면 중앙에 보이도록 즉시 위치를 맞춘다.
            self.update_follow_viewport()
            self._setup_follow_keyboard_shortcuts()

            print(
                "[INFO] Main Viewport forward follow camera enabled: "
                f"target={self._follow_drone_prim_path}, "
                f"camera={FOLLOW_CAMERA_PRIM_PATH}, "
                f"back_distance={FOLLOW_CAMERA_BACK_DISTANCE_M:.1f}m, "
                f"look_ahead={FOLLOW_CAMERA_LOOK_AHEAD_M:.1f}m"
            )
        except Exception as error:
            self._follow_viewport_api = None
            self._follow_camera_ready = False
            carb.log_error(
                "메인 Viewport 추적 카메라 설정 실패. "
                f"기본 Viewport를 유지합니다: {error}"
            )

    def _setup_follow_keyboard_shortcuts(self):
        """숫자키로 자유 시점·드론·사람 추적 대상을 바꿀 수 있게 한다."""
        if self._keyboard_subscription is not None:
            return

        try:
            import carb.input
            import omni.appwindow

            app_window = omni.appwindow.get_default_app_window()
            if app_window is None:
                raise RuntimeError("기본 AppWindow를 찾지 못했습니다.")

            self._keyboard = app_window.get_keyboard()
            self._input_interface = carb.input.acquire_input_interface()
            self._keyboard_subscription = (
                self._input_interface.subscribe_to_keyboard_events(
                    self._keyboard,
                    self._on_follow_keyboard_event,
                )
            )

            shortcuts = ", ".join(
                f"{index + 1}=quadrotor_{index + 1:02d}"
                for index in range(len(self._follow_drone_prim_paths))
            )
            print(
                "[FOLLOW VIEWPORT] 단축키 준비: "
                f"0=자유 시점; {shortcuts}; "
                "8=조난자 상공; 9=구조자 상공; F=다음 드론"
            )
        except Exception as error:
            self._input_interface = None
            self._keyboard = None
            self._keyboard_subscription = None
            carb.log_error(
                "메인 추적 Viewport 키보드 단축키 등록 실패: "
                f"{error}"
            )

    def _on_follow_keyboard_event(self, event):
        """키 입력을 드론 인덱스로 변환한다."""
        try:
            import carb.input

            if event.type != carb.input.KeyboardEventType.KEY_PRESS:
                return False

            key_to_index = {
                carb.input.KeyboardInput.KEY_1: 0,
                carb.input.KeyboardInput.KEY_2: 1,
                carb.input.KeyboardInput.KEY_3: 2,
                carb.input.KeyboardInput.KEY_4: 3,
                carb.input.KeyboardInput.NUMPAD_1: 0,
                carb.input.KeyboardInput.NUMPAD_2: 1,
                carb.input.KeyboardInput.NUMPAD_3: 2,
                carb.input.KeyboardInput.NUMPAD_4: 3,
            }

            if event.input in key_to_index:
                self.set_follow_drone(key_to_index[event.input])
                return True

            if event.input in (
                carb.input.KeyboardInput.KEY_0,
                carb.input.KeyboardInput.NUMPAD_0,
            ):
                self.release_follow_camera()
                return True

            if event.input in (
                carb.input.KeyboardInput.KEY_8,
                carb.input.KeyboardInput.NUMPAD_8,
            ):
                self.set_follow_person(
                    VICTIM_FOLLOW_PRIM_PATH,
                    "조난자",
                )
                return True

            if event.input in (
                carb.input.KeyboardInput.KEY_9,
                carb.input.KeyboardInput.NUMPAD_9,
            ):
                self.set_follow_person(
                    RESCUER_FOLLOW_PRIM_PATH,
                    "구조자",
                )
                return True

            if event.input == carb.input.KeyboardInput.F:
                if self._follow_drone_prim_paths:
                    next_index = (
                        self._follow_drone_index + 1
                    ) % len(self._follow_drone_prim_paths)
                    self.set_follow_drone(next_index)
                return True
        except Exception as error:
            carb.log_error(f"추적 드론 단축키 처리 실패: {error}")
        return False

    def set_follow_drone(self, drone_index):
        """왼쪽 메인 Viewport가 추적할 드론을 실행 중에 변경한다."""
        if not self._follow_drone_prim_paths:
            carb.log_warn("추적할 드론이 없습니다.")
            return False

        try:
            index = int(drone_index)
        except (TypeError, ValueError):
            carb.log_warn(f"잘못된 추적 드론 인덱스: {drone_index!r}")
            return False

        if not 0 <= index < len(self._follow_drone_prim_paths):
            carb.log_warn(
                "현재 생성되지 않은 드론입니다: "
                f"requested={index + 1}, "
                f"available=1~{len(self._follow_drone_prim_paths)}"
            )
            return False

        target_path = self._follow_drone_prim_paths[index]
        stage = omni.usd.get_context().get_stage()
        target_prim = stage.GetPrimAtPath(target_path)
        if not target_prim.IsValid():
            carb.log_warn(
                f"추적 대상 드론 Prim을 찾지 못했습니다: {target_path}"
            )
            return False

        self._activate_follow_camera()
        self._view_mode = "drone"
        self._person_follow_prim_path = None
        self._person_follow_label = None

        if (
            index == self._follow_drone_index
            and target_path == self._follow_drone_prim_path
        ):
            self.update_follow_viewport()
            return True

        self._follow_drone_index = index
        self._follow_drone_prim_path = target_path

        # 이전 드론의 위치와 진행방향을 새 드론에 섞지 않는다.
        self._follow_previous_position = None
        self._follow_direction_xy = None

        if self._follow_camera_ready:
            self.update_follow_viewport()

        print(
            "[FOLLOW VIEWPORT] 추적 대상 변경: "
            f"quadrotor_{index + 1:02d} "
            f"({self._follow_drone_prim_path})"
        )
        return True

    def _activate_follow_camera(self):
        """잠금 추적 모드에서 사용하는 전용 카메라를 다시 연결한다."""
        if self._follow_viewport_api is not None:
            self._follow_viewport_api.camera_path = FOLLOW_CAMERA_PRIM_PATH

    def release_follow_camera(self):
        """0번 키: 메인 Viewport를 마우스로 조작 가능한 자유 시점으로 돌린다."""
        if self._follow_viewport_api is None:
            return False

        self._view_mode = "free"
        self._person_follow_prim_path = None
        self._person_follow_label = None
        self._follow_viewport_api.camera_path = FREE_CAMERA_PRIM_PATH
        print(
            "[FOLLOW VIEWPORT] 잠금 해제: 자유 시점 "
            f"({FREE_CAMERA_PRIM_PATH})"
        )
        return True

    def set_follow_person(self, target_path, label):
        """8/9번 키: 조난자 또는 구조자를 상공에서 계속 내려다본다."""
        stage = omni.usd.get_context().get_stage()
        target_prim = stage.GetPrimAtPath(target_path)
        if not target_prim.IsValid():
            carb.log_warn(
                f"{label} 추적 Prim을 찾지 못했습니다: {target_path}. "
                "rescue_search 모드인지 확인하세요."
            )
            return False

        self._activate_follow_camera()
        self._view_mode = "person"
        self._person_follow_prim_path = target_path
        self._person_follow_label = label
        self._follow_previous_position = None
        self._follow_direction_xy = None
        self.update_follow_viewport()
        print(
            f"[FOLLOW VIEWPORT] {label} 상공 시점: {target_path}"
        )
        return True

    def shutdown(self):
        """키보드 이벤트 구독을 해제한다."""
        if (
            self._input_interface is not None
            and self._keyboard is not None
            and self._keyboard_subscription is not None
        ):
            try:
                self._input_interface.unsubscribe_to_keyboard_events(
                    self._keyboard,
                    self._keyboard_subscription,
                )
            except Exception as error:
                carb.log_warn(
                    "추적 Viewport 키보드 구독 해제 실패: "
                    f"{error}"
                )

        self._keyboard_subscription = None
        self._keyboard = None
        self._input_interface = None

    def update_follow_viewport(self):
        """드론 뒤에서 실제 진행방향 앞쪽을 바라보도록 카메라를 갱신한다."""
        if not self._follow_camera_ready:
            return
        if self._view_mode == "free":
            return
        if self._view_mode == "person":
            self._update_person_follow_viewport()
            return

        stage = omni.usd.get_context().get_stage()
        target_prim = stage.GetPrimAtPath(self._follow_drone_prim_path)
        if not target_prim.IsValid():
            return

        # 동적 Prim의 최신 World Transform을 읽기 위해 매 호출마다
        # 새로운 XformCache를 만든다.
        xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        world_matrix = xform_cache.GetLocalToWorldTransform(target_prim)
        drone_translation = world_matrix.ExtractTranslation()
        drone_position = np.array(
            [
                float(drone_translation[0]),
                float(drone_translation[1]),
                float(drone_translation[2]),
            ],
            dtype=np.float64,
        )

        measured_direction_xy = None

        # 이전 프레임과 현재 프레임의 XY 위치 차이로 실제 진행방향을 구한다.
        if self._follow_previous_position is not None:
            movement_xy = (
                drone_position[:2]
                - self._follow_previous_position[:2]
            )
            movement_distance = float(np.linalg.norm(movement_xy))
            if movement_distance >= FOLLOW_CAMERA_MIN_MOVEMENT_M:
                measured_direction_xy = movement_xy / movement_distance

        # 아직 이동하지 않았거나 위치 변화가 너무 작으면 드론 body의
        # 로컬 +X축을 World 좌표로 변환해 전방 방향으로 사용한다.
        if measured_direction_xy is None:
            body_forward_world = world_matrix.TransformDir(
                Gf.Vec3d(1.0, 0.0, 0.0)
            )
            body_forward_xy = np.array(
                [
                    float(body_forward_world[0]),
                    float(body_forward_world[1]),
                ],
                dtype=np.float64,
            )
            body_forward_norm = float(np.linalg.norm(body_forward_xy))
            if body_forward_norm > 1.0e-6:
                measured_direction_xy = (
                    body_forward_xy / body_forward_norm
                )
            else:
                measured_direction_xy = np.array(
                    [1.0, 0.0],
                    dtype=np.float64,
                )

        # 방향이 갑자기 바뀔 때 Viewport가 급회전하지 않도록 이전 방향과
        # 새 방향을 보간한 뒤 다시 단위 벡터로 정규화한다.
        if self._follow_direction_xy is None:
            smoothed_direction_xy = measured_direction_xy
        else:
            alpha = FOLLOW_CAMERA_DIRECTION_SMOOTHING
            smoothed_direction_xy = (
                (1.0 - alpha) * self._follow_direction_xy
                + alpha * measured_direction_xy
            )
            smoothed_norm = float(np.linalg.norm(smoothed_direction_xy))
            if smoothed_norm > 1.0e-6:
                smoothed_direction_xy /= smoothed_norm
            else:
                smoothed_direction_xy = measured_direction_xy

        self._follow_direction_xy = smoothed_direction_xy
        self._follow_previous_position = drone_position.copy()

        # 카메라는 드론의 진행방향 반대쪽에 놓고, 드론보다 앞쪽의
        # LOOK_AHEAD 지점을 바라본다.
        eye = np.array(
            [
                drone_position[0]
                - smoothed_direction_xy[0]
                * FOLLOW_CAMERA_BACK_DISTANCE_M,
                drone_position[1]
                - smoothed_direction_xy[1]
                * FOLLOW_CAMERA_BACK_DISTANCE_M,
                drone_position[2] + FOLLOW_CAMERA_HEIGHT_M,
            ],
            dtype=np.float64,
        )
        target = np.array(
            [
                drone_position[0]
                + smoothed_direction_xy[0]
                * FOLLOW_CAMERA_LOOK_AHEAD_M,
                drone_position[1]
                + smoothed_direction_xy[1]
                * FOLLOW_CAMERA_LOOK_AHEAD_M,
                drone_position[2]
                + FOLLOW_CAMERA_TARGET_HEIGHT_M,
            ],
            dtype=np.float64,
        )

        set_camera_view(
            eye=eye,
            target=target,
            camera_prim_path=FOLLOW_CAMERA_PRIM_PATH,
            viewport_api=self._follow_viewport_api,
        )

    def _update_person_follow_viewport(self):
        """사람을 중심에 두고 주변 지형도 보이는 사선 상공 시점을 갱신한다."""
        if not self._person_follow_prim_path:
            return

        stage = omni.usd.get_context().get_stage()
        target_prim = stage.GetPrimAtPath(self._person_follow_prim_path)
        if not target_prim.IsValid():
            return

        xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        world_matrix = xform_cache.GetLocalToWorldTransform(target_prim)
        translation = world_matrix.ExtractTranslation()
        person_position = np.array(
            [
                float(translation[0]),
                float(translation[1]),
                float(translation[2]),
            ],
            dtype=np.float64,
        )
        eye = np.array(
            [
                person_position[0] - PERSON_CAMERA_BACK_DISTANCE_M,
                person_position[1] - PERSON_CAMERA_SIDE_DISTANCE_M,
                person_position[2] + PERSON_CAMERA_HEIGHT_M,
            ],
            dtype=np.float64,
        )

        set_camera_view(
            eye=eye,
            target=person_position,
            camera_prim_path=FOLLOW_CAMERA_PRIM_PATH,
            viewport_api=self._follow_viewport_api,
        )

    @staticmethod
    def _enable_viewport_camera_light(viewport_api, viewport_title):
        """센서 Viewport Render Product의 Camera Light를 활성화한다."""
        stage = omni.usd.get_context().get_stage()
        render_product_path = str(
            viewport_api.get_render_product_path()
        )
        render_product_prim = stage.GetPrimAtPath(render_product_path)
        if not render_product_prim.IsValid():
            raise RuntimeError(
                f"{viewport_title} Render Product를 찾지 못했습니다: "
                f"{render_product_path}"
            )

        attribute = render_product_prim.GetAttribute(
            "omni:rtx:scene:useViewLightingMode"
        )
        if not attribute.IsValid():
            attribute = render_product_prim.CreateAttribute(
                "omni:rtx:scene:useViewLightingMode",
                Sdf.ValueTypeNames.Bool,
                custom=True,
            )
        attribute.Set(True)
        print(
            f"[CHECK] Camera Light ON: {viewport_title}, "
            f"render_product={render_product_path}"
        )
