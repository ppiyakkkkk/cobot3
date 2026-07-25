#!/usr/bin/env python3
"""rescue_search 전용 조난자·구조자 생성과 구조자 경로 추종."""

import math
import time

import carb
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path as PathMessage
import numpy as np
import omni.physx
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from pegasus.simulator.logic.people.person import Person

from sim_config import (
    FOR_TEST_VICTIM_SPAWN_ENABLED,
    FOR_TEST_VICTIM_WORLD_XYZ,
    PERSON_COLLIDER_CYLINDER_HEIGHT_M,
    PERSON_COLLIDER_RADIUS_M,
    PERSON_COLLIDER_TOTAL_HEIGHT_M,
    PERSON_GROUND_CLEARANCE_M,
    PERSON_CLOTHING_EXCLUDE_KEYWORDS,
    PERSON_CLOTHING_FALLBACK_TO_NON_SKIN_PARTS,
    PERSON_CLOTHING_INCLUDE_KEYWORDS,
    PERSON_ROLE_CLOTHING_COLOR_ENABLED,
    RESCUER_CHARACTER_KEYWORDS,
    RESCUER_MOVE_SPEED_M_S,
    RESCUER_PATH_TOPIC,
    RESCUER_POSE_PUBLISH_PERIOD_SEC,
    RESCUER_POSITION_TOPIC,
    RESCUER_STATUS_TOPIC,
    RESCUER_WAYPOINT_TOLERANCE_M,
    RESCUER_XY,
    RESCUER_CLOTHING_COLOR_RGB,
    VICTIM_CLOTHING_COLOR_RGB,
    VICTIM_PREFERRED_CHARACTER,
    VICTIM_SPAWN_POSITIONS,
)
from sim_utils import write_ground_truth


class PeopleManager:
    """두 사람과 충돌 프록시, 구조자 ROS Path 추종을 관리한다."""

    def __init__(self, terrain, rng, test_victim_spawn_world_enu=None):
        self.terrain = terrain
        self.rng = rng
        self.test_victim_spawn_world_enu = test_victim_spawn_world_enu
        self._person_physics_proxies = {}
        self.victim = None
        self.rescuer = None

        self._ros_node = None
        self._owns_rclpy_context = False
        self._rescuer_path = []
        self._rescuer_waypoint_index = 0
        self._rescuer_status = "NOT_INITIALIZED"
        self._last_pose_publish_at = float("-inf")
        self._last_navigation_log_sim_time = float("-inf")
        self._last_navigation_sim_time = None
        self._last_ground_z = None
        self._last_z_correction = 0.0
        self._last_move_command_applied = False
        self._last_ground_loss_log_at = float("-inf")
        self._last_large_ground_error_log_sim_time = float("-inf")
        self._ground_query_error_logged = False
        self._rescuer_xform_ops_logged = False
        self._rescuer_foot_offset_m = float(PERSON_GROUND_CLEARANCE_M)
        self._terrain_prim_path = str(self.terrain._terrain_prim.GetPath())
        self._timeline = omni.timeline.get_timeline_interface()
        self._scene_query = omni.physx.get_physx_scene_query_interface()
        self._path_subscription = None
        self._pose_publisher = None
        self._status_publisher = None

    @staticmethod
    def _select_people_assets(available_assets):
        """가능한 경우 조난자와 구조자에 서로 다른 모델을 선택한다."""
        available_assets = [str(value) for value in available_assets]
        if not available_assets:
            raise RuntimeError("No Pegasus person assets were found.")

        if VICTIM_PREFERRED_CHARACTER in available_assets:
            victim_asset = VICTIM_PREFERRED_CHARACTER
        else:
            victim_asset = available_assets[0]
            carb.log_warn(
                "Preferred victim asset was not found. "
                f"Using {victim_asset} instead."
            )

        candidates = [item for item in available_assets if item != victim_asset]
        if not candidates:
            carb.log_warn(
                "구조자에 사용할 다른 Character asset이 없어 조난자 모델을 "
                "같이 사용합니다."
            )
            return victim_asset, victim_asset

        keywords = tuple(str(value).lower() for value in RESCUER_CHARACTER_KEYWORDS)

        def score(asset_name):
            lowered = asset_name.lower()
            keyword_score = 0
            for index, keyword in enumerate(keywords):
                if keyword in lowered:
                    keyword_score = max(keyword_score, len(keywords) - index)
            return keyword_score

        rescuer_asset = max(candidates, key=lambda item: (score(item), item))
        return victim_asset, rescuer_asset

    @staticmethod
    def _normalize_material_name(value):
        """Prim/Material 이름을 키워드 비교용 소문자 문자열로 바꾼다."""
        return "".join(
            character.lower()
            for character in str(value)
            if character.isalnum()
        )

    @staticmethod
    def _bound_material_path(prim):
        """Prim에 현재 바인딩된 Material 경로를 반환한다."""
        try:
            material, _ = UsdShade.MaterialBindingAPI(
                prim
            ).ComputeBoundMaterial()
        except Exception:
            return ""

        if not material:
            return ""
        material_prim = material.GetPrim()
        if not material_prim or not material_prim.IsValid():
            return ""
        return str(material_prim.GetPath())

    @staticmethod
    def _create_role_color_material(stage, material_path, rgb):
        """UsdPreviewSurface 기반 단색 Material을 생성하거나 갱신한다."""
        red, green, blue = [
            min(1.0, max(0.0, float(value)))
            for value in rgb
        ]

        material = UsdShade.Material.Define(stage, material_path)
        shader = UsdShade.Shader.Define(
            stage,
            f"{material_path}/PreviewSurface",
        )
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput(
            "diffuseColor",
            Sdf.ValueTypeNames.Color3f,
        ).Set(Gf.Vec3f(red, green, blue))
        shader.CreateInput(
            "roughness",
            Sdf.ValueTypeNames.Float,
        ).Set(0.65)
        shader.CreateInput(
            "metallic",
            Sdf.ValueTypeNames.Float,
        ).Set(0.0)
        material.CreateSurfaceOutput().ConnectToSource(
            shader.ConnectableAPI(),
            "surface",
        )
        return material

    @staticmethod
    def _bind_role_material(prim, material):
        """기존 하위 Material보다 우선하도록 역할 Material을 바인딩한다."""
        try:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                material,
                UsdShade.Tokens.strongerThanDescendants,
            )
            return True
        except Exception:
            try:
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
                return True
            except Exception:
                return False

    def _apply_person_clothing_color(self, person, role_name, rgb):
        """Character의 피부를 제외한 의상 Material을 역할 색상으로 덮는다."""
        if not PERSON_ROLE_CLOTHING_COLOR_ENABLED:
            return

        stage = omni.usd.get_context().get_stage()
        root_path = str(
            getattr(person, "_stage_prefix", "")
            or getattr(person, "character_skel_root_stage_path", "")
        )
        root_prim = stage.GetPrimAtPath(root_path)
        if not root_path or not root_prim.IsValid():
            carb.log_warn(
                f"{role_name} Character Prim을 찾지 못해 의상 색상을 "
                f"적용하지 못했습니다: {root_path!r}"
            )
            return

        role_token = self._normalize_material_name(role_name)
        material_path = (
            f"/World/Looks/ForestRescuePeople/{role_token}_clothing"
        )
        material = self._create_role_color_material(stage, material_path, rgb)

        include_keywords = tuple(
            self._normalize_material_name(value)
            for value in PERSON_CLOTHING_INCLUDE_KEYWORDS
            if str(value).strip()
        )
        exclude_keywords = tuple(
            self._normalize_material_name(value)
            for value in PERSON_CLOTHING_EXCLUDE_KEYWORDS
            if str(value).strip()
        )

        preferred_targets = []
        fallback_targets = []
        for prim in Usd.PrimRange(root_prim):
            if not (prim.IsA(UsdGeom.Subset) or prim.IsA(UsdGeom.Mesh)):
                continue

            description = self._normalize_material_name(
                f"{prim.GetPath()} {self._bound_material_path(prim)}"
            )
            if any(keyword in description for keyword in exclude_keywords):
                continue

            if any(keyword in description for keyword in include_keywords):
                preferred_targets.append(prim)
            else:
                fallback_targets.append(prim)

        targets = preferred_targets
        selection_mode = "clothing_keyword"
        if not targets and PERSON_CLOTHING_FALLBACK_TO_NON_SKIN_PARTS:
            subset_targets = [
                prim for prim in fallback_targets
                if prim.IsA(UsdGeom.Subset)
            ]
            targets = subset_targets or [
                prim for prim in fallback_targets
                if prim.IsA(UsdGeom.Mesh)
            ]
            selection_mode = "non_skin_fallback"

        applied_paths = []
        failed_paths = []
        for prim in targets:
            if self._bind_role_material(prim, material):
                applied_paths.append(str(prim.GetPath()))
            else:
                failed_paths.append(str(prim.GetPath()))

        if not applied_paths:
            carb.log_warn(
                f"{role_name} 의상 색상 적용 대상이 없습니다. "
                f"root={root_path}, candidates={len(targets)}, "
                f"failed={len(failed_paths)}"
            )
            return

        preview = ", ".join(applied_paths[:8])
        if len(applied_paths) > 8:
            preview += f", ...(+{len(applied_paths) - 8})"

        print(
            f"[PERSON COLOR] {role_name}: "
            f"rgb={tuple(float(value) for value in rgb)}, "
            f"mode={selection_mode}, applied={len(applied_paths)}, "
            f"failed={len(failed_paths)}, targets=[{preview}]"
        )

    def spawn_people(self):
        """rescue_search 모드에서 조난자와 구조자를 생성한다."""
        victim_asset, rescuer_asset = self._select_people_assets(
            Person.get_character_asset_list()
        )
        print(f"[PERSON] Victim asset: {victim_asset}")
        print(f"[PERSON] Rescuer asset: {rescuer_asset}")

        if FOR_TEST_VICTIM_SPAWN_ENABLED:
            victim_position = getattr(
                self,
                "test_victim_spawn_world_enu",
                None,
            )
            if victim_position is None:
                raise RuntimeError(
                    "시험용 조난자 위치가 생성되지 않았습니다. "
                    "FOR_TEST_VICTIM_WORLD_XYZ를 확인하세요."
                )
            victim_position = [float(value) for value in victim_position]
            victim_index = -1
            spawn_description = (
                "TEST hardcoded XYZ="
                f"({FOR_TEST_VICTIM_WORLD_XYZ[0]:.1f}, "
                f"{FOR_TEST_VICTIM_WORLD_XYZ[1]:.1f}, "
                f"{FOR_TEST_VICTIM_WORLD_XYZ[2]:.1f})"
            )
        else:
            victim_index = int(
                self.rng.integers(len(VICTIM_SPAWN_POSITIONS))
            )
            victim_candidate = VICTIM_SPAWN_POSITIONS[victim_index]
            victim_x = float(victim_candidate[0])
            victim_y = float(victim_candidate[1])
            victim_ground_z = self.terrain.height(victim_x, victim_y)
            victim_position = [
                victim_x,
                victim_y,
                victim_ground_z + PERSON_GROUND_CLEARANCE_M,
            ]
            spawn_description = f"candidate {victim_index + 1}"

        write_ground_truth(victim_position, victim_index)
        self.victim = Person(
            "victim_01",
            victim_asset,
            init_pos=victim_position,
            init_yaw=0.0,
        )
        self._apply_person_clothing_color(
            self.victim,
            role_name="victim",
            rgb=VICTIM_CLOTHING_COLOR_RGB,
        )
        self._create_person_physics_proxy(
            "victim_01",
            self.victim,
            victim_position,
        )
        print(
            f"[INFO] Spawned victim at {spawn_description}: "
            f"({victim_position[0]:.3f}, "
            f"{victim_position[1]:.3f}, "
            f"{victim_position[2]:.3f})"
        )

        rescuer_x, rescuer_y = RESCUER_XY
        rescuer_ground_z = float(
            self.terrain.height(float(rescuer_x), float(rescuer_y))
        )
        rescuer_position = [
            float(rescuer_x),
            float(rescuer_y),
            rescuer_ground_z + float(PERSON_GROUND_CLEARANCE_M),
        ]
        self.rescuer = Person(
            "rescuer_01",
            rescuer_asset,
            init_pos=rescuer_position,
            init_yaw=0.0,
        )
        self._apply_person_clothing_color(
            self.rescuer,
            role_name="rescuer",
            rgb=RESCUER_CLOTHING_COLOR_RGB,
        )
        self._create_person_physics_proxy(
            "rescuer_01",
            self.rescuer,
            rescuer_position,
        )
        self._log_rescuer_xform_ops_once()
        self._measure_rescuer_foot_offset()
        print(
            "[INFO] Spawned rescuer beside drones at "
            f"({rescuer_position[0]:.3f}, "
            f"{rescuer_position[1]:.3f}, "
            f"{rescuer_position[2]:.3f})"
        )
        self._initialize_ros_bridge()

    def _initialize_ros_bridge(self):
        """ROS Path 수신과 구조자 위치·상태 발행 노드를 준비한다."""
        if self._ros_node is not None:
            return
        if not rclpy.ok():
            rclpy.init(args=[])
            self._owns_rclpy_context = True

        self._ros_node = rclpy.create_node("isaac_rescuer_bridge")
        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._path_subscription = self._ros_node.create_subscription(
            PathMessage,
            RESCUER_PATH_TOPIC,
            self._rescuer_path_callback,
            transient_qos,
        )
        self._pose_publisher = self._ros_node.create_publisher(
            PointStamped,
            RESCUER_POSITION_TOPIC,
            10,
        )
        self._status_publisher = self._ros_node.create_publisher(
            String,
            RESCUER_STATUS_TOPIC,
            transient_qos,
        )
        self._set_rescuer_status("IDLE")
        print(
            "[OK] Isaac 구조자 ROS bridge 준비: "
            f"path={RESCUER_PATH_TOPIC}, pose={RESCUER_POSITION_TOPIC}, "
            f"status={RESCUER_STATUS_TOPIC}"
        )

    def _rescuer_path_callback(self, message):
        if self.rescuer is None:
            return
        if message.header.frame_id and message.header.frame_id != "map":
            carb.log_error(
                "구조자 Path frame이 map이 아닙니다: "
                f"{message.header.frame_id}"
            )
            self._set_rescuer_status("PATH_REJECTED_FRAME")
            return

        waypoints = []
        for pose_stamped in message.poses:
            point = pose_stamped.pose.position
            waypoint = np.asarray(
                # Path의 Z는 보행에 사용하지 않는다. 실제 산 collision
                # 표면 높이는 이동 프레임마다 아래 방향 raycast로 구한다.
                [point.x, point.y, point.z],
                dtype=np.float64,
            )
            if waypoint.shape != (3,) or not np.all(np.isfinite(waypoint)):
                continue
            if waypoints and np.linalg.norm(waypoint - waypoints[-1]) < 0.05:
                continue
            waypoints.append(waypoint)

        if not waypoints:
            carb.log_error("수신한 구조자 Path에 유효한 Waypoint가 없습니다.")
            self._set_rescuer_status("PATH_REJECTED_EMPTY")
            return

        self._rescuer_path = waypoints
        self._rescuer_waypoint_index = 0
        self._set_rescuer_status("PATH_RECEIVED")
        self._skip_reached_waypoints()
        self._send_current_waypoint()
        print(
            "[RESCUER] Path 수신: "
            f"waypoints={len(self._rescuer_path)}, "
            f"speed={RESCUER_MOVE_SPEED_M_S:.1f}m/s"
        )

    def _current_rescuer_position(self):
        if self.rescuer is None:
            return None
        position = np.asarray(self.rescuer.state.position, dtype=np.float64)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            return None
        return position

    def _skip_reached_waypoints(self):
        position = self._current_rescuer_position()
        if position is None:
            return
        tolerance = float(RESCUER_WAYPOINT_TOLERANCE_M)
        while self._rescuer_waypoint_index < len(self._rescuer_path):
            target = self._rescuer_path[self._rescuer_waypoint_index]
            if float(np.linalg.norm(target[:2] - position[:2])) > tolerance:
                break
            self._rescuer_waypoint_index += 1

    def _send_current_waypoint(self):
        if self.rescuer is None or not self._rescuer_path:
            return
        if self._rescuer_waypoint_index >= len(self._rescuer_path):
            current_position = self._current_rescuer_position()
            if current_position is not None:
                self.rescuer.update_target_position(
                    current_position.tolist(),
                    walk_speed=0.0,
                )
            self._set_rescuer_status("ARRIVED")
            print("[RESCUER] 조난자 주변 최종 목표에 도착했습니다.")
            return

        target = self._rescuer_path[self._rescuer_waypoint_index]
        ground_z = self._raycast_terrain_z(float(target[0]), float(target[1]))
        if ground_z is None:
            self._stop_rescuer_for_ground_loss(
                "목표 waypoint 아래에서 산 지형 collision을 찾지 못함"
            )
            return
        target = target.copy()
        target[2] = ground_z + self._rescuer_foot_offset_m
        self.rescuer.update_target_position(
            target.tolist(),
            walk_speed=float(RESCUER_MOVE_SPEED_M_S),
        )
        self._last_move_command_applied = True
        self._set_rescuer_status(
            f"WALKING:{self._rescuer_waypoint_index + 1}/"
            f"{len(self._rescuer_path)}"
        )

    def update_rescuer_navigation(self):
        """매 시뮬레이션 프레임 ROS 콜백과 Waypoint 진행을 갱신한다."""
        if self._ros_node is None or self.rescuer is None:
            return
        rclpy.spin_once(self._ros_node, timeout_sec=0.0)
        self._publish_rescuer_pose_if_due()
        if not self._rescuer_path or self._rescuer_status == "ARRIVED":
            return

        position = self._current_rescuer_position()
        if position is None:
            return

        sim_time = float(self._timeline.get_current_time())
        if self._last_navigation_sim_time is None:
            sim_dt = 0.0
        else:
            sim_dt = max(0.0, sim_time - self._last_navigation_sim_time)
        self._last_navigation_sim_time = sim_time

        target = self._rescuer_path[self._rescuer_waypoint_index]
        distance_xy = float(np.linalg.norm(target[:2] - position[:2]))
        ground_z = self._raycast_terrain_z(
            float(position[0]), float(position[1])
        )
        self._last_move_command_applied = False
        if ground_z is None:
            self._last_ground_z = None
            self._last_z_correction = 0.0
            self._stop_rescuer_for_ground_loss(
                "현재 구조자 XY 아래에서 산 지형 collision을 찾지 못함"
            )
        else:
            self._last_ground_z = ground_z
            self._apply_ground_following(position, ground_z)
            self._refresh_current_ground_target(ground_z)

        diagnostic_position = self._current_rescuer_position()
        if diagnostic_position is None:
            diagnostic_position = position
        self._log_navigation_diagnostics(
            sim_time=sim_time,
            sim_dt=sim_dt,
            position=diagnostic_position,
            target=target,
            distance_xy=distance_xy,
        )

        if ground_z is None:
            return
        if distance_xy <= float(RESCUER_WAYPOINT_TOLERANCE_M):
            self._rescuer_waypoint_index += 1
            self._skip_reached_waypoints()
            self._send_current_waypoint()

    @staticmethod
    def _hit_prim_path(hit):
        """Isaac Sim 버전에 따라 dict 또는 RaycastHit 객체로 오는 hit를 처리한다."""
        for key in ("collision", "rigidBody", "material"):
            if isinstance(hit, dict):
                value = hit.get(key)
            else:
                value = getattr(hit, key, None)
            if value:
                return str(value)
        return ""

    @staticmethod
    def _hit_position(hit):
        """Raycast hit 위치를 dict/바인딩 객체 양쪽에서 안전하게 꺼낸다."""
        if isinstance(hit, dict):
            return hit.get("position")
        return getattr(hit, "position", None)

    def _is_terrain_hit(self, prim_path):
        """산 Terrain prim 및 그 하위 collision만 지면으로 인정한다."""
        terrain = self._terrain_prim_path.rstrip("/")
        candidate = str(prim_path).rstrip("/")
        return (
            candidate == terrain
            or candidate.startswith(f"{terrain}/")
            or terrain.startswith(f"{candidate}/")
        )

    def _raycast_terrain_z(self, x, y):
        """위에서 아래로 모든 hit를 검사해 산 Terrain 표면 Z만 반환한다."""
        ray_top = max(
            float(getattr(self.terrain, "z_max", 100.0)) + 20.0,
            float(self._current_rescuer_position()[2]) + 20.0,
        )
        ray_bottom = float(getattr(self.terrain, "z_min", -100.0)) - 20.0
        hits = []

        def report_hit(hit):
            prim_path = self._hit_prim_path(hit)
            if self._is_terrain_hit(prim_path):
                position = self._hit_position(hit)
                if position is not None:
                    hits.append((float(position[2]), prim_path))
            return True

        try:
            self._scene_query.raycast_all(
                carb.Float3(float(x), float(y), ray_top),
                carb.Float3(0.0, 0.0, -1.0),
                ray_top - ray_bottom,
                report_hit,
            )
        except Exception as error:
            if not self._ground_query_error_logged:
                carb.log_error(
                    "[RESCUER] PhysX 산 지형 raycast 실패: "
                    f"{type(error).__name__}: {error}"
                )
                self._ground_query_error_logged = True
            return None

        if not hits:
            return None
        self._ground_query_error_logged = False
        return max(hits, key=lambda item: item[0])[0]

    def _measure_rescuer_foot_offset(self):
        """Character visual 최저점과 발 기준 state 원점 사이 오프셋을 측정한다."""
        try:
            stage = omni.usd.get_context().get_stage()
            root_path = str(self.rescuer._stage_prefix)
            root_prim = stage.GetPrimAtPath(root_path)
            bbox_cache = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(),
                [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
            )
            world_range = bbox_cache.ComputeWorldBound(root_prim).ComputeAlignedRange()
            visual_min_z = float(world_range.GetMin()[2])
            state_z = float(self.rescuer.state.position[2])
            measured = state_z - visual_min_z
            if math.isfinite(measured) and -1.0 <= measured <= 1.0:
                self._rescuer_foot_offset_m = (
                    measured + float(PERSON_GROUND_CLEARANCE_M)
                )
            print(
                "[RESCUER] 발 기준 측정: "
                f"root={root_path}, state_z={state_z:.3f}, "
                f"visual_min_z={visual_min_z:.3f}, "
                f"foot_offset={self._rescuer_foot_offset_m:.3f}m"
            )
        except Exception as error:
            carb.log_warn(
                "[RESCUER] visual 발바닥 높이 측정 실패, 기본 여유 높이 사용: "
                f"{type(error).__name__}: {error}"
            )

    def _log_rescuer_xform_ops_once(self):
        """구조자 Prim의 기존 xformOp 구성을 변경 없이 한 번만 기록한다."""
        if self._rescuer_xform_ops_logged or self.rescuer is None:
            return
        self._rescuer_xform_ops_logged = True

        root_path = str(self.rescuer._stage_prefix)
        stage = omni.usd.get_context().get_stage()
        root_prim = stage.GetPrimAtPath(root_path)
        if not root_prim.IsValid():
            carb.log_warn(
                f"[RESCUER] Character root Prim을 찾지 못했습니다: {root_path}"
            )
            return

        try:
            xformable = UsdGeom.Xformable(root_prim)
            ordered_ops = xformable.GetOrderedXformOps()
            op_names = [
                str(op.GetAttr().GetName())
                for op in ordered_ops
            ]
            print(
                "[RESCUER] 기존 Character xformOp 유지: "
                f"root={root_path}, ordered_ops={op_names}"
            )
        except Exception as error:
            carb.log_warn(
                "[RESCUER] Character xformOp 확인 실패: "
                f"{type(error).__name__}: {error}"
            )

    def _apply_ground_following(self, position, ground_z):
        """지면 목표 Z를 계산한다. 실제 적용은 Person의 공식 목표 API가 맡는다."""
        desired_z = float(ground_z) + self._rescuer_foot_offset_m
        requested = desired_z - float(position[2])
        self._last_z_correction = requested

        if abs(requested) >= 0.75:
            sim_time = float(self._timeline.get_current_time())
            if (
                sim_time - self._last_large_ground_error_log_sim_time
                >= 1.0
            ):
                carb.log_warn(
                    "[RESCUER] 현재 지면 목표와 Z 차이가 큽니다: "
                    f"current_z={position[2]:.3f}, "
                    f"ground_z={ground_z:.3f}, "
                    f"desired_z={desired_z:.3f}, "
                    f"requested={requested:+.3f}m"
                )
                self._last_large_ground_error_log_sim_time = sim_time

    def _refresh_current_ground_target(self, current_ground_z):
        """raycast 지면 Z를 Person의 기존 XYZ 목표 갱신 경로로 적용한다."""
        if self._rescuer_waypoint_index >= len(self._rescuer_path):
            return
        target = self._rescuer_path[self._rescuer_waypoint_index].copy()
        target[2] = float(current_ground_z) + self._rescuer_foot_offset_m
        self.rescuer.update_target_position(
            target.tolist(),
            walk_speed=float(RESCUER_MOVE_SPEED_M_S),
        )
        self._last_move_command_applied = True
        if self._rescuer_status == "GROUND_LOST":
            self._set_rescuer_status(
                f"WALKING:{self._rescuer_waypoint_index + 1}/"
                f"{len(self._rescuer_path)}"
            )

    def _stop_rescuer_for_ground_loss(self, reason):
        position = self._current_rescuer_position()
        if position is not None:
            self.rescuer.update_target_position(
                position.tolist(),
                walk_speed=0.0,
            )
        self._last_move_command_applied = False
        self._set_rescuer_status("GROUND_LOST")
        now = time.monotonic()
        if now - self._last_ground_loss_log_at >= 1.0:
            carb.log_error(f"[RESCUER] 이동 정지: {reason}")
            self._last_ground_loss_log_at = now

    def _log_navigation_diagnostics(
        self, sim_time, sim_dt, position, target, distance_xy
    ):
        if sim_time - self._last_navigation_log_sim_time < 1.0:
            return
        self._last_navigation_log_sim_time = sim_time
        ground_text = (
            f"{self._last_ground_z:.3f}"
            if self._last_ground_z is not None
            else "NONE"
        )
        print(
            "[RESCUER][DIAG] "
            f"current=({position[0]:.3f},{position[1]:.3f},{position[2]:.3f}), "
            f"target=({target[0]:.3f},{target[1]:.3f},{target[2]:.3f}), "
            f"distance_xy={distance_xy:.3f}m, ground_z={ground_text}, "
            f"z_correction={self._last_z_correction:+.3f}m, "
            f"waypoint={self._rescuer_waypoint_index + 1}/"
            f"{len(self._rescuer_path)}, "
            f"move_command={self._last_move_command_applied}, "
            f"sim_dt={sim_dt:.4f}s"
        )

    def _publish_rescuer_pose_if_due(self):
        now = time.monotonic()
        if now - self._last_pose_publish_at < float(
            RESCUER_POSE_PUBLISH_PERIOD_SEC
        ):
            return
        position = self._current_rescuer_position()
        if position is None:
            return
        message = PointStamped()
        message.header.frame_id = "map"
        message.header.stamp = self._ros_node.get_clock().now().to_msg()
        message.point.x = float(position[0])
        message.point.y = float(position[1])
        message.point.z = float(position[2])
        self._pose_publisher.publish(message)
        self._last_pose_publish_at = now

    def _set_rescuer_status(self, status):
        status = str(status)
        if status == self._rescuer_status:
            return
        self._rescuer_status = status
        if self._status_publisher is not None:
            message = String()
            message.data = status
            self._status_publisher.publish(message)
        print(f"[RESCUER] 상태: {status}")

    def shutdown_ros(self):
        if self._ros_node is not None:
            self._ros_node.destroy_node()
            self._ros_node = None
        if self._owns_rclpy_context and rclpy.ok():
            rclpy.shutdown()
        self._owns_rclpy_context = False

    def _create_person_physics_proxy(
        self,
        person_name,
        person,
        foot_position,
    ):
        """사람을 따라 움직이는 보이지 않는 캡슐형 충돌체를 만든다."""
        stage = omni.usd.get_context().get_stage()
        collider_path = f"/World/person_colliders/{person_name}"

        capsule = UsdGeom.Capsule.Define(stage, collider_path)
        capsule.CreateAxisAttr().Set(UsdGeom.Tokens.z)
        capsule.CreateRadiusAttr().Set(PERSON_COLLIDER_RADIUS_M)
        capsule.CreateHeightAttr().Set(PERSON_COLLIDER_CYLINDER_HEIGHT_M)

        capsule_center = Gf.Vec3d(
            float(foot_position[0]),
            float(foot_position[1]),
            float(foot_position[2])
            + PERSON_COLLIDER_TOTAL_HEIGHT_M * 0.5,
        )
        translate_op = capsule.AddTranslateOp()
        translate_op.Set(capsule_center)

        capsule.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
        collider_prim = capsule.GetPrim()
        UsdPhysics.CollisionAPI.Apply(collider_prim)

        rigid_body = UsdPhysics.RigidBodyAPI.Apply(collider_prim)
        rigid_body.CreateRigidBodyEnabledAttr(True)
        rigid_body.CreateKinematicEnabledAttr(True)

        self._person_physics_proxies[person_name] = {
            "person": person,
            "translate_op": translate_op,
            "collider_path": collider_path,
        }

        print(
            f"[INFO] Physics proxy created: {collider_path}, "
            f"foot={foot_position}, center={tuple(capsule_center)}"
        )

    def sync_person_physics_proxies(self):
        """걷는 사람의 현재 World 위치로 물리 충돌체를 이동한다."""
        for person_name, proxy in self._person_physics_proxies.items():
            person_position = np.asarray(
                proxy["person"].state.position,
                dtype=np.float64,
            )

            if person_position.shape != (3,) or not np.all(
                np.isfinite(person_position)
            ):
                carb.log_warn(
                    f"Invalid person position for {person_name}: "
                    f"{person_position}"
                )
                continue

            capsule_center = Gf.Vec3d(
                float(person_position[0]),
                float(person_position[1]),
                float(person_position[2])
                + PERSON_COLLIDER_TOTAL_HEIGHT_M * 0.5,
            )
            proxy["translate_op"].Set(capsule_center)
