#!/usr/bin/env python3
"""조난자·구조자 생성과 사람 물리 충돌 프록시 동기화."""

import carb
import numpy as np
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics

from pegasus.simulator.logic.people.person import Person

from sim_config import (
    FOR_TEST_VICTIM_SPAWN_ENABLED,
    FOR_TEST_VICTIM_WORLD_XYZ,
    PERSON_COLLIDER_CYLINDER_HEIGHT_M,
    PERSON_COLLIDER_RADIUS_M,
    PERSON_COLLIDER_TOTAL_HEIGHT_M,
    PERSON_GROUND_CLEARANCE_M,
    RESCUER_FOOT_Z,
    RESCUER_XY,
    VICTIM_SPAWN_POSITIONS,
)
from sim_utils import write_ground_truth


class PeopleManager:
    """사람 객체와 사람이 사용하는 보이지 않는 물리 충돌체를 관리한다."""

    def __init__(self, terrain, rng, test_victim_spawn_world_enu=None):
        self.terrain = terrain
        self.rng = rng
        self.test_victim_spawn_world_enu = test_victim_spawn_world_enu
        self._person_physics_proxies = {}
        self.victim = None
        self.rescuer = None

    def spawn_people(self):
        """조난자 1명과 구조자 1명을 지정 조건에 맞게 정지 상태로 생성한다."""
        preferred_asset = "original_female_adult_business_02"
        available_assets = Person.get_character_asset_list()

        if not available_assets:
            raise RuntimeError("No Pegasus person assets were found.")

        if preferred_asset in available_assets:
            selected_asset = preferred_asset
        else:
            selected_asset = available_assets[0]
            carb.log_warn(
                f"Preferred person asset was not found. "
                f"Using {selected_asset} instead."
            )

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
            victim_position = [
                float(value) for value in victim_position
            ]
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

            # 후보 좌표의 실제 Terrain 높이를 다시 읽는다.
            victim_ground_z = self.terrain.height(victim_x, victim_y)
            victim_position = [
                victim_x,
                victim_y,
                victim_ground_z + PERSON_GROUND_CLEARANCE_M,
            ]
            spawn_description = f"candidate {victim_index + 1}"

        # Person을 실제로 생성할 때 사용하는 좌표를 그대로 RViz용
        # Ground Truth 파일에 기록한다.
        write_ground_truth(victim_position, victim_index)

        self.victim = Person(
            "victim_01",
            selected_asset,
            init_pos=victim_position,
            init_yaw=0.0,
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
        rescuer_position = [
            rescuer_x,
            rescuer_y,
            RESCUER_FOOT_Z,
        ]
        self.rescuer = Person(
            "rescuer_01",
            selected_asset,
            init_pos=rescuer_position,
            init_yaw=0.0,
        )
        self._create_person_physics_proxy(
            "rescuer_01",
            self.rescuer,
            rescuer_position,
        )
        print(
            "[INFO] Spawned rescuer beside drones at "
            f"({rescuer_position[0]:.3f}, "
            f"{rescuer_position[1]:.3f}, "
            f"{rescuer_position[2]:.3f})"
        )

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
        capsule.CreateHeightAttr().Set(
            PERSON_COLLIDER_CYLINDER_HEIGHT_M
        )

        # Capsule의 원점은 중심이므로 발 위치에서 사람 키의 절반만큼 올린다.
        capsule_center = Gf.Vec3d(
            float(foot_position[0]),
            float(foot_position[1]),
            float(foot_position[2])
            + PERSON_COLLIDER_TOTAL_HEIGHT_M * 0.5,
        )
        translate_op = capsule.AddTranslateOp()
        translate_op.Set(capsule_center)

        # 시각적 Person은 그대로 두고, 이 프록시만 물리 충돌에 사용한다.
        capsule.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
        collider_prim = capsule.GetPrim()
        UsdPhysics.CollisionAPI.Apply(collider_prim)

        rigid_body = UsdPhysics.RigidBodyAPI.Apply(collider_prim)
        rigid_body.CreateRigidBodyEnabledAttr(True)
        rigid_body.CreateKinematicEnabledAttr(True)

        # Person은 Animation Graph를 통해 이동하므로 캐릭터와 충돌체를
        # 부모-자식으로 묶지 않고, Person.state.position을 매 프레임 따라간다.
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

            # Person.state.position은 발 기준 World 좌표다.
            # 캡슐 Prim의 원점은 중심이므로 높이 절반을 더한다.
            capsule_center = Gf.Vec3d(
                float(person_position[0]),
                float(person_position[1]),
                float(person_position[2])
                + PERSON_COLLIDER_TOTAL_HEIGHT_M * 0.5,
            )
            proxy["translate_op"].Set(capsule_center)
