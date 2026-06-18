import rclpy
from rclpy.node import Node
from srvs_pkg.srv import GetTargetPose
from std_srvs.srv import SetBool, Trigger
import time
import math

class MasterNode(Node):
    def __init__(self):
        super().__init__('master_node')
        self.cli_v = self.create_client(GetTargetPose, '/get_target_pose')
        self.cli_r = self.create_client(GetTargetPose, '/robot_move_step')
        self.cli_g = self.create_client(SetBool, '/control_gripper')
        self.cli_h = self.create_client(Trigger, '/robot_home')
       
        # 8번 코드 오프셋 및 환경설정 반영
        self.Z_OFF = -95.0
        self.Z_MARGIN = 25.0
        self.BLOCK_H = 20.0
        self.WAIT_TIME = 1.5
        self.PRE_XY_LOWER = 70.0
        self.Z_PRE_LOWER_MARGIN = 220.0 # 목표물 위에서 미리 하강할 안전 여유 높이 (mm)
       
        self.STUD_PITCH = 0.016
        self.YAW_TUNE = 0.0
        self.TWO_BY_TWO_YAW_FOLD_DEG = 2.0

        # 8번 코드 인지 매핑
        self.CLASS_TO_TARGET_ID = {
            "2x2_red": "1",
            "2x2_green": "2",
            "2x2_blue": "3",
            "2x2_yellow": "4",
            "4x2_red": "5",
            "4x2_green": "6",
            "4x2_blue": "7",
            "4x2_yellow": "8",
            "2x4_red": "5",
            "2x4_green": "6",
            "2x4_blue": "7",
            "2x4_yellow": "8",
            "assembly": "999",
        }
       
        self.last_perfect_pose = None

    def call(self, cli, req):
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {cli.srv_name}...')
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def to_vision_target_id(self, target):
        target = str(target).strip()
        for prefix in ("count_", "far_"):
            if target.startswith(prefix):
                target = target[len(prefix):]

        if target.isdigit():
            return target

        target_id = self.CLASS_TO_TARGET_ID.get(target)
        if target_id is None:
            self.get_logger().error(f"❌ vision_node.py ID 매핑 없음: {target}")
            return target
        return target_id

    def request_target_pose(self, target):
        return self.call(
            self.cli_v,
            GetTargetPose.Request(target_color=self.to_vision_target_id(target))
        )

    def find_target_with_retry(self, color):
        p = self.request_target_pose(color)
        if p.success:
            return p
        self.get_logger().error(f"❌ [{color}] 타겟 인식 실패")
        return None

    def normalize_yaw(self, yaw):
        while yaw > 90.0:
            yaw -= 180.0
        while yaw < -90.0:
            yaw += 180.0
        return yaw

    def is_2x2_pose(self, pose):
        return str(getattr(pose, "class_name", "")).startswith("2x2_")

    def fold_2x2_yaw(self, yaw):
        yaw = self.normalize_yaw(yaw)
        if abs(yaw - 90.0) <= self.TWO_BY_TWO_YAW_FOLD_DEG:
            return yaw - 90.0
        if abs(yaw + 90.0) <= self.TWO_BY_TWO_YAW_FOLD_DEG:
            return yaw + 90.0
        return yaw

    # =========================================================================
    # 6번 코드 시퀀스 적용 (YAW 회전 후 XY 이동, 스캔 중간값 등)
    # =========================================================================
    def pick_target(self, color, layer_index=0, offset_studs_x=0.0, offset_studs_y=0.0):
        self.get_logger().info(f"\n--- PICK TARGET (정밀 모드): [{color.upper()}] ---")

        # 1. 최초 스캔 및 대략적인 XY 이동
        self.get_logger().info("1. 최초 스캔 및 중앙으로 대략적인 이동")
        p1 = self.find_target_with_retry(color)
        if not p1: return False

        # 🌟 치명적 버그 수정: 블라인드 이동 시 홈(Home) 기준 좌표가 필요하므로 p1을 저장합니다.
        self.last_perfect_pose = p1

        # 🌟 관절 한계 도달 방지: 목표물 위 150mm 지점까지 Z축 미리 하강
        z_move_total_initial = (p1.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)
        z_pre_to_move = z_move_total_initial - self.Z_PRE_LOWER_MARGIN
        z_already_moved = 0.0
        if z_pre_to_move > 0:
            self.get_logger().info(f"...Z축 미리 하강 ({z_pre_to_move:.1f}mm)")
            self.call(self.cli_r, GetTargetPose.Request(z=z_pre_to_move, target_size="Z"))
            time.sleep(self.WAIT_TIME)
            z_already_moved = z_pre_to_move

        req_xy1 = GetTargetPose.Request(x=p1.x, y=p1.y, target_size="XY")
        self.call(self.cli_r, req_xy1)
        time.sleep(self.WAIT_TIME)
        self.get_logger().info("...대략적인 중앙 이동 완료.")

        # 2. 중앙에서 정밀 재스캔
        self.get_logger().info("2. 중앙에서 정밀 재스캔 (X, Y, Z, Yaw)")
        p2 = self.find_target_with_retry(color)
        if not p2: return False

        # 3. YAW 회전
        self.get_logger().info("3. YAW 정렬 시작")
        target_yaw = p2.yaw + self.YAW_TUNE
        if self.is_2x2_pose(p2) or color.startswith("2x2_"):
            target_yaw = self.fold_2x2_yaw(target_yaw)
        else:
            target_yaw = self.normalize_yaw(target_yaw)

        req_y = GetTargetPose.Request(yaw=target_yaw, target_size="YAW")
        self.call(self.cli_r, req_y)
        time.sleep(self.WAIT_TIME)
        self.get_logger().info(f"...YAW 정렬 완료 ({target_yaw:.1f}도)")

        # 4. YAW 회전 후 최종 XY 위치 재스캔 및 이동
        self.get_logger().info("4. YAW 정렬 후 최종 XY 좌표 재스캔 및 이동")
        p3 = self.find_target_with_retry(color)
        if not p3: return False
        

        dx = offset_studs_x * self.STUD_PITCH
        dy = offset_studs_y * self.STUD_PITCH
        yaw_for_offset = self.fold_2x2_yaw(p3.yaw) if (self.is_2x2_pose(p3) or color.startswith("2x2_")) else p3.yaw
        yaw_rad = math.radians(yaw_for_offset)
        real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
        real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)

        target_x = p3.x + real_offset_x
        target_y = p3.y + real_offset_y

        req_xy2 = GetTargetPose.Request(x=target_x, y=target_y, target_size="XY")
        self.call(self.cli_r, req_xy2)
        time.sleep(self.WAIT_TIME)
        self.get_logger().info("...최종 XY 이동 완료.")

        # 5. Z 하강 및 집기
        self.get_logger().info("5. Z 하강 및 집기")
        z_move_total_final = (p3.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)
        z_remaining = z_move_total_final - z_already_moved
        
        self.call(self.cli_r, GetTargetPose.Request(z=z_remaining - self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME)
        
        self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME)

        self.call(self.cli_g, SetBool.Request(data=True))
        time.sleep(self.WAIT_TIME)
        self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
        time.sleep(self.WAIT_TIME)
        return True

    def blind_insert(self, base_pose, layer_index, yaw_offset=0.0, release_gripper=True, offset_studs_x=0.0, offset_studs_y=0.0):
        self.get_logger().info(f"\n--- BLIND STACK (메모리 사용): Layer {layer_index} (Y Offset: {offset_studs_y}) ---")
        time.sleep(1.0)

        # 0. Z 선하강 추가 (관절 한계 도달 방지)
        z_move_total = (base_pose.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)
        z_pre = z_move_total - self.Z_PRE_LOWER_MARGIN
        if z_pre > 0:
            self.get_logger().info(f"0. Z축 {z_pre:.1f}mm 미리 하강 (목표 {self.Z_PRE_LOWER_MARGIN}mm 상단)")
            self.call(self.cli_r, GetTargetPose.Request(z=z_pre, target_size="Z"))
            time.sleep(self.WAIT_TIME)
        else:
            z_pre = 0.0

        # 1. XY 이동
        dx = offset_studs_x * self.STUD_PITCH
        dy = offset_studs_y * self.STUD_PITCH

        yaw_for_offset = self.fold_2x2_yaw(base_pose.yaw) if self.is_2x2_pose(base_pose) else base_pose.yaw
        yaw_rad = math.radians(yaw_for_offset)
        real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
        real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)

        target_x = base_pose.x + real_offset_x
        target_y = base_pose.y + real_offset_y

        req_xy = GetTargetPose.Request()
        req_xy.x = target_x; req_xy.y = target_y; req_xy.target_size = "XY"
        self.call(self.cli_r, req_xy)
        time.sleep(self.WAIT_TIME)

        # 2. YAW 이동
        target_yaw = base_pose.yaw + yaw_offset + self.YAW_TUNE
        if self.is_2x2_pose(base_pose):
            target_yaw = self.fold_2x2_yaw(target_yaw)
        else:
            target_yaw = self.normalize_yaw(target_yaw)

        self.get_logger().info(f"🔄 [YAW 회전] 계산된 고정 각도 회전: {target_yaw:.1f}도")
        req_y = GetTargetPose.Request()
        req_y.yaw = target_yaw; req_y.target_size = "YAW"
        self.call(self.cli_r, req_y)
        time.sleep(self.WAIT_TIME)

        # 3. Z 이동 (미리 내려온 거리 z_pre를 제외하고 남은 거리만 이동)
        z_remaining = z_move_total - z_pre
       
        self.call(self.cli_r, GetTargetPose.Request(z=z_remaining - self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME)
        self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN , target_size="Z"))
        time.sleep(self.WAIT_TIME)

        if release_gripper:
            self.call(self.cli_g, SetBool.Request(data=False))
            time.sleep(self.WAIT_TIME)
            
        self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
        time.sleep(self.WAIT_TIME)
        return True

    def visual_insert(self, target_color, layer_index, release_gripper=True, yaw_offset=0.0, offset_studs_x=0.0, offset_studs_y=0.0):
        self.get_logger().info(f"\n--- VISUAL INSERT (정밀 모드): [{target_color.upper()}] ---")

        # 1. 최초 스캔 및 대략적인 XY 이동
        self.get_logger().info("1. 최초 스캔 및 중앙으로 대략적인 이동")
        p1 = self.find_target_with_retry(target_color)
        if not p1: return False

        # 🌟 치명적 버그 수정: 블라인드 이동 시 홈(Home) 기준 좌표가 필요하므로 p1을 저장합니다.
        self.last_perfect_pose = p1

        # 🌟 관절 한계 도달 방지: 목표물 위 150mm 지점까지 Z축 미리 하강
        z_move_total_initial = (p1.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)
        z_pre_to_move = z_move_total_initial - self.Z_PRE_LOWER_MARGIN
        z_already_moved = 0.0
        if z_pre_to_move > 0:
            self.get_logger().info(f"...Z축 미리 하강 ({z_pre_to_move:.1f}mm)")
            self.call(self.cli_r, GetTargetPose.Request(z=z_pre_to_move, target_size="Z"))
            time.sleep(self.WAIT_TIME)
            z_already_moved = z_pre_to_move

        req_xy1 = GetTargetPose.Request(x=p1.x, y=p1.y, target_size="XY")
        self.call(self.cli_r, req_xy1)
        time.sleep(self.WAIT_TIME)
        self.get_logger().info("...대략적인 중앙 이동 완료.")

        # 2. 중앙에서 정밀 재스캔
        self.get_logger().info("2. 중앙에서 정밀 재스캔 (X, Y, Z, Yaw)")
        p2 = self.find_target_with_retry(target_color)
        if not p2: return False

        # 3. YAW 회전
        self.get_logger().info("3. YAW 정렬 시작")
        target_yaw = p2.yaw + yaw_offset + self.YAW_TUNE
        if target_color.startswith("2x2_"):
            target_yaw = self.fold_2x2_yaw(target_yaw)
        else:
            target_yaw = self.normalize_yaw(target_yaw)

        self.get_logger().info(f"🔄 [YAW 회전] 시각 보정 기반 회전: {target_yaw:.1f}도")
        req_y = GetTargetPose.Request(yaw=target_yaw, target_size="YAW")
        self.call(self.cli_r, req_y)
        time.sleep(self.WAIT_TIME)
        self.get_logger().info(f"...YAW 정렬 완료 ({target_yaw:.1f}도)")

        # 4. YAW 회전 후 최종 XY 위치 재스캔 및 이동
        self.get_logger().info("4. YAW 정렬 후 최종 XY 좌표 재스캔 및 이동")
        p3 = self.find_target_with_retry(target_color)
        if not p3: return False
        

        dx = offset_studs_x * self.STUD_PITCH
        dy = offset_studs_y * self.STUD_PITCH
        yaw_for_offset = self.fold_2x2_yaw(p3.yaw) if (self.is_2x2_pose(p3) or target_color.startswith("2x2_")) else p3.yaw
        yaw_rad = math.radians(yaw_for_offset)
        real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
        real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)

        target_x = p3.x + real_offset_x
        target_y = p3.y + real_offset_y

        req_xy2 = GetTargetPose.Request(x=target_x, y=target_y, target_size="XY")
        self.call(self.cli_r, req_xy2)
        time.sleep(self.WAIT_TIME)
        self.get_logger().info("...최종 XY 이동 완료.")

        # 5. Z 하강 및 놓기
        self.get_logger().info("5. Z 하강 및 놓기")
        z_move_total_final = (p3.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)
        z_remaining = z_move_total_final - z_already_moved
        
        self.call(self.cli_r, GetTargetPose.Request(z=z_remaining - self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME)
        
        self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME)

        if release_gripper:
            self.call(self.cli_g, SetBool.Request(data=False))
            time.sleep(self.WAIT_TIME)
            
        self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
        time.sleep(self.WAIT_TIME)
        return True

    # =========================================================================
    # 8번 코드 레시피
    # =========================================================================
    def build_battery(self):
        self.get_logger().info("🔋 [배터리] 노란색(Pick) -> 파란색(Base)")
        if self.pick_target("2x2_yellow"):
            self.call(self.cli_h, Trigger.Request())
            self.visual_insert("2x2_blue", layer_index=1)
            self.get_logger().info("✅ 배터리 조립 완료!")

    def build_magnet(self):
        self.get_logger().info("🧲 [자석] 파란색(Pick) -> 빨간색(Base)")
        if self.pick_target("2x2_blue"):
            self.call(self.cli_h, Trigger.Request())
            self.visual_insert("2x2_red", layer_index=1)
            self.get_logger().info("✅ 자석 조립 완료!")

    def build_e_stop(self):
        self.get_logger().info("🛑 [비상정지] 빨간색(Pick) -> 노란색4x2(Base)")
        if self.pick_target("2x2_red"):
            self.call(self.cli_h, Trigger.Request())
            self.visual_insert("4x2_yellow", layer_index=1, yaw_offset=-90.0)
            self.get_logger().info("✅ 비상정지 조립 완료!")

    def build_carrot(self):
        self.get_logger().info("🥕 [당근] 노란색(Pick) -> 파란색(Base) -> 노란색(Pick)")
        if self.pick_target("2x2_yellow"):
            self.call(self.cli_h, Trigger.Request())
            if self.visual_insert("2x2_blue", layer_index=1):
                self.call(self.cli_h, Trigger.Request())
                if self.pick_target("2x2_yellow"):
                    self.call(self.cli_h, Trigger.Request())
                    self.visual_insert("2x2_yellow", layer_index=1)
                    self.get_logger().info("✅ 당근 완성!")

    def build_traffic_light(self):
        self.get_logger().info("🚦 [신호등] 노란색(Pick) -> 파란색(Base) -> 빨간색(Pick)")
        if self.pick_target("2x2_yellow"):
            self.call(self.cli_h, Trigger.Request())
            if self.visual_insert("2x2_blue", layer_index=1):
                self.call(self.cli_h, Trigger.Request())
                if self.pick_target("2x2_red"):
                    self.call(self.cli_h, Trigger.Request())
                    self.visual_insert("2x2_yellow", layer_index=1)
                    self.get_logger().info("✅ 신호등 완성!")

    def build_small_tree(self):
        self.get_logger().info("🌳 [작은 나무] 빨강4x2(Pick) -> 노랑2x2(Base) -> 빨강2x2(Pick)")
        if self.pick_target("4x2_green"):
            self.call(self.cli_h, Trigger.Request())
            if self.visual_insert("2x2_yellow", layer_index=1):
                self.call(self.cli_h, Trigger.Request())
                if self.pick_target("2x2_green"):
                    self.call(self.cli_h, Trigger.Request())
                    self.visual_insert("4x2_green", layer_index=)
                    self.get_logger().info("✅ 작은 나무 완성!")

    def build_hammer(self):
        self.get_logger().info("🔨 [망치] 빨강2x2(Pick) -> 빨강2x2(Base) -> 파랑4x2(Pick)")
        if self.pick_target("2x2_red"):
            self.call(self.cli_h, Trigger.Request())
            if self.visual_insert("2x2_red", layer_index=1):
                self.call(self.cli_h, Trigger.Request())
                if self.pick_target("4x2_blue"):
                    self.call(self.cli_h, Trigger.Request())
                    self.visual_insert("2x2_red", layer_index=1, yaw_offset=0.0)
                    self.get_logger().info("✅ 망치 완성!")

    def build_big_carrot(self):
        self.get_logger().info("🥕🥕 [큰 당근] 노랑2x2(Pick) -> 노랑2x2(Base) -> 노랑4x2(Pick) -> 파랑2x2(Pick)")
        if self.pick_target("2x2_yellow"):
            self.call(self.cli_h, Trigger.Request())
            if self.visual_insert("2x2_yellow", layer_index=1):
                self.call(self.cli_h, Trigger.Request())
                if self.pick_target("4x2_yellow"):
                    self.call(self.cli_h, Trigger.Request())
                    if self.visual_insert("2x2_yellow", layer_index=2, yaw_offset=0.0):
                        self.call(self.cli_h, Trigger.Request())
                        if self.pick_target("2x2_blue"):
                            self.call(self.cli_h, Trigger.Request())
                            self.visual_insert("4x2_yellow", layer_index=1)
                            self.get_logger().info("✅ 대왕 당근 완성!")

    def build_burger(self):
        self.get_logger().info("🍔 [버거] 노랑4x2(Base) -> 빨강4x2(Offset Y -1) -> 빨강2x2(Offset Y +2) -> 노랑4x2(Top)")
        if self.pick_target("4x2_red"):
            self.call(self.cli_h, Trigger.Request())
            if self.visual_insert("4x2_yellow", layer_index=1, offset_studs_y=-1.0):
                saved_base_bun_pose = self.last_perfect_pose
                self.call(self.cli_h, Trigger.Request())
                if self.pick_target("2x2_red"):
                    self.call(self.cli_h, Trigger.Request())
                    if self.visual_insert("4x2_red", layer_index=0, yaw_offset=0.0, offset_studs_y=3.0):
                        self.call(self.cli_h, Trigger.Request())
                        if self.pick_target("4x2_yellow"):
                            self.call(self.cli_h, Trigger.Request())
                            if saved_base_bun_pose:
                                self.get_logger().info("🧠 [메모리 사용] 최초 바닥 빵의 좌표를 기억해서 정중앙에 덮습니다!")
                                self.blind_insert(saved_base_bun_pose, layer_index=2, offset_studs_y=1.0)
                                self.get_logger().info("✅ 버거 완성!")
                            else:
                                self.get_logger().warn("❌ 저장된 좌표가 없습니다. 조립 실패.")

    def build_ice_cream(self):
        self.get_logger().info("🍦 [아이스크림] 모듈형 조립 전략")
        self.get_logger().info("[Phase 1] 하단 조립: 노랑4x2(Pick) -> 노랑2x2(Base)")
        if self.pick_target("4x2_yellow"):
            self.call(self.cli_h, Trigger.Request())
            if not self.visual_insert("2x2_yellow", layer_index=1):
                self.get_logger().warn("❌ 하단 모듈 조립 실패")
                return

        self.get_logger().info("[Phase 2-1] 파랑2x2(Pick) -> 빨강2x2 옆에 배치 (바닥)")
        self.call(self.cli_h, Trigger.Request())
        if self.pick_target("2x2_blue"):
            self.call(self.cli_h, Trigger.Request())
            if not self.visual_insert("2x2_red", layer_index=0, offset_studs_y=2.0):
                self.get_logger().warn("❌ 파란색 블록 배치 실패")
                return

        self.get_logger().info("[Phase 2-2] 노랑2x2(Pick) -> 파랑2x2(Base, offset_y=-1)로 결합")
        self.call(self.cli_h, Trigger.Request())
        if self.pick_target("2x2_yellow"):
            self.call(self.cli_h, Trigger.Request())
            if not self.visual_insert("2x2_blue", layer_index=1, offset_studs_y=-1.0):
                self.get_logger().warn("❌ 상단 모듈 결합 실패")
                return

        self.get_logger().info(" [Phase 3] 최종 결합: 상단 모듈 들어서 하단 모듈(노랑4x2) 위에 꽂기!")
        self.call(self.cli_h, Trigger.Request())
        if self.pick_target("2x2_yellow", layer_index=0.5):
            self.call(self.cli_h, Trigger.Request())
            if self.visual_insert("4x2_yellow", layer_index=2):
                self.get_logger().info("✅🎉 5단 아이스크림 완벽하게 완성!")
            else:
                self.get_logger().warn("❌ 최종 층 올리기 실패")

    def build_studs_y(self):
        self.get_logger().info("🧱 [초기화] 맨 처음 2x2_yellow 위치 스캔 및 기억")
        p_yellow = self.find_target_with_retry("2x2_yellow")
        if not p_yellow:
            self.get_logger().warn("❌ 바닥에 2x2_yellow가 안 보입니다. 조립을 취소합니다.")
            return
        saved_yellow_pose = p_yellow
        self.call(self.cli_h, Trigger.Request())

        self.get_logger().info("🧱 [Phase 1] 4x2_red(-1.85) 파지 -> 2x2_red(0.0) 결합 (그리퍼 유지)")
        if self.pick_target("4x2_red", offset_studs_y=-1.84):
            self.call(self.cli_h, Trigger.Request())
            if self.visual_insert("2x2_red", layer_index=1, offset_studs_y=0.0, release_gripper=False):
                self.call(self.cli_h, Trigger.Request())
                time.sleep(1.0) 

                self.get_logger().info("🧱 [Phase 2] 바닥의 다른 4x2_red 스캔 및 6x2 조립 (그리퍼 해제)")
                p_4x2_base = self.find_target_with_retry("4x2_red")
                if not p_4x2_base:
                    self.get_logger().warn("❌ 바닥에 다른 4x2_red가 안 보입니다.")
                    return
                saved_6x2_pose = p_4x2_base 
                
                if self.blind_insert(saved_6x2_pose, layer_index=1, offset_studs_y=-3.0, release_gripper=True):
                    self.call(self.cli_h, Trigger.Request())

                    self.get_logger().info("🧱 [Phase 3] 2x2_red 파지 -> 6x2 중심에 결합 (그리퍼 유지)")
                    if self.pick_target("2x2_red", offset_studs_y=-0.2):
                        self.call(self.cli_h, Trigger.Request())
                        if self.blind_insert(saved_6x2_pose, layer_index=2, offset_studs_y=-1.0, release_gripper=False):
                            self.call(self.cli_h, Trigger.Request())

                            self.get_logger().info("🧱 [Phase 4] 덩어리를 2x2_yellow 중앙에 최종 결합")
                            if self.blind_insert(saved_yellow_pose, layer_index=1.5, offset_studs_y=0.0):
                                self.get_logger().info("✅ 최종 조립 시퀀 완벽 종료!")

    # =========================================================================
    # 8번 코드 수동 실행 모드
    # =========================================================================
    def run(self):
        self.get_logger().info("🚀 STARTING VISUAL-STACK ASSEMBLY SEQUENCE (Keyboard Select Mode)")
        home_res = self.call(self.cli_h, Trigger.Request())
        if home_res is None or not home_res.success:
            self.get_logger().error("❌ 시작 HOME 이동 실패. 조립을 시작하지 않습니다.")
            return
        self.call(self.cli_g, SetBool.Request(data=False))
        time.sleep(1.0)
       
        actions = {
            "1": self.build_battery, "battery": self.build_battery, "배터리": self.build_battery,
            "2": self.build_magnet, "magnet": self.build_magnet, "자석": self.build_magnet,
            "3": self.build_e_stop, "estop": self.build_e_stop, "비상정지": self.build_e_stop,
            "4": self.build_carrot, "carrot": self.build_carrot, "당근": self.build_carrot,
            "5": self.build_traffic_light, "traffic": self.build_traffic_light, "신호등": self.build_traffic_light,
            "6": self.build_small_tree, "tree": self.build_small_tree, "작은나무": self.build_small_tree,
            "7": self.build_hammer, "hammer": self.build_hammer, "망치": self.build_hammer,
            "8": self.build_big_carrot, "bigcarrot": self.build_big_carrot, "큰당근": self.build_big_carrot,
            "9": self.build_burger, "burger": self.build_burger, "버거": self.build_burger,
            "10": self.build_ice_cream, "icecream": self.build_ice_cream, "아이스크림": self.build_ice_cream,
            "11": self.build_studs_y, "studs_y": self.build_studs_y
        }

        print("\n=== Master Node Assembly Keyboard Select ===")
        print("1: 배터리 / 2: 자석 / 3: 비상정지 / 4: 당근 / 5: 신호등 / 6: 작은나무")
        print("7: 망치 / 8: 큰당근 / 9: 버거 / 10: 아이스크림 / 11: studs_y")
        print("q: 종료")

        while rclpy.ok():
            user_input = input("\n조립할 항목을 선택하세요 [1~11/q]: ").strip().replace(" ", "").lower()
            if user_input in ("q", "quit", "exit", "종료"):
                self.get_logger().info("조립 시퀀스를 종료합니다.")
                break

            action = actions.get(user_input)
            if action is None:
                print("잘못된 입력입니다. 1~11 또는 q 중에서 선택하세요.")
                continue

            self.get_logger().info(f"▶️ 작업 시작: {user_input}")
            action()
            
            self.call(self.cli_h, Trigger.Request())
            time.sleep(1.0)
            self.get_logger().info("✅ 개별 조립 완료")

        self.call(self.cli_h, Trigger.Request())
        self.get_logger().info("🎉 ALL SEQUENCE DONE")

def main():
    rclpy.init()
    node = MasterNode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
