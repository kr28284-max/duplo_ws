# # # import rclpy
# # # from rclpy.node import Node
# # # from srvs_pkg.srv import GetTargetPose
# # # from std_srvs.srv import SetBool, Trigger
# # # import time
# # # import math

# # # class MasterNode(Node):
# # #     def __init__(self):
# # #         super().__init__('master_node')
# # #         self.cli_v = self.create_client(GetTargetPose, '/get_target_pose')
# # #         self.cli_r = self.create_client(GetTargetPose, '/robot_move_step')
# # #         self.cli_g = self.create_client(SetBool, '/control_gripper')
# # #         self.cli_h = self.create_client(Trigger, '/robot_home')
       
# # #         self.Z_OFF = -100.0
# # #         self.Z_MARGIN = 20.0
# # #         self.BLOCK_H = 16.0
# # #         self.WAIT_TIME = 1.5
       
# # #         self.STUD_PITCH = 0.016
# # #         self.YAW_TUNE = 0.0  # 필요시 여기서 영점 조절 (예: -1.5)

# # #         # vision_node.py는 target_color로 클래스명이 아니라 숫자 ID 문자열만 받는다.
# # #         self.CLASS_TO_TARGET_ID = {
# # #             "2x2_red": "1",
# # #             "2x2_green": "2",
# # #             "2x2_blue": "3",
# # #             "2x2_yellow": "4",
# # #             "4x2_red": "5",
# # #             "4x2_green": "6",
# # #             "4x2_blue": "7",
# # #             "4x2_yellow": "8",
# # #             "2x4_red": "5",
# # #             "2x4_green": "6",
# # #             "2x4_blue": "7",
# # #             "2x4_yellow": "8",
# # #             "assembly": "999",
# # #         }
       
# # #         # 🌟 가장 깨끗하게 인식되었을 때의 타겟 좌표를 기억하는 변수
# # #         self.last_perfect_pose = None

# # #     def call(self, cli, req):
# # #         while not cli.wait_for_service(timeout_sec=1.0):
# # #             self.get_logger().info(f'Waiting for {cli.srv_name}...')
# # #         future = cli.call_async(req)
# # #         rclpy.spin_until_future_complete(self, future)
# # #         return future.result()

# # #     def to_vision_target_id(self, target):
# # #         target = str(target).strip()
# # #         for prefix in ("count_", "far_"):
# # #             if target.startswith(prefix):
# # #                 target = target[len(prefix):]

# # #         if target.isdigit():
# # #             return target

# # #         target_id = self.CLASS_TO_TARGET_ID.get(target)
# # #         if target_id is None:
# # #             self.get_logger().error(f"❌ vision_node.py ID 매핑 없음: {target}")
# # #             return target
# # #         return target_id

# # #     def request_target_pose(self, target):
# # #         return self.call(
# # #             self.cli_v,
# # #             GetTargetPose.Request(target_color=self.to_vision_target_id(target))
# # #         )

# # #     def count_color(self, color):
# # #         p = self.request_target_pose(color)
# # #         return 1 if p.success else 0

# # #     def find_target_with_retry(self, color):
# # #         """재시도 없이 1회만 즉시 스캔"""
# # #         p = self.request_target_pose(color)
# # #         if p.success:
# # #             return p
# # #         self.get_logger().error(f"❌ [{color}] 타겟 인식 실패")
# # #         return None
    
# # #     def get_dist(self, p1, p2):
# # #         """두 지점 사이의 평면 거리(m)를 계산"""
# # #         return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

# # #     # =========================================================================
# # #     # Big_Tree,Ice_cream만 pick_fresh_target사용
# # #     # ==========================================================================
# # #     def pick_fresh_target(self, color, exclude_pose=None, threshold=0.035, layer_index=0):
# # #         target_req = f"far_{color}"
        
# # #         # [1번째 스캔] 홈 위치에서 멀리 있는 타겟 1회만 즉시 스캔
# # #         target_p = self.request_target_pose(target_req)
# # #         if not target_p.success:
# # #             return False

# # #         # --- 로봇 이동 시작 ---
# # #         z_move = (target_p.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)

# # #         # 🌟 [통합 이동] YAW, X, Y, Z(Z_MARGIN 위까지 대각선 하강) 한번에 이동
# # #         self.get_logger().info("➡️ [통합 이동] YAW, X, Y, Z 한번에 접근")
# # #         req_all = GetTargetPose.Request()
# # #         req_all.x = target_p.x
# # #         req_all.y = target_p.y
# # #         req_all.z = z_move - self.Z_MARGIN
# # #         req_all.yaw = target_p.yaw
# # #         req_all.target_size = "ALL"  # 통합 제어 식별자
# # #         self.call(self.cli_r, req_all)
# # #         time.sleep(self.WAIT_TIME)

# # #         # [수직 최종 접근]
# # #         self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
# # #         time.sleep(self.WAIT_TIME)

# # #         self.call(self.cli_g, SetBool.Request(data=True))
# # #         time.sleep(self.WAIT_TIME)
# # #         self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
# # #         return True


# # #     def pick_target(self, color, layer_index=0, offset_studs_x=0.0, offset_studs_y=0.0, exclude_pose=None):
# # #         self.get_logger().info(f"\n--- PICK TARGET: [{color.upper()}] ---")
        
# # #         # [1번째 스캔] 홈 위치에서 타겟의 전체 좌표 확보
# # #         p = self.find_target_with_retry(color)
# # #         if not p: return False
       
# # #         # 오프셋 계산
# # #         dx = offset_studs_x * self.STUD_PITCH
# # #         dy = offset_studs_y * self.STUD_PITCH
# # #         yaw_rad = math.radians(p.yaw)
# # #         real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
# # #         real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)

# # #         target_x = p.x + real_offset_x
# # #         target_y = p.y + real_offset_y
# # #         z_move = (p.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)

# # #         # 🌟 [통합 이동] YAW, X, Y, Z(Z_MARGIN 위까지 대각선 하강) 한번에 이동
# # #         self.get_logger().info("➡️ [통합 이동] YAW, X, Y, Z 한번에 접근")
# # #         req_all = GetTargetPose.Request()
# # #         req_all.x = target_x
# # #         req_all.y = target_y
# # #         req_all.z = z_move - self.Z_MARGIN
# # #         req_all.yaw = p.yaw
# # #         req_all.target_size = "ALL"  # 통합 제어 식별자
# # #         self.call(self.cli_r, req_all)
# # #         time.sleep(self.WAIT_TIME)
        
# # #         # [수직 최종 접근]
# # #         self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
# # #         time.sleep(self.WAIT_TIME)

# # #         self.call(self.cli_g, SetBool.Request(data=True))
# # #         time.sleep(self.WAIT_TIME)
# # #         self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
# # #         time.sleep(self.WAIT_TIME)
# # #         return True

# # #     def blind_insert(self, base_pose, layer_index, yaw_offset=0.0, release_gripper=True, regrip=False, offset_studs_x=0.0, offset_studs_y=0.0):
# # #         self.get_logger().info(f"\n--- BLIND STACK (메모리 사용): Layer {layer_index} (Y Offset: {offset_studs_y}) ---")
# # #         time.sleep(1.0)

# # #         # 오프셋 계산
# # #         dx = offset_studs_x * self.STUD_PITCH
# # #         dy = offset_studs_y * self.STUD_PITCH
# # #         yaw_rad = math.radians(base_pose.yaw)
# # #         real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
# # #         real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)

# # #         target_x = base_pose.x + real_offset_x
# # #         target_y = base_pose.y + real_offset_y
        
# # #         target_yaw = base_pose.yaw + yaw_offset + self.YAW_TUNE
# # #         while target_yaw > 90.0: target_yaw -= 180.0
# # #         while target_yaw < -90.0: target_yaw += 180.0

# # #         z_move = (base_pose.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)
       
# # #         # 🌟 [통합 이동] YAW, X, Y, Z(Z_MARGIN 위까지 대각선 하강) 한번에 이동
# # #         self.get_logger().info(f"➡️ [통합 이동] 계산된 목표지점으로 한번에 접근 (Yaw: {target_yaw:.1f})")
# # #         req_all = GetTargetPose.Request()
# # #         req_all.x = target_x
# # #         req_all.y = target_y
# # #         req_all.z = z_move - self.Z_MARGIN
# # #         req_all.yaw = target_yaw
# # #         req_all.target_size = "ALL"  # 통합 제어 식별자
# # #         self.call(self.cli_r, req_all)
# # #         time.sleep(self.WAIT_TIME)

# # #         # [수직 최종 접근]
# # #         self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN , target_size="Z"))
# # #         time.sleep(self.WAIT_TIME)

# # #         if release_gripper:
# # #             self.call(self.cli_g, SetBool.Request(data=False))
# # #             time.sleep(self.WAIT_TIME)
# # #         return True

# # #     def visual_insert(self, target_color, layer_index, release_gripper=True, yaw_offset=0.0, offset_studs_x=0.0, offset_studs_y=0.0):
# # #         self.get_logger().info(f"\n--- VISUAL STACK: [{target_color.upper()}] (Layer +{layer_index}, Y Offset: {offset_studs_y}) ---")
# # #         time.sleep(1.0)

# # #         p = self.find_target_with_retry(target_color)
# # #         if not p: return False
       
# # #         # 메모리 로직
# # #         self.last_perfect_pose = p

# # #         # 오프셋 계산
# # #         target_yaw = p.yaw + yaw_offset + self.YAW_TUNE
# # #         while target_yaw > 90.0: target_yaw -= 180.0
# # #         while target_yaw < -90.0: target_yaw += 180.0

# # #         dx = offset_studs_x * self.STUD_PITCH
# # #         dy = offset_studs_y * self.STUD_PITCH
# # #         yaw_rad = math.radians(p.yaw)
# # #         real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
# # #         real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)

# # #         target_x = p.x + real_offset_x
# # #         target_y = p.y + real_offset_y
# # #         z_move = (p.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)
       
# # #         # 🌟 [통합 이동] YAW, X, Y, Z(Z_MARGIN 위까지 대각선 하강) 한번에 이동
# # #         self.get_logger().info(f"➡️ [통합 이동] 시각 보정 기반 목표지점으로 한번에 접근 (Yaw: {target_yaw:.1f})")
# # #         req_all = GetTargetPose.Request()
# # #         req_all.x = target_x
# # #         req_all.y = target_y
# # #         req_all.z = z_move - self.Z_MARGIN
# # #         req_all.yaw = target_yaw
# # #         req_all.target_size = "ALL"  # 통합 제어 식별자
# # #         self.call(self.cli_r, req_all)
# # #         time.sleep(self.WAIT_TIME)

# # #         # [수직 최종 접근]
# # #         self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN , target_size="Z"))
# # #         time.sleep(self.WAIT_TIME)

# # #         if release_gripper:
# # #             self.call(self.cli_g, SetBool.Request(data=False))
# # #             time.sleep(self.WAIT_TIME)
# # #         self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
# # #         time.sleep(self.WAIT_TIME)
# # #         return True

# # #     def get_best_build_plan(self, current_inventory):
# # #         recipes = {
# # #             'studs_y': {'4x2_red': 2, '2x2_red': 2, '2x2_yellow': 1},
# # #             'battery': {'2x2_yellow': 1, '2x2_blue': 1},
# # #             'magnet': {'2x2_blue': 1, '2x2_red': 1},
# # #             'e_stop': {'2x2_red': 1, '4x2_yellow': 1},
# # #             'carrot': {'2x2_blue': 1, '2x2_yellow': 2},
# # #             'traffic_light': {'2x2_red': 1, '2x2_yellow': 1, '2x2_blue': 1},
# # #             'small_tree': {'2x2_red': 1, '4x2_red': 1, '2x2_yellow': 1},
# # #             'hammer': {'4x2_blue': 1, '2x2_red': 2},
# # #             'big_carrot': {'2x2_yellow': 2, '4x2_yellow': 1, '2x2_blue': 1},
# # #             'burger': {'4x2_yellow': 2, '4x2_red': 1, '2x2_red': 1},
# # #             'ice_cream': {'2x2_yellow': 2, '4x2_yellow': 1, '2x2_red': 1, '2x2_blue': 1},
# # #             'big_tree': {'2x2_yellow': 1, '2x2_red': 2, '4x2_red': 2}
# # #         }
# # #         best_plan = []
# # #         min_remainder = sum(current_inventory.values())

# # #         def dfs(inv, current_plan):
# # #             nonlocal best_plan, min_remainder
# # #             made_any = False
# # #             for name, recipe in recipes.items():
# # #                 can_make = True
# # #                 for color, count in recipe.items():
# # #                     if inv.get(color, 0) < count:
# # #                         can_make = False
# # #                         break
# # #                 if can_make:
# # #                     made_any = True
# # #                     new_inv = inv.copy()
# # #                     for color, count in recipe.items():
# # #                         new_inv[color] -= count
# # #                     dfs(new_inv, current_plan + [name])
           
# # #             if not made_any:
# # #                 remainder = sum(inv.values())
# # #                 if remainder < min_remainder:
# # #                     min_remainder = remainder
# # #                     best_plan = current_plan
# # #                 elif remainder == min_remainder:
# # #                     if len(current_plan) < len(best_plan):
# # #                         best_plan = current_plan

# # #         dfs(current_inventory, [])
# # #         return best_plan

# # #     # --- 2~3개 조합 (Visual Insert 적용) ---

# # #     def build_battery(self):
# # #         self.get_logger().info("🔋 [배터리] 노란색(Pick) -> 파란색(Base)")
# # #         if self.pick_target("2x2_yellow"):
# # #             self.call(self.cli_h, Trigger.Request())
# # #             self.visual_insert("2x2_blue", layer_index=1)
# # #             self.get_logger().info("✅ 배터리 조립 완료!")

# # #     def build_magnet(self):
# # #         self.get_logger().info("🧲 [자석] 파란색(Pick) -> 빨간색(Base)")
# # #         if self.pick_target("2x2_blue"):
# # #             self.call(self.cli_h, Trigger.Request())
# # #             self.visual_insert("2x2_red", layer_index=1)
# # #             self.get_logger().info("✅ 자석 조립 완료!")

# # #     def build_e_stop(self):
# # #         self.get_logger().info("🛑 [비상정지] 빨간색(Pick) -> 노란색4x2(Base)")
# # #         if self.pick_target("2x2_red"):
# # #             self.call(self.cli_h, Trigger.Request())
# # #             self.visual_insert("4x2_yellow", layer_index=1, yaw_offset=-90.0)
# # #             self.get_logger().info("✅ 비상정지 조립 완료!")

# # #     def build_carrot(self):
# # #         self.get_logger().info("🥕 [당근] 노란색(Pick) -> 파란색(Base) -> 노란색(Pick)")
# # #         if self.pick_target("2x2_yellow"):
# # #             self.call(self.cli_h, Trigger.Request())
# # #             if self.visual_insert("2x2_blue", layer_index=1):
# # #                 self.call(self.cli_h, Trigger.Request())
# # #                 if self.pick_target("2x2_yellow"):
# # #                     self.call(self.cli_h, Trigger.Request())
# # #                     # 1층에 놓인 노란색을 베이스로 삼아 1층 높이 더 올리기
# # #                     self.visual_insert("2x2_yellow", layer_index=1)
# # #                     self.get_logger().info("✅ 당근 완성!")

# # #     def build_traffic_light(self):
# # #         self.get_logger().info("🚦 [신호등] 노란색(Pick) -> 파란색(Base) -> 빨간색(Pick)")
# # #         if self.pick_target("2x2_yellow"):
# # #             self.call(self.cli_h, Trigger.Request())
# # #             if self.visual_insert("2x2_blue", layer_index=1):
# # #                 self.call(self.cli_h, Trigger.Request())
# # #                 if self.pick_target("2x2_red"):
# # #                     self.call(self.cli_h, Trigger.Request())
# # #                     # 파란색이 가려졌으니, 방금 놓은 노란색을 타겟으로!
# # #                     self.visual_insert("2x2_yellow", layer_index=1)
# # #                     self.get_logger().info("✅ 신호등 완성!")

# # #     def build_small_tree(self):
# # #         self.get_logger().info("🌳 [작은 나무] 빨강4x2(Pick) -> 노랑2x2(Base) -> 빨강2x2(Pick)")
# # #         if self.pick_target("4x2_red"):
# # #             self.call(self.cli_h, Trigger.Request())
# # #             if self.visual_insert("2x2_yellow", layer_index=1):
# # #                 self.call(self.cli_h, Trigger.Request())
# # #                 if self.pick_target("2x2_red"):
# # #                     self.call(self.cli_h, Trigger.Request())
# # #                     # 가려진 2x2 노랑 대신, 방금 놓은 4x2 빨강을 타겟으로!
# # #                     self.visual_insert("4x2_red", layer_index=1)
# # #                     self.get_logger().info("✅ 작은 나무 완성!")

# # #     def build_hammer(self):
# # #         self.get_logger().info("🔨 [망치] 빨강2x2(Pick) -> 빨강2x2(Base) -> 파랑4x2(Pick)")
# # #         if self.pick_target("2x2_red"):
# # #             self.call(self.cli_h, Trigger.Request())
# # #             if self.visual_insert("2x2_red", layer_index=1):
# # #                 self.call(self.cli_h, Trigger.Request())
# # #                 if self.pick_target("4x2_blue"):
# # #                     self.call(self.cli_h, Trigger.Request())
# # #                     # 가려진 0층 빨강 대신 1층 빨강을 타겟으로! 간섭 회피용 90도 회전
# # #                     self.visual_insert("2x2_red", layer_index=1, yaw_offset=0.0)
# # #                     self.get_logger().info("✅ 망치 완성!")

# # #     # --- 4개 조합 (Big Carrot, Burger) ---
# # #     def build_big_carrot(self):
# # #         self.get_logger().info("🥕🥕 [큰 당근] 노랑2x2(Pick) -> 노랑2x2(Base) -> 노랑4x2(Pick) -> 파랑2x2(Pick)")
# # #         if self.pick_target("2x2_yellow"):
# # #             self.call(self.cli_h, Trigger.Request())
# # #             if self.visual_insert("2x2_yellow", layer_index=1):
               
# # #                 self.call(self.cli_h, Trigger.Request())
# # #                 if self.pick_target("4x2_yellow"):
# # #                     self.call(self.cli_h, Trigger.Request())
# # #                     if self.visual_insert("2x2_yellow", layer_index=2, yaw_offset=0.0):
                       
# # #                         self.call(self.cli_h, Trigger.Request())
# # #                         if self.pick_target("2x2_blue"):
# # #                             self.call(self.cli_h, Trigger.Request())
# # #                             self.visual_insert("4x2_yellow", layer_index=1)
# # #                             self.get_logger().info("✅ 대왕 당근 완성!")

# # #     def build_burger(self):
# # #         self.get_logger().info("🍔 [버거] 노랑4x2(Base) -> 빨강4x2(Offset Y -1) -> 빨강2x2(Offset Y +2) -> 노랑4x2(Top)")
# # #         if self.pick_target("4x2_red"):
# # #             self.call(self.cli_h, Trigger.Request())
# # #             if self.visual_insert("4x2_yellow", layer_index=1, offset_studs_y=-1.0):
# # #                 saved_base_bun_pose = self.last_perfect_pose
               
# # #                 self.call(self.cli_h, Trigger.Request())
# # #                 if self.pick_target("2x2_red"):
# # #                     self.call(self.cli_h, Trigger.Request())
                   
# # #                     # 🌟 간섭 회피를 위해 yaw_offset=0.0 으로 복구
# # #                     if self.visual_insert("4x2_red", layer_index=0, yaw_offset=0.0, offset_studs_y=3.0):
                       
# # #                         self.call(self.cli_h, Trigger.Request())
# # #                         if self.pick_target("4x2_yellow"):
# # #                             self.call(self.cli_h, Trigger.Request())
                           
# # #                             if saved_base_bun_pose:
# # #                                 self.get_logger().info("🧠 [메모리 사용] 덩어리 인식 오류 방지: 최초 바닥 빵의 좌표를 기억해서 정중앙에 덮습니다!")
# # #                                 self.blind_insert(saved_base_bun_pose, layer_index=2, offset_studs_y=1.0)
# # #                                 self.get_logger().info("✅ 버거 완성!")
# # #                             else:
# # #                                 self.get_logger().warn("❌ 저장된 좌표가 없습니다. 조립 실패.")

# # #     def build_ice_cream(self):
# # #         self.get_logger().info("🍦 [아이스크림] 모듈형 조립 전략")

# # #         self.get_logger().info("[Phase 1] 하단 조립: 노랑4x2(Pick) -> 노랑2x2(Base)")
# # #         if self.pick_target("4x2_yellow"):
# # #             self.call(self.cli_h, Trigger.Request())
# # #             if not self.visual_insert("2x2_yellow", layer_index=1):
# # #                 self.get_logger().warn("❌ 하단 모듈 조립 실패")
# # #                 return

# # #         self.get_logger().info("[Phase 2-1] 파랑2x2(Pick) -> 빨강2x2 옆에 배치 (바닥)")
# # #         self.call(self.cli_h, Trigger.Request())
# # #         if self.pick_target("2x2_blue"):
# # #             self.call(self.cli_h, Trigger.Request())
# # #             if not self.visual_insert("2x2_red", layer_index=0, offset_studs_y=2.0):
# # #                 self.get_logger().warn("❌ 파란색 블록 배치 실패")
# # #                 return

# # #         self.get_logger().info("[Phase 2-2] 노랑2x2(Pick) -> 파랑2x2(Base, offset_y=-1)로 결합")
# # #         self.call(self.cli_h, Trigger.Request())
# # #         if self.pick_target("2x2_yellow"):
# # #             self.call(self.cli_h, Trigger.Request())
# # #             if not self.visual_insert("2x2_blue", layer_index=1, offset_studs_y=-1.0):
# # #                 self.get_logger().warn("❌ 상단 모듈 결합 실패")
# # #                 return

# # #         self.get_logger().info(" [Phase 3] 최종 결합: 상단 모듈 들어서 하단 모듈(노랑4x2) 위에 꽂기!")
# # #         self.call(self.cli_h, Trigger.Request())
# # #         if self.pick_target("2x2_yellow", layer_index=0.5):
# # #             self.call(self.cli_h, Trigger.Request())
# # #             if self.visual_insert("4x2_yellow", layer_index=2):
# # #                 self.get_logger().info("✅🎉 5단 아이스크림 완벽하게 완성!")
# # #             else:
# # #                 self.get_logger().warn("❌ 최종 층 올리기 실패")

# # #     def build_studs_y(self):
# # #         self.get_logger().info("🧱 [초기화] 맨 처음 2x2_yellow 위치 스캔 및 기억")
# # #         # 페이즈 4를 위해 노란색 블록의 위치를 가장 먼저 찾아 저장합니다.
# # #         p_yellow = self.find_target_with_retry("2x2_yellow")
# # #         if not p_yellow:
# # #             self.get_logger().warn("❌ 바닥에 2x2_yellow가 안 보입니다. 조립을 취소합니다.")
# # #             return
# # #         saved_yellow_pose = p_yellow
# # #         self.call(self.cli_h, Trigger.Request())

# # #         self.get_logger().info("🧱 [Phase 1] 4x2_red(-1.85) 파지 -> 2x2_red(0.0) 결합 (그리퍼 유지)")
# # #         if self.pick_target("4x2_red", offset_studs_y=-1.84):
# # #             self.call(self.cli_h, Trigger.Request())
            
# # #             # 그리퍼 열지 않고 결합
# # #             if self.visual_insert("2x2_red", layer_index=1, offset_studs_y=0.0, release_gripper=False):
# # #                 self.call(self.cli_h, Trigger.Request())
# # #                 time.sleep(1.0) # 카메라가 흔들림을 잡고 바닥을 볼 수 있도록 약간의 대기

# # #                 self.get_logger().info("🧱 [Phase 2] 바닥의 다른 4x2_red 스캔 및 6x2 조립 (그리퍼 해제)")
# # #                 p_4x2_base = self.find_target_with_retry("4x2_red")
# # #                 if not p_4x2_base:
# # #                     self.get_logger().warn("❌ 바닥에 다른 4x2_red가 안 보입니다.")
# # #                     return
                
# # #                 # 시야가 가려지기 전의 정확한 바닥 좌표를 저장!
# # #                 saved_6x2_pose = p_4x2_base 
                
# # #                 if self.blind_insert(saved_6x2_pose, layer_index=1, offset_studs_y=-3.0, release_gripper=True):
# # #                     self.call(self.cli_h, Trigger.Request())

# # #                     self.get_logger().info("🧱 [Phase 3] 2x2_red 파지 -> 6x2 중심에 결합 (그리퍼 유지)")
# # #                     if self.pick_target("2x2_red", offset_studs_y=-0.2):
# # #                         self.call(self.cli_h, Trigger.Request())
                        
# # #                         # 아까 저장해둔 깨끗한 6x2 베이스 좌표로 블라인드 이동
# # #                         if self.blind_insert(saved_6x2_pose, layer_index=2, offset_studs_y=-1.0, release_gripper=False):
# # #                             self.call(self.cli_h, Trigger.Request())

# # #                             self.get_logger().info("🧱 [Phase 4] 덩어리를 2x2_yellow 중앙에 최종 결합")
# # #                             # 맨 처음에 기억해둔 노란색 블록 위치로 블라인드 이동 후 그리퍼 해제
# # #                             if self.blind_insert(saved_yellow_pose, layer_index=1.5, offset_studs_y=0.0):
# # #                                 self.get_logger().info("✅ 최종 조립 시퀀스 완벽 종료!")

# # #     def run(self):
# # #         self.get_logger().info("🚀 STARTING VISUAL-STACK ASSEMBLY SEQUENCE (Full Recipe Mode)")
# # #         self.call(self.cli_h, Trigger.Request())
# # #         self.call(self.cli_g, SetBool.Request(data=False))
# # #         time.sleep(1.0)
       
# # #         self.get_logger().info("👀 필드 블록 스캔 중...")
# # #         inventory = {
# # #             "2x2_yellow": self.count_color("2x2_yellow"),
# # #             "2x2_blue": self.count_color("2x2_blue"),
# # #             "2x2_red": self.count_color("2x2_red"),
# # #             "2x2_green": self.count_color("2x2_green"),
# # #             "4x2_yellow": self.count_color("4x2_yellow"),
# # #             "4x2_red": self.count_color("4x2_red"),
# # #             "4x2_blue": self.count_color("4x2_blue")
# # #         }
# # #         self.get_logger().info(f"📦 현재 인벤토리: {inventory}")

# # #         best_plan = self.get_best_build_plan(inventory)
       
# # #         if not best_plan:
# # #             self.get_logger().warn("❌ 조립 가능한 조합이 없습니다.")
# # #         else:
# # #             self.get_logger().info(f"🧠 최적 계획: {best_plan}")
# # #             for item in best_plan:
# # #                 self.get_logger().info(f"▶️ 작업 시작: {item.upper()}")
# # #                 if item == 'battery': self.build_battery()
# # #                 elif item == 'studs_y': self.build_studs_y()
# # #                 elif item == 'magnet': self.build_magnet()
# # #                 elif item == 'e_stop': self.build_e_stop()
# # #                 elif item == 'carrot': self.build_carrot()
# # #                 elif item == 'traffic_light': self.build_traffic_light()
# # #                 elif item == 'small_tree': self.build_small_tree()
# # #                 elif item == 'hammer': self.build_hammer()
# # #                 elif item == 'big_carrot': self.build_big_carrot()
# # #                 elif item == 'burger': self.build_burger()
# # #                 elif item == 'ice_cream': self.build_ice_cream()
# # #                 elif item == 'big_tree': self.build_big_tree()
                   
# # #                 self.call(self.cli_h, Trigger.Request())
# # #                 time.sleep(1.0)

# # #         self.call(self.cli_h, Trigger.Request())
# # #         self.get_logger().info("🎉 ALL SEQUENCE DONE")

# # # def main():
# # #     rclpy.init()
# # #     node = MasterNode()
# # #     node.run()
# # #     rclpy.shutdown()

# # # if __name__ == '__main__':
# # #     main()

# import rclpy
# from rclpy.node import Node
# from srvs_pkg.srv import GetTargetPose
# from std_srvs.srv import SetBool, Trigger
# import time
# import math

# class MasterNode(Node):
#     def __init__(self):
#         super().__init__('master_node')
#         self.cli_v = self.create_client(GetTargetPose, '/get_target_pose')
#         self.cli_r = self.create_client(GetTargetPose, '/robot_move_step')
#         self.cli_g = self.create_client(SetBool, '/control_gripper')
#         self.cli_h = self.create_client(Trigger, '/robot_home')
       
#         self.Z_OFF = -100.0
#         self.Z_MARGIN = 20.0
#         self.BLOCK_H = 16.0
#         self.WAIT_TIME = 1.5
       
#         self.STUD_PITCH = 0.016
#         self.YAW_TUNE = 0.0  # 필요시 여기서 영점 조절 (예: -1.5)

#         # vision_node.py는 target_color로 클래스명이 아니라 숫자 ID 문자열만 받는다.
#         self.CLASS_TO_TARGET_ID = {
#             "2x2_red": "1",
#             "2x2_green": "2",
#             "2x2_blue": "3",
#             "2x2_yellow": "4",
#             "4x2_red": "5",
#             "4x2_green": "6",
#             "4x2_blue": "7",
#             "4x2_yellow": "8",
#             "2x4_red": "5",
#             "2x4_green": "6",
#             "2x4_blue": "7",
#             "2x4_yellow": "8",
#             "assembly": "999",
#         }
       
#         # 🌟 가장 깨끗하게 인식되었을 때의 타겟 좌표를 기억하는 변수
#         self.last_perfect_pose = None

#     def call(self, cli, req):
#         while not cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info(f'Waiting for {cli.srv_name}...')
#         future = cli.call_async(req)
#         rclpy.spin_until_future_complete(self, future)
#         return future.result()

#     def to_vision_target_id(self, target):
#         target = str(target).strip()
#         for prefix in ("count_", "far_"):
#             if target.startswith(prefix):
#                 target = target[len(prefix):]

#         if target.isdigit():
#             return target

#         target_id = self.CLASS_TO_TARGET_ID.get(target)
#         if target_id is None:
#             self.get_logger().error(f"❌ vision_node.py ID 매핑 없음: {target}")
#             return target
#         return target_id

#     def request_target_pose(self, target):
#         return self.call(
#             self.cli_v,
#             GetTargetPose.Request(target_color=self.to_vision_target_id(target))
#         )

#     def count_color(self, color):
#         p = self.request_target_pose(color)
#         return 1 if p.success else 0

#     def find_target_with_retry(self, color):
#         """재시도 없이 1회만 즉시 스캔"""
#         p = self.request_target_pose(color)
#         if p.success:
#             return p
#         self.get_logger().error(f"❌ [{color}] 타겟 인식 실패")
#         return None
    
#     def get_dist(self, p1, p2):
#         """두 지점 사이의 평면 거리(m)를 계산"""
#         return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

#     # =========================================================================
#     # Big_Tree,Ice_cream만 pick_fresh_target사용
#     # ==========================================================================
#     def pick_fresh_target(self, color, exclude_pose=None, threshold=0.035, layer_index=0):
#         target_req = f"far_{color}"
        
#         # [1번째 스캔] 홈 위치에서 멀리 있는 타겟 1회만 즉시 스캔
#         target_p = self.request_target_pose(target_req)
#         if not target_p.success:
#             return False

#         # --- 로봇 이동 시작 ---
#         z_move = (target_p.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)

#         # 🌟 [통합 이동] YAW, X, Y, Z(Z_MARGIN 위까지 대각선 하강) 한번에 이동
#         self.get_logger().info("➡️ [통합 이동] YAW, X, Y, Z 한번에 접근")
#         req_all = GetTargetPose.Request()
#         req_all.x = target_p.x
#         req_all.y = target_p.y
#         req_all.z = z_move - self.Z_MARGIN
#         req_all.yaw = target_p.yaw
#         req_all.target_size = "ALL"
#         self.call(self.cli_r, req_all)
#         time.sleep(self.WAIT_TIME)

#         # [수직 최종 접근]
#         self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
#         time.sleep(self.WAIT_TIME)

#         self.call(self.cli_g, SetBool.Request(data=True))
#         time.sleep(self.WAIT_TIME)
#         self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
#         return True


#     def pick_target(self, color, layer_index=0, offset_studs_x=0.0, offset_studs_y=0.0, exclude_pose=None):
#         self.get_logger().info(f"\n--- PICK TARGET: [{color.upper()}] ---")
        
#         # [1번째 스캔] 홈 위치에서 타겟의 전체 좌표 확보
#         p = self.find_target_with_retry(color)
#         if not p: return False
       
#         # 오프셋 계산
#         dx = offset_studs_x * self.STUD_PITCH
#         dy = offset_studs_y * self.STUD_PITCH
#         yaw_rad = math.radians(p.yaw)
#         real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
#         real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)

#         target_x = p.x + real_offset_x
#         target_y = p.y + real_offset_y
#         z_move = (p.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)

#         # 🌟 [통합 이동] YAW, X, Y, Z(Z_MARGIN 위까지 대각선 하강) 한번에 이동
#         self.get_logger().info("➡️ [통합 이동] YAW, X, Y, Z 한번에 접근")
#         req_all = GetTargetPose.Request()
#         req_all.x = target_x
#         req_all.y = target_y
#         req_all.z = z_move - self.Z_MARGIN
#         req_all.yaw = p.yaw
#         req_all.target_size = "ALL"
#         self.call(self.cli_r, req_all)
#         time.sleep(self.WAIT_TIME)
        
#         # [수직 최종 접근]
#         self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
#         time.sleep(self.WAIT_TIME)

#         self.call(self.cli_g, SetBool.Request(data=True))
#         time.sleep(self.WAIT_TIME)
#         self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
#         time.sleep(self.WAIT_TIME)
#         return True

#     def blind_insert(self, base_pose, layer_index, yaw_offset=0.0, release_gripper=True, regrip=False, offset_studs_x=0.0, offset_studs_y=0.0):
#         self.get_logger().info(f"\n--- BLIND STACK (메모리 사용): Layer {layer_index} (Y Offset: {offset_studs_y}) ---")
#         time.sleep(1.0)

#         # 오프셋 계산
#         dx = offset_studs_x * self.STUD_PITCH
#         dy = offset_studs_y * self.STUD_PITCH
#         yaw_rad = math.radians(base_pose.yaw)
#         real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
#         real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)

#         target_x = base_pose.x + real_offset_x
#         target_y = base_pose.y + real_offset_y
        
#         target_yaw = base_pose.yaw + yaw_offset + self.YAW_TUNE
#         while target_yaw > 90.0: target_yaw -= 180.0
#         while target_yaw < -90.0: target_yaw += 180.0

#         z_move = (base_pose.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)
       
#         # 🌟 [통합 이동] YAW, X, Y, Z(Z_MARGIN 위까지 대각선 하강) 한번에 이동
#         self.get_logger().info(f"➡️ [통합 이동] 계산된 목표지점으로 한번에 접근 (Yaw: {target_yaw:.1f})")
#         req_all = GetTargetPose.Request()
#         req_all.x = target_x
#         req_all.y = target_y
#         req_all.z = z_move - self.Z_MARGIN
#         req_all.yaw = target_yaw
#         req_all.target_size = "ALL"
#         self.call(self.cli_r, req_all)
#         time.sleep(self.WAIT_TIME)

#         # [수직 최종 접근]
#         self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN , target_size="Z"))
#         time.sleep(self.WAIT_TIME)

#         if release_gripper:
#             self.call(self.cli_g, SetBool.Request(data=False))
#             time.sleep(self.WAIT_TIME)
#         return True

#     def visual_insert(self, target_color, layer_index, release_gripper=True, yaw_offset=0.0, offset_studs_x=0.0, offset_studs_y=0.0):
#         self.get_logger().info(f"\n--- VISUAL STACK: [{target_color.upper()}] (Layer +{layer_index}, Y Offset: {offset_studs_y}) ---")
#         time.sleep(1.0)

#         p = self.find_target_with_retry(target_color)
#         if not p: return False
       
#         # 메모리 로직
#         self.last_perfect_pose = p

#         # 오프셋 계산
#         target_yaw = p.yaw + yaw_offset + self.YAW_TUNE
#         while target_yaw > 90.0: target_yaw -= 180.0
#         while target_yaw < -90.0: target_yaw += 180.0

#         dx = offset_studs_x * self.STUD_PITCH
#         dy = offset_studs_y * self.STUD_PITCH
#         yaw_rad = math.radians(p.yaw)
#         real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
#         real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)

#         target_x = p.x + real_offset_x
#         target_y = p.y + real_offset_y
#         z_move = (p.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)
       
#         # 🌟 [통합 이동] YAW, X, Y, Z(Z_MARGIN 위까지 대각선 하강) 한번에 이동
#         self.get_logger().info(f"➡️ [통합 이동] 시각 보정 기반 목표지점으로 한번에 접근 (Yaw: {target_yaw:.1f})")
#         req_all = GetTargetPose.Request()
#         req_all.x = target_x
#         req_all.y = target_y
#         req_all.z = z_move - self.Z_MARGIN
#         req_all.yaw = target_yaw
#         req_all.target_size = "ALL"
#         self.call(self.cli_r, req_all)
#         time.sleep(self.WAIT_TIME)

#         # [수직 최종 접근]
#         self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN , target_size="Z"))
#         time.sleep(self.WAIT_TIME)

#         if release_gripper:
#             self.call(self.cli_g, SetBool.Request(data=False))
#             time.sleep(self.WAIT_TIME)
#         self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
#         time.sleep(self.WAIT_TIME)
#         return True

#     # --- 2~3개 조합 (Visual Insert 적용) ---

#     def build_battery(self):
#         self.get_logger().info("🔋 [배터리] 노란색(Pick) -> 파란색(Base)")
#         if self.pick_target("2x2_yellow"):
#             self.call(self.cli_h, Trigger.Request())
#             self.visual_insert("2x2_blue", layer_index=1)
#             self.get_logger().info("✅ 배터리 조립 완료!")

#     def build_magnet(self):
#         self.get_logger().info("🧲 [자석] 파란색(Pick) -> 빨간색(Base)")
#         if self.pick_target("2x2_blue"):
#             self.call(self.cli_h, Trigger.Request())
#             self.visual_insert("2x2_red", layer_index=1)
#             self.get_logger().info("✅ 자석 조립 완료!")

#     def build_e_stop(self):
#         self.get_logger().info("🛑 [비상정지] 빨간색(Pick) -> 노란색4x2(Base)")
#         if self.pick_target("2x2_red"):
#             self.call(self.cli_h, Trigger.Request())
#             self.visual_insert("4x2_yellow", layer_index=1, yaw_offset=-90.0)
#             self.get_logger().info("✅ 비상정지 조립 완료!")

#     def build_carrot(self):
#         self.get_logger().info("🥕 [당근] 노란색(Pick) -> 파란색(Base) -> 노란색(Pick)")
#         if self.pick_target("2x2_yellow"):
#             self.call(self.cli_h, Trigger.Request())
#             if self.visual_insert("2x2_blue", layer_index=1):
#                 self.call(self.cli_h, Trigger.Request())
#                 if self.pick_target("2x2_yellow"):
#                     self.call(self.cli_h, Trigger.Request())
#                     self.visual_insert("2x2_yellow", layer_index=1)
#                     self.get_logger().info("✅ 당근 완성!")

#     def build_traffic_light(self):
#         self.get_logger().info("🚦 [신호등] 노란색(Pick) -> 파란색(Base) -> 빨간색(Pick)")
#         if self.pick_target("2x2_yellow"):
#             self.call(self.cli_h, Trigger.Request())
#             if self.visual_insert("2x2_blue", layer_index=1):
#                 self.call(self.cli_h, Trigger.Request())
#                 if self.pick_target("2x2_red"):
#                     self.call(self.cli_h, Trigger.Request())
#                     self.visual_insert("2x2_yellow", layer_index=1)
#                     self.get_logger().info("✅ 신호등 완성!")

#     def build_small_tree(self):
#         self.get_logger().info("🌳 [작은 나무] 빨강4x2(Pick) -> 노랑2x2(Base) -> 빨강2x2(Pick)")
#         if self.pick_target("4x2_red"):
#             self.call(self.cli_h, Trigger.Request())
#             if self.visual_insert("2x2_yellow", layer_index=1):
#                 self.call(self.cli_h, Trigger.Request())
#                 if self.pick_target("2x2_red"):
#                     self.call(self.cli_h, Trigger.Request())
#                     self.visual_insert("4x2_red", layer_index=1)
#                     self.get_logger().info("✅ 작은 나무 완성!")

#     def build_hammer(self):
#         self.get_logger().info("🔨 [망치] 빨강2x2(Pick) -> 빨강2x2(Base) -> 파랑4x2(Pick)")
#         if self.pick_target("2x2_red"):
#             self.call(self.cli_h, Trigger.Request())
#             if self.visual_insert("2x2_red", layer_index=1):
#                 self.call(self.cli_h, Trigger.Request())
#                 if self.pick_target("4x2_blue"):
#                     self.call(self.cli_h, Trigger.Request())
#                     self.visual_insert("2x2_red", layer_index=1, yaw_offset=0.0)
#                     self.get_logger().info("✅ 망치 완성!")

#     # --- 4개 조합 (Big Carrot, Burger) ---
#     def build_big_carrot(self):
#         self.get_logger().info("🥕🥕 [큰 당근] 노랑2x2(Pick) -> 노랑2x2(Base) -> 노랑4x2(Pick) -> 파랑2x2(Pick)")
#         if self.pick_target("2x2_yellow"):
#             self.call(self.cli_h, Trigger.Request())
#             if self.visual_insert("2x2_yellow", layer_index=1):
#                 self.call(self.cli_h, Trigger.Request())
#                 if self.pick_target("4x2_yellow"):
#                     self.call(self.cli_h, Trigger.Request())
#                     if self.visual_insert("2x2_yellow", layer_index=2, yaw_offset=0.0):
#                         self.call(self.cli_h, Trigger.Request())
#                         if self.pick_target("2x2_blue"):
#                             self.call(self.cli_h, Trigger.Request())
#                             self.visual_insert("4x2_yellow", layer_index=1)
#                             self.get_logger().info("✅ 대왕 당근 완성!")

#     def build_burger(self):
#         self.get_logger().info("🍔 [버거] 노랑4x2(Base) -> 빨강4x2(Offset Y -1) -> 빨강2x2(Offset Y +2) -> 노랑4x2(Top)")
#         if self.pick_target("4x2_red"):
#             self.call(self.cli_h, Trigger.Request())
#             if self.visual_insert("4x2_yellow", layer_index=1, offset_studs_y=-1.0):
#                 saved_base_bun_pose = self.last_perfect_pose
#                 self.call(self.cli_h, Trigger.Request())
#                 if self.pick_target("2x2_red"):
#                     self.call(self.cli_h, Trigger.Request())
#                     if self.visual_insert("4x2_red", layer_index=0, yaw_offset=0.0, offset_studs_y=3.0):
#                         self.call(self.cli_h, Trigger.Request())
#                         if self.pick_target("4x2_yellow"):
#                             self.call(self.cli_h, Trigger.Request())
#                             if saved_base_bun_pose:
#                                 self.get_logger().info("🧠 [메모리 사용] 최초 바닥 빵의 좌표를 기억해서 정중앙에 덮습니다!")
#                                 self.blind_insert(saved_base_bun_pose, layer_index=2, offset_studs_y=1.0)
#                                 self.get_logger().info("✅ 버거 완성!")
#                             else:
#                                 self.get_logger().warn("❌ 저장된 좌표가 없습니다. 조립 실패.")

#     def build_ice_cream(self):
#         self.get_logger().info("🍦 [아이스크림] 모듈형 조립 전략")
#         self.get_logger().info("[Phase 1] 하단 조립: 노랑4x2(Pick) -> 노랑2x2(Base)")
#         if self.pick_target("4x2_yellow"):
#             self.call(self.cli_h, Trigger.Request())
#             if not self.visual_insert("2x2_yellow", layer_index=1):
#                 self.get_logger().warn("❌ 하단 모듈 조립 실패")
#                 return

#         self.get_logger().info("[Phase 2-1] 파랑2x2(Pick) -> 빨강2x2 옆에 배치 (바닥)")
#         self.call(self.cli_h, Trigger.Request())
#         if self.pick_target("2x2_blue"):
#             self.call(self.cli_h, Trigger.Request())
#             if not self.visual_insert("2x2_red", layer_index=0, offset_studs_y=2.0):
#                 self.get_logger().warn("❌ 파란색 블록 배치 실패")
#                 return

#         self.get_logger().info("[Phase 2-2] 노랑2x2(Pick) -> 파랑2x2(Base, offset_y=-1)로 결합")
#         self.call(self.cli_h, Trigger.Request())
#         if self.pick_target("2x2_yellow"):
#             self.call(self.cli_h, Trigger.Request())
#             if not self.visual_insert("2x2_blue", layer_index=1, offset_studs_y=-1.0):
#                 self.get_logger().warn("❌ 상단 모듈 결합 실패")
#                 return

#         self.get_logger().info(" [Phase 3] 최종 결합: 상단 모듈 들어서 하단 모듈(노랑4x2) 위에 꽂기!")
#         self.call(self.cli_h, Trigger.Request())
#         if self.pick_target("2x2_yellow", layer_index=0.5):
#             self.call(self.cli_h, Trigger.Request())
#             if self.visual_insert("4x2_yellow", layer_index=2):
#                 self.get_logger().info("✅🎉 5단 아이스크림 완벽하게 완성!")
#             else:
#                 self.get_logger().warn("❌ 최종 층 올리기 실패")

#     def build_studs_y(self):
#         self.get_logger().info("🧱 [초기화] 맨 처음 2x2_yellow 위치 스캔 및 기억")
#         p_yellow = self.find_target_with_retry("2x2_yellow")
#         if not p_yellow:
#             self.get_logger().warn("❌ 바닥에 2x2_yellow가 안 보입니다. 조립을 취소합니다.")
#             return
#         saved_yellow_pose = p_yellow
#         self.call(self.cli_h, Trigger.Request())

#         self.get_logger().info("🧱 [Phase 1] 4x2_red(-1.85) 파지 -> 2x2_red(0.0) 결합 (그리퍼 유지)")
#         if self.pick_target("4x2_red", offset_studs_y=-1.84):
#             self.call(self.cli_h, Trigger.Request())
#             if self.visual_insert("2x2_red", layer_index=1, offset_studs_y=0.0, release_gripper=False):
#                 self.call(self.cli_h, Trigger.Request())
#                 time.sleep(1.0) 

#                 self.get_logger().info("🧱 [Phase 2] 바닥의 다른 4x2_red 스캔 및 6x2 조립 (그리퍼 해제)")
#                 p_4x2_base = self.find_target_with_retry("4x2_red")
#                 if not p_4x2_base:
#                     self.get_logger().warn("❌ 바닥에 다른 4x2_red가 안 보입니다.")
#                     return
#                 saved_6x2_pose = p_4x2_base 
                
#                 if self.blind_insert(saved_6x2_pose, layer_index=1, offset_studs_y=-3.0, release_gripper=True):
#                     self.call(self.cli_h, Trigger.Request())

#                     self.get_logger().info("🧱 [Phase 3] 2x2_red 파지 -> 6x2 중심에 결합 (그리퍼 유지)")
#                     if self.pick_target("2x2_red", offset_studs_y=-0.2):
#                         self.call(self.cli_h, Trigger.Request())
#                         if self.blind_insert(saved_6x2_pose, layer_index=2, offset_studs_y=-1.0, release_gripper=False):
#                             self.call(self.cli_h, Trigger.Request())

#                             self.get_logger().info("🧱 [Phase 4] 덩어리를 2x2_yellow 중앙에 최종 결합")
#                             if self.blind_insert(saved_yellow_pose, layer_index=1.5, offset_studs_y=0.0):
#                                 self.get_logger().info("✅ 최종 조립 시퀀스 완벽 종료!")

#     # 🌟 키보드 입력 기반 조립 메뉴 로직으로 변경
#     def run(self):
#         self.get_logger().info("🚀 STARTING VISUAL-STACK ASSEMBLY SEQUENCE (Keyboard Select Mode)")
#         self.call(self.cli_h, Trigger.Request())
#         self.call(self.cli_g, SetBool.Request(data=False))
#         time.sleep(1.0)
       
#         actions = {
#             "1": self.build_battery, "battery": self.build_battery, "배터리": self.build_battery,
#             "2": self.build_magnet, "magnet": self.build_magnet, "자석": self.build_magnet,
#             "3": self.build_e_stop, "estop": self.build_e_stop, "비상정지": self.build_e_stop,
#             "4": self.build_carrot, "carrot": self.build_carrot, "당근": self.build_carrot,
#             "5": self.build_traffic_light, "traffic": self.build_traffic_light, "신호등": self.build_traffic_light,
#             "6": self.build_small_tree, "tree": self.build_small_tree, "작은나무": self.build_small_tree,
#             "7": self.build_hammer, "hammer": self.build_hammer, "망치": self.build_hammer,
#             "8": self.build_big_carrot, "bigcarrot": self.build_big_carrot, "큰당근": self.build_big_carrot,
#             "9": self.build_burger, "burger": self.build_burger, "버거": self.build_burger,
#             "10": self.build_ice_cream, "icecream": self.build_ice_cream, "아이스크림": self.build_ice_cream,
#             "11": self.build_studs_y, "studs_y": self.build_studs_y
#         }

#         print("\n=== Master Node Assembly Keyboard Select ===")
#         print("1: 배터리 / 2: 자석 / 3: 비상정지 / 4: 당근 / 5: 신호등 / 6: 작은나무")
#         print("7: 망치 / 8: 큰당근 / 9: 버거 / 10: 아이스크림 / 11: studs_y")
#         print("q: 종료")

#         while rclpy.ok():
#             user_input = input("\n조립할 항목을 선택하세요 [1~11/q]: ").strip().replace(" ", "").lower()
#             if user_input in ("q", "quit", "exit", "종료"):
#                 self.get_logger().info("조립 시퀀스를 종료합니다.")
#                 break

#             action = actions.get(user_input)
#             if action is None:
#                 print("잘못된 입력입니다. 1~11 또는 q 중에서 선택하세요.")
#                 continue

#             self.get_logger().info(f"▶️ 작업 시작: {user_input}")
#             action()
            
#             self.call(self.cli_h, Trigger.Request())
#             time.sleep(1.0)
#             self.get_logger().info("✅ 개별 조립 완료")

#         self.call(self.cli_h, Trigger.Request())
#         self.get_logger().info("🎉 ALL SEQUENCE DONE")

# def main():
#     rclpy.init()
#     node = MasterNode()
#     node.run()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


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
       
        self.Z_OFF = -100.0
        self.Z_MARGIN = 20.0
        self.BLOCK_H = 16.0
        self.WAIT_TIME = 2.5
       
        self.STUD_PITCH = 0.016
        self.YAW_TUNE = 0.0

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

    def count_color(self, color):
        p = self.request_target_pose(color)
        return 1 if p.success else 0

    def find_target_with_retry(self, color):
        p = self.request_target_pose(color)
        if p.success:
            return p
        self.get_logger().error(f"❌ [{color}] 타겟 인식 실패")
        return None
    
    def get_dist(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def pick_fresh_target(self, color, exclude_pose=None, threshold=0.035, layer_index=0):
        target_req = f"far_{color}"
        
        target_p = self.request_target_pose(target_req)
        if not target_p.success:
            return False

        z_move = (target_p.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)

        self.get_logger().info("➡️ [통합 이동] YAW, X, Y, Z 한번에 접근")
        req_all = GetTargetPose.Request()
        req_all.x = target_p.x
        req_all.y = target_p.y
        req_all.z = z_move - self.Z_MARGIN
        req_all.yaw = target_p.yaw
        req_all.target_size = "ALL"
        self.call(self.cli_r, req_all)
        time.sleep(self.WAIT_TIME)

        self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME)

        self.call(self.cli_g, SetBool.Request(data=True))
        time.sleep(self.WAIT_TIME)
        
        self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
        time.sleep(self.WAIT_TIME)
        return True


    def pick_target(self, color, layer_index=0, offset_studs_x=0.0, offset_studs_y=0.0, exclude_pose=None):
        self.get_logger().info(f"\n--- PICK TARGET: [{color.upper()}] ---")
        
        p = self.find_target_with_retry(color)
        if not p: return False
       
        dx = offset_studs_x * self.STUD_PITCH
        dy = offset_studs_y * self.STUD_PITCH
        yaw_rad = math.radians(p.yaw)
        real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
        real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)

        target_x = p.x * 1000.0 + real_offset_x # 수정 된거 1000곱했음
        target_y = p.y * 1000.0 + real_offset_y
        z_move = (p.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)

        self.get_logger().info("➡️ [통합 이동] YAW, X, Y, Z 한번에 접근")
        req_all = GetTargetPose.Request()
        req_all.x = target_x
        req_all.y = target_y
        req_all.z = z_move - self.Z_MARGIN
        req_all.yaw = p.yaw
        req_all.target_size = "ALL"
        self.call(self.cli_r, req_all)
        time.sleep(self.WAIT_TIME)
        
        self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME)

        self.call(self.cli_g, SetBool.Request(data=True))
        time.sleep(self.WAIT_TIME)
        
        self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
        time.sleep(self.WAIT_TIME)
        return True

    def blind_insert(self, base_pose, layer_index, yaw_offset=0.0, release_gripper=True, regrip=False, offset_studs_x=0.0, offset_studs_y=0.0):
        self.get_logger().info(f"\n--- BLIND STACK (메모리 사용): Layer {layer_index} (Y Offset: {offset_studs_y}) ---")
        time.sleep(1.0)

        dx = offset_studs_x * self.STUD_PITCH
        dy = offset_studs_y * self.STUD_PITCH
        yaw_rad = math.radians(base_pose.yaw)
        real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
        real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)

        target_x = base_pose.x + real_offset_x
        target_y = base_pose.y + real_offset_y
        
        target_yaw = base_pose.yaw + yaw_offset + self.YAW_TUNE
        while target_yaw > 90.0: target_yaw -= 180.0
        while target_yaw < -90.0: target_yaw += 180.0

        z_move = (base_pose.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)
       
        self.get_logger().info(f"➡️ [통합 이동] 계산된 목표지점으로 한번에 접근 (Yaw: {target_yaw:.1f})")
        req_all = GetTargetPose.Request()
        req_all.x = target_x
        req_all.y = target_y
        req_all.z = z_move - self.Z_MARGIN
        req_all.yaw = target_yaw
        req_all.target_size = "ALL"
        self.call(self.cli_r, req_all)
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
        self.get_logger().info(f"\n--- VISUAL STACK: [{target_color.upper()}] (Layer +{layer_index}, Y Offset: {offset_studs_y}) ---")
        time.sleep(1.0)

        p = self.find_target_with_retry(target_color)
        if not p: return False
       
        self.last_perfect_pose = p

        target_yaw = p.yaw + yaw_offset + self.YAW_TUNE
        while target_yaw > 90.0: target_yaw -= 180.0
        while target_yaw < -90.0: target_yaw += 180.0

        dx = offset_studs_x * self.STUD_PITCH
        dy = offset_studs_y * self.STUD_PITCH
        yaw_rad = math.radians(p.yaw)
        real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
        real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)

        target_x = p.x + real_offset_x
        target_y = p.y + real_offset_y
        z_move = (p.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)
       
        self.get_logger().info(f"➡️ [통합 이동] 시각 보정 기반 목표지점으로 한번에 접근 (Yaw: {target_yaw:.1f})")
        req_all = GetTargetPose.Request()
        req_all.x = target_x
        req_all.y = target_y
        req_all.z = z_move - self.Z_MARGIN
        req_all.yaw = target_yaw
        req_all.target_size = "ALL"
        self.call(self.cli_r, req_all)
        time.sleep(self.WAIT_TIME)

        self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN , target_size="Z"))
        time.sleep(self.WAIT_TIME)

        if release_gripper:
            self.call(self.cli_g, SetBool.Request(data=False))
            time.sleep(self.WAIT_TIME)
            
        self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
        time.sleep(self.WAIT_TIME)
        return True

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
        if self.pick_target("4x2_red"):
            self.call(self.cli_h, Trigger.Request())
            if self.visual_insert("2x2_yellow", layer_index=1):
                self.call(self.cli_h, Trigger.Request())
                if self.pick_target("2x2_red"):
                    self.call(self.cli_h, Trigger.Request())
                    self.visual_insert("4x2_red", layer_index=1)
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

    def run(self):
        self.get_logger().info("🚀 STARTING VISUAL-STACK ASSEMBLY SEQUENCE (Keyboard Select Mode)")
        self.call(self.cli_h, Trigger.Request())
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