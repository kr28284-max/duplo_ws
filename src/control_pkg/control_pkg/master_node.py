# import rclpy
# from rclpy.node import Node
# from srvs_pkg.srv import GetTargetPose
# from std_srvs.srv import SetBool, Trigger
# import time

# class MasterNode(Node):
#     def __init__(self):
#         super().__init__('master_node')
#         self.cli_v = self.create_client(GetTargetPose, '/get_target_pose')
#         self.cli_r = self.create_client(GetTargetPose, '/robot_move_step')
#         self.cli_g = self.create_client(SetBool, '/control_gripper')
#         self.cli_h = self.create_client(Trigger, '/robot_home')
        
#         self.Z_OFF = -85.0
#         self.Z_MARGIN = 20.0
#         self.BLOCK_H = 16.0
#         self.WAIT_TIME = 1.5 

#     def call(self, cli, req):
#         while not cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info(f'Waiting for {cli.srv_name}...')
#         future = cli.call_async(req)
#         rclpy.spin_until_future_complete(self, future)
#         return future.result()

#     def check_color_exists(self, color):
#         p = self.call(self.cli_v, GetTargetPose.Request(target_color=color))
#         return p.success

#     def find_target_with_retry(self, color, retries=4):
#         for i in range(retries):
#             p = self.call(self.cli_v, GetTargetPose.Request(target_color=color))
#             if p.success:
#                 return p
#             self.get_logger().warn(f"⚠️ [{color}] 찾는 중... 대기 ({i+1}/{retries})")
#             time.sleep(1.0) 
#         return None

#     def pick_target(self, color):
#         self.get_logger().info(f"\n--- PICK TARGET: [{color.upper()}] ---")
        
#         # 1. YAW
#         p = self.find_target_with_retry(color)
#         if not p: return False
#         req = GetTargetPose.Request(); req.yaw = p.yaw; req.target_size = "YAW"
#         self.call(self.cli_r, req)
#         time.sleep(self.WAIT_TIME) 

#         # 2. XY
#         p = self.find_target_with_retry(color)
#         if not p: return False
#         req = GetTargetPose.Request(); req.x = p.x; req.y = p.y; req.target_size = "XY"
#         self.call(self.cli_r, req)
#         time.sleep(self.WAIT_TIME) 

#         # 3. Z
#         p = self.find_target_with_retry(color)
#         if not p: return False
#         z_move = p.z * 1000.0 + self.Z_OFF
#         self.call(self.cli_r, GetTargetPose.Request(z=z_move - self.Z_MARGIN, target_size="Z"))
#         time.sleep(self.WAIT_TIME) 
#         self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
#         time.sleep(self.WAIT_TIME) 

#         # 4. Gripper Close
#         self.call(self.cli_g, SetBool.Request(data=True))
#         time.sleep(self.WAIT_TIME) 
#         self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
#         time.sleep(self.WAIT_TIME) 
#         return True

#     # 💡 새로 추가된 함수: 로봇을 고정된 특정 좌표로 이동시켜 블록을 내려놓습니다.
#     def place_at_fixed_position(self, x, y, z, yaw):
#         self.get_logger().info(f"\n--- DROP AT FIXED POSE (X:{x}, Y:{y}, Z:{z}, YAW:{yaw}) ---")
#         time.sleep(1.0)

#         # 1. YAW 이동
#         req_y = GetTargetPose.Request()
#         req_y.yaw = yaw 
#         req_y.target_size = "YAW"
#         self.call(self.cli_r, req_y)
#         time.sleep(self.WAIT_TIME) 
        
#         # 2. XY 이동
#         req_xy = GetTargetPose.Request()
#         req_xy.x = x
#         req_xy.y = y
#         req_xy.target_size = "XY"
#         self.call(self.cli_r, req_xy)
#         time.sleep(self.WAIT_TIME) 

#         # 3. Z 이동 (단위: 기존 코드의 z값 단위와 맞추어 사용)
#         # z가 mm 단위라면 아래 수식을 알맞게 변경해야 할 수 있습니다. 
#         # (기존 코드를 참고하여 z * 1000 형식 유지)
#         z_move = (z * 1000.0 + self.Z_OFF) - self.BLOCK_H
#         self.call(self.cli_r, GetTargetPose.Request(z=z_move - self.Z_MARGIN, target_size="Z"))
#         time.sleep(self.WAIT_TIME) 
#         self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
#         time.sleep(self.WAIT_TIME) 

#         # 4. Gripper Open (놓기)
#         self.call(self.cli_g, SetBool.Request(data=False))
#         time.sleep(self.WAIT_TIME) 
        
#         # 5. Z 상승 후 빠져나오기
#         self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
#         time.sleep(self.WAIT_TIME) 
#         return True


#     def run(self):
#         self.get_logger().info("🚀 STARTING 3 BLUE BLOCKS PICK & PLACE SEQUENCE")
        
#         self.call(self.cli_h, Trigger.Request())
#         self.call(self.cli_g, SetBool.Request(data=False))
#         time.sleep(1.0) 
        
#         # 🎯 타겟 색상 정의 (상황에 맞게 이름 변경 필요)
#         target_color = "2x2_blue"   
        
#         # 🎯 블록을 놓을 단일 고정 좌표 (❗실제 환경 로봇 좌표에 맞게 꼭 수정하세요)
#         # 단위: X, Y, Z는 기존 p.x, p.y 데이터와 동일한 스케일을 사용해야 합니다.
#         DROP_X = 0.40   
#         DROP_Y = 0.05  
#         DROP_Z = 0.50     #0.5
#         DROP_YAW = 0.0   

#         # 최대 3개의 파란색 블록을 차례로 집어서 옮깁니다.
#         for i in range(1, 4):
#             self.get_logger().info(f"🔍 [{i}/3] 파란색 블록을 스캔합니다...")
            
#             # 파란색 블록이 시야에 있는지 확인
#             if self.check_color_exists(target_color):
#                 self.get_logger().info(f"🎯 [{i}/3] 파란색 블록 발견! 픽업 시작.")
                
#                 if self.pick_target(target_color):
#                     self.call(self.cli_h, Trigger.Request()) 
                    
#                     # 고정 좌표에 블록 내려놓기
#                     if self.place_at_fixed_position(DROP_X, DROP_Y, DROP_Z, DROP_YAW):
#                         self.get_logger().info(f"✅ [{i}/3] 지정된 위치에 블록 놓기 완료!")
#                     else:
#                         self.get_logger().error(f"❌ [{i}/3] 블록 놓기 실패!")
#                 else:
#                     self.get_logger().error(f"❌ [{i}/3] 픽업 실패!")
#             else:
#                 # 더 이상 파란 블록이 없으면 조기 종료
#                 self.get_logger().info(f"⏭️ 시야에 파란색 블록이 더 이상 없습니다. (현재 {i-1}개 이동 완료)")
#                 break
                
#             # 하나를 완료한 뒤 홈으로 복귀하여 다음 스캔 준비
#             self.call(self.cli_h, Trigger.Request())
#             time.sleep(1.0)

#         # 모든 루프 종료
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
        # 서비스 클라이언트 설정
        self.cli_v = self.create_client(GetTargetPose, '/get_target_pose')
        self.cli_r = self.create_client(GetTargetPose, '/robot_move_step')
        self.cli_g = self.create_client(SetBool, '/control_gripper')
        self.cli_h = self.create_client(Trigger, '/robot_home')
        
        # --- 🛠️ [이 값들을 직접 튜닝하세요! 단위: m] ---
        self.CAM_TO_GRIPPER_X = 0.00
        self.CAM_TO_GRIPPER_Y = 0.001
        self.CAM_TO_GRIPPER_Z = -0.003 
        
        # --- [기존 환경 변수 유지 (단위: mm)] ---
        self.Z_OFF = -85.0
        self.Z_MARGIN = 20.0
        self.BLOCK_H = 16.0
        self.WAIT_TIME = 1.5 
        
        # --- [가동 범위 판단 기준 (단위: m)] ---
        self.MAX_REACH = 0.65 
        self.MIN_REACH = 0.12

    def call(self, cli, req):
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {cli.srv_name}...')
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def is_reachable(self, x, y, z):
        """판단 로직: 로봇 베이스 기준 좌표가 가동 범위 내인지 확인하고 메시지 출력"""
        distance = math.sqrt(x**2 + y**2 + z**2)
        
        if distance > self.MAX_REACH:
            self.get_logger().error(f"❌ [작업 불가] 물체가 너무 멉니다! (현재 거리: {distance:.3f}m / 제한: {self.MAX_REACH}m)")
            return False
        if distance < self.MIN_REACH:
            self.get_logger().error(f"❌ [작업 불가] 물체가 너무 가깝습니다! (현재 거리: {distance:.3f}m / 최소: {self.MIN_REACH}m)")
            return False
            
        self.get_logger().info(f"✅ [작업 가능] 거리 확인됨: {distance:.3f}m")
        return True

    # 💡 [수정] retries 값을 4에서 2로 변경
    def find_target_with_retry(self, color, retries=2):
        for i in range(retries):
            p = self.call(self.cli_v, GetTargetPose.Request(target_color=color))
            if p.success: return p
            self.get_logger().warn(f"⚠️ [{color}] 스캔 중... ({i+1}/{retries})")
            time.sleep(1.0) 
        return None

    def check_color_exists(self, color):
        p = self.call(self.cli_v, GetTargetPose.Request(target_color=color))
        return p.success

    def pick_target(self, color):
        self.get_logger().info(f"\n--- PICK TARGET: [{color.upper()}] ---")
        
        # 1. YAW 이동 전 판단
        p = self.find_target_with_retry(color)
        if not p: return False
        
        if not self.is_reachable(p.x + self.CAM_TO_GRIPPER_X, p.y + self.CAM_TO_GRIPPER_Y, p.z - self.CAM_TO_GRIPPER_Z):
            return False

        req = GetTargetPose.Request(); req.yaw = p.yaw; req.target_size = "YAW"
        self.call(self.cli_r, req)
        time.sleep(self.WAIT_TIME) 

        # 2. XY 이동 (보정 적용)
        p = self.find_target_with_retry(color)
        if not p: return False
        
        corrected_x = p.x + self.CAM_TO_GRIPPER_X 
        corrected_y = p.y + self.CAM_TO_GRIPPER_Y 
        
        if not self.is_reachable(corrected_x, corrected_y, p.z - self.CAM_TO_GRIPPER_Z):
            return False

        req = GetTargetPose.Request(); req.x = corrected_x; req.y = corrected_y; req.target_size = "XY"
        self.call(self.cli_r, req)
        time.sleep(self.WAIT_TIME) 

        # 3. Z 이동 (보정 적용)
        p = self.find_target_with_retry(color)
        if not p: return False
        
        corrected_z = p.z - self.CAM_TO_GRIPPER_Z
        z_move = corrected_z * 1000.0 + self.Z_OFF
        
        self.call(self.cli_r, GetTargetPose.Request(z=z_move - self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME) 
        self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME) 

        # 4. Gripper 작동
        self.call(self.cli_g, SetBool.Request(data=True))
        time.sleep(self.WAIT_TIME) 
        self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
        time.sleep(self.WAIT_TIME) 
        return True

    def place_at_fixed_position(self, x, y, z, yaw):
        self.get_logger().info(f"\n--- DROP AT FIXED POSE ---")
        if not self.is_reachable(x, y, z): return False

        self.call(self.cli_r, GetTargetPose.Request(yaw=yaw, target_size="YAW"))
        time.sleep(self.WAIT_TIME) 
        self.call(self.cli_r, GetTargetPose.Request(x=x, y=y, target_size="XY"))
        time.sleep(self.WAIT_TIME) 

        z_move = (z * 1000.0 + self.Z_OFF) - self.BLOCK_H
        self.call(self.cli_r, GetTargetPose.Request(z=z_move - self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME) 
        self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME) 

        self.call(self.cli_g, SetBool.Request(data=False))
        time.sleep(self.WAIT_TIME) 
        self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
        time.sleep(self.WAIT_TIME) 
        return True

    def run(self):
        self.get_logger().info("🚀 STARTING SEQUENCE (TUNING MODE)")
        self.call(self.cli_h, Trigger.Request()) 
        self.call(self.cli_g, SetBool.Request(data=False)) 
        time.sleep(1.0) 
        
        target_color = "2x2_blue"   
        DROP_X, DROP_Y, DROP_Z, DROP_YAW = 0.40, 0.05, 0.35, 0.0   

        for i in range(1, 4):
            if self.check_color_exists(target_color):
                if self.pick_target(target_color):
                    self.call(self.cli_h, Trigger.Request()) 
                    self.place_at_fixed_position(DROP_X, DROP_Y, DROP_Z, DROP_YAW)
                else:
                    self.get_logger().error(f"❌ [{i}/3] 작업 범위 밖의 물체입니다.")
            else:
                break
            self.call(self.cli_h, Trigger.Request()); time.sleep(1.0)
        self.get_logger().info("🎉 ALL SEQUENCE DONE")

def main():
    rclpy.init()
    node = MasterNode()
    try:
        node.run()
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__':
    main()