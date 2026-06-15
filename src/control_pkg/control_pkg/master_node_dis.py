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
        
        self.Z_OFF = -85.0
        self.Z_MARGIN = 20.0
        self.BLOCK_H = 16.0
        self.WAIT_TIME = 1.5 
        
        self.STUD_PITCH = 0.016 
        self.YAW_TUNE = 0.0  # 필요시 여기서 영점 조절 (예: -1.5)
        
        # 🌟 가장 깨끗하게 인식되었을 때의 타겟 좌표를 기억하는 변수
        self.last_perfect_pose = None 

    def call(self, cli, req):
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {cli.srv_name}...')
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def count_color(self, color):
        p = self.call(self.cli_v, GetTargetPose.Request(target_color=f"count_{color}"))
        return int(p.x) if p.success else 0

    def find_target_with_retry(self, color, retries=4):
        for i in range(retries):
            p = self.call(self.cli_v, GetTargetPose.Request(target_color=color))
            if p.success:
                return p
            self.get_logger().warn(f"⚠️ [{color}] 타겟 찾는 중... ({i+1}/{retries})")
            time.sleep(1.0) 
        return None

    def pick_target(self, color, layer_index=0):
        self.get_logger().info(f"\n--- PICK TARGET: [{color.upper()}] ---")
        p = self.find_target_with_retry(color)
        if not p: return False
        
        req = GetTargetPose.Request(); req.yaw = p.yaw; req.target_size = "YAW"
        self.call(self.cli_r, req)
        time.sleep(self.WAIT_TIME) 

        p = self.find_target_with_retry(color)
        if not p: return False
        req = GetTargetPose.Request(); req.x = p.x; req.y = p.y; req.target_size = "XY"
        self.call(self.cli_r, req)
        time.sleep(self.WAIT_TIME) 

        p = self.find_target_with_retry(color)
        if not p: return False
        z_move = (p.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)
        self.call(self.cli_r, GetTargetPose.Request(z=z_move - self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME) 
        self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME) 

        self.call(self.cli_g, SetBool.Request(data=True))
        time.sleep(self.WAIT_TIME) 
        self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
        time.sleep(self.WAIT_TIME) 
        return True

    def blind_insert(self, base_pose, layer_index, yaw_offset=0.0, offset_studs_x=0.0, offset_studs_y=0.0):
        self.get_logger().info(f"\n--- BLIND STACK (메모리 사용): Layer {layer_index} (Y Offset: {offset_studs_y}) ---")
        time.sleep(1.0)

        dx = offset_studs_x * self.STUD_PITCH
        dy = offset_studs_y * self.STUD_PITCH

        yaw_rad = math.radians(base_pose.yaw)
        real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
        real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)

        target_x = base_pose.x + real_offset_x
        target_y = base_pose.y + real_offset_y

        req_xy = GetTargetPose.Request()
        req_xy.x = target_x; req_xy.y = target_y; req_xy.target_size = "XY"
        self.call(self.cli_r, req_xy)
        time.sleep(self.WAIT_TIME) 

        target_yaw = base_pose.yaw + yaw_offset + self.YAW_TUNE
        while target_yaw > 90.0: target_yaw -= 180.0
        while target_yaw < -90.0: target_yaw += 180.0

        self.get_logger().info(f"🔄 [YAW 회전] 계산된 고정 각도 회전: {target_yaw:.1f}도")
        req_y = GetTargetPose.Request()
        req_y.yaw = target_yaw; req_y.target_size = "YAW"
        self.call(self.cli_r, req_y)
        time.sleep(self.WAIT_TIME) 

        z_move = (base_pose.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)
        
        self.call(self.cli_r, GetTargetPose.Request(z=z_move - self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME) 
        self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME) 

        self.call(self.cli_g, SetBool.Request(data=False))
        time.sleep(self.WAIT_TIME) 
        self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
        time.sleep(self.WAIT_TIME) 
        return True

    def visual_insert(self, target_color, layer_index, yaw_offset=0.0, offset_studs_x=0.0, offset_studs_y=0.0):
        self.get_logger().info(f"\n--- VISUAL STACK: [{target_color.upper()}] (Layer +{layer_index}, Y Offset: {offset_studs_y}) ---")
        time.sleep(1.0)

        p = self.find_target_with_retry(target_color)
        if not p: return False
        
        target_yaw = p.yaw + yaw_offset + self.YAW_TUNE
        while target_yaw > 90.0: target_yaw -= 180.0
        while target_yaw < -90.0: target_yaw += 180.0

        self.get_logger().info(f"🔄 [YAW 회전] 시각 보정 기반 회전: {target_yaw:.1f}도")
        req_y = GetTargetPose.Request()
        req_y.yaw = target_yaw; req_y.target_size = "YAW"
        self.call(self.cli_r, req_y)
        time.sleep(self.WAIT_TIME)

        p = self.find_target_with_retry(target_color)
        if not p: return False

        # 메모리 로직
        self.last_perfect_pose = p

        dx = offset_studs_x * self.STUD_PITCH
        dy = offset_studs_y * self.STUD_PITCH
        yaw_rad = math.radians(p.yaw)
        real_offset_x = dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad)
        real_offset_y = dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad)

        target_x = p.x + real_offset_x
        target_y = p.y + real_offset_y

        self.get_logger().info(f"➡️ [XY 이동] 시각 보정 기반 최적 오프셋 적용")
        req_xy = GetTargetPose.Request()
        req_xy.x = target_x; req_xy.y = target_y; req_xy.target_size = "XY"
        self.call(self.cli_r, req_xy)
        time.sleep(self.WAIT_TIME) 

        z_move = (p.z * 1000.0 + self.Z_OFF) - (self.BLOCK_H * layer_index)
        
        self.call(self.cli_r, GetTargetPose.Request(z=z_move - self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME) 
        self.call(self.cli_r, GetTargetPose.Request(z=self.Z_MARGIN, target_size="Z"))
        time.sleep(self.WAIT_TIME) 

        self.call(self.cli_g, SetBool.Request(data=False))
        time.sleep(self.WAIT_TIME) 
        self.call(self.cli_r, GetTargetPose.Request(z=-50.0, target_size="Z"))
        time.sleep(self.WAIT_TIME) 
        return True

    # --- 분해 시퀀스 ---
    def disassemble_battery(self):
        self.get_logger().info("\n🔋 [배터리 분해] 파란색 2x2 위에 있는 2층 노란색 2x2(Pick) -> 분해")
        
        # 🌟 핵심: 2층에 떠 있는 블록을 '집기' 위해서는 몸통 깊숙이 내려가야 하므로, 
        # layer_index=0 으로 주어 바닥 높이까지 파지 위치를 내립니다.
        if self.pick_target("4x2_yellow", layer_index=0):
            self.get_logger().info("⬆️ 노란색 블록 깊숙이 파지 및 상승(Z+ 5cm 수준) 완료!")
            
            # 들어 올린 노란색 블록을 처리하기 위해 홈으로 이동
            self.call(self.cli_h, Trigger.Request())
            
            # 그리퍼를 열어서 블록을 내려놓음(Drop)
            self.call(self.cli_g, SetBool.Request(data=False))
            self.get_logger().info("✅ 배터리 조합 분해 완료! (노란색 블록 분리)")
        else:
            self.get_logger().warn("❌ 배터리 상단의 2층 노란색 블록을 찾지 못했습니다.")

    def run(self):
        self.get_logger().info("🚀 STARTING DISASSEMBLY SEQUENCE")
        self.call(self.cli_h, Trigger.Request())
        self.call(self.cli_g, SetBool.Request(data=False))
        time.sleep(1.0) 
        
        self.get_logger().info("👀 필드 블록 스캔 중...")
        inventory = {
            "4x2_yellow": self.count_color("4x2_yellow"),
            "2x2_blue": self.count_color("2x2_blue")
        }
        self.get_logger().info(f"📦 현재 필드 상태: {inventory}")

        # 다른 조합 무시하고, 배터리 분해만 실행
        self.get_logger().info("▶️ 작업 시작: BATTERY DISASSEMBLY")
        self.disassemble_battery()

        self.call(self.cli_h, Trigger.Request())
        self.get_logger().info("🎉 ALL SEQUENCE DONE")

def main():
    rclpy.init()
    node = MasterNode()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()