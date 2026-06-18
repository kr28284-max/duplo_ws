import cv2
import numpy as np
import rclpy
import threading
from rclpy.node import Node
from srvs_pkg.srv import GetTargetPose
from vision_pkg import INUVisionCall as ivc
from vision_pkg import INUVisionLib as ivl


WINDOW_NAME = 'vision_node2 detections'
SCAN_PERIOD_SEC = 1.0


def _project_pose_center(item, intrinsics):
    center_m = np.array([item["x_mm"], item["y_mm"], item["z_mm"]], dtype=np.float64) / 1000.0
    return ivl.project_point_to_image(center_m, intrinsics)


def _draw_text_lines(image, lines, origin, font_scale=0.43):
    x, y = origin
    for i, line in enumerate(lines):
        yy = y + i * 17
        cv2.putText(
            image,
            line,
            (x, yy),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            line,
            (x, yy),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )


def _draw_pose_overlay(color_rgb, pose_table, intrinsics, target_pose=None):
    vis_bgr = cv2.cvtColor(color_rgb.copy(), cv2.COLOR_RGB2BGR)

    for item in pose_table or []:
        obj = item.get("object_ref") or item.get("raw_contour_object") or {}
        is_target = item is target_pose
        color = (0, 255, 255) if is_target else (80, 220, 80)
        thickness = 3 if is_target else 2

        cx = cy = None
        box_2d = None
        if obj.get("obb_3d") is not None:
            box_2d = np.asarray(obj["obb_3d"].get("box_2d"), dtype=np.int32)

        if box_2d is not None and len(box_2d) == 4:
            cv2.drawContours(vis_bgr, [box_2d], 0, color, thickness)
            cx = int(np.mean(box_2d[:, 0]))
            cy = int(np.mean(box_2d[:, 1]))
        elif item.get("center_uv") is not None:
            cx, cy = map(int, item["center_uv"])
            cv2.rectangle(vis_bgr, (cx - 25, cy - 25), (cx + 25, cy + 25), color, thickness)
        else:
            center_uv = _project_pose_center(item, intrinsics)
            if center_uv is not None:
                cx, cy = center_uv
                cv2.rectangle(vis_bgr, (cx - 25, cy - 25), (cx + 25, cy + 25), color, thickness)

        if cx is None or cy is None:
            continue

        cv2.circle(vis_bgr, (cx, cy), 4, (0, 0, 255), -1)

        label = item.get("class_name", "unknown")
        local_id = item.get("local_id", "-")
        x_mm = item.get("x_mm", 0.0)
        y_mm = item.get("y_mm", 0.0)
        z_mm = item.get("z_mm", 0.0)
        yaw_deg = item.get("yaw_deg", 0.0)

        lines = [
            f"{label} id:{local_id}",
            f"XYZ: {x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f} mm",
            f"Yaw: {yaw_deg:.1f} deg",
        ]
        _draw_text_lines(vis_bgr, lines, (cx + 8, max(18, cy - 28)))

    return vis_bgr

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node2')
        self.srv = self.create_service(GetTargetPose, '/get_target_pose', self.get_pose_cb)
        self.get_logger().info('[VISION] 초기화 중... VisionManager 로드')
        
        self.vision = ivc.VisionManager()
        self.vision_lock = threading.Lock()
        self.last_error = None
        self.scan_count = 0
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        self.scan_timer = self.create_timer(SCAN_PERIOD_SEC, self.scan_timer_cb)
        self.get_logger().info(
            f'[VISION] vision_node2 시작 완료 - {SCAN_PERIOD_SEC:.1f}초마다 블럭 확인/화면 갱신'
        )

    def scan_timer_cb(self):
        if not self.vision_lock.acquire(blocking=False):
            return

        try:
            self.vision.capture_camera(visualize=False)
            self.vision.run_search(visualize=False)
            self.show_detection_window_locked()

            self.scan_count += 1
            self.last_error = None
            self.get_logger().info(
                f'[VISION] 주기 스캔 완료 #{self.scan_count}: '
                f'{len(self.vision.pose_table or [])}개 객체'
            )
        except Exception as e:
            self.last_error = str(e)
            self.get_logger().error(f'[VISION] 주기 스캔 실패: {e}')
        finally:
            self.vision_lock.release()

    def show_detection_window(self, target_pose=None):
        with self.vision_lock:
            self.show_detection_window_locked(target_pose=target_pose)

    def show_detection_window_locked(self, target_pose=None):
        if self.vision.color_rgb is None or self.vision.pose_table is None:
            return

        try:
            overlay = _draw_pose_overlay(
                color_rgb=self.vision.color_rgb,
                pose_table=self.vision.pose_table,
                intrinsics=self.vision.intrinsics,
                target_pose=target_pose,
            )
            cv2.imshow(WINDOW_NAME, overlay)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warning(f'[VISION] 디버그 창 표시 실패: {e}')

    def get_pose_cb(self, request, response):
        # 1. target_color 필드를 통해 ID 문자열을 받음 (예: "7", "999")
        target_str = request.target_color.strip()
        self.get_logger().info(f'[VISION] 서비스 요청 수신 - target ID: {target_str}')

        try:
            # 입력값이 숫자인지 확인
            if not target_str.isdigit():
                self.get_logger().error(f'[VISION] 잘못된 입력입니다. 숫자 ID를 입력하세요: {target_str}')
                response.success = False
                return response
            
            target_id = int(target_str)

            self.vision_lock.acquire()
            try:
                # 2. 일반 듀플로는 주기 스캔의 최신 결과를 사용
                if 1 <= target_id <= 8:
                    self.get_logger().info(f'[VISION] 일반 브릭(ID:{target_id}) 최신 스캔 결과 조회')
                    if self.vision.class_index is None:
                        self.get_logger().error('[VISION] 아직 주기 스캔 결과가 없습니다. 잠시 후 다시 요청하세요.')
                        response.success = False
                        return response

                # 3. 조립체/기타 모드는 요청 시 별도 분석
                elif target_id == 999:
                    self.get_logger().info('[VISION] 조립체(ID:999) 요청 분석 실행')
                    self.vision.capture_camera(visualize=False)
                    self.vision.run_search_assembly(visualize=False)
                else:
                    self.get_logger().info(f'[VISION] 기타 객체(ID:{target_id}) 요청 분석 실행')
                    self.vision.capture_camera(visualize=False)
                    self.vision.run_search(visualize=False)

                # 4. 탐색된 결과에서 특정 타겟의 Pose 추출
                pose = self.vision.get_pose_by_id(target_id=target_id, local_id=0)
                self.show_detection_window_locked(target_pose=pose)
            finally:
                self.vision_lock.release()

            # 5. 결과 반환 (Service Response)
            if pose is not None:
                response.success = True
                # ROS 표준 단위(미터)에 맞게 mm -> m 변환
                response.x = float(pose["x_mm"] / 1000.0)
                response.y = float(pose["y_mm"] / 1000.0)
                response.z = float(pose["z_mm"] / 1000.0)
                response.yaw = float(pose["yaw_deg"])
                # srv에 추가된 class_name 반환
                response.class_name = str(pose.get("class_name", ""))
                
                self.get_logger().info(
                    f'[VISION] 타겟({target_id}) 발견! X:{response.x*1000:.1f} Y:{response.y*1000:.1f} Yaw:{response.yaw:.1f} Class:{response.class_name}'
                )
            else:
                self.get_logger().error(f'[VISION] 시야에서 타겟(ID:{target_id})을 찾을 수 없습니다.')
                response.success = False

        except Exception as e:
            self.get_logger().error(f'[VISION] 처리 중 심각한 오류 발생: {e}')
            response.success = False

        return response

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
