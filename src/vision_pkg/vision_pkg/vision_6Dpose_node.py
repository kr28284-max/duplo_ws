import rclpy
from rclpy.node import Node
from srvs_pkg.srv import GetTargetPose
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import cv2
import time

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.srv = self.create_service(GetTargetPose, '/get_target_pose', self.get_pose_cb)
        self.model = YOLO("/home/da/duplo_ws/best.pt")

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        self.latest_color = None
        self.latest_depth = None
        self.latest_results = None

        self.create_timer(0.033, self.visualize_callback)
        self.get_logger().info("✅ Vision Node: 1층 높이 필터링 및 Blind Stack 지원 모드 가동")

    def calculate_refined_yaw(self, rect):
        (cx, cy), (w, h), angle = rect
        if w < h:
            yaw = angle
        else:
            yaw = angle + 90.0

        if yaw > 90: yaw -= 180
        if yaw < -90: yaw += 180
        return yaw

    def visualize_callback(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            aligned = self.align.process(frames)
            self.latest_depth = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            
            if not color_frame or not self.latest_depth: return

            self.latest_color = np.asanyarray(color_frame.get_data())
            self.latest_results = self.model(self.latest_color, verbose=False)[0]

            display_img = self.latest_results.plot()
            cv2.circle(display_img, (320, 240), 5, (0, 0, 255), -1)

            if self.latest_results.boxes is not None:
                for i, box in enumerate(self.latest_results.boxes):
                    xyxy = box.xyxy[0].cpu().numpy()
                    u, v = int((xyxy[0] + xyxy[2]) / 2), int((xyxy[1] + xyxy[3]) / 2)
                    yaw = 0.0

                    if self.latest_results.masks is not None and len(self.latest_results.masks.xy) > i:
                        pts = np.int32([self.latest_results.masks.xy[i]])
                        M = cv2.moments(pts)
                        if M["m00"] != 0:
                            u, v = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                            rect = cv2.minAreaRect(pts)
                            yaw = self.calculate_refined_yaw(rect)
                    
                    z_val = self.latest_depth.get_distance(u, v)
                    if z_val > 0:
                        x_r, y_r, _ = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], z_val)
                        cv2.putText(display_img, f"X:{x_r*1000:.1f} Y:{y_r*1000:.1f}", (u - 60, v + 25), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.putText(display_img, f"Yaw:{yaw:.1f}", (u - 60, v + 45), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            cv2.imshow("6D Pose (Refined Yaw)", display_img)
            cv2.waitKey(1)
        except Exception:
            pass

    def get_valid_depth(self, depth_frame, u, v, search_radius=10):
        z = depth_frame.get_distance(u, v)
        if z > 0: return z
        for r in range(1, search_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    nu, nv = u + dx, v + dy
                    if 0 <= nu < 640 and 0 <= nv < 480:
                        z = depth_frame.get_distance(nu, nv)
                        if z > 0: return z
        return 0.0

    def get_pose_cb(self, request, response):
        target = request.target_color.lower()

        # [개수 파악 로직 유지]
        if target.startswith("count_"):
            search_color = target.replace("count_", "")
            start_time = time.time()
            max_count = 0
            while time.time() - start_time < 0.5:
                try:
                    frames = self.pipeline.wait_for_frames(timeout_ms=500)
                    aligned = self.align.process(frames)
                    color_f = aligned.get_color_frame()
                    if not color_f: continue

                    img = np.asanyarray(color_f.get_data())
                    results = self.model(img, verbose=False)[0]
                    
                    if results.boxes is not None:
                        current_count = sum(1 for box in results.boxes if search_color in results.names[int(box.cls[0])].lower())
                        max_count = max(max_count, current_count)
                except Exception:
                    pass
            response.success = True
            response.x, response.y, response.z, response.yaw = float(max_count), 0.0, 0.0, 0.0
            return response

        # -------------------------------------------------------------
        # 🌟 Z값(높이) 필터링이 적용된 포즈 측정
        self.get_logger().info(f"🔍 '{target}' 정밀 측정 (바닥 1층 블록만 탐색 중...)")
        
        samples = []
        start_time = time.time()
        
        while time.time() - start_time < 1.2:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=500)
                aligned = self.align.process(frames)
                depth_f = aligned.get_depth_frame()
                color_f = aligned.get_color_frame()
                
                if not color_f or not depth_f: continue

                img = np.asanyarray(color_f.get_data())
                results = self.model(img, verbose=False)[0]
                if results.boxes is None: continue

                frame_targets = []
                for i, box in enumerate(results.boxes):
                    cls_name = results.names[int(box.cls[0])].lower()
                    if target in cls_name:
                        u, v, yaw = 0, 0, 0.0
                        if results.masks is not None and len(results.masks.xy) > i:
                            pts = np.int32([results.masks.xy[i]])
                            M = cv2.moments(pts)
                            if M["m00"] != 0:
                                u, v = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                                rect = cv2.minAreaRect(pts)
                                yaw = self.calculate_refined_yaw(rect)
                        else:
                            xyxy = box.xyxy[0].cpu().numpy()
                            u, v = int((xyxy[0] + xyxy[2]) / 2), int((xyxy[1] + xyxy[3]) / 2)

                        z = self.get_valid_depth(depth_f, u, v)
                        if z > 0:
                            dist = ((u - 320) ** 2 + (v - 240) ** 2) ** 0.5
                            frame_targets.append({'u': u, 'v': v, 'z': z, 'yaw': yaw, 'dist': dist})

                if frame_targets:
                    # 🌟 핵심 트릭: 카메라에서 가장 먼(Z값이 가장 큰) 거리를 찾음 = 가장 밑바닥
                    max_z = max(t['z'] for t in frame_targets)
                    
                    # 바닥(max_z)에서 1.5cm(0.015) 이내에 있는 블록들만 남김 (2, 3층은 제외됨)
                    ground_targets = [t for t in frame_targets if abs(max_z - t['z']) < 0.015]
                    
                    # 바닥에 있는 블록들 중에서 중앙에 가장 가까운 놈을 선택
                    best = min(ground_targets, key=lambda t: t['dist'])
                    x, y, z = rs.rs2_deproject_pixel_to_point(self.intrinsics, [best['u'], best['v']], best['z'])
                    samples.append([x, y, z, best['yaw']])
                
                time.sleep(0.01)
            except Exception:
                continue

        if len(samples) < 5:
            self.get_logger().error(f"❌ 인식 실패 (수집 프레임 부족)")
            response.success = False
            return response

        samples = np.array(samples)
        median_pose = np.median(samples, axis=0)
        
        response.success = True
        response.x, response.y, response.z, response.yaw = \
            float(median_pose[0]), float(median_pose[1]), float(median_pose[2]), float(median_pose[3])
        
        self.get_logger().info(f"🎯 타겟 확정: X:{response.x*1000:.1f}, Y:{response.y*1000:.1f}, Yaw:{response.yaw:.1f}")
        return response

def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# # import rclpy
# # from rclpy.node import Node
# # from srvs_pkg.srv import GetTargetPose
# # import numpy as np
# # import pyrealsense2 as rs
# # from ultralytics import YOLO
# # import cv2
# # import time

# # class VisionNode(Node):
# #     def __init__(self):
# #         super().__init__('vision_node')
# #         self.srv = self.create_service(GetTargetPose, '/get_target_pose', self.get_pose_cb)
# #         self.model = YOLO("/home/da/duplo_ws/best.pt")

# #         self.pipeline = rs.pipeline()
# #         config = rs.config()
# #         config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# #         config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# #         profile = self.pipeline.start(config)
# #         self.align = rs.align(rs.stream.color)
# #         self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

# #         self.latest_color = None
# #         self.latest_depth = None
# #         self.latest_results = None

# #         self.create_timer(0.033, self.visualize_callback)
# #         self.get_logger().info("✅ Vision Node: 1층 높이 필터링 및 2.0초 정밀 스캔 모드 가동")

# #     def calculate_refined_yaw(self, rect):
# #         (cx, cy), (w, h), angle = rect
# #         if w < h:
# #             yaw = angle
# #         else:
# #             yaw = angle + 90.0

# #         if yaw > 90: yaw -= 180
# #         if yaw < -90: yaw += 180
# #         return yaw

# #     def visualize_callback(self):
# #         try:
# #             frames = self.pipeline.wait_for_frames(timeout_ms=1000)
# #             aligned = self.align.process(frames)
# #             self.latest_depth = aligned.get_depth_frame()
# #             color_frame = aligned.get_color_frame()
            
# #             if not color_frame or not self.latest_depth: return

# #             self.latest_color = np.asanyarray(color_frame.get_data())
# #             self.latest_results = self.model(self.latest_color, verbose=False)[0]

# #             display_img = self.latest_results.plot()
# #             cv2.circle(display_img, (320, 240), 5, (0, 0, 255), -1)

# #             if self.latest_results.boxes is not None:
# #                 for i, box in enumerate(self.latest_results.boxes):
# #                     xyxy = box.xyxy[0].cpu().numpy()
# #                     u, v = int((xyxy[0] + xyxy[2]) / 2), int((xyxy[1] + xyxy[3]) / 2)
# #                     yaw = 0.0

# #                     if self.latest_results.masks is not None and len(self.latest_results.masks.xy) > i:
# #                         pts = np.int32([self.latest_results.masks.xy[i]])
# #                         M = cv2.moments(pts)
# #                         if M["m00"] != 0:
# #                             u, v = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
# #                             rect = cv2.minAreaRect(pts)
# #                             yaw = self.calculate_refined_yaw(rect)
                    
# #                     z_val = self.latest_depth.get_distance(u, v)
# #                     if z_val > 0:
# #                         x_r, y_r, _ = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], z_val)
# #                         cv2.putText(display_img, f"X:{x_r*1000:.1f} Y:{y_r*1000:.1f}", (u - 60, v + 25), 
# #                                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
# #                         cv2.putText(display_img, f"Yaw:{yaw:.1f}", (u - 60, v + 45), 
# #                                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

# #             cv2.imshow("6D Pose (Refined Yaw)", display_img)
# #             cv2.waitKey(1)
# #         except Exception:
# #             pass

# #     def get_valid_depth(self, depth_frame, u, v, search_radius=10):
# #         z = depth_frame.get_distance(u, v)
# #         if z > 0: return z
# #         for r in range(1, search_radius + 1):
# #             for dx in range(-r, r + 1):
# #                 for dy in range(-r, r + 1):
# #                     nu, nv = u + dx, v + dy
# #                     if 0 <= nu < 640 and 0 <= nv < 480:
# #                         z = depth_frame.get_distance(nu, nv)
# #                         if z > 0: return z
# #         return 0.0

# #     def get_pose_cb(self, request, response):
# #         target = request.target_color.lower()

# #         if target.startswith("count_"):
# #             search_color = target.replace("count_", "")
# #             start_time = time.time()
# #             max_count = 0
# #             while time.time() - start_time < 0.5:
# #                 try:
# #                     frames = self.pipeline.wait_for_frames(timeout_ms=500)
# #                     aligned = self.align.process(frames)
# #                     color_f = aligned.get_color_frame()
# #                     if not color_f: continue

# #                     img = np.asanyarray(color_f.get_data())
# #                     results = self.model(img, verbose=False)[0]
                    
# #                     if results.boxes is not None:
# #                         current_count = sum(1 for box in results.boxes if search_color in results.names[int(box.cls[0])].lower())
# #                         max_count = max(max_count, current_count)
# #                 except Exception:
# #                     pass
# #             response.success = True
# #             response.x, response.y, response.z, response.yaw = float(max_count), 0.0, 0.0, 0.0
# #             return response

# #         # -------------------------------------------------------------
# #         # 🌟 정밀 스캔 데이터 수집 시간을 2.0초로 늘려 안정성 극대화
# #         self.get_logger().info(f"🔍 '{target}' 정밀 측정 (바닥 1층 탐색, 2.0초 데이터 수집 중...)")
        
# #         samples = []
# #         start_time = time.time()
        
# #         # 💡 1.2 -> 2.0으로 늘려서 흔들림 없는 확실한 중간값을 뽑아냅니다.
# #         while time.time() - start_time < 2.0:
# #             try:
# #                 frames = self.pipeline.wait_for_frames(timeout_ms=500)
# #                 aligned = self.align.process(frames)
# #                 depth_f = aligned.get_depth_frame()
# #                 color_f = aligned.get_color_frame()
                
# #                 if not color_f or not depth_f: continue

# #                 img = np.asanyarray(color_f.get_data())
# #                 results = self.model(img, verbose=False)[0]
# #                 if results.boxes is None: continue

# #                 frame_targets = []
# #                 for i, box in enumerate(results.boxes):
# #                     cls_name = results.names[int(box.cls[0])].lower()
# #                     if target in cls_name:
# #                         u, v, yaw = 0, 0, 0.0
# #                         if results.masks is not None and len(results.masks.xy) > i:
# #                             pts = np.int32([results.masks.xy[i]])
# #                             M = cv2.moments(pts)
# #                             if M["m00"] != 0:
# #                                 u, v = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
# #                                 rect = cv2.minAreaRect(pts)
# #                                 yaw = self.calculate_refined_yaw(rect)
# #                         else:
# #                             xyxy = box.xyxy[0].cpu().numpy()
# #                             u, v = int((xyxy[0] + xyxy[2]) / 2), int((xyxy[1] + xyxy[3]) / 2)

# #                         z = self.get_valid_depth(depth_f, u, v)
# #                         if z > 0:
# #                             dist = ((u - 320) ** 2 + (v - 240) ** 2) ** 0.5
# #                             frame_targets.append({'u': u, 'v': v, 'z': z, 'yaw': yaw, 'dist': dist})

# #                 if frame_targets:
# #                     max_z = max(t['z'] for t in frame_targets)
# #                     ground_targets = [t for t in frame_targets if abs(max_z - t['z']) < 0.015]
# #                     best = min(ground_targets, key=lambda t: t['dist'])
# #                     x, y, z = rs.rs2_deproject_pixel_to_point(self.intrinsics, [best['u'], best['v']], best['z'])
# #                     samples.append([x, y, z, best['yaw']])
                
# #                 time.sleep(0.01)
# #             except Exception:
# #                 continue

# #         if len(samples) < 5:
# #             self.get_logger().error(f"❌ 인식 실패 (수집 프레임 부족)")
# #             response.success = False
# #             return response

# #         samples = np.array(samples)
# #         median_pose = np.median(samples, axis=0)
        
# #         response.success = True
# #         response.x, response.y, response.z, response.yaw = \
# #             float(median_pose[0]), float(median_pose[1]), float(median_pose[2]), float(median_pose[3])
        
# #         self.get_logger().info(f"🎯 타겟 확정: X:{response.x*1000:.1f}, Y:{response.y*1000:.1f}, Yaw:{response.yaw:.1f}")
# #         return response

# # def main():
# #     rclpy.init()
# #     node = VisionNode()
# #     try:
# #         rclpy.spin(node)
# #     except KeyboardInterrupt:
# #         pass
# #     finally:
# #         cv2.destroyAllWindows()
# #         node.destroy_node()
# #         rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()

# import rclpy
# from rclpy.node import Node
# from srvs_pkg.srv import GetTargetPose
# import numpy as np
# import pyrealsense2 as rs
# from ultralytics import YOLO
# import cv2
# import time

# class VisionNode(Node):
#     def __init__(self):
#         super().__init__('vision_node')
#         self.srv = self.create_service(GetTargetPose, '/get_target_pose', self.get_pose_cb)
#         self.model = YOLO("/home/da/duplo_ws/best.pt")

#         self.pipeline = rs.pipeline()
#         config = rs.config()
#         config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#         config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#         profile = self.pipeline.start(config)
#         self.align = rs.align(rs.stream.color)
#         self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

#         self.latest_color = None
#         self.latest_depth = None
#         self.latest_results = None

#         self.create_timer(0.033, self.visualize_callback)
#         self.get_logger().info("✅ Vision Node: 높이 제한 해제 (모든 층 인식 모드) 가동")

#     def calculate_refined_yaw(self, rect):
#         (cx, cy), (w, h), angle = rect
#         if w < h:
#             yaw = angle
#         else:
#             yaw = angle + 90.0

#         if yaw > 90: yaw -= 180
#         if yaw < -90: yaw += 180
#         return yaw

#     def visualize_callback(self):
#         try:
#             frames = self.pipeline.wait_for_frames(timeout_ms=1000)
#             aligned = self.align.process(frames)
#             self.latest_depth = aligned.get_depth_frame()
#             color_frame = aligned.get_color_frame()
            
#             if not color_frame or not self.latest_depth: return

#             self.latest_color = np.asanyarray(color_frame.get_data())
#             self.latest_results = self.model(self.latest_color, verbose=False)[0]

#             display_img = self.latest_results.plot()
#             cv2.circle(display_img, (320, 240), 5, (0, 0, 255), -1)

#             if self.latest_results.boxes is not None:
#                 for i, box in enumerate(self.latest_results.boxes):
#                     xyxy = box.xyxy[0].cpu().numpy()
#                     u, v = int((xyxy[0] + xyxy[2]) / 2), int((xyxy[1] + xyxy[3]) / 2)
#                     yaw = 0.0

#                     if self.latest_results.masks is not None and len(self.latest_results.masks.xy) > i:
#                         pts = np.int32([self.latest_results.masks.xy[i]])
#                         M = cv2.moments(pts)
#                         if M["m00"] != 0:
#                             u, v = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
#                             rect = cv2.minAreaRect(pts)
#                             yaw = self.calculate_refined_yaw(rect)
                    
#                     z_val = self.latest_depth.get_distance(u, v)
#                     if z_val > 0:
#                         x_r, y_r, _ = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], z_val)
#                         cv2.putText(display_img, f"X:{x_r*1000:.1f} Y:{y_r*1000:.1f}", (u - 60, v + 25), 
#                                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
#                         cv2.putText(display_img, f"Yaw:{yaw:.1f}", (u - 60, v + 45), 
#                                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

#             cv2.imshow("6D Pose (Refined Yaw)", display_img)
#             cv2.waitKey(1)
#         except Exception:
#             pass

#     def get_valid_depth(self, depth_frame, u, v, search_radius=10):
#         z = depth_frame.get_distance(u, v)
#         if z > 0: return z
#         for r in range(1, search_radius + 1):
#             for dx in range(-r, r + 1):
#                 for dy in range(-r, r + 1):
#                     nu, nv = u + dx, v + dy
#                     if 0 <= nu < 640 and 0 <= nv < 480:
#                         z = depth_frame.get_distance(nu, nv)
#                         if z > 0: return z
#         return 0.0

#     def get_pose_cb(self, request, response):
#         target = request.target_color.lower()

#         # [개수 파악 로직 유지]
#         if target.startswith("count_"):
#             search_color = target.replace("count_", "")
#             start_time = time.time()
#             max_count = 0
#             while time.time() - start_time < 0.5:
#                 try:
#                     frames = self.pipeline.wait_for_frames(timeout_ms=500)
#                     aligned = self.align.process(frames)
#                     color_f = aligned.get_color_frame()
#                     if not color_f: continue

#                     img = np.asanyarray(color_f.get_data())
#                     results = self.model(img, verbose=False)[0]
                    
#                     if results.boxes is not None:
#                         current_count = sum(1 for box in results.boxes if search_color in results.names[int(box.cls[0])].lower())
#                         max_count = max(max_count, current_count)
#                 except Exception:
#                     pass
#             response.success = True
#             response.x, response.y, response.z, response.yaw = float(max_count), 0.0, 0.0, 0.0
#             return response

#         # -------------------------------------------------------------
#         # 🌟 Z값(높이) 필터링 제거: 층수에 상관없이 해당 색상의 블록을 찾습니다.
#         self.get_logger().info(f"🔍 '{target}' 정밀 측정 (모든 층 탐색 중...)")
        
#         samples = []
#         start_time = time.time()
        
#         while time.time() - start_time < 1.2:
#             try:
#                 frames = self.pipeline.wait_for_frames(timeout_ms=500)
#                 aligned = self.align.process(frames)
#                 depth_f = aligned.get_depth_frame()
#                 color_f = aligned.get_color_frame()
                
#                 if not color_f or not depth_f: continue

#                 img = np.asanyarray(color_f.get_data())
#                 results = self.model(img, verbose=False)[0]
#                 if results.boxes is None: continue

#                 frame_targets = []
#                 for i, box in enumerate(results.boxes):
#                     cls_name = results.names[int(box.cls[0])].lower()
#                     if target in cls_name:
#                         u, v, yaw = 0, 0, 0.0
#                         if results.masks is not None and len(results.masks.xy) > i:
#                             pts = np.int32([results.masks.xy[i]])
#                             M = cv2.moments(pts)
#                             if M["m00"] != 0:
#                                 u, v = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
#                                 rect = cv2.minAreaRect(pts)
#                                 yaw = self.calculate_refined_yaw(rect)
#                         else:
#                             xyxy = box.xyxy[0].cpu().numpy()
#                             u, v = int((xyxy[0] + xyxy[2]) / 2), int((xyxy[1] + xyxy[3]) / 2)

#                         z = self.get_valid_depth(depth_f, u, v)
#                         if z > 0:
#                             dist = ((u - 320) ** 2 + (v - 240) ** 2) ** 0.5
#                             frame_targets.append({'u': u, 'v': v, 'z': z, 'yaw': yaw, 'dist': dist})

#                 if frame_targets:
#                     # 🌟 기존의 바닥(1층) 필터링 로직 제거
#                     # 단순히 화면 중앙에 가장 가까운 타겟을 최우선으로 선택합니다.
#                     best = min(frame_targets, key=lambda t: t['dist'])
                    
#                     x, y, z = rs.rs2_deproject_pixel_to_point(self.intrinsics, [best['u'], best['v']], best['z'])
#                     samples.append([x, y, z, best['yaw']])
                
#                 time.sleep(0.01)
#             except Exception:
#                 continue

#         if len(samples) < 5:
#             self.get_logger().error(f"❌ 인식 실패 (수집 프레임 부족)")
#             response.success = False
#             return response

#         samples = np.array(samples)
#         median_pose = np.median(samples, axis=0)
        
#         response.success = True
#         response.x, response.y, response.z, response.yaw = \
#             float(median_pose[0]), float(median_pose[1]), float(median_pose[2]), float(median_pose[3])
        
#         self.get_logger().info(f"🎯 타겟 확정: X:{response.x*1000:.1f}, Y:{response.y*1000:.1f}, Yaw:{response.yaw:.1f}")
#         return response

# def main():
#     rclpy.init()
#     node = VisionNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()