import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from px4_msgs.msg import OffboardControlMode, VehicleCommand, TrajectorySetpoint, VehicleLocalPosition
from ros_gz_interfaces.msg import Contacts 
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import math
import time

class AutonomousControlNode(Node):
    def __init__(self):
        super().__init__('autonomous_control_node')
        
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bottom_cam_sub = self.create_subscription(Image, '/bottom_camera/image_raw', self.bottom_image_callback, 10)
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_pos_callback, qos_profile_sensor_data)
        
        self.left_contact_sub = self.create_subscription(Contacts, '/contact/left_finger', self.left_contact_callback, 10)
        self.right_contact_sub = self.create_subscription(Contacts, '/contact/right_finger', self.right_contact_callback, 10)
        self.left_contacted = False
        self.right_contacted = False
        
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint,'/fmu/in/trajectory_setpoint', 10)
        
        self.elbow_pub = self.create_publisher(Float64, '/arm_cmd/elbow', 10)
        self.wrist_pub = self.create_publisher(Float64, '/arm_cmd/wrist', 10)
        self.left_finger_pub = self.create_publisher(Float64, '/arm_cmd/left_finger', 10)
        self.right_finger_pub = self.create_publisher(Float64, '/arm_cmd/right_finger', 10)

        self.bridge = CvBridge()
        
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        self.marker_length = 0.5 
        
        self.obj_points = np.array([
            [-self.marker_length/2,  self.marker_length/2, 0], 
            [ self.marker_length/2,  self.marker_length/2, 0], 
            [ self.marker_length/2, -self.marker_length/2, 0], 
            [-self.marker_length/2, -self.marker_length/2, 0]  
        ], dtype=np.float32)
        
        self.camera_matrix = np.array([[1435.5, 0.0, 320.5], 
                                       [0.0, 1435.5, 240.5], 
                                       [0.0, 0.0, 1.0]], dtype=np.float32) 
        self.dist_coeffs = np.zeros((4,1)) 

        self.state = "INIT"  
        self.tick_count = 0
        self.start_z = None
        self.current_x, self.current_y, self.current_z, self.current_yaw = 0.0, 0.0, 0.0, 0.0 
        self.hold_x, self.hold_y = 0.0, 0.0
        self.current_vx, self.current_vy, self.current_vz = 0.0, 0.0, 0.0 
        
        self.target_id = 0         
        self.wait_start_time = None 
        self.search_start_time = None 
        self.grab_time = 0.0
        self.target_yaw, self.setpoint_yaw = 0.0, 0.0    
        
        self.TARGET_ALTITUDE = 1.5 
        
        # 👇 [수학적 계산 완벽 반영] 몸체 0.45m - 팔 0.41m = 땅에서 4cm 높이(박스 정중앙)
        self.PICKING_ALTITUDE = 0.25 

        self.box_px_err_x, self.box_px_err_y = 0.0, 0.0
        self.box_visible = False
        self.has_payload = False     
        self.vision_dual_check = False 
        
        self.cmd_elbow = 0.0 
        self.cmd_wrist = 0.0
        self.cmd_left_finger = 0.0 
        self.cmd_right_finger = 0.0

        self.is_touched = False
        self.current_grip = 0.0
        
        self.timer = self.create_timer(0.05, self.timer_callback)

    def left_contact_callback(self, msg):
        self.left_contacted = len(msg.contacts) > 0

    def right_contact_callback(self, msg):
        self.right_contacted = len(msg.contacts) > 0

    def local_pos_callback(self, msg):
        self.current_x, self.current_y, self.current_z, self.current_yaw = msg.x, msg.y, msg.z, msg.heading
        if self.start_z is None: 
            self.target_yaw = self.setpoint_yaw = msg.heading
            self.start_z = msg.z
            self.state = "ARMING"

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command, msg.param1, msg.param2 = command, float(param1), float(param2)
        msg.target_system, msg.target_component, msg.source_system, msg.source_component = 1, 1, 255, 0
        msg.from_external = True 
        self.vehicle_command_pub.publish(msg)

    def bottom_image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w = cv_image.shape[:2]
            cx, cy = w / 2, h / 2
            
            current_alt = self.start_z - self.current_z if self.start_z else 1.5
            
            # 👇 [핵심 수정: 원근법 보정] 카메라와 박스 윗면(8cm) 사이의 실제 거리 H 계산
            # current_alt는 이륙 기준 높이이므로 랜딩기어(0.20m)를 더해야 실제 몸체 높이가 나옵니다.
            H = current_alt + 0.20 - 0.02 - 0.08
            if H < 0.1: H = 0.1 
            
            fy = 320.0 
            gripper_px_x = int(cx)
            gripper_px_y = int(cy + (fy * 0.15 / H))
            
            cv2.drawMarker(cv_image, (int(cx), int(cy)), (0, 255, 255), cv2.MARKER_CROSS, 15, 1)
            cv2.drawMarker(cv_image, (gripper_px_x, gripper_px_y), (0, 0, 255), cv2.MARKER_CROSS, 20, 3)

            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_mag, upper_mag = np.array([140, 50, 50]), np.array([170, 255, 255])
            mask = cv2.inRange(hsv, lower_mag, upper_mag)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            self.box_visible = False
            self.vision_dual_check = False
            
            if contours:
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                
                if not self.has_payload and area > 50: 
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        box_cx, box_cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                        cv2.circle(cv_image, (box_cx, box_cy), 5, (0, 255, 0), -1)
                        cv2.line(cv_image, (gripper_px_x, gripper_px_y), (box_cx, box_cy), (255, 255, 0), 2)
                        
                        self.box_px_err_x = box_cx - gripper_px_x
                        self.box_px_err_y = box_cy - gripper_px_y
                        self.box_visible = True

                elif self.has_payload:
                    if area > 2000: 
                        self.vision_dual_check = True
                        cv2.drawContours(cv_image, [c], -1, (0, 255, 0), 3)
                        cv2.putText(cv_image, "DUAL VERIFICATION: OK", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    else:
                        cv2.putText(cv_image, "WARNING: BOX DROPPED!", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 3)

            contact_text = f"L_SEN: {self.left_contacted} | R_SEN: {self.right_contacted}"
            color = (0, 255, 0) if (self.left_contacted and self.right_contacted) else (0, 165, 255)
            cv2.putText(cv_image, contact_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            state_color = (0, 255, 0) if self.state == "ALIGN_BOX" else (0, 255, 255)
            cv2.putText(cv_image, f"STATE: {self.state} | PAYLOAD: {self.has_payload}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, state_color, 2)
            cv2.imshow("Bottom Camera", cv_image)
            cv2.waitKey(1)
        except Exception: pass

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w = cv_image.shape[:2]
            cx, cy = w / 2, h / 2
            
            self.camera_matrix[0, 0], self.camera_matrix[1, 1] = 1435.5, 1435.5
            self.camera_matrix[0, 2], self.camera_matrix[1, 2] = cx, cy    

            cv2.drawMarker(cv_image, (int(cx), int(cy)), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = self.detector.detectMarkers(gray)
            current_alt = self.start_z - self.current_z if self.start_z else 0.0
            
            cv2.putText(cv_image, f"STATE: {self.state} | TARGET ID: {self.target_id}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(cv_image, f"ALT: {current_alt:.2f}m", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)

            if self.state == "DONE":
                cv2.putText(cv_image, "MISSION COMPLETE!", (int(cx)-150, int(cy)), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

            target_vz_alt = 0.0
            if current_alt < (self.TARGET_ALTITUDE - 0.5): target_vz_alt = -0.3 
            elif current_alt > (self.TARGET_ALTITUDE + 0.5): target_vz_alt = 0.3  

            found_target = False
            target_idx = -1
            if ids is not None:
                for i, marker_id in enumerate(ids):
                    if marker_id[0] == self.target_id:
                        found_target = True
                        target_idx = i
                        break

            if found_target:
                aruco.drawDetectedMarkers(cv_image, [corners[target_idx]], np.array([[self.target_id]]))
                success, rvec, tvec = cv2.solvePnP(self.obj_points, corners[target_idx], self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
                
                if success:
                    actual_dist = math.sqrt(tvec[0][0]**2 + tvec[1][0]**2 + tvec[2][0]**2)
                    dx, dy = tvec[0][0], tvec[1][0] 
                    cv2.putText(cv_image, f"DIST to ID {self.target_id}: {actual_dist:.2f}m", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                    if self.state in ["TRACKING", "WAITING"]:
                        error_dist = actual_dist - 2.5 
                        px_err_x = np.mean(corners[target_idx][0][:, 0]) - cx
                        px_err_y = np.mean(corners[target_idx][0][:, 1]) - cy
                        
                        if abs(px_err_x) < 60: dx = 0.0
                        if abs(px_err_y) < 60: dy = 0.0
                        if abs(error_dist) < 0.15: error_dist = 0.0
                        
                        is_perfect = (dx == 0.0 and dy == 0.0 and error_dist == 0.0)

                        if self.state == "TRACKING" and is_perfect:
                            self.state = "WAITING"
                            self.wait_start_time = time.time()

                        if self.state == "WAITING":
                            elapsed = time.time() - self.wait_start_time
                            cv2.putText(cv_image, f"WAITING: {elapsed:.1f}s / 5.0s", (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 3)
                            
                            if elapsed > 5.0:
                                if not self.has_payload:
                                    self.state = "ALIGN_BOX"
                                else:
                                    self.hold_x, self.hold_y = self.current_x, self.current_y
                                    self.state = "DESCENDING"
                                self.wait_start_time = None

                        if is_perfect:
                            self.current_vx, self.current_vy = 0.0, 0.0
                            target_vz_cam = 0.0
                        else:
                            self.current_vx = (0.8 * self.current_vx) + (0.2 * np.clip(0.4 * error_dist, -0.3, 0.3))
                            self.current_vy = (0.8 * self.current_vy) + (0.2 * np.clip(0.4 * dx, -0.3, 0.3))
                            target_vz_cam = np.clip(0.4 * dy, -0.2, 0.2)     
                            
                        final_target_vz = target_vz_alt if target_vz_alt != 0.0 else target_vz_cam
                        self.current_vz = 0.0 if (is_perfect and target_vz_alt == 0.0) else (0.8 * self.current_vz) + (0.2 * final_target_vz)
            else:
                self.current_vx *= 0.7
                self.current_vy *= 0.7
                self.current_vz = (0.8 * self.current_vz) + (0.2 * target_vz_alt) 
                if self.state == "WAITING":
                    self.state = "TRACKING"
                    self.wait_start_time = None

            cv2.imshow("Front Camera", cv_image)
            cv2.waitKey(1)
        except Exception: pass

    def timer_callback(self):
        if self.state in ["INIT", "LANDING"]: return
        
        if self.state in ["ALIGN_BOX", "DESCENDING", "ACTION"] or self.has_payload:
            self.cmd_elbow = 1.5708  
            self.cmd_wrist = 0.0      
        else:
            self.cmd_elbow = 0.0     
            self.cmd_wrist = 0.0
        
        self.tick_count += 1
        timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = timestamp
        setpoint_msg = TrajectorySetpoint()
        setpoint_msg.timestamp = timestamp
        current_alt = self.start_z - self.current_z if self.start_z else 0.0

        if self.state == "ARMING":
            offboard_msg.position = True; offboard_msg.velocity = False
            self.setpoint_yaw = self.target_yaw
            if self.tick_count > 20 and self.tick_count % 10 == 0:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                if self.tick_count > 60: self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            offboard_msg.position = True; offboard_msg.velocity = False
            setpoint_msg.position = [float('nan'), float('nan'), self.start_z - self.TARGET_ALTITUDE] 
            self.setpoint_yaw = self.target_yaw
            if current_alt > (self.TARGET_ALTITUDE - 0.5) and self.tick_count > 100: self.state = "TRACKING"

        elif self.state in ["TRACKING", "WAITING"]:
            offboard_msg.position = False; offboard_msg.velocity = True
            self.setpoint_yaw = self.target_yaw
            ned_vx = self.current_vx * math.cos(self.current_yaw) - self.current_vy * math.sin(self.current_yaw)
            ned_vy = self.current_vx * math.sin(self.current_yaw) + self.current_vy * math.cos(self.current_yaw)
            setpoint_msg.position = [float('nan'), float('nan'), float('nan')]
            setpoint_msg.velocity = [float(ned_vx), float(ned_vy), float(self.current_vz)]

        elif self.state == "ALIGN_BOX":
            offboard_msg.position = False; offboard_msg.velocity = True
            self.setpoint_yaw = self.target_yaw
            
            if self.box_visible:
                self.search_start_time = None
                
                target_vx = np.clip(-0.005 * self.box_px_err_y, -0.2, 0.2)
                target_vy = np.clip(0.005 * self.box_px_err_x, -0.2, 0.2)
                
                self.current_vx = (0.5 * self.current_vx) + (0.5 * target_vx)
                self.current_vy = (0.5 * self.current_vy) + (0.5 * target_vy)
                
                if abs(self.box_px_err_x) < 10 and abs(self.box_px_err_y) < 10:
                    self.hold_x, self.hold_y = self.current_x, self.current_y
                    self.state = "DESCENDING"
            else:
                if self.search_start_time is None:
                    self.search_start_time = time.time()
                search_elapsed = time.time() - self.search_start_time
                sweep_vx = 0.15 * math.sin(search_elapsed * (2 * math.pi / 4.0)) 
                self.current_vx = (0.8 * self.current_vx) + (0.2 * sweep_vx)
                self.current_vy *= 0.8
            
            target_vz_alt = 0.0
            if current_alt < (self.TARGET_ALTITUDE - 0.1): target_vz_alt = -0.3
            elif current_alt > (self.TARGET_ALTITUDE + 0.1): target_vz_alt = 0.3
            self.current_vz = (0.8 * self.current_vz) + (0.2 * target_vz_alt)

            ned_vx = self.current_vx * math.cos(self.current_yaw) - self.current_vy * math.sin(self.current_yaw)
            ned_vy = self.current_vx * math.sin(self.current_yaw) + self.current_vy * math.cos(self.current_yaw)
            setpoint_msg.position = [float('nan'), float('nan'), float('nan')]
            setpoint_msg.velocity = [float(ned_vx), float(ned_vy), float(self.current_vz)]

        elif self.state == "DESCENDING":
            offboard_msg.position = True; offboard_msg.velocity = False
            self.setpoint_yaw = self.target_yaw
            
            setpoint_msg.position = [self.hold_x, self.hold_y, self.start_z - self.PICKING_ALTITUDE]
            
            if not self.has_payload:
                if self.left_contacted or self.right_contacted:
                    self.contact_z = self.current_z 
                    self.state = "ACTION"
                    self.wait_start_time = time.time()
                    self.get_logger().info("🎯 하강 중 박스 접촉 감지! 즉시 파지를 시작합니다!")
                elif current_alt < (self.PICKING_ALTITUDE + 0.02):
                    self.contact_z = self.current_z 
                    self.state = "ACTION"
                    self.wait_start_time = time.time()
            else:
                if current_alt < (self.PICKING_ALTITUDE + 0.02):
                    self.contact_z = self.current_z
                    self.state = "ACTION"
                    self.wait_start_time = time.time()

        elif self.state == "ACTION":
            offboard_msg.position = True; offboard_msg.velocity = False
            self.setpoint_yaw = self.target_yaw
            
            if hasattr(self, 'contact_z'):
                setpoint_msg.position = [self.hold_x, self.hold_y, self.contact_z]
            else:
                setpoint_msg.position = [self.hold_x, self.hold_y, self.current_z]
                
            elapsed = time.time() - self.wait_start_time
            
            if not self.has_payload:
                if not self.is_touched:
                    self.current_grip += 0.002 
                    if self.current_grip >= 0.050: 
                        self.current_grip = 0.050
                    
                    self.cmd_left_finger = -self.current_grip
                    self.cmd_right_finger = self.current_grip

                    if self.left_contacted and self.right_contacted and self.current_grip > 0.01:
                        self.is_touched = True
                        self.grab_time = time.time() 
                        self.get_logger().info("✅ 완벽 파지 성공! 박스를 물었습니다. 이륙 준비!")
                else:
                    if time.time() - self.grab_time > 0.5:
                        self.has_payload = True
                        self.is_touched = False
                        self.state = "ASCENDING"

            else: 
                # 박스 내려놓기
                if 1.0 < elapsed < 2.0:
                    self.cmd_left_finger = 0.0
                    self.cmd_right_finger = 0.0
                if elapsed > 2.0:
                    self.has_payload = False
                    self.state = "ASCENDING_END" 

        elif self.state == "ASCENDING":
            offboard_msg.position = True; offboard_msg.velocity = True
            self.setpoint_yaw = self.target_yaw
            setpoint_msg.velocity = [0.0, 0.0, -1.0] 
            setpoint_msg.position = [self.hold_x, self.hold_y, self.start_z - self.TARGET_ALTITUDE]
            
            if abs(current_alt - self.TARGET_ALTITUDE) < 0.2:
                self.state = "ROTATING"
                self.wait_start_time = time.time()
                self.target_id = 1 
                self.target_yaw += math.pi
                if self.target_yaw > math.pi: self.target_yaw -= 2*math.pi

        elif self.state == "ROTATING":
            offboard_msg.position = True; offboard_msg.velocity = False
            setpoint_msg.position = [self.hold_x, self.hold_y, self.start_z - self.TARGET_ALTITUDE] 
            
            yaw_err = self.target_yaw - self.setpoint_yaw
            yaw_err = (yaw_err + math.pi) % (2 * math.pi) - math.pi
            self.setpoint_yaw += yaw_err * 0.04
            self.setpoint_yaw = (self.setpoint_yaw + math.pi) % (2 * math.pi) - math.pi
            
            yaw_diff = abs(self.target_yaw - self.current_yaw)
            if yaw_diff > math.pi: yaw_diff = 2*math.pi - yaw_diff
            if yaw_diff < 0.1 or (time.time() - self.wait_start_time > 6.0):
                self.state = "TRACKING"

        elif self.state == "ASCENDING_END":
            offboard_msg.position = True; offboard_msg.velocity = True
            self.setpoint_yaw = self.target_yaw
            setpoint_msg.velocity = [0.0, 0.0, -1.0]
            setpoint_msg.position = [self.hold_x, self.hold_y, self.start_z - self.TARGET_ALTITUDE]
            if abs(current_alt - self.TARGET_ALTITUDE) < 0.2:
                self.state = "DONE"

        elif self.state == "DONE":
            offboard_msg.position = True; offboard_msg.velocity = False
            self.setpoint_yaw = self.target_yaw
            setpoint_msg.position = [self.hold_x, self.hold_y, self.start_z - self.TARGET_ALTITUDE]

        setpoint_msg.yaw = self.setpoint_yaw

        self.elbow_pub.publish(Float64(data=self.cmd_elbow))
        self.wrist_pub.publish(Float64(data=self.cmd_wrist))
        self.left_finger_pub.publish(Float64(data=self.cmd_left_finger))
        self.right_finger_pub.publish(Float64(data=self.cmd_right_finger))

        self.offboard_mode_pub.publish(offboard_msg)
        self.trajectory_pub.publish(setpoint_msg)

    def land(self):
        self.state = "LANDING"
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0) 
        for _ in range(5):
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousControlNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.land()
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()