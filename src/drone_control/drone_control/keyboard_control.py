#키보드 키를 이용해서 제어 시도(실패)
import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
import sys
import termios
import tty
import threading

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        #publisher(offboard control, velocity, command(mode))
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode,'/fmu/in/offboard_control_mode',qos_profile_sensor_data)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint,'/fmu/in/trajectory_setpoint',qos_profile_sensor_data)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand,'/fmu/in/vehicle_command',qos_profile_system_default)
        #카메라 data와 t265로 가정한 카메라로 부터 얻은 현재 위치 data subscriber
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition,'/fmu/out/vehicle_local_position_v1',self.local_pos_callback,qos_profile_sensor_data) #drone의 현재 위치 받아오기
        self.timer = self.create_timer(0.05, self.timer_callback)
        #고도 변수
        self.current_z = 0.0
        self.target_z = 0.0
        self.hold_position = False
        #속도 변수
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.get_logger().info("키보드 제어 노드 시작! (드론 이륙 10회 시도)")
        self.timer_arm = self.create_timer(1.0, self.arm_and_offboard) #시동 및 offboard mode 명령 10회 보내기
        self.arm_attempts = 0
        #keyboard 입력에 따른 값 받기
        self.input_thread = threading.Thread(target=self.keyboard_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def local_pos_callback(self, msg):
        #기압계를 통한 고도(Z)는 꽤 정확하므로 이것만 업데이트
        self.current_z = msg.z

    def arm_and_offboard(self):
        if self.arm_attempts < 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.arm_attempts += 1
            self.get_logger().info(f"시동 및 Offboard 명령 전송 중... ({self.arm_attempts}/10)")
        else:
            self.get_logger().info("키보드로 조종하세요,")
            self.timer_arm.cancel()

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        #시동과 offboard mode를 위한 값
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 255 
        msg.source_component = 0
        msg.from_external = True 
        self.vehicle_command_pub.publish(msg)

    def timer_callback(self):
        #offboard mode를 위한 변수 설정
        timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = timestamp
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        #drone의 움직임을 제어하기 위한 변수
        setpoint_msg = TrajectorySetpoint()
        setpoint_msg.timestamp = timestamp
        setpoint_msg.yaw = float('nan') #뱅글뱅글 도는 현상 원천 차단(yaw를 강제로 0으로 잡지 않고 현재 상태 유지(NaN))
        setpoint_msg.yawspeed = 0.0
        #혼합 제어 모드
        if self.hold_position:
            #위치와 속도를 둘 다 켭니다.
            offboard_msg.position = True
            offboard_msg.velocity = True
            #X, Y는 위치 무시(NaN)하고 속도 0으로 브레이크, Z는 현재 높이 유진
            setpoint_msg.position = [float('nan'), float('nan'), self.target_z]
            setpoint_msg.velocity = [0.0, 0.0, float('nan')]
        #이동 모드
        else:
            #속도 제어만 켭니다.
            offboard_msg.position = False
            offboard_msg.velocity = True
            #정해진 속도로 설정       
            setpoint_msg.position = [float('nan'), float('nan'), float('nan')]
            setpoint_msg.velocity = [self.vx, self.vy, self.vz]
        self.offboard_mode_pub.publish(offboard_msg)
        self.trajectory_pub.publish(setpoint_msg)

    def keyboard_loop(self):
        old_attr = termios.tcgetattr(sys.stdin) 
        tty.setcbreak(sys.stdin.fileno())
        speed = 1.0
        try:
            while True:
                key = sys.stdin.read(1)
                if key in ['w', 's', 'a', 'd', 'e', 'z']:
                    self.hold_position = False
                    if key == 'w': self.vx = speed; self.vy = 0.0; self.vz = 0.0; self.get_logger().info("입력: [W] 전진")
                    elif key == 's': self.vx = -speed; self.vy = 0.0; self.vz = 0.0; self.get_logger().info("입력: [S] 후진")
                    elif key == 'a': self.vx = 0.0; self.vy = -speed; self.vz = 0.0; self.get_logger().info("입력: [A] 좌측")
                    elif key == 'd': self.vx = 0.0; self.vy = speed; self.vz = 0.0; self.get_logger().info("입력: [D] 우측")
                    elif key == 'e': self.vx = 0.0; self.vy = 0.0; self.vz = -speed; self.get_logger().info("입력: [E] 상승")
                    elif key == 'z': self.vx = 0.0; self.vy = 0.0; self.vz = speed; self.get_logger().info("입력: [Z] 하강")
                #X를 누르면 현재 고도(Z)를 캡처하고 혼합 제어 모드 돌입!
                elif key == 'x': 
                    self.hold_position = True
                    self.target_z = self.current_z
                    self.get_logger().info(f"입력: [X] 브레이크! (현재 고도 {abs(self.target_z):.2f}m 유지)")
                elif key == '\x03': break #ctrl+c
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()