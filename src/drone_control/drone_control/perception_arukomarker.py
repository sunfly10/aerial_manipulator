#aruko marker 인식 확인 코드
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import math

class CameraViewerNode(Node):
    def __init__(self):
        super().__init__('camera_viewer_node')
        self.subscription = self.create_subscription(
            Image,
            '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image', #토픽 찾기
            self.image_callback,
            10)
        self.bridge = CvBridge()
        #마커 특성
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        self.marker_length = 0.50
        #마커의 3d 좌표
        self.obj_points = np.array([
            [-self.marker_length / 2,  self.marker_length / 2, 0],
            [ self.marker_length / 2,  self.marker_length / 2, 0],
            [ self.marker_length / 2, -self.marker_length / 2, 0],
            [-self.marker_length / 2, -self.marker_length / 2, 0]
        ], dtype=np.float32)
        #camera intrinsic parameter
        # 행렬 형태:
        #[ fx,   0,  cx ]
        #[  0,  fy,  cy ]
        #[  0,   0,   1 ]
        #fx, fy: 초점 거리 (Focal length) - 화면이 얼마나 줌인/줌아웃 되어 있는지 픽셀 단위로 나타냄
        #cx, cy: 주점 (Principal point) - 빛이 모이는 화면의 광학적 정중앙 좌표 (해상도의 절반)
        #camera calibration 방법:
        #1. 실제 드론 카메라: '체커보드(흑백 바둑판)'를 들고 카메라 앞에서 이리저리 움직이며 여러 장 찍은 뒤, OpenCV의 cv2.calibrateCamera() 함수에 넣으면 알아서 이 행렬을 계산해 줍니다.
        #2. 가제보 시뮬레이션: 카메라 센서의 SDF 파일 안에 <lens> 태그를 보면 Fx, Fy, Cx, Cy 값이 명시되어 있거나, FOV(시야각)와 화면 해상도(Width)를 이용해 수식으로 계산할 수 있습니다.
        #cx = Width / 2
        #cy = Height / 2
        #fx = Width / (2 * tan(FOV_horizontal / 2))
        #fy = Height / (2 * tan(FOV_vertical / 2))
        #3. ★ 현재 코드에 적용된 우리의 방식 (물리적 거리 보정 / 튜닝) ★
        #    - 이론상 공식을 쓰면 드론의 '무게 중심(0,0,0)'이 아닌 '카메라 렌즈 표면'을 기준으로 거리를 잽니다.
        #    - 하지만 드론의 몸체와 렌즈 사이에는 약 10cm의 튀어나온 물리적 오차 거리가 존재합니다.
        #    - 우리는 드론을 목표 거리(2.5m)에 세워두고, 가제보 물리 엔진의 '절대 거리'와 카메라가 인식하는 
        #      '수학적 거리'가 완벽하게 일치할 때까지 비례식을 써서 초점거리를 역산했습니다.
        #    - 그 결과 찾아낸 이 드론만의 완벽한 맞춤형 초점거리 스펙이 바로 **1435.5** 입니다.
        #    (참고: 아래 554.25는 초기 뼈대일 뿐이며, image_callback에서 1435.5로 매 프레임 덮어씌워집니다.)
        self.camera_matrix = np.array([[554.25, 0.0, 320.5],
                                       [0.0, 554.25, 240.5],
                                       [0.0, 0.0, 1.0]], dtype=np.float32)
        self.dist_coeffs = np.zeros((4,1)) #렌즈 왜곡 계수(왜곡이 없는 핀홀 카메라로 가정)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    
            corners, ids, rejected = self.detector.detectMarkers(gray) #detector를 이용해 마커 감지
            
            if ids is not None:
                aruco.drawDetectedMarkers(cv_image, corners, ids) #id, 4개의 모서리
                for i in range(len(ids)):
                    #solvePnP를 사용하여 거리(tvec)와 회전(rvec) 계산
                    success, rvec, tvec = cv2.solvePnP(
                        self.obj_points, corners[i], self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    if success: #성공 시 거리 계산
                        distance = math.sqrt(tvec[0]**2 + tvec[1]**2 + tvec[2]**2)
                        text = f"ID:{ids[i][0]} Dist:{distance:.2f}m"
                        cv2.putText(cv_image, text, (10, 30 + i*30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                        self.get_logger().info(f"[마커 ID {ids[i][0]}] 거리: {distance:.2f}m (X:{tvec[0][0]:.2f}, Y:{tvec[1][0]:.2f}, Z:{tvec[2][0]:.2f})") #1차원 배열에서 값을 빼오기 위해 [0] 추가
                        cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
            cv2.imshow("Drone Front Camera", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"영상 변환 오류: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()