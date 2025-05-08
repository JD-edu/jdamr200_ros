
## jdAMR200 분산형 SLAM 지도 생성 – 라즈베리파이 실행 Launch 파일 설명서

### 1. 개요

이 문서는 **PC–라즈베리파이 분리형 방식**에서 `jdAMR200` 로봇을 이용해 **Cartographer SLAM으로 지도를 생성하는 과정 중, 라즈베리파이에서 실행할 노드를 하나의 Launch 파일로 구성한 예시**를 설명합니다.

사용된 Launch 파일 이름은 예시로 `jdamr200_lidar_node_launch.py`라 하며, 이 파일은 다음 두 가지 주요 노드를 실행합니다:

1. **jdamr200\_node** – 로봇 하드웨어 제어 (모터, IMU, 엔코더, cmd\_vel 처리)
2. **ld14.launch.py** – LD14 Lidar 구동

---

### 2. 분산형 SLAM 지도 생성 구조

| 실행 위치                 | 실행 노드 또는 Launch 파일                                                      | 역할                      |
| --------------------- | ----------------------------------------------------------------------- | ----------------------- |
| **라즈베리파이 (jdAMR200)** | `jdamr200_node`, `ld14.launch.py`                                       | 모터 및 센서 제어, 라이다 데이터 송신  |
| **PC**                | `cartographer_node`, `occupancy_grid.launch.py`, `rviz2`, `teleop_node` | SLAM 계산, 지도 시각화 및 로봇 조작 |

---

### 3. jdamr200\_lidar\_node\_launch.py 파일의 역할

이 Launch 파일은 **라즈베리파이에서 실행할 모든 핵심 노드**를 하나로 통합한 구성입니다. 이 파일을 실행함으로써 라즈베리파이 측에서 필요한 모든 ROS2 노드가 자동으로 실행됩니다.

* **로봇 제어 노드(jdamr200\_node)**:
  모터 제어, IMU 데이터 수집, 엔코더 읽기, `/cmd_vel` 수신 처리
* **LD14 Lidar 드라이버(ld14.launch.py)**:
  라이다를 통해 `/scan` 토픽으로 환경 정보를 발행
* **URDF 로봇 모델 포함**:
  RViz2 또는 다른 시각화 노드가 있는 경우 로봇 형상을 사용 가능하도록 파라미터 설정

---

### 4. 소스코드 설명

#### 📁 파일 위치

`launch/jdamr200_lidar_node_launch.py`

#### 🔍 주요 코드 설명

```python
use_sim_time = LaunchConfiguration('use_sim_time', default='false')
```

* 시뮬레이션 시간(`/clock`) 사용 여부를 런치 인자(argument)로 받을 수 있도록 설정

```python
jdamr200_urdf = os.path.join(get_package_share_directory('jdamr200_description'), 'urdf', 'jdamr200.urdf')
with open(jdamr200_urdf, 'r') as file:
    jdamr200_desc = file.read()
```

* URDF 파일을 불러와 `robot_description` 파라미터에 포함
* 로봇 형상은 RViz 또는 시뮬레이션 환경에서 활용 가능

```python
ld14lidar_launch_file_dir = os.path.join(get_package_share_directory('ldlidar_sl_ros2'), 'launch')
```

* 외부 제공된 LD14 Lidar 패키지의 Launch 디렉토리 경로를 가져옴

```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([ld14lidar_launch_file_dir, '/ld14.launch.py']),
    launch_arguments={'use_sim_time': use_sim_time}.items()
)
```

* `ld14.launch.py`를 포함하여 라이다 드라이버 노드를 실행

```python
Node(
    package='jdamr200_node',
    executable='jdamr200_node',
    output='screen',
)
```

* 아두이노 또는 ESP32와 시리얼 통신하여 모터/IMU/엔코더를 제어하는 노드를 실행
* `/cmd_vel`을 수신하여 로봇을 이동시킴

---

### 5. 실행 방법 (라즈베리파이에서)

1. SSH를 통해 라즈베리파이에 접속

```bash
ssh pi@192.168.0.xxx
```

2. ROS2 워크스페이스 환경 설정

```bash
source ~/jdamr200_ws/install/setup.bash
```

3. Launch 파일 실행

```bash
ros2 launch jdamr200_bringup jdamr200_lidar_node_launch.py
```

* `jdamr200_node`와 `ld14.launch.py`가 함께 실행됨
* `/scan`, `/odom`, `/imu`, `/cmd_vel` 등의 ROS2 토픽이 네트워크를 통해 PC로 전송됨

---

### 6. 이후 과정 (PC 측)

* 별도의 터미널에서 PC에 설치된 ROS2 워크스페이스에서 다음 명령어를 실행

```bash
source ~/jdamr200_ws/install/setup.bash
ros2 launch jdamr200_cartographer pc_only_cartographer.launch.py
```

* Cartographer SLAM 실행 및 지도 시각화가 진행됨
* `teleop_node` 실행 시 로봇을 조종할 수 있음

---

이 Launch 파일은 분산 환경에서 라즈베리파이 쪽의 로봇 노드와 라이다 노드를 일괄 실행하는 매우 중요한 구성 요소입니다.
이 문서는 SLAM 실습을 위한 라즈베리파이 측 실행 절차와 파일 구조 이해에 도움이 됩니다.


