## jdAMR200 Cartographer SLAM – PC–라즈베리파이 분산 방식 안내서

### 1. 개요

이 문서는 `jdAMR200` 로봇에서 Cartographer SLAM을 통해 지도를 생성하는 \*\*분산 실행 방식(PC-Raspberry Pi)\*\*을 설명합니다.
라즈베리파이(RPi)는 로봇에 탑재되어 **모터 제어 및 라이다 데이터 송신**을 담당하며, 상대적으로 고성능인 **PC에서는 SLAM 처리와 지도 시각화를 담당**합니다.

이 방식은 RPi의 연산 자원이 제한적이라는 점을 고려하여 **성능 부담이 큰 Cartographer SLAM은 PC에서 실행**되도록 분리한 구조입니다.

---

### 2. 시스템 구성

| 장치              | 실행 노드 및 역할                                                     |
| --------------- | -------------------------------------------------------------- |
| **라즈베리파이(RPi)** | `jdamr200_node` (모터, IMU, 엔코더 제어), `ld14.launch.py` (라이다 드라이버) |
| **PC**          | `cartographer_node`, `occupancy_grid`, `rviz2`, `jdamr200_teleop`  |

두 장치는 동일한 **Wi-Fi 네트워크 상에 있어야 하며**, ROS2의 Fast DDS나 Multicast Discovery를 통해 서로 통신할 수 있어야 합니다.

---

### 3. 실행 순서

#### \[라즈베리파이에서 수행]

1. ROS2 워크스페이스 환경 설정

   ```bash
   source ~/jdamr200_ws/install/setup.bash
   ```

2. 라이다 노드 실행

   ```bash
   ros2 launch ldlidar_sl_ros2 ld14.launch.py
   ```

3. 모터/IMU 노드 실행

   ```bash
   ros2 run jdamr200_node jdamr200_node
   ```
4. 라이다노드와 모터/IMU 노드를 하나의 launch 파일로 묶어서 실행할 수 있습니다. 

---

#### \[PC에서 수행]

1. ROS2 환경 설정

   ```bash
   source ~/jdamr200_ws/install/setup.bash
   ```

2. Cartographer + RViz + TF + Occupancy Grid 실행

   ```bash
   ros2 launch jdamr200_cartographer cartographer.launch_no_lidar.py
   ```

3. Teleop 노드 실행 (로봇을 조작하기 위해)

   ```bash
   ros2 run jdamr200_teleop teleop_node
   ```
이 launch 파일에서 라이다 관련 노드를 삭제한이유는 실제 라이다는 라즈베리파이 쪽에서 처리하기 때문입니다. 

---

### 4. 동작 흐름 요약

1. **라즈베리파이**에서 `/scan`, `/odom`, `/imu`, `/tf`, `/cmd_vel` 처리를 담당한다.
2. **PC**는 라즈베리파이로부터 `/scan`, `/odom`, `/tf` 등의 토픽을 구독해 `cartographer_node`에서 SLAM을 수행한다.
3. 생성된 `/map`은 `rviz2`를 통해 시각화된다.
4. PC에서 실행된 `jdamr200_teleop` `/cmd_vel`을 발행하고, 이 메시지는 라즈베리파이의 `jdamr200_node`로 전달되어 로봇을 움직인다.

---

### 5. PC용 Launch 파일 구성 설명 (라이다 제외)

`pc_only_cartographer.launch.py`는 라이다 실행 코드를 제거하고, TF 설정, RViz, Cartographer, Occupancy Grid 노드만 실행하는 구성입니다.
이는 **PC에서만 실행**되며, 라이다 노드와 모터 노드는 라즈베리파이에서 별도로 실행됩니다.

#### 주요 구성 요소 설명:

1. **TF 정적 변환 설정**

```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
)
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'odom', 'body_link']
)
```

* `map → odom`, `odom → body_link` 연결을 수동으로 생성하여 SLAM이 정확히 동작하게 함.

2. **RViz2 실행**

```python
Node(package='rviz2', executable='rviz2')
```

* 지도를 시각화하기 위한 RViz 실행.

3. **Cartographer 노드 실행**

```python
Node(
    package='cartographer_ros',
    executable='cartographer_node',
    name='cartographer_node',
    parameters=[{ 'use_sim_time': use_sim_time }],
    arguments=[
        '-configuration_directory', jdamr200_config_dir,
        '-configuration_basename', configuration_basename
    ]
)
```

* SLAM 계산을 담당하는 Cartographer 핵심 노드.

4. **OccupancyGrid 발행**

```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([.../occupancy_grid.launch.py]),
    launch_arguments={...}.items()
)
```

* 2D `/map`을 생성하여 RViz에서 사용할 수 있도록 함.

5. **설정 인자 선언**

```python
DeclareLaunchArgument('use_sim_time', default_value='false')
DeclareLaunchArgument('configuration_basename', default_value='jdamr200_lidar.lua')
DeclareLaunchArgument('resolution', default_value='0.05')
DeclareLaunchArgument('publish_period_sec', default_value='1.0')
```

* 사용자가 필요에 따라 launch 시 파라미터를 쉽게 조절할 수 있도록 선언됨.

---

### 6. 네트워크 설정 팁

* 라즈베리파이와 PC는 같은 서브넷에 있어야 하며, `ROS_DOMAIN_ID`를 일치시켜야 한다.
* Fast DDS는 기본적으로 multicast를 통해 자동 연결되지만, 복잡한 네트워크 환경에서는 `ROS_LOCALHOST_ONLY=0`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` 환경변수를 활용하여 연결을 명확히 해야 함.
* PC쪽이 실제 리눅스머신이 아닌 Virtual Box를 사용할 경우 연결이 되지 않을 수 있습니다. 
---

### 7. 실습 시 주의사항

* 라이다와 로봇의 좌표계(`scan_frame`, `base_link`, `odom`)가 올바르게 설정되어 있어야 Cartographer가 올바르게 동작함.
* RViz에서 Fixed Frame은 반드시 `map`으로 설정되어야 함.
* 라이다와 로봇의 위치가 일치하지 않거나 TF가 누락되면 `/map`이 생성되지 않거나 왜곡됨.

* 물론입니다. 요청하신 내용을 문서 맨 마지막에 **SSH를 사용하여 라즈베리파이에서 원격으로 jdamr200 노드를 실행하는 방법** 항목으로 추가해드렸습니다. 실전 명령어 예시와 함께 바로 사용할 수 있도록 구성했습니다.

---

### 8. 라즈베리파이 원격 접속 및 실행 방법 (SSH 사용)

분산 방식에서는 라이다 드라이버(`ld14.launch.py`)와 로봇 제어 노드(`jdamr200_node`)를 라즈베리파이에서 실행해야 합니다.
이때 라즈베리파이 화면이나 키보드를 직접 사용할 필요 없이, **PC에서 SSH로 원격 접속**하여 필요한 노드를 실행할 수 있습니다.

---

#### ■ 1단계: SSH로 라즈베리파이에 접속

먼저, PC 터미널에서 아래 명령어로 라즈베리파이에 원격 접속합니다.

```bash
ssh pi@192.168.0.123
```

* `pi`: 라즈베리파이 사용자 이름 (보통 기본값)
* `192.168.0.123`: 라즈베리파이의 IP 주소 (본인 환경에 맞게 수정 필요)
* 비밀번호 입력 후 접속됩니다.

※ IP 주소는 라즈베리파이에서 `hostname -I` 명령어로 확인하거나 공유기 관리자 페이지에서 찾을 수 있습니다.

---

#### ■ 2단계: ROS2 환경 설정

SSH 접속 후, ROS2 워크스페이스의 환경 설정을 수행합니다:

```bash
source ~/jdamr200_ws/install/setup.bash
```

※ `setup.bash` 경로는 사용자의 워크스페이스 이름에 따라 다를 수 있습니다.

---

#### ■ 3단계: 노드 실행

1. **jdamr200\_node 실행 (모터제어 및 센서 통신)**

```bash
ros2 run jdamr200_node jdamr200_node
```

2. **라이다 드라이버 실행**

```bash
ros2 launch ldlidar_sl_ros2 ld14.launch.py
```

※ 각각 **별도의 터미널 창 또는 tmux 세션**에서 실행하는 것이 안정적입니다.

---

#### 🧩 팁: 하나의 SSH 세션에서 여러 개의 터미널을 관리하려면

`tmux` 또는 `screen` 등의 터미널 멀티플렉서 툴을 사용하면 하나의 SSH 접속에서 여러 세션을 관리할 수 있습니다:

```bash
sudo apt install tmux
tmux new -s lidar
# 이 안에서 라이다 실행 후 Ctrl+b, d 로 백그라운드로 보내고 다시 접속 가능
```
