## cartographer.launch.py 사용방법을 정리한 jdAMR200 로봇 기반  Cartographer SLAM 설명서

## jdAMR200 Cartographer SLAM (All-in-One 방식)

### 1. 개요

이 문서는 `jdAMR200` 로봇을 이용하여 **Cartographer SLAM 기반의 지도 생성을 수행하는 방법**을 설명한다. 본 설명서는 특히 **All-in-One 방식**, 즉 모든 노드를 하나의 제어 컴퓨터에서 실행하는 환경을 기준으로 작성되었다.

### 2. 지도 생성 방식 비교

1. **All-in-One 방식**

   * 로봇의 제어 컴퓨터(예: Jetson, 미니PC 등)의 성능이 충분한 경우 사용.
   * 라이다, 모터제어, SLAM, RViz 등 모든 노드를 한 컴퓨터에서 실행함.
   * 본 문서에서 설명하는 Launch 파일이 이 방식을 위한 구성이다.

2. **PC - Raspberry Pi 분산 방식**

   * 라즈베리파이의 연산 성능으로는 Cartographer 실행에 어려움이 있으므로, SLAM은 PC에서 실행하고 로봇 제어와 라이다는 라즈베리파이에서 수행한다.
   * 두 시스템은 동일한 ROS 네트워크에서 동작해야 한다.

---

### 3. All-in-One 방식 실행 절차

1. `ld14.launch.py` 라이다 노드와 `jdamr200_node` 모터제어 노드를 실행한다.
2. Cartographer 관련 노드를 포함한 launch 파일(`all_in_one_cartographer.launch.py`)을 실행한다.
3. `map → odom`과 `odom → body_link` 연결을 위한 정적 TF 노드를 실행한다.
4. `rviz2`를 실행하여 시각화 환경을 구성한다.
5. 별도의 터미널에서 `teleop_node`를 실행하여 키보드로 로봇을 조작한다.
6. 로봇이 움직이며 라이다로 주변 환경을 스캔하면, `cartographer_node`가 이를 분석하여 `/map` 토픽으로 발행한다.
7. RViz에서 실시간으로 맵이 시각화되며 점차 확장된다.

---

### 4. Launch 파일 구성 설명 (cartographer.launch.py)

* **라이다 노드 실행**
  `ld14.launch.py`를 IncludeLaunchDescription으로 불러와 LP14 Lidar를 구동함.

* **정적 TF 브로드캐스트 노드 (2개)**

  * `map → odom`: SLAM의 월드 기준 좌표 설정
  * `odom → body_link`: 로봇의 실시간 위치 추적을 위한 TF 설정

* **RViz 실행**
  시각화를 위한 RViz2 노드 실행

* **Cartographer SLAM 노드 실행**
  `cartographer_node`를 통해 `/scan`, `/odom`, `/tf` 정보를 이용하여 SLAM 수행

* **OccupancyGrid 발행 노드 실행**
  생성된 SLAM 데이터를 `/map` 토픽으로 주기적으로 발행

---

### 5. 주요 설정 요소

* `use_sim_time`: 시뮬레이션 시간 사용 여부
* `configuration_basename`: `.lua` 기반의 Cartographer 파라미터 설정 파일 이름
* `resolution`: occupancy grid 맵의 해상도 (예: 0.05m)
* `publish_period_sec`: `/map`을 발행하는 주기 (초)

---

### 6. 전체 토픽 흐름 구조

* `teleop_node` → `/cmd_vel`
* `jdamr200_node` → `/odom`, 모터 제어 → 실제 로봇 움직임
* `ld14.launch.py` → `/scan`
* `cartographer_node` → `/map`
* `rviz2` → `/map`, `/tf`, `/scan` 시각화

---

### 7. 실습 시 주의사항

* 로봇은 천천히 움직이며, 구석구석 공간을 스캔해야 정확한 맵이 생성됨.
* RViz에서는 다음 요소들을 시각화해야 함:

  * `Fixed Frame`: `map`
  * `/scan`, `/map`, `/tf`, `/odom`
* 맵이 완성되면 `/map` 토픽을 저장하여 Nav2 자율주행 등에 활용 가능

---

### 8. 관련 파일 정리

| 파일명                                  | 설명                      |
| ------------------------------------ | ----------------------- |
| jdamr200\_lidar.lua                  | Cartographer 파라미터 설정 파일 |
| occupancy\_grid.launch.py            | OccupancyGrid 맵 발행 설정   |
| ld14.launch.py                       | LP14 라이다 실행             |
| all\_in\_one\_cartographer.launch.py | 전체 노드 통합 실행 Launch 파일   |

---

### 9. 참고 자료

* Cartographer 공식 문서: [https://google-cartographer.readthedocs.io](https://google-cartographer.readthedocs.io)
* ROS2 TF2 개요: [https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Understanding-Tf2.html](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Understanding-Tf2.html)
* RViz 사용법: [https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Visualizing-One-Topic.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Visualizing-One-Topic.html)

좋습니다. 아래는 기존 설명서 하단에 **`all_in_one_cartographer.launch.py` 파이썬 Launch 파일에 대한 간략한 코드 설명**을 추가한 내용입니다. 전체 설명서 마무리에 적절하게 삽입하도록 구성했습니다.

---

### 10. `cartographer.launch.py` 파이썬 코드 간략 설명

이 파일은 ROS2의 여러 노드를 조합하여 Cartographer 기반 SLAM을 한 번에 실행하는 Launch 파일입니다. 주요 구성 요소는 다음과 같습니다:

1. **파라미터 선언부**

   ```python
   DeclareLaunchArgument('use_sim_time', default_value='false', ...)
   DeclareLaunchArgument('configuration_basename', default_value='jdamr200_lidar.lua', ...)
   ```

   * 런치 시 전달 가능한 인자값들을 정의하며, 주로 시뮬레이션 시간 사용 여부, 설정 파일 경로 등을 제어합니다.

2. **라이다 노드 실행**

   ```python
   IncludeLaunchDescription(PythonLaunchDescriptionSource([.../ld14.launch.py]), ...)
   ```

   * LP14 LDLiDAR 장치를 위한 별도 Launch 파일을 포함시켜 실행합니다.

3. **정적 TF 노드 2개 실행**

   ```python
   static_transform_publisher ... 'map' → 'odom'
   static_transform_publisher ... 'odom' → 'body_link'
   ```

   * 로봇의 좌표계를 연결하기 위한 필수 TF 설정입니다.

4. **RViz2 노드 실행**

   ```python
   Node(package='rviz2', executable='rviz2')
   ```

   * RViz2 시각화 도구를 실행합니다.

5. **Cartographer SLAM 노드 실행**

   ```python
   Node(package='cartographer_ros', executable='cartographer_node', ...)
   ```

   * `.lua` 설정 파일을 기반으로 SLAM 처리를 수행하는 핵심 노드입니다.

6. **OccupancyGrid 발행 Launch 포함**

   ```python
   IncludeLaunchDescription(PythonLaunchDescriptionSource([.../occupancy_grid.launch.py]), ...)
   ```

   * Cartographer가 계산한 맵을 2D Grid 형태로 `/map` 토픽에 발행하는 노드입니다.

이러한 구성을 통해 사용자는 단 하나의 Launch 파일로 라이다 실행, TF 연결, SLAM 수행, 시각화까지 **지도 생성에 필요한 모든 프로세스를 자동화**할 수 있습니다.


