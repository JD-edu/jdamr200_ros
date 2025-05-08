
## jdAMR200 URDF 시각화용 Launch 파일 설명서

### 1. 개요

이 문서는 `jdAMR200` 로봇의 URDF 모델을 **RViz2를 통해 시각화**하기 위한 전용 Launch 파일을 설명한다.
주 목적은 로봇의 링크 구조, 조인트 구조, 센서 위치 등을 시각적으로 확인하고, TF 트리 구조를 검증하며 URDF 모델을 디버깅하는 데 있다.

---

### 2. 활용 목적

| 활용 목적     | 설명                                |
| --------- | --------------------------------- |
| URDF 시각화  | 로봇 모델링이 정확하게 되었는지 RViz로 확인        |
| TF 프레임 확인 | `/tf` 트리를 통해 조인트 및 링크 연결 구조 점검    |
| 센서 위치 확인  | URDF에 포함된 LiDAR, IMU 등의 위치 시각화 가능 |
| 디버깅 및 문서화 | 로봇 모델을 시각적으로 설명하거나 리포트에 활용        |

---

### 3. Launch 파일 구성요소 설명

#### 1) `use_sim_time` 런치 인자 선언

```python
DeclareLaunchArgument(
    'use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true'
)
```

* 시뮬레이션 시간(`clock` 토픽)을 사용할지 여부 설정
* 일반적으로 실제 로봇 없이 RViz만 띄우는 경우 `true`로 설정

---

#### 2) URDF 파일 로딩

```python
jdamr200_urdf = os.path.join(
    get_package_share_directory('jdamr200_description'),
    'urdf',
    'jdamr200.urdf')
with open(jdamr200_urdf, 'r') as file:
    jdamr200_desc = file.read()
```

* `jdamr200_description` 패키지에 포함된 URDF 파일을 읽어 `robot_description` 파라미터로 설정
* 이 파라미터는 이후 `robot_state_publisher`에 전달되어 로봇의 링크-조인트 구조를 퍼블리시함

---

#### 3) `robot_state_publisher` 실행

```python
Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[robot_param, {'use_sim_time': use_sim_time}],
)
```

* `robot_description`을 기준으로 TF 트리를 생성하고 퍼블리시함
* 실시간으로 로봇의 링크 위치 및 관절 상태를 `/tf`로 전송

---

#### 4) `joint_state_publisher` 실행

```python
Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
)
```

* 로봇의 조인트를 수동으로 조작 가능하게 해줌
* URDF가 고정된 조인트(예: 회전 가능한 LIDAR, 팔 관절 등)를 포함할 경우 조작 시 시각적 변화 확인 가능

---

#### 5) `rviz2` 실행

```python
Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_file],
)
```

* 지정된 RViz 설정 파일(`jdamr200_display_rviz.rviz`)을 불러와 시각화 실행
* 로봇 모델과 TF, Grid, Axes 등이 포함된 기본 시각화 구성을 불러옴

---

### 4. 실행 방법

1. ROS2 환경 설정

```bash
source ~/jdamr200_ws/install/setup.bash
```

2. Launch 실행

```bash
ros2 launch jdamr200_bringup jdamr200_urdf_launch.py
```

3. RViz 창이 열리고 로봇 모델이 로드됨

   * 좌측 Displays에서 `/robot_description`, `/tf`, `/joint_states` 등을 확인
   * 로봇 모델이 제대로 보이지 않으면 URDF 오류 가능성 있음

---

### 5. 디버깅 팁

* URDF에 오류가 있는 경우 RViz에 로봇이 보이지 않거나 콘솔에 오류 메시지가 출력됨
* TF가 꼬이거나 누락된 경우, RViz 좌측 하단의 TF Tree를 열어 확인할 수 있음
* RViz에서 Fixed Frame은 반드시 `base_link` 또는 `body_link`로 설정

---

### 6. 관련 패키지 및 파일 구조

| 파일/패키지                                             | 설명            |
| -------------------------------------------------- | ------------- |
| `jdamr200_description/urdf/jdamr200.urdf`          | 로봇 모델 정의      |
| `jdamr200_bringup/rviz/jdamr200_display_rviz.rviz` | 기본 RViz 설정 파일 |
| `robot_state_publisher`                            | TF 구조 퍼블리셔    |
| `joint_state_publisher`                            | 조인트 수동 조작기    |
| `rviz2`                                            | 시각화 도구        |

---

### 7. 결론

이 Launch 파일은 URDF 기반 로봇 모델을 확인하고 시각화할 수 있는 필수적인 도구로서,
로봇 모델의 구조적 완성도와 센서 위치 검증을 위한 **디버깅 및 개발 초기 필수 구성**이다.

특히 SLAM 또는 자율주행 시스템에 통합되기 전, 로봇 모델링이 정확한지 검증하는 데 큰 도움이 된다.


