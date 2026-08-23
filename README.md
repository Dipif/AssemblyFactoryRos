# AssemblyFactoryRos

ROS 2를 사용하여 NVIDIA Isaac Sim 6.0.1에서 구성한 Scene의 협동 로봇들을 제어하는 프로젝트입니다.

Isaac Sim Scene은 별도의 [AssemblyFactoryScene](https://github.com/Dipif/AssemblyFactoryScene) 저장소에서 관리합니다.

## 실행 환경

| 항목 | 설정 |
| --- | --- |
| 호스트 OS | Windows 11 |
| 시뮬레이터 | NVIDIA Isaac Sim 6.0.1 |
| ROS 2 환경 | WSL2 Ubuntu 22.04 + ROS 2 Humble |
| RMW | Fast DDS (`rmw_fastrtps_cpp`) |

## 프로젝트 구성

```text
AssemblyFactoryRos/
├── fastdds.xml
├── isaac_ros_env.bash
└── README.md
```

- `fastdds.xml`: Windows Isaac Sim과 WSL2 ROS 2 사이의 Fast DDS UDPv4 설정
- `isaac_ros_env.bash`: ROS 2 환경변수를 설정하는 스크립트

## WSL2 ROS 2 환경

Isaac Sim과의 연동을 위해 터미널 실행시 아래의 명령을 실행해야 합니다.

```bash
source ~/project/AssemblyFactoryRos/isaac_ros_env.bash
```

Windows와 WSL2는 모두 Domain ID `42`, Fast DDS 및 UDPv4 프로파일을 사용해야 합니다. 

## 관련 문서

- [Isaac Sim 6.0.1 Documentation](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/)
- [ROS 2 Installation on Other Platforms](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/install_ros_other_platforms.html)
- [ROS 2 Joint Control Tutorial](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/ros2_tutorials/tutorial_ros2_manipulation.html)