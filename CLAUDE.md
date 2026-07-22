# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Language

**Always respond to the user in Korean (한국어).** Documentation, commit messages, and PR descriptions are written in Korean too.

## Requirements (항상 참고)

**Before any design or implementation decision, consult `docs/requirements/요구사항.md`.** It is the consolidated digest of the competition rules, the team's Pipeline architecture doc, and meeting notes (raw sources in `docs/requirements/raw/`).

Treat it as a **draft, not a fixed spec** — the team is still converging and the docs contradict each other and the current code. Its §8 lists the live mismatches (DOF 5 vs 6, motor model XL430 vs XM540/XC430, direct-servo vs MoveIt2 path, CAN vs UDP/TCP, etc.) — check there before trusting any single number. Priority is a working implementation over faithfully following the docs.

## What this is

ROS 2 Humble workspace for the 2025 극한로봇 (Extreme Robot) competition: a robot arm that visually tracks a target with YOLO and drives Dynamixel servos to follow it. The dev environment is fully containerized so the team shares one identical setup. Documentation and commit messages are in Korean.

## Environment model (important)

**All ROS 2 commands run *inside* the Docker container, not on the host.** The host is only used for `git` and `docker compose`.

- `./ros2_ws` is bind-mounted to `/root/ros2_ws` in the container, so host edits to `ros2_ws/src/` appear instantly inside.
- **Only `ros2_ws/src/` is version-controlled.** Build outputs (`build/`, `install/`, `log/`) are gitignored — each developer runs `colcon build` in their own container.
- Compose files: `docker-compose.yml` (기본) + `docker-compose.gpu.yml` (Jetson GPU override — `-f`로 얹어 쓴다). WSL2 지원은 제거됨. 컨테이너는 `ros2_humble`, `privileged: true` + `network_mode: host` + **`ipc: host`**.
- **`ipc: host`는 파워트레인 연동에 필수다.** 파워트레인은 같은 Jetson의 **별도 컨테이너**에서 ROS 2 노드를 돌리고 DDS로만 통신한다. Fast-DDS는 같은 호스트면 공유메모리(`/dev/shm`)로 데이터를 보내는데 Docker는 컨테이너마다 별도 `/dev/shm`을 준다 → `ipc: host`가 없으면 **discovery는 되는데 데이터가 한 건도 안 오는 조용한 실패**가 난다. 양쪽 컨테이너 모두 필요하다.
- The container runs `privileged`, so host devices (e.g. the Dynamixel USB serial adapter at `/dev/ttyUSB0`, the camera) are reachable without explicit `devices:` mappings — but the hardware must actually be plugged into the host.

## Common commands

Start container + enter (host):
```bash
xhost +local:docker && docker compose up -d
# Jetson GPU 가속까지 쓸 때만
xhost +local:docker && docker compose -f docker-compose.yml -f docker-compose.gpu.yml up -d

docker exec -it ros2_humble bash   # ROS already sourced via .bashrc
```

Build & run (inside container):
```bash
cd /root/ros2_ws
colcon build
source install/setup.bash
```

- Build one package: `colcon build --packages-select <pkg>`
- Resolve missing deps before reporting a build failure: `rosdep install --from-paths src --ignore-src -r -y`
- Rebuild image after a `Dockerfile` change: `docker compose build` then `up -d`.

## Dependency policy

System dependencies go in the **`Dockerfile`**, not ad-hoc `apt install`, so the team's environment stays reproducible. Already installed there: `dynamixel-sdk`, `dynamixel-workbench` (apt), `joint-state-publisher-gui`, **`moveit`, `ros2-control`, `ros2-controllers`** (apt, for MoveIt + mock hardware/controllers), a full `gstreamer1.0` plugin set (`base`/`good`/`bad`/`ugly`/`libav` + dev headers, for `stream_node`'s SRT streaming), and `ultralytics` + `numpy<2` + `pyrealsense2` + `onnx`/`onnxslim`(TensorRT export path, below) via pip (with `opencv-python` uninstalled so it doesn't clash with ROS's `cv_bridge` OpenCV).

**GPU 가속(CUDA/cuDNN/TensorRT)은 pip/apt로 컨테이너 안에 다시 설치하지 않고, 호스트 JetPack에 이미 있는 걸 `docker-compose.gpu.yml`로 마운트한다** (이미지 용량·빌드 시간 절감 + 호스트 드라이버와의 버전 불일치 회피):
- `torch`/`torchvision`만 예외 — Dockerfile에서 `pypi.jetson-ai-lab.io/jp6/cu126` 인덱스로 JetPack 6.2(CUDA 12.6) 호환 빌드를 `--no-deps`로 설치(PyPI 기본 torch는 더 최신 CUDA로 빌드돼 있어 `torch.cuda.is_available()`가 `False`로 떨어지고 YOLO가 CPU 폴백된다 — 2026-07-22 실측·수정, `torch.cuda.is_available()=True`/`device='Orin'` 확인됨).
- `CUDA_HOME`(기본 `/usr/local/cuda-12.6`) + `CUDNN_LIBS`(기본 `~/.cudnn-libs`, cuDNN 9.3 `.so` curated 디렉터리) + `TENSORRT_LIBS`/`TENSORRT_PYTHON`(기본 `~/.tensorrt-libs`/`~/.tensorrt-python`, TensorRT 10.3 `.so`+파이썬 바인딩 curated 디렉터리)를 read-only 마운트, `LD_LIBRARY_PATH`/`PYTHONPATH`에 연결(`docker-compose.gpu.yml` 주석에 curated 디렉터리 준비 명령 있음). Tegra 드라이버 레벨 라이브러리(`libcuda`/`libnvrm_*`/`libnvdla_*`/`libcudla`)는 `runtime: nvidia`가 자동으로 넣어주므로 별도 마운트 불필요.
- **TensorRT 백엔드(`perception_node`의 `backend:=trt` 파라미터) 실측(2026-07-22, box seg 모델 848×480)**: CUDA(pt) 17.1Hz → TensorRT FP16 25.1~25.5Hz(`/detected_objects` 실측). 첫 실행 시 `.pt→.onnx→.engine` 변환에 ~8분 소요, 이후는 캐시된 `.engine` 재사용(같은 입력 크기 한정).
  ⚠️ **함정**: `.engine` 파일은 task 메타데이터를 보존하지 않아 ultralytics가 자동 추정 시 seg 모델도 `task='detect'`로 오판하고 `r0.masks`가 **에러 없이 조용히 `None`**이 돼 markerless pose가 깨진다 — `model_presets.py`의 `task` 필드를 `YOLO(path, task=...)`에 명시적으로 넘겨서 해결(perception_node.py 참고). 새 preset 추가 시 `task` 필드 채우는 것 잊지 말 것.

> Base image is `ros:humble-ros-base` + `ros-humble-desktop` layered on top (split from the former `osrf/ros:humble-desktop-full` monolith), built for `linux/arm64` (`docker-compose.yml`) — this targets the Jetson deployment, not just dev laptops.

> Note: the Dynamixel libraries are pulled via apt (`ros-humble-dynamixel-sdk`, `ros-humble-dynamixel-workbench`) — **not** git submodules. An earlier broken submodule/gitlink for these was removed.

## 파워트레인 계약 (중요)

파워트레인 팀([power-train-sw](https://github.com/lightminn/power-train-sw))과 **같은 Jetson의 별도 컨테이너**에서 각자 ROS 2 노드를 돌리고 **DDS로만** 통신한다. 워크스페이스를 서로 오버레이하지 않으며, 공유하는 것은 메시지 계약뿐이다. 파워트레인은 `robot_arm_msgs`의 `.msg`만 벤더링해 자기들이 직접 빌드한다(ROS 2는 wire에서 **패키지명 + 구조 해시**로 매칭하므로 동일한 `.msg`로 각자 빌드하면 붙는다).

- **값 어휘의 단일 출처는 `dynamixel_control/contract.py`** — 파워트레인의 `powertrain_ros/contract.py`와 짝이다. **여기 없는 status 문자열을 새로 만들지 말 것.** 파워트레인은 `contract.ARM_STATUSES` 밖의 값을 받으면 즉시 `CONTRACT_VIOLATION` + motion hold를 건다. 어휘 변경은 **양 팀 합의 사항**이다.
- QoS는 `dynamixel_control/qos_profiles.py`. heartbeat 계열은 **KeepLast 1**이다 — depth를 키우면 낡은 샘플이 큐에 쌓여 파워트레인의 신선도 판정이 어긋난다.
- **`/arm_status`는 `arm_fsm`의 heartbeat 타이머 한 곳에서만 발행한다.** 상태 핸들러는 `_set_status()`로 값만 바꾼다. 발행 경로가 둘이 되면 `header.stamp` 순서가 뒤집힐 수 있는데, 파워트레인은 stamp가 0.5초 이상 역행하면 **영구 latch**(프로세스 재시작 전까지 해제 불가)를 건다.
- **`MISSION_STOP`만이 팔 작업 허가다.** `DRIVING`을 포함한 나머지 mode는 전부 잠금(default-deny).
  - **`arm_fsm_node.py`는 2026-07-15부터 이를 준수한다** — `LOCK_MODES`를 `contract.py`(파워트레인과 짝인 단일 출처) 걸로 통일해 `DRIVING` 수신 시 `_on_chassis_mode()`가 `_enter_locked()`를 건다. 언락은 `_try_advance()`의 `MISSION_STOP` + 같은 mission_id `ArrivalStatus` conjunction 경로 하나뿐(자동 언락 분기 없음).
- **차가 움직이려면 팔이 `STOWED_LOCKED` 또는 `CARRYING_LOCKED`를 신선하게 발행해야 한다**(`contract.DRIVE_READY_STATUSES`). 그 외 status는 전부 주행 불가다.
  - **이제 둘 다 발행한다**(`_is_settled()` 게이트 통과 후 `_set_status()`) — 접힘 모션(`_begin_stow_move`)도 구현됨.
  - ⚠️ 단, **`stow_joint_positions` 기본값(`[0.0, -0.6, 1.2]`)은 CAD 미검증 placeholder다.** all-zero home은 계약상 접힘 자세로 금지(구조 충돌·역구동 위험). **실기 검증 전까지 이 기본값으로 실제 서보를 구동하지 말 것** — 노드도 기동 시 이 경고를 로그로 찍는다.

### robot_arm_msgs (ament_cmake) — 공통 메시지 패키지
양팀(로봇팔·파워트레인)이 공유하는 커스텀 메시지 5개: `DetectedObject`(class_id/name/confidence/`geometry_msgs/Pose`/bbox), `DetectedObjectArray`(header + objects[]), `ArrivalStatus`, `ChassisMode`, `ArmStatus`. 인터페이스 상세는 `CLAUDE_Plan.md` §1 참고.

### robot_arm_perception (ament_python) — markerless 인식 노드
`perception_node` + `stream_node` 두 개. `perception_node`: RealSense D435i color+depth → YOLO **segmentation** 추론 → `/detected_objects`(`DetectedObjectArray`) 30Hz publish. **markerless pose**(대회 규정상 타겟 마커 부착 금지): translation은 마스크 centroid의 depth median deproject(`yolo_depth_3d.py` 로직 포팅, align 생략), orientation은 마스크 (u,v) 픽셀 **2D PCA** 주축각 → optical Z yaw quaternion. 카메라 intrinsics는 RealSense 스트림에서 직접 취득(calibration yaml 불필요). 또한 `/pick_target`(`DetectedObject`, transient_local latched)을 publish: `pick_classes` 화이트리스트 ∩ `pick_min_conf` 이상 ∩ depth 조건(`require_depth`) 만족 객체 중 confidence 최고 하나(신호등/정지선 등 관찰 전용은 화이트리스트로 자동 제외). 파라미터: `model_path`(**seg 모델이면 markerless pose 전체 활성, detection 전용이어도 bbox 중심 depth로 translation은 폴백** — 기본값이 2026-07-08부터 `models/best.pt`, 대회 타겟으로 Roboflow에서 커스텀 학습한 모델로 교체됨; 이전 COCO 사전학습 `yolov8n-seg.pt`는 `ros2_ws/` 루트에 그대로 남아있고 gitignore 대상), `camera_mode`(`realsense`|`test`), `conf_threshold`, `classes`/`pick_classes`(새 모델의 실제 클래스명에 맞게 실행 시 지정 필요 — 아직 미확인), `pick_min_conf`, `require_depth`, `frame_id` 등. ArUco 경로는 제거됨. 진행 상황은 `CLAUDE_Plan.md`·`WORK_STATUS.md`.

`perception_node`는 구독자가 있을 때만 `/perception/debug_image`(bbox·마스크·거리 오버레이, pick 타겟=초록/나머지=파란색)를 publish한다. `stream_node`는 이 토픽을 구독해 `gst-launch-1.0` 서브프로세스(x264enc→SRT)로 원격 PC에 H.264/SRT 스트리밍(`recv_stream.sh`로 수신) — 하드웨어 테스트 중 원격 모니터링용, `/pick_target` 등 제어 경로와 무관.

### dynamixel_control (ament_python) — the core runtime
Two runtimes share this package (entry points in `setup.py`): a **legacy YOLO→servo P-control pipeline** (3 nodes, below) and the **Phase 3 MoveIt/FSM pipeline** (`moveit_dynamixel_bridge` + `arm_fsm`) — the latter is the real 구간2 pick path.

```
yolo_detection ──/yolo/target_center──▶ yolo_bridge ──/dynamixel/goal_position──▶ position_node ──▶ physical XL430 servos
   (camera+YOLO)     [cx, cy]            (P-control)        [id, goal_pos]                          + /joint_states, /dynamixel/state
```

- `yolo_detection` (`yolo_detection_node.py`): opens the camera with `cv2.VideoCapture`, runs `ultralytics` YOLO, publishes the best target's pixel center to `/yolo/target_center`. **Does not use `rclpy.spin`** — it runs its own blocking `while rclpy.ok()` loop in `run()`; an OpenCV preview window (`show_window` param) needs X/GUI forwarding. Tunable params: `model_path`, `target_class`, `conf_threshold`, `camera_device`, etc.
- `yolo_bridge` (`yolo_to_dynamixel_bridge.py`): converts pixel error `cx - 320` into a goal position via simple proportional gain, publishes `[id=1, goal]` to `/dynamixel/goal_position`. Currently hardcoded to motor ID 1.
- `position_node` (`dynamixel_position_node.py`): touches hardware for the *legacy* pipeline. Talks to 5× XL430 (`DXL_IDS = [0..4]`) over `/dev/ttyUSB0` at 1 Mbps, protocol 2.0. Subscribes `/dynamixel/goal_position`, enables torque on startup, and at 10 Hz reads pos/vel/current/temp → publishes `/dynamixel/state` and a `/joint_states` (`JointState`) for RViz/MoveIt. Raw 0–4095 ↔ radians is approximated as `(raw-2048)*2π/4096`.

**MoveIt/FSM pipeline (Phase 3 — the real pick path; both nodes touch `/dev/ttyUSB0`/MoveIt, don't run alongside `position_node` on the same bus):**
- `moveit_dynamixel_bridge` (`moveit_dynamixel_bridge.py`): hardware node for the MoveIt path. Implements `/arm_controller/follow_joint_trajectory` + `/gripper_controller/follow_joint_trajectory` action servers, so MoveIt/`arm_fsm` execute on real servos (a lighter substitute for a full `ros2_control` HW interface). Reads `PRESENT_CURRENT`(126,2 signed)~`PRESENT_POSITION`(132,4) in one 10-byte SyncRead → publishes `/joint_states` with **position + effort (raw signed current)**. Gripper = single servo, both fingers mirrored; `gripper_ids`/`gripper_open_tick`/`gripper_close_tick`/`gripper_open_m`/`gripper_close_m` are params, defaulted per-module from `gripper_presets.GRIPPER_PRESETS` via a new `gripper_type` param (default `gripper_a`, matching `robot_arm_description`'s `xacro:arg gripper`) — still individually overridable by CLI/launch, empty `gripper_ids` disables the gripper → mock-friendly. Arm `JOINT_CONFIG` currently covers `joint_1..joint_3` (ids 0,1,2) — extend when arm DOF is finalized. **Only IDs whose torque-enable actually succeeds get registered in the SyncRead group** — a missing/unpowered servo no longer breaks readback for the rest of the bus.
- `arm_fsm` (`arm_fsm_node.py`): the 구간2 pick FSM (12 states `IDLE`~`LOCKED`, MoveIt 단일 경로 '가'). Subscribes `/pick_target`(latched)·`/arrival_status`·`/chassis_mode`·`/joint_states`, publishes `/arm_status`. Sends pose goals to MoveIt `move_action`; grasp/DROP decided from `/joint_states.effort` (raw-current thresholds). Gripper params (`gripper_joints`/`gripper_open`/`gripper_close`/`grasp_effort_thresh`/`drop_effort_thresh`/`gripper_action_time`) default from the same `gripper_presets.py`/`gripper_type` mechanism as the bridge (kept in sync intentionally — a mismatch here previously left both nodes defaulting to stale `left_finger_joint`/`right_finger_joint` names that didn't match the `gripper_a.xacro`/SRDF joints `gripper_a_joint5`/`gripper_a_joint6`). `_carry_pose()` looks up TF (`base_frame`←`tip_link`) for a base_link +Z lift (`lift_height`) → needs `tf2_ros` (in `package.xml`). Status string enums (`ARRIVED_PICKUP`/`DONE`/…) are **provisional, pending powertrain-team agreement**. Hardware-free smoke test: launch + mock-pub `/pick_target`(transient_local) + `/arrival_status` → expect `IDLE→PERCEIVE→PLAN→DESCEND` then a `move_action 미준비` warning (no move_group).
  - **`gripper_presets.py` (신규):** shared preset dict (`GRIPPER_PRESETS`, keyed by gripper name) consumed by both nodes above — adding a new gripper module (e.g. `gripper_b`) means adding one preset entry here, not editing either node's code. Currently only `gripper_a` is defined; its tick calibration (`gripper_open_tick=2446`/`gripper_close_tick=3186`) is the HW-8 real-servo measurement (id 5), the meter-domain calibration points (`gripper_open_m`/`gripper_close_m`) and effort thresholds are still placeholders pending real calibration.
  - **IK note (HW-7, 2026-07-05):** the URDF currently models only 3 of the arm's 5 axes (`joint_1..joint_3`, CAD still WIP), so MoveIt's 6DOF pose IK returns `NO_IK_SOLUTION` even for the live tip pose — confirmed on real hardware, not just a planning-difficulty issue. Default `ik_mode='analytic'` bypasses MoveGroup: FK service (`/compute_fk`, called from a **separate helper node** `arm_fsm_fk_client` — calling it from `self` inside the `_tick` timer callback deadlocks via reentrant spin) + a finite-difference Jacobian solves position-only 3DOF IK, publishing straight to `/arm_controller/joint_trajectory` (orientation is dropped). The MoveGroup path (`ik_mode='moveit'`) is kept, not removed — switch back once the URDF covers all 5 axes. Real-hardware verified end-to-end: bottle detection → analytic IK → descend → gripper close → effort-based grasp check.

### robot_arm_description (ament_cmake)
Compiles nothing — `CMakeLists.txt` only installs `urdf/`, `launch/`, `rviz/`, `config/` to `share/`. Adding a resource dir requires adding it to the `install(DIRECTORY ...)` block.
- **`urdf/robot_arm.urdf` — 2026-07-15 Isaac Sim 전면 재export (현재 소스, 5-DOF+그리퍼 통합).** 이전 Fusion360/fusion2urdf 계열("VerNOmiddle")과 그 위 시도였던 `gripper_a.xacro` URDF-레벨 모듈 스왑 방식은 **둘 다 폐기됨**(RViz에서 joint_2/3 축이 계속 이상해서 사용자 요청으로 전면 교체, `robot_arm.urdf` 파일 헤더 주석 참고). `urdf_and_usd/robotarm/robotarm_urdf_20260711.urdf` + 대응 USD를 원본에 대한 **순수 문자열 치환만**(mesh 경로, `<robot name>`) 적용해 그대로 가져옴 — 링크/조인트 재조립 없음. 결과: **58 link / 57 joint**, `link_001`..`link_057` 이름 체계(이전 fusion2urdf 이름·이 문서가 과거에 쓰던 `joint_1..joint_6`/`gripper_mount` 둘 다 아님), 관절 이름은 `arm_joint_1..5` + `gripper_drive_joint`(그리퍼 구동 조인트 1개, 나머지 평행 4절링크 8개 조인트는 URDF `<mimic>` 태그로 정식 종속 — 이전 `gripper_a_joint5/6` 미러링 방식에서 갈아탐). **그리퍼는 이제 이 트리에 완전히 통합**돼 있어(`gripper_a_base_link`가 `link_051`의 자식) URDF 레벨 그리퍼 교체(`urdf/grippers/*.xacro` 스왑)는 더 이상 없음 — 그리퍼 모듈화는 이제 **로직 레벨**(`dynamixel_control`의 `gripper_type`/`gripper_presets.py`)에서만 유지됨. `urdf/robot_arm.urdf.xacro`는 이 `robot_arm.urdf`를 그대로 `xacro:include`하는 얇은 wrapper.
- **손목 카메라는 아직 이 트리에 없음** — `wrist_camera_link`는 URDF 조인트가 아니라 `camera_tf.launch.py`가 별도로 발행하는 **홈 포즈 고정 static TF**다(아래 참고). 통합 절차는 Notion "wrist_camera_link 동적 TF 통합 — CAD 실측·검증 절차 (2026-07-22)" 참고.
- `launch/display.launch.py`: `robot_arm.urdf.xacro`를 `xacro.process_file()`로 처리해 로드(예전엔 raw `.urdf` 직접 로드였으나 이미 xacro 경유로 전환됨) + robot_state_publisher + joint_state_publisher_gui + rviz2. RViz launches with no saved config, so the model is invisible until you set Fixed Frame to `base_link`, add a RobotModel display, and set its Description Topic durability to `Transient Local` (see README).
- `launch/camera_tf.launch.py`: 카메라 2대분 static TF 발행. 전방 RGB-D(차체 고정): `base_link→camera_link`(장착 오프셋 launch arg `cam_x/y/z`·`cam_roll/pitch/yaw`, **CAD 실측값** 기본 `x=0.123, z=0.082, pitch=-0.26`) + `camera_link→camera_color_optical_frame`(REP-103 optical 회전 `-π/2,0,-π/2` 고정). 손목 RGB(그리퍼 위): `base_link→wrist_camera_link`(`wrist_cam_x/y/z`·`wrist_cam_roll/pitch/yaw`, CAD 실측값 기본 `x=0.040, z=0.295`) — **홈 포즈 기준 static placeholder**라 팔이 움직이면 실제 위치와 어긋남, URDF 관절 통합은 미착수(Notion 절차 문서는 있음). `perception_node`가 TF를 발행하지 않으므로, MoveIt이 `/pick_target`(camera frame) 목표를 `base_link`로 변환하려면 이 launch가 떠 있어야 함.
- `config/controllers.yaml`: `arm_controller`(`joint_trajectory_controller`)가 **`arm_joint_1`..`arm_joint_3`만**(update_rate 100) — URDF는 5축이지만 이 파일은 아직 실서보 배선이 끝난 3축만 등록(§ Watch out for 참고). 그리퍼 조인트 없음. `robot_arm_moveit_config/config/ros2_controllers.yaml`(아래)이 5축+그리퍼 전체를 다루는 더 최신/완전한 버전이니 실제로는 그쪽을 참고할 것.

### robot_arm_moveit_config (ament_cmake) — MoveIt 경로 계산용
Generated by MoveIt Setup Assistant; structure is complete and ready for motion planning. Use this package for path/trajectory planning.
- **Planning groups (`config/robot_arm.srdf`, 2026-07-15 URDF 교체에 맞춰 재생성됨):** `arm` is the kinematic chain `base_link` → `link_051`(`arm_joint_1..5`); `gripper` group = `gripper_drive_joint`, end effector parented to `link_051`. Named state `home` = all arm joints at 0. `gripper_open`/`gripper_closed` named states are **0/0 placeholder**(캘리브 전). Virtual joint `world` → `base_link` (fixed).
- **Collision matrix (`disable_collisions`)**: 두 종류로 구성 — "Adjacent"(부모-자식 조인트에서 기계적으로 100% 도출, 값 조작 없음)와 "Default"(2026-07-16, `/check_state_validity`로 관절 리밋 안쪽 무작위 자세 40개 샘플링해 항상 충돌하는 쌍만 등록 — 정식 MoveIt Setup Assistant GUI "Regenerate Default Collision Matrix"의 대체, 디스플레이 없는 환경이라 GUI 재실행 불가했음). **⚠️ 샘플 수(40)가 Setup Assistant 기본값(수천)보다 훨씬 적어 근거가 약함** — 디스플레이 있는 환경에서 기회 되면 GUI로 재생성해 이 블록을 정식 결과로 덮어쓸 것(SRDF 파일 안에 이 위험도 주석으로 남아있음).
- **IK solver (`config/kinematics.yaml`):** KDL (`kdl_kinematics_plugin/KDLKinematicsPlugin`) for the `arm` group.
- **Controllers:** MoveIt sends `FollowJointTrajectory` to `arm_controller` and `gripper_controller` (`config/moveit_controllers.yaml`); the matching `ros2_control` controllers are in `config/ros2_controllers.yaml` — `arm_controller`는 **`arm_joint_1..5`(5축 전체)**, `gripper_controller`는 `gripper_drive_joint` (update_rate 100 Hz, position command interface). 이 파일이 `robot_arm_description/config/controllers.yaml`(3축만)보다 최신/완전하다.
- **`demo.launch.py` is mock-only, not real hardware.** `config/robot_arm.ros2_control.xacro` loads the `mock_components/GenericSystem` plugin (the SetupAssistant `FakeSystem`), so `demo.launch.py` plans against fake joints — it does **not** drive the physical Dynamixels, and you must **not** run it alongside the bridge (its mock `ros2_control_node` competes for `/joint_states` and `/arm_controller`). To execute MoveIt plans on **real servos**, run `move_group.launch.py` + `rsp.launch.py` and let `dynamixel_control`'s `moveit_dynamixel_bridge` act as the controller (it implements the `/arm_controller`+`/gripper_controller` action servers MoveIt drives — a lighter alternative to a full `ros2_control` HW interface). **실서보는 현재 3축만 배선돼 있어**(`dynamixel_control`의 `JOINT_CONFIG`) 5축 planning 결과를 그대로 실행할 순 없음 — 4/5축 서보 배선·ID 확정이 선행 과제.
- **MoveIt mock demo works (재검증 2026-07-22).** `ros-humble-moveit` + `ros-humble-ros2-control` + `ros-humble-ros2-controllers` are in the Dockerfile. Run with:
  ```bash
  cd /root/ros2_ws && colcon build --packages-select robot_arm_description robot_arm_moveit_config
  source install/setup.bash && ros2 launch robot_arm_moveit_config demo.launch.py
  ```
  This brings up `move_group` + mock `ros2_control` + RViz MotionPlanning; all 3 controllers (`arm_controller`/`gripper_controller`/`joint_state_broadcaster`) go `active`, logs "You can start planning now!". Plan & Execute in RViz drives the *mock* joints only (not real servos).
- **Fixes baked in (don't regress):** (1) `urdf/robot_arm.urdf`의 Gazebo `ign_ros2_control/IgnitionSystem` 블록이 MoveIt mock `FakeSystem`과 충돌해 `ros2_control_node`를 크래시시켰던 문제 — 주석 처리됨(Gazebo Ignition 쓸 때만 재활성화). (2) `config/moveit_controllers.yaml`에 `action_ns: follow_joint_trajectory`가 빠져 MoveIt이 컨트롤러 0개로 봤던 문제 — 추가됨.
- The MoveIt SRDF uses link names `link_001`..`link_057`(`link_051`이 tip); confirm these match `robot_arm_description/urdf/robot_arm.urdf` when editing the URDF, or planning/collision checks break.

### pick_test_pkg (ament_python)
Standalone gripper test: `pick_test_node` listens on `/fake_object_position` (`Point`) and sends a `FollowJointTrajectory` action to `/gripper_controller/follow_joint_trajectory` for `left_finger_joint`/`right_finger_joint`.

## Watch out for

- **Joint-count mismatches across files are a live source of bugs.** `position_node` publishes only `joint_1`..`joint_5`, but the URDF and `controllers.yaml` define `joint_1`..`joint_6` (plus gripper joints). Keep `DXL_IDS`/`JOINT_NAMES`, the URDF, and the controller config in sync when editing any one of them.
- Hardware nodes fail without the real devices: `position_node` / `moveit_dynamixel_bridge` need the servo bus on `/dev/ttyUSB0` (and must not share the bus — pick one runtime); `yolo_detection` / `perception_node` need a camera (RealSense for `perception_node`). All rely on `privileged` for device access.
- `wrist_camera_link`'s static TF (`camera_tf.launch.py`) is a **home-pose-only placeholder** — it does not move with the arm. Don't trust it for pick geometry once the arm has left home; real eye-in-hand tracking needs the wrist camera integrated as a URDF joint (not done yet).
- **`ros2 run`/`ros2 launch` leak child nodes:** `kill <PID>`/`Ctrl-C` often kills only the wrapper, leaving the python node or `static_transform_publisher` running (→ CPU spin, `/arm_status` noise, stale TF). Clean up with `pkill -f <node>` and verify via `ps aux | grep ros2`.
- Branch strategy: `main` stays stable; feature work on `feat/*` branches.
</content>
