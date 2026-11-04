"""LIMO 애커먼 모델을 Isaac Sim 4.5에 올리고 ROS2와 연결하는 예제입니다.

WeGo-Robotics/LimoIsaacSIM (sim 브랜치) limo_ros2_diff.py를 바탕으로 바꾼 점:
- 로봇: limo_diff_thin.usd → limo_ackermann.usd
- 주행: Twist + DifferentialController → ROS2 Subscribe AckermannDrive + Ackermann Controller
  (조향 조인트는 위치 명령, 바퀴 조인트는 속도 명령)
- 라이다: 애커먼 모델의 구형 PhysX 라이다 대신, 차동 예제와 같은 RTX 라이다(RPLidar_S2e.usd)를 laser_link에 붙임
- IMU: 애커먼 모델에 IMU 센서가 없어 그래프를 뺐습니다(우리 코드는 IMU를 쓰지 않음)
- 토픽 이름을 실차와 같게: /scan, /odom, /camera/color/image_raw (+ camera_info)
- 트랙·장애물·라바콘·출발 위치를 scenario.json에서 읽음

실차 /limo/ack_cmd는 AckermannDrive(헤더 없음)인데 Isaac 구독 노드는 AckermannDriveStamped를 받습니다.
그래서 시뮬레이터 쪽 명령 토픽은 /sim/ackermann_cmd 이고, tools/ack_stamp_relay.py가 중간에서 바꿉니다.

※ 이 파일은 Isaac Sim이 없는 환경에서 작성했고 실행 확인을 하지 않았습니다. 안내서의 '첫 실행 점검' 순서대로 확인하십시오.
"""
import json
import math
import os

import carb
import numpy as np
import omni
import omni.graph.core as og
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCone, FixedCone, FixedCuboid, FixedCylinder
from isaacsim.core.prims import SingleArticulation, XFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
from isaacsim.examples.interactive.base_sample import BaseSample
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics


def _yaw_quat(yaw_deg):
    h = math.radians(yaw_deg) / 2.0
    return np.array([math.cos(h), 0.0, 0.0, math.sin(h)])  # (w, x, y, z)


class LimoROS2Ackermann(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        if not omni.kit.app.get_app().get_extension_manager().is_extension_enabled("isaacsim.ros2.bridge"):
            carb.log_error("ROS2 Bridge is not enabled. Please enable the extension to use this feature.")

        self._base_dir = os.path.dirname(os.path.abspath(__file__))
        self._usd_dir = os.path.normpath(os.path.join(self._base_dir, "../../USD/WegoLimo"))
        self._path_to_robot_usd = os.path.join(self._usd_dir, "Limo/limo_ackermann.usd")
        self._path_to_lidar_usd = os.path.join(self._usd_dir, "Limo/RPLidar_S2e.usd")
        self._path_to_wego_track_usd = os.path.join(self._usd_dir, "LimoTrack/LIMO_simulation_table.usd")
        self._scenario_path = os.path.join(self._base_dir, "scenario.json")

        self._robot_prim_path = "/World/Limo"
        self._track_prim_path = "/World/Track"
        self._articulation = None

        # limo_ackermann.usd에서 읽은 값 (usd-core로 확인, 2026-11-04)
        self._wheelBase = 0.20        # 앞바퀴 x=+0.10, 뒷바퀴 x=-0.10
        self._trackWidth = 0.14       # 좌우 y=±0.07
        self._wheelRadius = 0.045
        self._maxWheelRotation = 0.42  # 실차 조향 한계(위키). 모델 조인트 한계는 ±30°(0.52 rad)
        self._maxWheelVelocity = 30.0  # rad/s ≈ 1.35 m/s (추정값, 실차 최고 속도보다 넉넉히)
        self._maxAcceleration = 1.0    # NVIDIA 튜토리얼 값
        self._maxSteeringAngleVelocity = 2.0  # rad/s, 추정값
        self._steer_joints = ["left_steering_hinge_wheel", "right_steering_hinge_wheel"]
        self._wheel_joints = ["rear_left_wheel", "rear_right_wheel", "front_left_wheel", "front_right_wheel"]

        self._basePrim = "/World/Limo/base_link"
        self._cameraPath = "/World/Limo/depth_link/rgb_camera"
        self._lidarPath = "/World/Limo/laser_link/RPLidar_S2e/RPLIDAR_S2E"
        self._sensorPrims = ["/World/Limo/depth_link", "/World/Limo/laser_link"]
        self._scenario = self._load_scenario()

    # ------------------------------------------------------------------ 시나리오
    def _load_scenario(self):
        default = {
            "track_usd": "",
            "track_scale": 1.0,
            "robot_start": {"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
            "invert_steering": False,
            "camera_width": 640,
            "camera_height": 480,
            "objects": [],
        }
        try:
            with open(self._scenario_path, encoding="utf-8") as f:
                default.update(json.load(f))
        except Exception as e:  # 파일이 없거나 틀리면 기본값으로 진행
            carb.log_warn(f"scenario.json을 읽지 못해 기본값을 씁니다: {e}")
        return default

    # ------------------------------------------------------------------ 장면
    def add_light(self):
        light = UsdLux.SphereLight.Define(get_current_stage(), Sdf.Path("/World/SphereLight"))
        light.CreateRadiusAttr(2)
        light.CreateIntensityAttr(100000)
        XFormPrim(str(light.GetPath())).set_world_poses(np.array([[0, 0, 12]]))
        dome = UsdLux.DomeLight.Define(get_current_stage(), Sdf.Path("/World/DomeLight"))
        dome.CreateIntensityAttr(500)

    def _set_xform(self, prim, scale=None, translate=None, orient=None):
        xf = UsdGeom.Xformable(prim)
        ops = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}
        if scale is not None:
            (ops.get("xformOp:scale") or xf.AddScaleOp()).Set(Gf.Vec3f(*scale))
        if translate is not None:
            (ops.get("xformOp:translate") or xf.AddTranslateOp()).Set(Gf.Vec3d(*translate))
        if orient is not None:
            op = ops.get("xformOp:orient") or xf.AddOrientOp()
            q = Gf.Quatf(*orient) if op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat else Gf.Quatd(*orient)
            op.Set(q)

    def add_track(self):
        track = self._scenario.get("track_usd", "")
        if track:
            path = track if os.path.isabs(track) else os.path.join(self._base_dir, track)
            add_reference_to_stage(usd_path=path, prim_path=self._track_prim_path)
            s = float(self._scenario.get("track_scale", 1.0))
            self._set_xform(self._stage.GetPrimAtPath(self._track_prim_path), scale=(s, s, s))
        else:
            # WeGo 기본 트랙 (WeGo 예제와 같은 위치)
            add_reference_to_stage(usd_path=self._path_to_wego_track_usd, prim_path=self._track_prim_path)
            self._set_xform(self._stage.GetPrimAtPath(self._track_prim_path), scale=(0.01, 0.01, 0.01),
                            translate=(0.28, 0.49, 0.01), orient=(0.707, 0.0, 0.0, 0.707))

    def add_objects(self):
        """scenario.json의 objects: 정차 LIMO(box), 라바콘(cone), 기둥(cylinder)."""
        for i, o in enumerate(self._scenario.get("objects", [])):
            if "type" not in o:  # 설명용 항목은 건너뜀
                continue
            kind = o["type"]
            name = o.get("name", f"{kind}_{i}")
            x, y = float(o["x"]), float(o["y"])
            yaw = _yaw_quat(float(o.get("yaw_deg", 0.0)))
            color = np.array(o.get("color", [0.9, 0.4, 0.0] if kind == "cone" else [0.2, 0.2, 0.2]))
            path = f"/World/Obs/{name}"
            if kind == "box":
                sx, sy, sz = o.get("size", [0.32, 0.23, 0.25])  # LIMO 외곽(모델 bbox 기준)
                obj = FixedCuboid(prim_path=path, name=name, position=np.array([x, y, sz / 2]),
                                  orientation=yaw, scale=np.array([sx, sy, sz]), size=1.0, color=color)
            elif kind == "cone":
                r, h = o.get("radius", 0.10), o.get("height", 0.20)  # 추정값: 실제 라바콘 실측해 바꾸십시오
                cls = DynamicCone if o.get("dynamic", True) else FixedCone
                kw = {"mass": o.get("mass", 0.2)} if cls is DynamicCone else {}
                obj = cls(prim_path=path, name=name, position=np.array([x, y, h / 2]),
                          radius=r, height=h, color=color, **kw)
            else:
                r, h = o.get("radius", 0.05), o.get("height", 0.3)
                obj = FixedCylinder(prim_path=path, name=name, position=np.array([x, y, h / 2]),
                                    radius=r, height=h, color=color)
            self._world.scene.add(obj)

    def add_robot(self):
        add_reference_to_stage(self._path_to_robot_usd, self._robot_prim_path)
        # limo_ackermann.usd에는 ArticulationRoot가 base_footprint와 base_link 두 곳에 있습니다.
        # WeGo가 고친 차동 모델(limo_diff_thin.usd)은 base_link에만 있으므로 같게 맞춥니다.
        fp = self._stage.GetPrimAtPath(self._robot_prim_path + "/base_footprint")
        if fp.IsValid() and fp.HasAPI(UsdPhysics.ArticulationRootAPI):
            fp.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            if fp.HasAPI(PhysxSchema.PhysxArticulationAPI):
                fp.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
        # 차동 예제와 같은 RTX 라이다를 laser_link에 붙입니다.
        add_reference_to_stage(self._path_to_lidar_usd, self._robot_prim_path + "/laser_link/RPLidar_S2e")

        st = self._scenario.get("robot_start", {})
        XFormPrim(self._robot_prim_path).set_world_poses(
            positions=np.array([[float(st.get("x", 0.0)), float(st.get("y", 0.0)), 0.0]]),
            orientations=np.array([_yaw_quat(float(st.get("yaw_deg", 0.0)))]))

        self._articulation = SingleArticulation(self._robot_prim_path)
        self._world.scene.add(self._articulation)

    def add_physics(self):
        scene = UsdPhysics.Scene.Define(self._stage, Sdf.Path("/physicsScene"))
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
        api = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        api.CreateSolverTypeAttr().Set("PGS")
        api.CreateEnableCCDAttr().Set(True)
        api.CreateEnableStabilizationAttr().Set(True)
        api.CreateEnableGPUDynamicsAttr().Set(False)

    # ------------------------------------------------------------------ OmniGraph
    def og_setup(self):
        K = og.Controller.Keys
        try:
            og.Controller.edit(
                {"graph_path": "/ROS2AckermannDrive", "evaluator_name": "execution"},
                {
                    K.CREATE_NODES: [
                        ("onPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("context", "isaacsim.ros2.bridge.ROS2Context"),
                        ("ackSubscriber", "isaacsim.ros2.bridge.ROS2SubscribeAckermannDrive"),
                        ("ackController", "isaacsim.robot.wheeled_robots.AckermannController"),
                        ("steerArticulation", "isaacsim.core.nodes.IsaacArticulationController"),
                        ("wheelArticulation", "isaacsim.core.nodes.IsaacArticulationController"),
                    ],
                    K.SET_VALUES: [
                        ("ackSubscriber.inputs:topicName", "/sim/ackermann_cmd"),
                        ("ackController.inputs:wheelBase", self._wheelBase),
                        ("ackController.inputs:trackWidth", self._trackWidth),
                        ("ackController.inputs:frontWheelRadius", self._wheelRadius),
                        ("ackController.inputs:backWheelRadius", self._wheelRadius),
                        ("ackController.inputs:maxWheelRotation", self._maxWheelRotation),
                        ("ackController.inputs:maxWheelVelocity", self._maxWheelVelocity),
                        ("ackController.inputs:maxAcceleration", self._maxAcceleration),
                        ("ackController.inputs:maxSteeringAngleVelocity", self._maxSteeringAngleVelocity),
                        ("ackController.inputs:invertSteering", bool(self._scenario.get("invert_steering", False))),
                        ("steerArticulation.inputs:targetPrim", Sdf.Path(self._basePrim)),
                        ("steerArticulation.inputs:jointNames", self._steer_joints),
                        ("wheelArticulation.inputs:targetPrim", Sdf.Path(self._basePrim)),
                        ("wheelArticulation.inputs:jointNames", self._wheel_joints),
                    ],
                    K.CONNECT: [
                        ("onPlaybackTick.outputs:tick", "ackSubscriber.inputs:execIn"),
                        ("context.outputs:context", "ackSubscriber.inputs:context"),
                        ("onPlaybackTick.outputs:tick", "ackController.inputs:execIn"),
                        ("onPlaybackTick.outputs:deltaSeconds", "ackController.inputs:dt"),
                        ("ackSubscriber.outputs:speed", "ackController.inputs:speed"),
                        ("ackSubscriber.outputs:steeringAngle", "ackController.inputs:steeringAngle"),
                        ("ackController.outputs:execOut", "steerArticulation.inputs:execIn"),
                        ("ackController.outputs:wheelAngles", "steerArticulation.inputs:positionCommand"),
                        ("ackController.outputs:execOut", "wheelArticulation.inputs:execIn"),
                        ("ackController.outputs:wheelRotationVelocity", "wheelArticulation.inputs:velocityCommand"),
                    ],
                },
            )

            og.Controller.edit(
                {"graph_path": "/ROS2Odom", "evaluator_name": "execution"},
                {
                    K.CREATE_NODES: [
                        ("onPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("context", "isaacsim.ros2.bridge.ROS2Context"),
                        ("readSimulationTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("computeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                        ("publishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                        ("rawTFTree", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                        ("TFTree", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    ],
                    K.SET_VALUES: [
                        ("computeOdometry.inputs:chassisPrim", Sdf.Path(self._basePrim)),
                        ("TFTree.inputs:parentPrim", Sdf.Path(self._basePrim)),
                        ("TFTree.inputs:targetPrims", self._sensorPrims),
                        ("publishOdometry.inputs:topicName", "/odom"),
                    ],
                    K.CONNECT: [
                        ("onPlaybackTick.outputs:tick", "computeOdometry.inputs:execIn"),
                        ("onPlaybackTick.outputs:tick", "rawTFTree.inputs:execIn"),
                        ("onPlaybackTick.outputs:tick", "TFTree.inputs:execIn"),
                        ("context.outputs:context", "publishOdometry.inputs:context"),
                        ("context.outputs:context", "rawTFTree.inputs:context"),
                        ("context.outputs:context", "TFTree.inputs:context"),
                        ("readSimulationTime.outputs:simulationTime", "rawTFTree.inputs:timeStamp"),
                        ("readSimulationTime.outputs:simulationTime", "TFTree.inputs:timeStamp"),
                        ("readSimulationTime.outputs:simulationTime", "publishOdometry.inputs:timeStamp"),
                        ("computeOdometry.outputs:execOut", "publishOdometry.inputs:execIn"),
                        ("computeOdometry.outputs:angularVelocity", "publishOdometry.inputs:angularVelocity"),
                        ("computeOdometry.outputs:linearVelocity", "publishOdometry.inputs:linearVelocity"),
                        ("computeOdometry.outputs:position", "publishOdometry.inputs:position"),
                        ("computeOdometry.outputs:orientation", "publishOdometry.inputs:orientation"),
                        ("computeOdometry.outputs:position", "rawTFTree.inputs:translation"),
                        ("computeOdometry.outputs:orientation", "rawTFTree.inputs:rotation"),
                    ],
                },
            )

            og.Controller.edit(
                {"graph_path": "/ROS2Camera", "evaluator_name": "execution"},
                {
                    K.CREATE_NODES: [
                        ("onPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("runOneSimulationFrame", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                        ("rendererProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                        ("rosContext", "isaacsim.ros2.bridge.ROS2Context"),
                        ("RGBcameraHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                        ("cameraInfoHelper", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                    ],
                    K.SET_VALUES: [
                        ("rendererProduct.inputs:cameraPrim", Sdf.Path(self._cameraPath)),
                        ("rendererProduct.inputs:width", int(self._scenario.get("camera_width", 640))),
                        ("rendererProduct.inputs:height", int(self._scenario.get("camera_height", 480))),
                        ("RGBcameraHelper.inputs:frameId", "depth_link"),
                        ("RGBcameraHelper.inputs:topicName", "/camera/color/image_raw"),
                        ("RGBcameraHelper.inputs:type", "rgb"),
                        ("cameraInfoHelper.inputs:frameId", "depth_link"),
                        ("cameraInfoHelper.inputs:topicName", "/camera/color/camera_info"),
                    ],
                    K.CONNECT: [
                        ("onPlaybackTick.outputs:tick", "runOneSimulationFrame.inputs:execIn"),
                        ("runOneSimulationFrame.outputs:step", "rendererProduct.inputs:execIn"),
                        ("rendererProduct.outputs:execOut", "RGBcameraHelper.inputs:execIn"),
                        ("rendererProduct.outputs:renderProductPath", "RGBcameraHelper.inputs:renderProductPath"),
                        ("rosContext.outputs:context", "RGBcameraHelper.inputs:context"),
                        ("rendererProduct.outputs:execOut", "cameraInfoHelper.inputs:execIn"),
                        ("rendererProduct.outputs:renderProductPath", "cameraInfoHelper.inputs:renderProductPath"),
                        ("rosContext.outputs:context", "cameraInfoHelper.inputs:context"),
                    ],
                },
            )

            og.Controller.edit(
                {"graph_path": "/ROS2Lidar", "evaluator_name": "execution"},
                {
                    K.CREATE_NODES: [
                        ("onPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("runOneSimulationFrame", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                        ("rendererProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                        ("rosContext", "isaacsim.ros2.bridge.ROS2Context"),
                        ("RtxLidarHelper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                    ],
                    K.SET_VALUES: [
                        ("rendererProduct.inputs:cameraPrim", Sdf.Path(self._lidarPath)),
                        ("RtxLidarHelper.inputs:frameId", "laser_link"),
                        ("RtxLidarHelper.inputs:topicName", "/scan"),
                        ("RtxLidarHelper.inputs:type", "laser_scan"),
                    ],
                    K.CONNECT: [
                        ("onPlaybackTick.outputs:tick", "runOneSimulationFrame.inputs:execIn"),
                        ("runOneSimulationFrame.outputs:step", "rendererProduct.inputs:execIn"),
                        ("rendererProduct.outputs:execOut", "RtxLidarHelper.inputs:execIn"),
                        ("rendererProduct.outputs:renderProductPath", "RtxLidarHelper.inputs:renderProductPath"),
                        ("rosContext.outputs:context", "RtxLidarHelper.inputs:context"),
                    ],
                },
            )

            og.Controller.edit(
                {"graph_path": "/ROS2Clock", "evaluator_name": "execution"},
                {
                    K.CREATE_NODES: [
                        ("onPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("readSimulationTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("rosContext", "isaacsim.ros2.bridge.ROS2Context"),
                        ("publishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ],
                    K.CONNECT: [
                        ("onPlaybackTick.outputs:tick", "publishClock.inputs:execIn"),
                        ("readSimulationTime.outputs:simulationTime", "publishClock.inputs:timeStamp"),
                        ("rosContext.outputs:context", "publishClock.inputs:context"),
                    ],
                },
            )
        except Exception as e:
            carb.log_error(f"OmniGraph 구성 실패: {e}")

    # ------------------------------------------------------------------ BaseSample
    def setup_scene(self):
        self._world = World.instance()
        self._stage = get_current_stage()
        self.add_physics()
        self.add_light()
        self.add_track()
        self.add_robot()
        self.add_objects()
        self.og_setup()

    async def setup_post_load(self):
        return

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        return

    def world_cleanup(self):
        return
