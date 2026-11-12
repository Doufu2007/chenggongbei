"""Bounded ground commands are gated by real matching and fresh observations."""
import importlib
import importlib.util
import math
import sys
from types import ModuleType, SimpleNamespace

import pytest


def api():
    name = "inspection_mission.bounded_ground_probe"
    assert importlib.util.find_spec(name) is not None, "bounded ground probe is missing"
    return importlib.import_module(name)


def odom(stamp=10., *, x=0., speed=0., child="base_footprint"):
    sec = int(stamp)
    return {"header": {"stamp": {"sec": sec, "nanosec": round((stamp-sec)*1e9)}, "frame_id": "odom"},
            "child_frame_id": child,
            "pose": {"pose": {"position": {"x": x, "y": 0., "z": .055},
                              "orientation": {"x": 0., "y": 0., "z": 0., "w": 1.}}},
            "twist": {"twist": {"linear": {"x": speed, "y": 0., "z": 0.},
                                "angular": {"x": 0., "y": 0., "z": 0.}}}}


def scan(stamp=10., ranges=None):
    sec = int(stamp)
    return {"header": {"stamp": {"sec": sec, "nanosec": round((stamp-sec)*1e9)}, "frame_id": "laser_link"},
            "angle_min": -math.pi, "angle_max": math.pi,
            "angle_increment": 2*math.pi/359, "range_min": .1, "range_max": 12.,
            "ranges": [2.] * 360 if ranges is None else ranges}


def endpoint(kind="geometry_msgs/msg/Twist", node="ican_inspection_public_bridge_197"):
    return {"node": "/" + node, "type": kind, "reliability": "RELIABLE", "durability": "VOLATILE"}


def guard():
    g = api()
    # Explicit experimental bounds from the public candidate model, not verified platform dimensions.
    return g.GroundMotionGuard(g.MotionLimits(.447, .114, .02), started_wall_s=0.)


def ready(state):
    state.update_endpoints([endpoint()], matched_subscriptions=1)
    for index in range(12):
        wall = index / 10
        stamp = 10. + wall
        state.update_clock(stamp, wall_s=wall)
        state.update_odom(odom(stamp), wall_s=wall)
        state.update_scan(scan(stamp), wall_s=wall)
        decision = state.step(wall_s=wall)
    return decision


def refresh(state, wall, *, x=0.):
    stamp = 10. + wall
    state.update_clock(stamp, wall_s=wall)
    state.update_odom(odom(stamp, x=x), wall_s=wall)
    state.update_scan(scan(stamp), wall_s=wall)


def test_stable_fresh_matched_ground_observations_allow_only_small_forward_speed():
    state = guard()
    decision = ready(state)
    assert decision.linear_x == .03 and decision.state == "PULSE"
    assert state.first_odom["child_frame_id"] == "base_footprint"
    assert state.last_odom["pose"]["pose"]["orientation"] == {"x": 0., "y": 0., "z": 0., "w": 1.}


@pytest.mark.parametrize("kind", ["absent", "wrong_type", "only_self", "not_matched"])
def test_no_actual_external_matching_subscriber_never_moves(kind):
    state = guard()
    ready(state)
    endpoints, matches = [endpoint()], 1
    if kind == "absent": endpoints = []
    elif kind == "wrong_type": endpoints = [endpoint("std_msgs/msg/String")]
    elif kind == "only_self": endpoints = [endpoint(node="inspection_bounded_ground_probe")]
    else: matches = 0
    state.update_endpoints(endpoints, matched_subscriptions=matches)
    assert state.step(wall_s=1.11).linear_x == 0.


@pytest.mark.parametrize("kind", ["odom_wall_old", "odom_sim_old", "scan_wall_old", "scan_sim_old", "clock_old"])
def test_stale_wall_or_sim_sensor_data_stops_an_active_pulse(kind):
    state = guard(); ready(state)
    if kind == "odom_wall_old":
        state.update_clock(11.15, wall_s=1.4); state.update_scan(scan(11.15), wall_s=1.4)
    elif kind == "odom_sim_old":
        state.update_clock(11.5, wall_s=1.15); state.update_scan(scan(11.5), wall_s=1.15)
        state.update_odom(odom(11.1), wall_s=1.15)
    elif kind == "scan_wall_old":
        state.update_clock(11.15, wall_s=1.4); state.update_odom(odom(11.15), wall_s=1.4)
    elif kind == "scan_sim_old":
        state.update_clock(11.5, wall_s=1.15); state.update_odom(odom(11.5), wall_s=1.15)
        state.update_scan(scan(11.1), wall_s=1.15)
    assert state.step(wall_s=1.4 if kind.endswith("wall_old") or kind == "clock_old" else 1.15).linear_x == 0.


@pytest.mark.parametrize("kind", ["narrow", "too_close", "nan", "zero", "negative_inf", "inf_short_range", "wrong_frame"])
def test_incomplete_or_unsafe_ring_scan_cannot_allow_ground_motion(kind):
    state = guard(); ready(state)
    message = scan(11.1)
    if kind == "narrow": message["ranges"] = [2.] * 30; message["angle_max"] = message["angle_min"] + 29*message["angle_increment"]
    elif kind == "too_close": message["ranges"][180] = .6
    elif kind == "nan": message["ranges"][1] = float("nan")
    elif kind == "zero": message["ranges"][1] = 0.
    elif kind == "negative_inf": message["ranges"][1] = -math.inf
    elif kind == "inf_short_range": message["ranges"] = [math.inf]*360; message["range_max"] = .6
    else: message["header"]["frame_id"] = "unknown"
    state.update_scan(message, wall_s=1.1)
    decision = state.step(wall_s=1.11)
    assert decision.linear_x == 0. and decision.reason


def test_positive_infinity_only_means_clear_to_declared_sufficient_range():
    state = guard(); ready(state)
    state.update_scan(scan(11.1, [math.inf]*360), wall_s=1.1)
    assert state.step(wall_s=1.11).linear_x == .03


def test_displacement_limit_latches_stop_and_cannot_restart():
    state = guard(); ready(state)
    refresh(state, 1.2, x=.06)
    decision = state.step(wall_s=1.2)
    assert decision.linear_x == 0. and decision.reason == "displacement_limit"
    refresh(state, 1.3, x=0.)
    assert state.step(wall_s=1.3).linear_x == 0.


@pytest.mark.parametrize("kind", ["clock", "wall", "odom"])
def test_time_reversal_latches_zero(kind):
    state = guard(); ready(state)
    if kind == "clock": state.update_clock(9., wall_s=1.15)
    elif kind == "odom": state.update_odom(odom(9.), wall_s=1.15)
    decision = state.step(wall_s=.5 if kind == "wall" else 1.15)
    assert decision.linear_x == 0. and "reversed" in decision.reason


def test_action_wall_timeout_stops_even_while_simulation_pauses_and_repeats_zero():
    state = guard(); ready(state)
    refresh(state, 2.1)
    decision = state.step(wall_s=2.1)
    assert decision.linear_x == 0. and decision.reason == "pulse_time_limit"
    assert all(state.step(wall_s=t).linear_x == 0. for t in (2.15, 2.2, 2.3))


def test_wrong_body_or_moving_start_cannot_establish_stability():
    state = guard()
    state.update_endpoints([endpoint()], matched_subscriptions=1)
    for index in range(12):
        t=index/10
        state.update_clock(10+t, wall_s=t)
        state.update_odom(odom(10+t, speed=.02), wall_s=t)
        state.update_scan(scan(10+t), wall_s=t)
        assert state.step(wall_s=t).linear_x == 0.
    state.update_odom(odom(11.1, child="uav/base_link"), wall_s=1.1)
    assert state.step(wall_s=1.11).linear_x == 0.


def test_missing_or_excessive_experimental_limits_are_rejected():
    g = api()
    for changes in ({"robot_radius_m": None}, {"speed_mps": .031}, {"pulse_wall_s": 1.6}, {"max_displacement_m": .061}):
        with pytest.raises(ValueError):
            g.MotionLimits(**{"robot_radius_m": .447, "sensor_offset_bound_m": .114, "clearance_m": .02, **changes})


def test_sparse_full_circle_does_not_count_as_effective_ring_coverage():
    state = guard(); ready(state)
    message = scan(11.1, [2.]*12)
    message["angle_increment"] = 2*math.pi/11
    state.update_scan(message, wall_s=1.1)
    assert state.step(wall_s=1.11).linear_x == 0.


def install_ros_transport(monkeypatch, *, graph=()):
    callbacks, published, publishers = {}, [], []

    class Node:
        def __init__(self, name): self.name = name
        @property
        def subscriptions(self): return iter(())
        def get_topic_names_and_types(self): return list(graph)
        def create_publisher(self, cls, name, qos):
            publishers.append(name)
            return SimpleNamespace(publish=lambda msg: published.append((name, msg.linear.x, msg.angular.z)),
                                   get_subscription_count=lambda: 1)
        def create_subscription(self, cls, name, cb, qos): callbacks[name] = cb; return cb
        def get_subscriptions_info_by_topic(self, name):
            return [SimpleNamespace(node_name="ican_inspection_public_bridge_197", node_namespace="/",
                                    topic_type="geometry_msgs/msg/Twist",
                                    qos_profile=SimpleNamespace(reliability="RELIABLE", durability="VOLATILE"))]
        def get_publishers_info_by_topic(self, name): return []
        def get_logger(self): return SimpleNamespace(info=lambda _: None, warning=lambda _: None)
    class Twist:
        def __init__(self): self.linear = SimpleNamespace(x=0.,y=0.,z=0.); self.angular = SimpleNamespace(x=0.,y=0.,z=0.)
    modules = {name: ModuleType(name) for name in ("rclpy", "rclpy.node", "rclpy.qos", "rosidl_runtime_py",
               "rosidl_runtime_py.convert", "rosidl_runtime_py.utilities", "geometry_msgs", "geometry_msgs.msg")}
    modules["rclpy.node"].Node = Node
    modules["rclpy.qos"].QoSProfile = lambda **kwargs: SimpleNamespace(**kwargs)
    modules["rclpy.qos"].ReliabilityPolicy = SimpleNamespace(RELIABLE="RELIABLE", BEST_EFFORT="BEST_EFFORT")
    modules["rclpy.qos"].DurabilityPolicy = SimpleNamespace(VOLATILE="VOLATILE", TRANSIENT_LOCAL="TRANSIENT_LOCAL")
    modules["rosidl_runtime_py.convert"].message_to_ordereddict = lambda msg: msg
    modules["rosidl_runtime_py.utilities"].get_message = lambda name: type("Message", (), {})
    modules["geometry_msgs.msg"].Twist = Twist
    for name,value in modules.items(): monkeypatch.setitem(sys.modules, name, value)
    return callbacks, published, publishers


def test_ros_adapter_only_publishes_cmd_vel_and_records_all_distinct_tf_edges(monkeypatch):
    g = api()
    callbacks, published, publishers = install_ros_transport(monkeypatch)
    assert hasattr(g, "create_bounded_node"), "ground-only ROS adapter is missing"
    wall = [0.]
    node = g.create_bounded_node(g.MotionLimits(.447,.114,.02), wall_clock=lambda:wall[0])
    assert set(publishers) == {"/cmd_vel","/uav/cmd_vel","/referee/current_target","/inspection/detections",
                               "/referee/cruise_event","/uav/laser_fire"}
    for index in range(12):
        wall[0] = index/10
        stamp = 10.+wall[0]; sec=int(stamp)
        callbacks["/clock"]({"clock":{"sec":sec,"nanosec":round((stamp-sec)*1e9)}})
        callbacks["/odom"](odom(stamp))
        callbacks["/scan"](scan(stamp))
        node.tick()
    assert any(linear == .03 for _,linear,_ in published)
    assert all(topic == "/cmd_vel" for topic,_,_ in published)
    for child in ("camera_link", "laser_link"):
        callbacks["/tf_static"]({"transforms":[{"header":{"stamp":{"sec":0,"nanosec":0},"frame_id":"base_footprint"},
                                              "child_frame_id":child,
                                              "transform":{"translation":{"x":0.,"y":0.,"z":0.},
                                                           "rotation":{"x":0.,"y":0.,"z":0.,"w":1.}}}]})
    snapshot = node.evidence_snapshot()
    assert snapshot["read_only"] is False and snapshot["mode"] == "bounded_ground_motion"
    assert {item["child_frame_id"] for item in snapshot["tf_edges"].values()} == {"camera_link","laser_link"}
    assert snapshot["odom_first"]["child_frame_id"] == snapshot["odom_last"]["child_frame_id"] == "base_footprint"
    node.stop("test_finished")
    node.tick()
    assert published[-1] == ("/cmd_vel",0.,0.)
    node.close_watchdog()


def test_watchdog_stops_with_repeated_zeros_when_ros_spin_is_not_running(monkeypatch):
    import time
    g=api(); _,published,_ = install_ros_transport(monkeypatch)
    assert hasattr(g,"create_bounded_node"), "ground-only ROS adapter is missing"
    node = g.create_bounded_node(g.MotionLimits(.447,.114,.02,pulse_wall_s=.05), wall_clock=time.monotonic)
    start = time.monotonic()
    node.arm_watchdog()
    while time.monotonic()-start < .15 and len(published)<2:
        time.sleep(.005)
    assert len(published) >= 2 and all(linear == 0. for _,linear,_ in published)
    node.close_watchdog()


def test_endpoint_history_preserves_initial_and_intermediate_match_with_bounded_storage(monkeypatch):
    g=api(); install_ros_transport(monkeypatch)
    wall=[0.]
    node=g.create_bounded_node(g.MotionLimits(.447,.114,.02), wall_clock=lambda:wall[0])
    assert len(node.evidence_snapshot()["command_endpoint_history"]) == 1
    # Match counts vary, ending disconnected; retain initial and recent changes.
    for count in range(1, 20):
        wall[0]=float(count)
        for publisher in node.publisher_by_topic.values():
            publisher.get_subscription_count=lambda count=count:count
        node.refresh_endpoints()
    for publisher in node.publisher_by_topic.values():
        publisher.get_subscription_count=lambda:0
    node.refresh_endpoints()
    snapshot=node.evidence_snapshot()
    history=snapshot["command_endpoint_history"]
    assert len(history) == 16
    assert history[0]["wall_elapsed_s"] == 0.
    assert history[0]["endpoints"]["/cmd_vel"]["matched_subscriptions"] == 1
    assert any(item["endpoints"]["/cmd_vel"]["matched_subscriptions"] > 1 for item in history)
    assert history[-1]["endpoints"]["/cmd_vel"]["matched_subscriptions"] == 0
    assert snapshot["command_endpoint_history_dropped"] > 0
    node.close_watchdog()


def test_started_event_and_final_snapshot_identify_bounded_active_experiment(monkeypatch):
    import json
    g=api(); install_ros_transport(monkeypatch)
    events=[]
    class Reporter:
        def emit(self, event, payload): events.append((event,payload)); return True
        def send_now(self, event, payload, timeout_s): events.append((event,payload)); return True
    node=g.create_bounded_node(g.MotionLimits(.447,.114,.02), Reporter(), wall_clock=lambda:0.)
    node.report_started(45.)
    node.report_final()
    started=[payload for event,payload in events if event=="probe_started"]
    assert len(started)==1
    assert started[0]["read_only"] is False and started[0]["mode"]=="bounded_ground_motion"
    assert started[0]["duration_s"]==45.
    final=[payload for event,payload in events if event=="probe_snapshot"]
    assert final[0]["startup"] == started[0]
    assert final[0]["section"]=="ground_motion_core"
    json.dumps(final[0], allow_nan=False)
    node.close_watchdog()


def test_watchdog_deadline_is_measured_from_actual_first_pulse_not_late_arming(monkeypatch):
    import time
    g=api(); _,published,_=install_ros_transport(monkeypatch)
    node=g.create_bounded_node(g.MotionLimits(.447,.114,.02,pulse_wall_s=.05), wall_clock=time.monotonic)
    node.guard.pulse_started=time.monotonic()-.1
    node.arm_watchdog()
    start=time.monotonic()
    while time.monotonic()-start < .03 and not published:
        time.sleep(.001)
    assert published and all(linear==0. for _,linear,_ in published)
    node.close_watchdog()


def test_post_stop_odom_drift_is_retained_in_max_observed_displacement():
    state=guard(); ready(state)
    state.stop("test_stop", 1.15)
    refresh(state,1.2,x=.04)
    assert state.step(wall_s=1.2).linear_x==0.
    assert state.max_observed_displacement_m==pytest.approx(.04)


def test_start_stability_cannot_wait_past_wall_budget():
    state=guard()
    state.update_endpoints([endpoint()],matched_subscriptions=1)
    for wall in (0.,20.1):
        state.update_clock(10.+wall,wall_s=wall)
        state.update_odom(odom(10.+wall,speed=.02),wall_s=wall)
        state.update_scan(scan(10.+wall),wall_s=wall)
        decision=state.step(wall_s=wall)
    assert decision.state=="STOP" and decision.reason.startswith("wait_timeout:")


def test_ros_adapter_skips_conflicting_output_types_instead_of_adding_a_new_type(monkeypatch):
    g=api()
    _,published,publishers=install_ros_transport(monkeypatch,graph=(
        ("/cmd_vel",["std_msgs/msg/String"]),("/uav/cmd_vel",["std_msgs/msg/String"])))
    node=g.create_bounded_node(g.MotionLimits(.447,.114,.02),wall_clock=lambda:0.)
    assert "/cmd_vel" not in publishers and "/uav/cmd_vel" not in publishers
    assert "output_type_conflict:/cmd_vel" in node.errors
    assert "output_type_conflict:/uav/cmd_vel" in node.errors
    decision=node.tick()
    assert decision.linear_x==0. and decision.reason=="no_external_matching_cmd_subscriber"
    assert published==[]
    node.close_watchdog()


def test_final_snapshot_and_first_last_real_sensor_images_fit_receiver_contract(monkeypatch):
    import base64
    import json
    from array import array
    from inspection_mission.telemetry import DiagnosticReporter
    cv2=pytest.importorskip("cv2"); np=pytest.importorskip("numpy")
    g=api(); callbacks,published,_=install_ros_transport(monkeypatch)
    events=[]
    class Reporter:
        def emit(self,event,payload):
            events.append(json.loads(DiagnosticReporter._encode(event,payload))); return True
        def send_now(self,event,payload,timeout_s):
            events.append(json.loads(DiagnosticReporter._encode(event,payload))); return True
    node=g.create_bounded_node(g.MotionLimits(.447,.114,.02),Reporter(),wall_clock=lambda:0.)
    node.report_started(45.)
    # Different true pixel buffers and stamps verify that end frames are not a
    # repeated cached start JPEG. No ROS Image serialization stub is involved.
    for topic in ("/camera/image","/uav/camera/image"):
        for stamp,color in ((1,(255,0,0)),(2,(0,255,0))):
            msg=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp,nanosec=0),
                frame_id="camera_link" if topic=="/camera/image" else "uav/camera_link"),
                width=64,height=32,step=64*3,encoding="rgb8",data=array("B",bytes(color)*64*32))
            callbacks[topic](msg)
    callbacks["/camera/camera_info"]({"header":{"frame_id":"camera_link"},"k":[476.7,0.,400.5,0.,476.7,400.5,0.,0.,1.]})
    callbacks["/referee/ground_tasks"]({"data":"ground_1,ground_4,ground_0"})
    callbacks["/referee/recognition_status"]({"data":"runtime-message"})
    node.report_final()
    snapshots=[event["data"] for event in events if event["event"]=="probe_snapshot"]
    assert len(snapshots)==2 and snapshots[0]["section"]=="ground_motion_core"
    assert snapshots[0]["read_only"] is False and snapshots[0]["startup"]["duration_s"]==45.
    assert snapshots[1]["public_messages"]["/camera/camera_info"]["last"]["k"][8]==1.
    assert len(snapshots[1]["image_records"])==4
    images=[event["data"] for event in events if event["event"]=="image_sample"]
    assert len(images)==4
    for topic in ("/camera/image","/uav/camera/image"):
        records=[item for item in images if item["topic"]==topic]
        assert [item["stamp"] for item in records]==[1.,2.]
        decoded=[cv2.imdecode(np.frombuffer(base64.b64decode(item["jpeg_base64"]),np.uint8),cv2.IMREAD_COLOR)
                 for item in records]
        assert decoded[0][:,:,2].mean()>245 and decoded[1][:,:,1].mean()>245
    assert all(topic=="/cmd_vel" for topic,_,_ in published)
    node.close_watchdog()


def test_flooding_endpoint_and_motion_changes_have_explicit_bounded_evidence(monkeypatch):
    g=api(); install_ros_transport(monkeypatch)
    wall=[0.]
    node=g.create_bounded_node(g.MotionLimits(.447,.114,.02),wall_clock=lambda:wall[0])
    node.get_subscriptions_info_by_topic=lambda topic:[SimpleNamespace(node_name="bridge_"+str(i),
        node_namespace="/",topic_type="geometry_msgs/msg/Twist",
        qos_profile=SimpleNamespace(reliability="RELIABLE",durability="VOLATILE")) for i in range(8)]
    for i in range(100):
        wall[0]=i/100
        node.publisher_by_topic["/cmd_vel"].get_subscription_count=lambda i=i:i%2
        node.tick()
    snapshot=node.evidence_snapshot()
    assert len(snapshot["motion_log"])==64 and snapshot["motion_log_dropped"]==36
    assert snapshot["motion_log"][0]["wall_elapsed_s"]==0.
    for value in snapshot["command_endpoints"].values():
        assert len(value["subscriber_endpoints"])==4 and value["subscriber_endpoints_omitted"]==4
    node.close_watchdog()


def test_oversize_final_evidence_cannot_suppress_motion_core_or_other_sensor_images(monkeypatch):
    import json
    from array import array
    from inspection_mission.telemetry import DiagnosticReporter
    pytest.importorskip("cv2"); pytest.importorskip("numpy")
    g=api(); callbacks,_,_=install_ros_transport(monkeypatch)
    events=[]
    class Reporter:
        def emit(self,event,payload): return True
        def send_now(self,event,payload,timeout_s):
            events.append(json.loads(DiagnosticReporter._encode(event,payload))); return True
    node=g.create_bounded_node(g.MotionLimits(.447,.114,.02),Reporter(),wall_clock=lambda:0.)
    node.report_started(45.)
    # Exceed the envelope using valid public sensor evidence rather than a
    # fabricated score; every emitted fragment must pass the real encoder.
    for i in range(128):
        callbacks["/tf_static"]({"transforms":[{"header":{"stamp":{"sec":0,"nanosec":0},"frame_id":"base_footprint"},
            "child_frame_id":"observed_sensor_"+str(i),"transform":{"translation":{"x":0.,"y":0.,"z":0.},
            "rotation":{"x":0.,"y":0.,"z":0.,"w":1.}}}]})
    node.public_messages["/scan"]={"first":scan(ranges=[2.123456789]*8192),"last":scan(ranges=[2.123456789]*8192)}
    callbacks["/camera/image"](SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=0),
        frame_id="camera_link"),width=8,height=8,step=24,encoding="rgb8",data=array("B",bytes((255,0,0))*64)))
    node.report_final()
    assert events[0]["event"]=="probe_snapshot" and events[0]["data"]["section"]=="ground_motion_core"
    assert any(item["event"]=="image_sample" for item in events)
    parts=[item["data"] for item in events if item["event"]=="probe_snapshot"]
    assert any(part.get("section")=="tf_edges" for part in parts)
    assert not any(part.get("section")=="ground_motion_full" for part in parts)
    assert any(part.get("section")=="evidence_errors" for part in parts)
    node.close_watchdog()


def test_v5_sized_sensor_tf_and_endpoint_history_snapshot_fits_actual_encoder(monkeypatch):
    import copy
    from inspection_mission.telemetry import DiagnosticReporter
    g=api(); callbacks,_,_=install_ros_transport(monkeypatch)
    wall=[0.]
    node=g.create_bounded_node(g.MotionLimits(.447,.114,.02),wall_clock=lambda:wall[0])
    node.report_started(45.)
    for t in (10.,11.):
        wall[0]=t-10
        for topic in ("/odom","/uav/odom"):
            payload=odom(t)
            if topic=="/uav/odom": payload["child_frame_id"]="uav/base_link"
            payload["pose"]["covariance"]=[0.]*36
            payload["twist"]["covariance"]=[0.]*36
            callbacks[topic](payload)
        callbacks["/scan"](scan(t))
        callbacks["/clock"]({"clock":{"sec":int(t),"nanosec":0}})
        for topic in ("/camera/camera_info","/uav/camera/camera_info"):
            callbacks[topic]({"header":{"frame_id":"camera_link","stamp":{"sec":int(t),"nanosec":0}},
                "width":800,"height":800,"k":[476.7,0.,400.5,0.,476.7,400.5,0.,0.,1.],
                "r":[1.,0.,0.,0.,1.,0.,0.,0.,1.],"p":[476.7,0.,400.5,0.,0.,476.7,400.5,0.,0.,0.,1.,0.],
                "d":[0.]*5,"distortion_model":"plumb_bob"})
    for index in range(3):
        callbacks["/tf_static"]({"transforms":[{"header":{"frame_id":"base_footprint","stamp":{"sec":0,"nanosec":0}},
            "child_frame_id":"sensor_"+str(index),"transform":{"translation":{"x":0.,"y":0.,"z":.07},
            "rotation":{"x":0.,"y":0.,"z":0.,"w":1.}}}]})
    node.command_endpoint_history=[copy.deepcopy(node.command_endpoint_history[0]) for _ in range(16)]
    snapshot=node.evidence_snapshot()
    encoded=DiagnosticReporter._encode("probe_snapshot",{**snapshot,"section":"ground_motion_full","partial":False})
    assert 20000<len(encoded)<65536
    node.close_watchdog()


def multirate_samples(state, until_wall, *, clock_enabled=True):
    decisions=[]
    # Public v6 rates: /clock 10 Hz, ground odometry 50 Hz, laser 8 Hz.
    # Arrivals are wall-fresh; new odometry can precede the next clock sample.
    for index in range(round(until_wall*1000)+1):
        wall=index/1000
        if clock_enabled and index%100==0:
            state.update_clock(10.+wall,wall_s=wall)
        if index%20==0:
            state.update_odom(odom(10.+wall),wall_s=wall)
        if index%125==0:
            state.update_scan(scan(10.+wall),wall_s=wall)
        if index%20==0:
            decisions.append(state.step(wall_s=wall))
    return decisions


def test_normal_10hz_clock_50hz_odom_8hz_scan_can_hold_start_stability_for_a_second():
    g=api()
    legacy=g.GroundMotionGuard(g.MotionLimits(.447,.114,.02,clock_sampling_lag_s=.05),started_wall_s=0.)
    legacy.update_endpoints([endpoint()],matched_subscriptions=1)
    assert not any(item.state=="PULSE" for item in multirate_samples(legacy,1.2))
    state=guard()
    state.update_endpoints([endpoint()],matched_subscriptions=1)
    decisions=multirate_samples(state,1.2)
    assert any(item.state=="PULSE" and item.linear_x==.03 for item in decisions)
    assert state.stable_anchor is not None
    assert state.limits.sensor_age_s==.2


@pytest.mark.parametrize("topic",["odom","scan"])
def test_sensor_stamp_beyond_explicit_clock_sampling_lag_bound_stops(topic):
    state=guard(); ready(state)
    stamp=state.clock[0]+.121
    getattr(state,"update_"+topic)((odom if topic=="odom" else scan)(stamp),wall_s=1.11)
    assert state.step(wall_s=1.11).linear_x==0.
    assert state.state=="STOP"


def test_normal_clock_sampling_lag_does_not_allow_a_stopped_clock_to_keep_motion():
    state=guard(); ready(state)
    # Same epoch sensor headers, arriving recently, cannot replace /clock.
    state.update_odom(odom(11.1),wall_s=1.31)
    state.update_scan(scan(11.1),wall_s=1.31)
    decision=state.step(wall_s=1.31)
    assert decision.linear_x==0. and "stale" in decision.reason


def test_repeated_frozen_clock_messages_do_not_admit_new_sensor_epochs():
    state=guard(); ready(state)
    for wall in (1.15,1.2,1.25):
        # Fresh wall delivery of an unadvanced clock still cannot make newer
        # sensor timestamps part of its bounded epoch forever.
        state.update_clock(11.1,wall_s=wall)
        state.update_odom(odom(10.+wall),wall_s=wall)
        state.update_scan(scan(10.+wall),wall_s=wall)
        decision=state.step(wall_s=wall)
    assert decision.linear_x==0. and decision.state=="STOP"
    assert decision.reason=="odom_stale_or_invalid"


@pytest.mark.parametrize("topic",["odom","scan"])
def test_sensor_past_age_stays_at_point_two_after_future_phase_fix(topic):
    state=guard(); ready(state)
    old_stamp=state.clock[0]-.201
    # A same-lifecycle late sample must not look fresh solely because its
    # delivery time is recent; odom reversal also fails shut as before.
    getattr(state,"update_"+topic)((odom if topic=="odom" else scan)(old_stamp),wall_s=1.11)
    assert state.step(wall_s=1.11).linear_x==0.


def test_clock_sampling_lag_limit_is_explicit_bounded_and_not_a_larger_sensor_age():
    g=api()
    limits=g.MotionLimits(.447,.114,.02)
    assert limits.clock_sampling_lag_s==.12 and limits.sensor_age_s==.2
    with pytest.raises(ValueError):
        g.MotionLimits(.447,.114,.02,clock_sampling_lag_s=.121)


def test_condition_changes_store_actual_stamps_wall_ages_and_clock_delta(monkeypatch):
    g=api(); callbacks,_,_=install_ros_transport(monkeypatch)
    wall=[0.]
    node=g.create_bounded_node(g.MotionLimits(.447,.114,.02),wall_clock=lambda:wall[0])
    callbacks["/clock"]({"clock":{"sec":10,"nanosec":0}})
    wall[0]=.08
    callbacks["/odom"](odom(10.08))
    callbacks["/scan"](scan(10.025))
    node.tick()
    record=node.evidence_snapshot()["motion_log"][-1]
    assert record["reason"]=="start_stability_dwell"
    timing=record["timing"]
    assert timing["clock"]["stamp_s"]==10.
    assert timing["clock"]["wall_age_s"]==pytest.approx(.08)
    assert timing["odom"]["stamp_s"]==pytest.approx(10.08)
    assert timing["odom"]["wall_age_s"]==0.
    assert timing["odom"]["clock_minus_stamp_s"]==pytest.approx(-.08)
    assert timing["scan"]["stamp_s"]==pytest.approx(10.025)
    node.close_watchdog()


def test_control_loop_receives_fast_callbacks_but_commands_at_most_20hz():
    g=api()
    assert hasattr(g,"spin_control_loop"),"independent wall command cadence is missing"
    wall=[0.]; ticks=[]; spins=[]
    node=SimpleNamespace(tick=lambda:(ticks.append(wall[0]) or SimpleNamespace(state="WAIT")))
    class Ros:
        @staticmethod
        def ok(): return True
        @staticmethod
        def spin_once(node,timeout_sec):
            spins.append(timeout_sec)
            wall[0]+=.002  # 500 sensor callbacks/second, without blocking.
    g.spin_control_loop(node,Ros(),duration_s=1.,wall_clock=lambda:wall[0])
    assert len(spins)>=450
    assert 19<=len(ticks)<=21
    assert min(b-a for a,b in zip(ticks,ticks[1:]))>=.05-1e-9


def test_watchdog_still_publishes_zero_while_endpoint_graph_refresh_blocks(monkeypatch):
    import threading
    import time
    g=api(); _,published,_=install_ros_transport(monkeypatch)
    node=g.create_bounded_node(g.MotionLimits(.447,.114,.02,pulse_wall_s=.05),wall_clock=time.monotonic)
    entered,release=threading.Event(),threading.Event()
    original=node.refresh_endpoints
    def blocked_graph_query():
        entered.set()
        release.wait(.5)
        original()
    node.refresh_endpoints=blocked_graph_query
    node.guard.pulse_started=time.monotonic()
    node.guard.state="PULSE"
    node.arm_watchdog()
    worker=threading.Thread(target=node.tick,daemon=True)
    worker.start()
    observed_zero=False
    try:
        assert entered.wait(.15)
        deadline=time.monotonic()+.13
        while time.monotonic()<deadline and not published:
            time.sleep(.002)
        observed_zero=any(topic=="/cmd_vel" and linear==0. for topic,linear,_ in published)
    finally:
        release.set()
        worker.join(timeout=.5)
        node.close_watchdog()
    assert observed_zero,"ROS graph queries must not hold the watchdog's publish lock"
    assert all(linear==0. for _,linear,_ in published)
    assert node.guard.state in ("STOP","FINISHED")


def test_graph_refresh_exception_immediately_latches_stop_and_publishes_zero(monkeypatch):
    g=api(); _,published,_=install_ros_transport(monkeypatch)
    node=g.create_bounded_node(g.MotionLimits(.447,.114,.02),wall_clock=lambda:0.)
    def graph_failure(): raise RuntimeError("endpoint graph query failed")
    node.refresh_endpoints=graph_failure
    decision=node.tick()
    assert decision.state=="STOP" and decision.linear_x==0.
    assert decision.reason.startswith("endpoint_refresh_error:")
    assert published[-1]==("/cmd_vel",0.,0.)
    node.refresh_endpoints=lambda:None
    assert node.tick().linear_x==0.
    node.close_watchdog()


def test_slow_command_tick_does_not_cause_an_immediate_followup_command():
    g=api()
    wall=[0.]; outputs=[]
    class Ros:
        @staticmethod
        def ok(): return True
        @staticmethod
        def spin_once(node,timeout_sec): wall[0]+=.001
    class Node:
        def tick(self):
            if not outputs: wall[0]+=.1
            outputs.append(wall[0])
            return SimpleNamespace(state="WAIT")
    g.spin_control_loop(Node(),Ros(),duration_s=.18,wall_clock=lambda:wall[0])
    assert len(outputs)==2
    assert min(b-a for a,b in zip(outputs,outputs[1:]))>=.05-1e-9


@pytest.mark.parametrize("topic",["odom","scan"])
def test_active_invalid_sensor_cannot_be_overwritten_before_next_decision(topic):
    state=guard();ready(state)
    broken=(odom if topic=="odom" else scan)(11.11)
    if topic=="odom":broken["pose"]["pose"]["orientation"]["w"]=0.
    else:broken["ranges"][0]=float("nan")
    getattr(state,"update_"+topic)(broken,wall_s=1.11)
    assert state.state=="STOP","active invalid input must latch before any subsequent callback"
    original_reason=state.reason
    refresh(state,1.12)
    decision=state.step(wall_s=1.12)
    assert decision.linear_x==decision.angular_z==0. and decision.reason==original_reason


@pytest.mark.parametrize("topic",["odom","scan"])
def test_wait_invalid_sensor_can_recover_but_still_requires_full_stable_dwell(topic):
    state=guard()
    broken=(odom if topic=="odom" else scan)(10.)
    if topic=="odom":broken["child_frame_id"]="unknown_body"
    else:broken["ranges"][0]=0.
    getattr(state,"update_"+topic)(broken,wall_s=0.)
    assert state.state=="WAIT"
    assert ready(state).linear_x==.03


@pytest.mark.parametrize("fault",["speed","yaw_speed","displacement","tilt"])
def test_each_active_odometry_callback_latches_transient_physical_violation(fault):
    state=guard();ready(state)
    payload=odom(11.11,x=.061 if fault=="displacement" else 0.,speed=.101 if fault=="speed" else 0.)
    if fault=="yaw_speed":payload["twist"]["twist"]["angular"]["z"]=.101
    if fault=="tilt":payload["pose"]["pose"]["orientation"]={"x":math.sin(.031/2),"y":0.,"z":0.,"w":math.cos(.031/2)}
    state.update_odom(payload,wall_s=1.11)
    assert state.state=="STOP"
    reason=state.reason
    refresh(state,1.12)
    assert state.step(wall_s=1.12).linear_x==0. and state.reason==reason


@pytest.mark.parametrize("topic",["clock","odom","scan"])
@pytest.mark.parametrize("fault",["arrival_reversed","arrival_nonfinite","arrival_gap"])
def test_callback_time_fault_cannot_be_hidden_by_following_fresh_sample(topic,fault):
    state=guard();ready(state)
    wall=.9 if fault=="arrival_reversed" else (float("nan") if fault=="arrival_nonfinite" else 1.31)
    stamp=11.11 if fault!="arrival_gap" else 11.31
    if topic=="clock":state.update_clock(stamp,wall_s=wall)
    else:getattr(state,"update_"+topic)((odom if topic=="odom" else scan)(stamp),wall_s=wall)
    assert state.state=="STOP"
    reason=state.reason
    refresh(state,1.32)
    assert state.step(wall_s=1.32).linear_x==0. and state.reason==reason
    assert finite_stop_time(state.stopped_wall)


def finite_stop_time(value):return isinstance(value,(float,int)) and math.isfinite(value)


@pytest.mark.parametrize("topic",["odom","scan"])
@pytest.mark.parametrize("offset",[-.201,.121])
def test_active_sensor_epoch_violation_latches_at_callback(topic,offset):
    state=guard();ready(state)
    message=(odom if topic=="odom" else scan)(state.clock[0]+offset)
    getattr(state,"update_"+topic)(message,wall_s=1.11)
    assert state.state=="STOP"
    refresh(state,1.12)
    assert state.step(wall_s=1.12).linear_x==0.


def test_scan_stamp_reversal_latches_even_if_following_scan_is_valid():
    state=guard();ready(state)
    state.update_scan(scan(11.09),wall_s=1.11)
    assert state.state=="STOP" and "reversed" in state.reason
    refresh(state,1.12)
    assert state.step(wall_s=1.12).linear_x==0.


def test_repeated_frozen_clock_and_sensor_stamps_cannot_refresh_motion_epoch_forever():
    state=guard();ready(state)
    for wall in (1.15,1.2,1.25,1.3,1.35):
        state.update_clock(11.1,wall_s=wall)
        state.update_odom(odom(11.1),wall_s=wall)
        state.update_scan(scan(11.1),wall_s=wall)
    assert state.state=="STOP" and "clock" in state.reason


def test_clock_epoch_jump_latches_stale_sensor_observation_before_replacement():
    state=guard();ready(state)
    state.update_clock(11.301,wall_s=1.11)
    assert state.state=="STOP"
    refresh(state,1.32)
    assert state.step(wall_s=1.32).linear_x==0.
