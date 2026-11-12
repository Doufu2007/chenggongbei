from inspection_mission.probe import PUBLIC_EXACT_TOPICS, select_public_topics, summarize_payload


def test_select_public_topics_excludes_forbidden_and_keeps_public_interfaces():
    topics = [
        ('/cmd_vel', 'geometry_msgs/msg/Twist'),
        ('/uav/cmd_vel', 'geometry_msgs/msg/Twist'),
        ('/odom', 'nav_msgs/msg/Odometry'),
        ('/uav/odom', 'nav_msgs/msg/Odometry'),
        ('/referee/current_target', 'std_msgs/msg/String'),
        ('/referee/ground_tasks', 'std_msgs/msg/String'),
        ('/referee/recognition_status', 'std_msgs/msg/String'),
        ('/referee/events_public', 'std_msgs/msg/String'),
        ('/inspection/detections', 'std_msgs/msg/String'),
        ('/referee/cruise_event', 'std_msgs/msg/String'),
        ('/uav/laser_fire', 'std_msgs/msg/Bool'),
        ('/uav/laser_state', 'std_msgs/msg/Bool'),
        ('/clock', 'rosgraph_msgs/msg/Clock'),
        ('/referee/events', 'std_msgs/msg/String'),
        ('/gazebo/model_states', 'gazebo_msgs/msg/ModelStates'),
    ]
    selected = select_public_topics(topics)
    names = {name for name, _ in selected}
    assert set(PUBLIC_EXACT_TOPICS) <= names
    assert '/referee/events' not in names
    assert '/gazebo/model_states' not in names


def test_summarize_payload_is_bounded_and_keeps_nested_keys():
    payload = {'header': {'stamp': {'sec': 4, 'nanosec': 7}, 'frame_id': 'map'}, 'pose': {'x': 1.0}, 'ranges': list(range(100))}
    summary = summarize_payload(payload, max_items=4)
    assert summary['header']['frame_id'] == 'map'
    assert summary['pose']['x'] == 1.0
    assert len(summary['ranges']) <= 4
def test_probe_does_not_treat_an_unknown_odometry_topic_as_public():
    assert select_public_topics([('/ground_truth/odom', 'nav_msgs/msg/Odometry'), ('/other_odom', 'nav_msgs/msg/Odometry')]) == []


def test_probe_discovers_sensor_topics_without_allowing_referee_private_routes():
    topics = [('/scan', 'sensor_msgs/msg/LaserScan'), ('/camera/image_raw', 'sensor_msgs/msg/Image'),
              ('/referee/hidden_image', 'sensor_msgs/msg/Image'), ('/gazebo/scan', 'sensor_msgs/msg/LaserScan')]
    assert select_public_topics(topics) == [('/camera/image_raw', 'sensor_msgs/msg/Image'), ('/scan', 'sensor_msgs/msg/LaserScan')]
