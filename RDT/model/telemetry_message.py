from datetime import datetime
import json


class TelemetryMessage:
    def __init__(self, robot_id, robot_speed, robot_joints, robot_state_end_effector):
        self.robot_id = robot_id
        self.robot_speed = robot_speed
        self.robot_joints = robot_joints
        self.robot_state_end_effector = robot_state_end_effector
        self.timestamp = datetime.timestamp(datetime.now())

    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__)
