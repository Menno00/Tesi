import json


class RobotInfo:
    def __init__(self, robot_id, robot_speed, robot_action, robot_current_action=None, robot_operator_id=None):
        self.robot_id = robot_id
        self.robot_speed = robot_speed
        self.robot_action = robot_action
        self.robot_current_action = robot_current_action
        self.robot_operator_id = robot_operator_id

    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__)
