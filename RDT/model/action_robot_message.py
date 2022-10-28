import json


class ActionRobotMessage:

    def __init__(self, robot_id=None, action_type=None, task_name=None, speed=None, movement_type=None, home_pose=None, place_pose=None, pick_pose_list=None, operator_id = None):
        self.robot_id = robot_id
        self.action_type = action_type
        self.task_name = task_name
        self.speed = speed
        self.movement_type = movement_type
        self.home_pose = home_pose
        self.place_pose = place_pose
        self.pick_pose_list = pick_pose_list
        self.operator_id = operator_id

    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__)

