import json


class ActionOperatorMessage:

    def __init__(self, operator_id=None, robot_id=None, action_type=None, task_name=None):
        self.operator_id = operator_id
        self.robot_id = robot_id
        self.action_type = action_type
        self.task_name = task_name

    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__)

