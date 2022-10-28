import json

class OperatorDescriptor:
    def __init__(self, operator_id, operator_level, operator_action, operator_current_action=None, operator_robot_id=None, operator_status=True):
        self.operator_id = operator_id
        self.operator_level = operator_level
        self.operator_action = operator_action
        self.operator_current_action = operator_current_action
        self.operator_robot_id = operator_robot_id
        self.operator_status = operator_status
        self.operator_stress = False

    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__)
