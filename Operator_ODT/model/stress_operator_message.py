import json


class StressOperatorMessage:
    def __init__(self, operator_id, operator_stress_alert):
        self.operator_id = operator_id
        self.operator_stress_alert = operator_stress_alert

    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__)
