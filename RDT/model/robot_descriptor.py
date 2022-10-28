class RobotDescriptor:
    def __init__(self, robot_id, robot_ip, robot_action, robot_current_action=None, robot_operator_id=None):
        self.robot_id = robot_id
        self.robot_ip = robot_ip
        self.robot_joints = [0, 0, 0, 0, 0, 0]
        self.robot_speed = 2
        self.robot_end_effector = False
        self.robot_action = robot_action
        self.robot_current_action = robot_current_action
        self.robot_operator_id = robot_operator_id
