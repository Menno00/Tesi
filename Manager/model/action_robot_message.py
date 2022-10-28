import json


class ActionRobotMessage:

    def __init__(self):
        self.robot_id = ''
        self.action_type = ''
        self.task_name = ''
        self.speed = ''
        self.movement_type = ''
        self.home_pose = []
        self.place_pose = []
        self.pick_pose_list = []
        self.operator_id = ''

    def get_from_yaml(self, dict_yaml):
        self.robot_id = dict_yaml['id']
        self.action_type = dict_yaml['action_type']
        self.task_name = dict_yaml['task']
        self.speed = dict_yaml['speed']
        self.movement_type = dict_yaml['movement_type']
        self.home_pose = self.get_pose_from_yaml(dict_yaml['home_pose'])
        self.place_pose = self.get_pose_from_yaml(dict_yaml['place_pose'])
        self.pick_pose_list = self.pick_pose_list_from_yaml(dict_yaml['pick_list'])


    @staticmethod
    def get_pose_from_yaml(yaml_dict):
        list_coord = [yaml_dict['x'], yaml_dict['y'], yaml_dict['z'],
                      yaml_dict['roll'], yaml_dict['pitch'], yaml_dict['yaw']]
        return list_coord

    def pick_pose_list_from_yaml(self, yaml_list):
        list_pick_pose = []
        for yaml_key, yaml_dict in yaml_list.items():
            list_pick_pose.append(
                self.get_pose_from_yaml(yaml_dict)
            )
        return list_pick_pose

    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__)



