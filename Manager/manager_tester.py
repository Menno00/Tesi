import json

import yaml
import paho.mqtt.client as mqtt
from threading import Thread
import time
from conf.mqtt_conf_params import MqttConfigurationParameters
from model.action_robot_message import ActionRobotMessage
from model.action_operator_message import ActionOperatorMessage
from model.operator_info import OperatorInfo
from model.robot_info import RobotInfo
from model.stress_operator_message import StressOperatorMessage


def read_info():
    robot_info_topic = "{0}/{1}/+/{2}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.ROBOT_TOPIC,
        MqttConfigurationParameters.ROBOT_INFO_TOPIC)

    mqtt_client.subscribe(robot_info_topic)

    print("Subscribed to: " + robot_info_topic)


def read_telemetry():
    robot_telemetry_topic = "{0}/{1}/+/{2}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.ROBOT_TOPIC,
        MqttConfigurationParameters.ROBOT_TELEMETRY_TOPIC)

    mqtt_client.subscribe(robot_telemetry_topic)

    print("Subscribed to: " + robot_telemetry_topic)


def read_event():
    robot_event_topic = "{0}/{1}/+/{2}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.ROBOT_TOPIC,
        MqttConfigurationParameters.ROBOT_EVENT_TOPIC)

    mqtt_client.subscribe(robot_event_topic)

    print("Subscribed to: " + robot_event_topic)


def send_new_mission():
    name_file_yaml = input("\nEnter configuration file name:")

    with open(name_file_yaml) as f:
        try:
            dict_yaml = yaml.load(f, Loader=yaml.FullLoader)
            print(dict_yaml)
        except yaml.YAMLError as e:
            print(e)
    time.sleep(3)
    action_robot_message = ActionRobotMessage()
    action_robot_message.get_from_yaml(dict_yaml)
    mission_send = False
    if (action_robot_message.robot_id in robot_dict.keys()
            and robot_dict[action_robot_message.robot_id].robot_current_action is None
            and action_robot_message.task_name in robot_dict[action_robot_message.robot_id].robot_action):
        for op_dict in operator_dict.values():
            if (action_robot_message.task_name in op_dict.operator_action
                    and op_dict.operator_robot_id is None
                    and op_dict.operator_status is True):
                action_robot_message.operator_id = op_dict.operator_id
                action_operator_message = ActionOperatorMessage(op_dict.operator_id, action_robot_message.robot_id,
                                                                action_robot_message.action_type,
                                                                action_robot_message.task_name)

                robot_action_topic = "{0}/{1}/{2}/{3}".format(
                    MqttConfigurationParameters.MQTT_BASIC_TOPIC,
                    MqttConfigurationParameters.ROBOT_TOPIC,
                    action_robot_message.robot_id,
                    MqttConfigurationParameters.ROBOT_ACTION_TOPIC)

                operator_action_topic = "{0}/{1}/{2}/{3}".format(
                    MqttConfigurationParameters.MQTT_BASIC_TOPIC,
                    MqttConfigurationParameters.OPERATOR_TOPIC,
                    op_dict.operator_id,
                    MqttConfigurationParameters.OPERATOR_ACTION_TOPIC)

                robot_payload_string = action_robot_message.to_json()
                operator_payload_string = action_operator_message.to_json()

                mqtt_client.publish(robot_action_topic, robot_payload_string, 0, False)
                mqtt_client.publish(operator_action_topic, operator_payload_string, 0, False)

                print(f"New Mission Published: Topic: {robot_action_topic} Payload: {robot_payload_string}")
                print(f"New Mission Published: Topic: {operator_action_topic} Payload: {operator_payload_string}")

                mission_send = True
            if mission_send is True:
                return
        if mission_send is not True:
            print("No operator able to satisfy the request")
    else:
        print("Robot not usable")


def send_stop_robot(robot_id):
    stop_action_robot_message = ActionRobotMessage()
    stop_action_robot_message.robot_id = robot_id
    stop_action_robot_message.action_type = 'stop'
    stop_action_robot_message.home_pose = [0, 0, 0, 0, 0, 0]

    robot_action_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.ROBOT_TOPIC,
        stop_action_robot_message.robot_id,
        MqttConfigurationParameters.ROBOT_ACTION_TOPIC)

    robot_payload_string = stop_action_robot_message.to_json()
    mqtt_client.publish(robot_action_topic, robot_payload_string, 0, False)
    print(f"Stop Mission Published: Topic: {robot_action_topic} Payload: {robot_payload_string}")


def send_stop_operator(operator_id):
    stop_action_operator_message = ActionOperatorMessage()
    stop_action_operator_message.operator_id = operator_id
    stop_action_operator_message.action_type = 'stop'

    operator_action_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.OPERATOR_TOPIC,
        operator_id,
        MqttConfigurationParameters.OPERATOR_ACTION_TOPIC)

    operator_payload_string = stop_action_operator_message.to_json()
    mqtt_client.publish(operator_action_topic, operator_payload_string, 0, False)
    print(f"Stop Mission Published: Topic: {operator_action_topic} Payload: {operator_payload_string}")


def stop_mission(robot_id):
    operator_id = robot_dict[robot_id].robot_operator_id
    send_stop_operator(operator_id)
    send_stop_robot(robot_id)


def modify_speed_robot(robot_id, new_speed):
    modify_speed_robot_message = ActionRobotMessage()
    modify_speed_robot_message.robot_id = robot_id
    modify_speed_robot_message.action_type = 'speed'
    modify_speed_robot_message.speed = new_speed

    robot_action_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.ROBOT_TOPIC,
        modify_speed_robot_message.robot_id,
        MqttConfigurationParameters.ROBOT_ACTION_TOPIC)

    robot_payload_string = modify_speed_robot_message.to_json()

    mqtt_client.publish(robot_action_topic, robot_payload_string, 0, False)

    print(f"Modify Speed Published: Topic: {robot_action_topic} Payload: {robot_payload_string}")


def on_message(client, userdata, message):
    message_payload = str(message.payload.decode("utf-8"))
    print(f"Received IoT Message: Topic: {message.topic} Payload: {message_payload}")

    global operator_dict
    global robot_dict

    if "/operator/" in message.topic:
        if "/stress" in message.topic:
            stress = StressOperatorMessage(**json.loads(message_payload))
            if stress.operator_stress_alert is True:
                modify_speed_robot(operator_dict[stress.operator_id].operator_robot_id, 5)
            if stress.operator_stress_alert is False:
                modify_speed_robot(operator_dict[stress.operator_id].operator_robot_id, 2)
        else:
            operator = OperatorInfo(**json.loads(message_payload))
            operator_dict[operator.operator_id] = operator
            print(operator_dict)

    if "/robot/" in message.topic:
        robot = RobotInfo(**json.loads(message_payload))
        if robot.robot_id in robot_dict and robot.robot_current_action is None and robot_dict[robot.robot_id].robot_operator_id is not None \
                and operator_dict[robot_dict[robot.robot_id].robot_operator_id].operator_current_action is not None:
            send_stop_operator(robot_dict[robot.robot_id].robot_operator_id)
        robot_dict[robot.robot_id] = robot
        print(robot_dict)


def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    operator_info_topic = "{0}/{1}/+/{2}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.OPERATOR_TOPIC,
        MqttConfigurationParameters.OPERATOR_INFO_TOPIC)
    operator_stress_topic = "{0}/{1}/+/{2}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.OPERATOR_TOPIC,
        MqttConfigurationParameters.OPERATOR_STRESS_TOPIC)
    robot_info_topic = "{0}/{1}/+/{2}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.ROBOT_TOPIC,
        MqttConfigurationParameters.ROBOT_INFO_TOPIC)

    mqtt_client.subscribe(operator_info_topic)
    mqtt_client.subscribe(robot_info_topic)
    mqtt_client.subscribe(operator_stress_topic)


def menu():
    while True:
        n_input = input("\nSelect: \
                \n1: Read robot info\
                \n2: Read telemetry\
                \n3: Read event \
                \n4: Start new mission \
                \n5: Stop mission \n")
        if n_input == '1':
            read_info()
        elif n_input == '2':
            read_telemetry()
        elif n_input == '3':
            read_event()
        elif n_input == '4':
            send_new_mission()
        elif n_input == '5':
            robot_id = input("Which robot to terminate?")
            stop_mission(robot_id)
        else:
            print("Input wrong")


operator_dict = {}
robot_dict = {}

mdt_id = "MDT_Tester_00"

mqtt_client = mqtt.Client(mdt_id)
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

# Set Account Username & Password
mqtt_client.username_pw_set(MqttConfigurationParameters.MQTT_USERNAME, MqttConfigurationParameters.MQTT_PASSWORD)

print("Connecting to " + MqttConfigurationParameters.BROKER_ADDRESS + " port: " + str(
    MqttConfigurationParameters.BROKER_PORT))
mqtt_client.connect(MqttConfigurationParameters.BROKER_ADDRESS, MqttConfigurationParameters.BROKER_PORT)

menu_thread = Thread(target=menu)
menu_thread.start()

mqtt_client.loop_forever()
