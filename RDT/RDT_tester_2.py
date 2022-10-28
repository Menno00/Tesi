import json
import time
import paho.mqtt.client as mqtt
from threading import Thread
from pyniryo2 import *

from conf.mqtt_conf_params import MqttConfigurationParameters
from model.action_robot_message import ActionRobotMessage
from model.robot_descriptor import RobotDescriptor
from model.event_message import EventMessage
from model.info_robot_message import InfoRobotMessage
from model.telemetry_message import TelemetryMessage


def publish_robot_info():
    robot_info_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.ROBOT_TOPIC,
        robot_descriptor.robot_id,
        MqttConfigurationParameters.ROBOT_INFO_TOPIC)
    info_robot_message = InfoRobotMessage(robot_descriptor.robot_id, robot_descriptor.speed,
                                          robot_descriptor.action, robot_descriptor.current_action,
                                          robot_descriptor.operator_id).to_json()
    mqtt_client.publish(robot_info_topic, info_robot_message, 0, True)


def publish_robot_event(event_type, metadata):
    robot_event_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.ROBOT_TOPIC,
        robot_descriptor.robot_id,
        MqttConfigurationParameters.ROBOT_EVENT_TOPIC)
    event_message = EventMessage(event_type, metadata).to_json()
    mqtt_client.publish(robot_event_topic, event_message)


def publish_robot_telemetry():
    robot_telemetry_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.ROBOT_TOPIC,
        robot_descriptor.robot_id,
        MqttConfigurationParameters.ROBOT_TELEMETRY_TOPIC)

    old_joints = robot_descriptor.joints
    while not flag_pick_and_place:

        robot_descriptor.joints = robot.arm.get_joints()
        if old_joints != robot_descriptor.joints:
            telemetry_message = TelemetryMessage(robot_descriptor.robot_id, robot_descriptor.speed, robot_descriptor.joints,
                                                 robot_descriptor.gripper).to_json()
            mqtt_client.publish(robot_telemetry_topic, telemetry_message, 0, False)
        old_joints = robot_descriptor.joints
        time.sleep(0.01)


def pick_n_place(pick_pose, place_pose):
    pick_pose = PoseObject(pick_pose[0], pick_pose[1], pick_pose[2], pick_pose[3], pick_pose[4], pick_pose[5])

    height_offset = 0.05  # Offset according to Z-Axis to go over pick & place poses
    gripper_speed = 400

    pick_pose_high = pick_pose.copy_with_offsets(z_offset=height_offset)
    place_pose_high = place_pose.copy_with_offsets(z_offset=height_offset)

    # Going Over Object
    robot.arm.move_pose(pick_pose_high)
    # Opening Gripper
    robot.tool.release_with_tool()
    robot_descriptor.gripper = False
    # Going to picking place and closing gripper
    robot.arm.move_pose(pick_pose)
    robot.tool.grasp_with_tool()
    robot_descriptor.gripper = True
    # Raising
    robot.arm.move_pose(pick_pose_high)

    # Going Over Place pose
    robot.arm.move_pose(place_pose_high)
    # Going to Place pose
    robot.arm.move_pose(place_pose)
    # Opening Gripper
    robot.tool.release_with_tool()
    robot_descriptor.gripper = False
    # Raising
    robot.arm.move_pose(place_pose_high)


def pick_pose_task1():
    pick_pose_number = 1

    global flag_pick_and_place
    flag_pick_and_place = False

    publish_telemetry_thread = Thread(target=publish_robot_telemetry)
    #publish_telemetry_thread.start()

    action_robot_message.place_pose = \
        PoseObject(action_robot_message.place_pose[0], action_robot_message.place_pose[1], action_robot_message.place_pose[2],
                   action_robot_message.place_pose[3], action_robot_message.place_pose[4], action_robot_message.place_pose[5])

    for pick_pose in action_robot_message.pick_pose_list:
        if not stop_movement:
            #pick_n_place(pick_pose, action_robot_message.place_pose)
            print("Pick and place " + str(pick_pose_number) + " complete")
            publish_robot_event("Pick and place complete", str(pick_pose_number) + " complete")
            pick_pose_number += 1
            time.sleep(5)

    #robot.arm.move_joints(action_robot_message.home_pose)
    flag_pick_and_place = True
    if not stop_movement:
        publish_robot_event("Task complete", action_robot_message.to_json())
    else:
        publish_robot_event("Task stopped", action_robot_message.to_json())

    robot_descriptor.operator_id = None
    robot_descriptor.current_action = None

    publish_robot_info()


######## Lettura da MQTT

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))

    robot_action_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.ROBOT_TOPIC,
        robot_descriptor.robot_id,
        MqttConfigurationParameters.ROBOT_ACTION_TOPIC)

    mqtt_client.subscribe(robot_action_topic)

    print("Subscribed to: " + robot_action_topic)


def on_message(client, userdata, message):
    global action_robot_message
    global stop_movement

    if robot_descriptor.current_action is None:
        message_payload = str(message.payload.decode("utf-8"))
        action_robot_message = ActionRobotMessage(**json.loads(message_payload))

        if action_robot_message.action_type == 'task' and action_robot_message.task_name in robot_descriptor.action:
            stop_movement = False
            print(f"Received IoT Message: Topic: {message.topic} Payload: {message_payload}")
            publish_robot_event("New Mission", message_payload)

            robot_descriptor.current_action = action_robot_message.task_name
            robot_descriptor.operator_id = action_robot_message.operator_id
            robot_descriptor.speed = action_robot_message.speed
            #robot.arm.set_arm_max_velocity(robot_descriptor.speed)
            publish_robot_info()

            pick_pose_thread = Thread(target=pick_pose_task1)
            #pick_pose_thread.start()
            return

        elif action_robot_message.action_type == 'stop':
            stop_movement = True
            publish_robot_event("Error", "Robot not in motion")

        else:
            publish_robot_event("Error", "Mission not correct")

    else:
        message_payload = str(message.payload.decode("utf-8"))
        second_action_robot_message = ActionRobotMessage(**json.loads(message_payload))
        if second_action_robot_message.action_type == 'stop':
            stop_movement = True
            robot_descriptor.current_action = None
            robot_descriptor.operator_id = None
            publish_robot_info()
            publish_robot_event("Stop movement", "Stop pick and place")
            print(f"Received IoT Message: Topic: {message.topic} Payload: {message_payload}")

        elif second_action_robot_message.action_type == 'speed':
            print(second_action_robot_message.speed)
            robot_descriptor.speed = second_action_robot_message.speed
            #robot.arm.set_arm_max_velocity(robot_descriptor.speed)
            publish_robot_info()
            publish_robot_event("Change speed", "New speed set")
            print(f"Received IoT Message: Topic: {message.topic} Payload: {message_payload}")

        else:
            print(f"CONFLICT - Robot already in motion")
            publish_robot_event("Conflict", "Robot already in motion")


flag_pick_and_place = False
stop_movement = False
action_robot_message = ActionRobotMessage()

#robot_descriptor = RobotDescriptor("NiryoNed", "169.254.200.200", ["Duck", "Tree"])
robot_descriptor = RobotDescriptor("NiryoNed_2", "10.10.10.10", ["Duck"])

mqtt_client = mqtt.Client(robot_descriptor.robot_id)
mqtt_client.on_message = on_message
mqtt_client.on_connect = on_connect

# Set Account Username & Password
mqtt_client.username_pw_set(MqttConfigurationParameters.MQTT_USERNAME, MqttConfigurationParameters.MQTT_PASSWORD)

print("Connecting to " + MqttConfigurationParameters.BROKER_ADDRESS + " port: " + str(
    MqttConfigurationParameters.BROKER_PORT))
mqtt_client.connect(MqttConfigurationParameters.BROKER_ADDRESS, MqttConfigurationParameters.BROKER_PORT)


###### Connessione
home_pose = [0, 0, 0, 0, 0, 0]
#robot = NiryoRobot(robot_descriptor.robot_ip)
print("connect")
publish_robot_event("Start", "Robot connected")

###### Calibrazione e movimento

#robot.arm.calibrate_auto()
#robot.tool.update_tool()

publish_robot_info()
publish_robot_event("Calibration", "Robot calibrated")

#robot.arm.move_joints(home_pose)
#robot_descriptor.joints = robot.arm.get_joints()
publish_robot_event("Ready", "Robot ready")

mqtt_client.loop_forever()

#robot.end()
