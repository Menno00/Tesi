import json
import time
import paho.mqtt.client as mqtt
from threading import Thread

from conf.mqtt_conf_params import MqttConfigurationParameters
from model.action_operator_message import ActionOperatorMessage
from model.operator_descriptor import OperatorDescriptor
from model.stress_operator_message import StressOperatorMessage


def publish_operator_info():
    operator_info_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.OPERATOR_TOPIC,
        operator_descriptor.operator_id,
        MqttConfigurationParameters.OPERATOR_INFO_TOPIC)
    info_operator_message = operator_descriptor.to_json()
    mqtt_client.publish(operator_info_topic, info_operator_message, 0, True)


def publish_stress_alert(stress_flag):
    operator_stress_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.OPERATOR_TOPIC,
        operator_descriptor.operator_id,
        MqttConfigurationParameters.OPERATOR_STRESS_TOPIC)
    stress_operator_message = StressOperatorMessage(operator_descriptor.operator_id, stress_flag).to_json()
    mqtt_client.publish(operator_stress_topic, stress_operator_message, 0, False)


def control_operator_stress():
    while True:
        if operator_descriptor.operator_current_action is not None and operator_descriptor.operator_stress is False:
            time.sleep(50)
            if operator_descriptor.operator_current_action is not None:
                operator_descriptor.operator_stress = True
                publish_stress_alert(True)
                publish_operator_info()
                time.sleep(50)
        elif operator_descriptor.operator_current_action is not None and operator_descriptor.operator_stress is True:
            if operator_descriptor.operator_current_action is not None:
                operator_descriptor.operator_stress = False
                publish_stress_alert(False)
                publish_operator_info()

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))

    publish_operator_info()

    operator_action_topic = "{0}/{1}/{2}/{3}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.OPERATOR_TOPIC,
        operator_descriptor.operator_id,
        MqttConfigurationParameters.OPERATOR_ACTION_TOPIC)

    mqtt_client.subscribe(operator_action_topic)

    print("Subscribed to: " + operator_action_topic)


def on_message(client, userdata, message):
    message_payload = str(message.payload.decode("utf-8"))
    print(f"Received IoT Message: Topic: {message.topic} Payload: {message_payload}")
    action_operator_message = ActionOperatorMessage(**json.loads(message_payload))
    global operator_descriptor

    if action_operator_message.action_type == "task":
        operator_descriptor.operator_robot_id = action_operator_message.robot_id
        operator_descriptor.operator_current_action = action_operator_message.task_name
    elif action_operator_message.action_type == "stop":
        operator_descriptor.operator_robot_id = None
        operator_descriptor.operator_current_action = None
        operator_descriptor.operator_stress = False

    publish_operator_info()


operator_descriptor = OperatorDescriptor("Operator_0001", "junior", ["Faro", "Albero"])
time.sleep(10)

mqtt_client = mqtt.Client(operator_descriptor.operator_id)
mqtt_client.on_message = on_message
mqtt_client.on_connect = on_connect

# Set Account Username & Password
mqtt_client.username_pw_set(MqttConfigurationParameters.MQTT_USERNAME, MqttConfigurationParameters.MQTT_PASSWORD)

print("Connecting to " + MqttConfigurationParameters.BROKER_ADDRESS + " port: " + str(
    MqttConfigurationParameters.BROKER_PORT))
mqtt_client.connect(MqttConfigurationParameters.BROKER_ADDRESS, MqttConfigurationParameters.BROKER_PORT)

stress_thread = Thread(target=control_operator_stress)
stress_thread.start()



mqtt_client.loop_forever()





