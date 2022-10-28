import paho.mqtt.client as mqtt

from conf.mqtt_conf_params import MqttConfigurationParameters
import time

starting_point = 0
# Define a callback method to receive asynchronous messages
def on_message(client, userdata, message):
    message_payload = str(message.payload.decode("utf-8"))
    print(f"Received IoT Message: Topic: {message.topic} Payload: {message_payload}")
    #print(int(time.time() - starting_point))


def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    global starting_point
    starting_point = time.time()
    robot_info_topic = "{0}/{1}/+/{2}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.ROBOT_TOPIC,
        MqttConfigurationParameters.ROBOT_INFO_TOPIC)

    robot_action_topic = "{0}/{1}/+/{2}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.ROBOT_TOPIC,
        MqttConfigurationParameters.ROBOT_ACTION_TOPIC)

    robot_event_topic = "{0}/{1}/+/{2}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.ROBOT_TOPIC,
        MqttConfigurationParameters.ROBOT_EVENT_TOPIC)

    robot_telemetry_topic = "{0}/{1}/+/{2}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.ROBOT_TOPIC,
        MqttConfigurationParameters.ROBOT_TELEMETRY_TOPIC)

    operator_info_topic = "{0}/{1}/+/{2}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.OPERATOR_TOPIC,
        MqttConfigurationParameters.OPERATOR_INFO_TOPIC)

    operator_action_topic = "{0}/{1}/+/{2}".format(
        MqttConfigurationParameters.MQTT_BASIC_TOPIC,
        MqttConfigurationParameters.OPERATOR_TOPIC,
        MqttConfigurationParameters.OPERATOR_ACTION_TOPIC)

    mqtt_client.subscribe(operator_info_topic)
    print("Subscribed to: " + operator_info_topic)

    mqtt_client.subscribe(operator_action_topic)
    print("Subscribed to: " + operator_action_topic)

    mqtt_client.subscribe(robot_info_topic)
    print("Subscribed to: " + robot_info_topic)

    mqtt_client.subscribe(robot_action_topic)
    print("Subscribed to: " + robot_action_topic)

    mqtt_client.subscribe(robot_event_topic)
    print("Subscribed to: " + robot_event_topic)

    #mqtt_client.subscribe(robot_telemetry_topic)
    #print("Subscribed to: " + robot_telemetry_topic)


mdt_id = "MDT_Consumer_00"
message_limit = 1000

mqtt_client = mqtt.Client(mdt_id)
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

# Set Account Username & Password
mqtt_client.username_pw_set(MqttConfigurationParameters.MQTT_USERNAME, MqttConfigurationParameters.MQTT_PASSWORD)

print("Connecting to " + MqttConfigurationParameters.BROKER_ADDRESS + " port: " + str(MqttConfigurationParameters.BROKER_PORT))
mqtt_client.connect(MqttConfigurationParameters.BROKER_ADDRESS, MqttConfigurationParameters.BROKER_PORT)


mqtt_client.loop_forever()
