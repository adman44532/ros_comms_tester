# For Godot ROS2 Module Package
extends Node

@export var websocket_url = "ws://10.19.0.215:9090" # Assuming rosbridge is running locally on port 9090
@export var publish_topic = "/latency_test_response"  # Topic to publish messages to
@export var subscribe_topic = "/latency_test_request"  # Topic to receive messages from

var socket = WebSocketPeer.new()

func _ready():
	# Initiate connection to the ROS2 bridge WebSocket URL.
	var err = socket.connect_to_url(websocket_url)
	if err != OK:
		print("Unable to connect")
		set_process(false)
	else:
		# Wait for the socket to connect.
		await get_tree().create_timer(2).timeout

		# Subscribe to the specified ROS2 topic
		subscribe_to_ros_topic(subscribe_topic)


func _process(_delta):
	socket.poll()

	var state = socket.get_ready_state()

	if state == WebSocketPeer.STATE_OPEN:
		while socket.get_available_packet_count():
			# Receive and parse JSON messages from ROS2
			var raw_message = socket.get_packet().get_string_from_utf8()
			var json = JSON.new()  # Create an instance of the JSON class
			var json_result = json.parse(raw_message)
			
			if json_result == OK:
				var parsed_message = json.get_data()
				# Check if the parsed_message is a dictionary and contains the key "data"
				if typeof(parsed_message) == TYPE_DICTIONARY and parsed_message.has("msg"):
					var string_data = str(parsed_message["msg"]["data"])  # Get the string data
					print("Received message from ROS2: ")
					send_ros_message(string_data)
				else:
					print("Parsed JSON does not contain 'data': ", parsed_message)
			else:
				print("Failed to parse JSON: ", raw_message)

	elif state == WebSocketPeer.STATE_CLOSING:
		pass

	elif state == WebSocketPeer.STATE_CLOSED:
		var code = socket.get_close_code()
		print("WebSocket closed with code: %d. Clean: %s" % [code, code != -1])
		set_process(false)

# Function to subscribe to a ROS2 topic
func subscribe_to_ros_topic(topic_name: String):
	if socket.get_ready_state() == WebSocketPeer.STATE_OPEN:
		var subscribe_message = {
			"op": "subscribe",
			"topic": topic_name,
			"type": "std_msgs/String"
		}
		socket.send_text(JSON.stringify(subscribe_message))
	else:
		print("Socket not open. Cannot subscribe to topic.")

# Function to send a message to the ROS2 publish topic
func send_ros_message(data: String):
	if socket.get_ready_state() == WebSocketPeer.STATE_OPEN:
		var publish_message = {
			"op": "publish",
			"topic": publish_topic,
			"msg": {"data": data}
		}
		socket.send_text(JSON.stringify(publish_message))
	else:
		print("Socket not open. Can't send message.")
