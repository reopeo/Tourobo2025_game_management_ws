import rclpy
from rclpy.node import Node
from game_state_interfaces.msg import Match
import socket
import json
import time

# Raspberry Pi Connection Information
HOSTNAME = 'raspberrypi.local'
PORT = 65432

MAX_RETRIES = 3
RETRY_DELAY_SECONDS = 2

def send_tape_led_command(pi_ip, port, tape_led_states):
    retries = 0
    while retries <= MAX_RETRIES:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((pi_ip, port))
                initial_response = s.recv(4096).decode('utf-8')
                status_data = json.loads(initial_response)
                if status_data.get("status") == "ok":
                    command = {"control_tape_leds": tape_led_states}
                    s.sendall(json.dumps(command).encode('utf-8'))
                    response = s.recv(4096).decode('utf-8')
                    return json.loads(response)
                else:
                    return {"status": "error", "message": "Pi server not ready or reported error."}
        except (ConnectionRefusedError, socket.error):
            retries += 1
            time.sleep(RETRY_DELAY_SECONDS)
        except (json.JSONDecodeError, Exception) as e:
            return {"status": "error", "message": str(e)}
    return {"status": "error", "message": f"Failed after {MAX_RETRIES} retries."}

class LedManagerNode(Node):
    def __init__(self):
        super().__init__('led_manager')
        self.subscription = self.create_subscription(
            Match,
            '/match/status',
            self.match_status_callback,
            10
        )

    def match_status_callback(self, msg: Match):
        # type_1_a～type_3_cをLED 0～8に割り当て
        type_fields = [
            msg.type_3_c, msg.type_2_c, msg.type_1_c,
            msg.type_3_b, msg.type_2_b, msg.type_1_b,
            msg.type_2_a, msg.type_1_a, msg.type_3_a
        ]
        # LED状態: {led_id: color_code}
        led_states = {i: type_fields[i] for i in range(9)}
        self.get_logger().info(f'Sending LED states: {led_states}')
        result = send_tape_led_command(HOSTNAME, PORT, led_states)
        self.get_logger().info(f'LED command result: {result}')


def main():
    rclpy.init()
    node = LedManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
