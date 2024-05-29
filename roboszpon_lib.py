import can
import struct

MSG_MOTOR_COMMAND = 0x01
MSG_ACTION_REQUEST = 0x02

ACTION_ARM = 0x00
ACTION_DISARM = 0x01


def send_can_frame(interface, frame_id, data):
    try:
        canbus = can.interface.Bus(channel=interface, bustype="socketcan")
        frame = can.Message(
            arbitration_id=frame_id,
            data=data.to_bytes(8, byteorder="big"),
            is_extended_id=False,
        )
        canbus.send(frame)
        canbus.shutdown()
    except Exception as e:
        print(f"Error sending CAN frame: {e}")


def build_frame_id(node_id, message_id):
    return ((node_id & 0b11111) << 6) + (message_id & 0b111111)


def send_action_request(interface, node_id, action_id):
    send_can_frame(interface, build_frame_id(node_id, MSG_ACTION_REQUEST), action_id)


def arm(interface, node_id):
    send_action_request(interface, node_id, ACTION_ARM)


def disarm(interface, node_id):
    send_action_request(interface, node_id, ACTION_DISARM)


def float_to_bits(value):
    value_bits = struct.pack("f", value)
    return struct.unpack("i", value_bits)[0]


def send_motor_command(interface, node_id, motor_command_type, command):
    data = ((motor_command_type & 0xFF) << 56) + (
        (float_to_bits(command) & 0xFFFFFFFF) << 24
    )
    send_can_frame(interface, build_frame_id(node_id, MSG_MOTOR_COMMAND), data)


def send_duty_command(interface, node_id, command):
    send_motor_command(interface, node_id, 0, command)


def send_velocity_command(interface, node_id, command):
    send_motor_command(interface, node_id, 1, command)


def send_position_command(interface, node_id, command):
    send_motor_command(interface, node_id, 2, command)


def emergency_stop(interface):
    send_can_frame(interface, 0x001, 0)
