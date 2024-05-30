import can
import struct

MSG_MOTOR_COMMAND = 0x01
MSG_ACTION_REQUEST = 0x02
MSG_STATUS_REPORT = 0x03
MSG_AXIS_REPORT = 0x04
MSG_MOTOR_REPORT = 0x05

ACTION_ARM = 0x00
ACTION_DISARM = 0x01

ROBOSZPON_MODE_STOPPED = 0x00
ROBOSZPON_MODE_RUNNING = 0x01
ROBOSZPON_MODE_ERROR = 0x02
ROBOSZPON_MODES = {0: "STOPPED", 1: "RUNNING", 2: "ERROR"}


def send_can_frame(canbus, frame_id, data):
    try:
        frame = can.Message(
            arbitration_id=frame_id,
            data=data.to_bytes(8, byteorder="big"),
            is_extended_id=False,
        )
        canbus.send(frame)
    except Exception as e:
        print(f"Error sending CAN frame: {e}")


def build_frame_id(node_id, message_id):
    return ((node_id & 0b11111) << 6) + (message_id & 0b111111)


def decode_message(frame_id, data):
    node_id = (frame_id >> 6) & 0b11111
    message_id = frame_id & 0b111111
    if message_id == MSG_STATUS_REPORT:
        mode = (data >> 62) & 0b11
        flags = data & 0x3FFFFFFFFFFFFFFF
        return {
            "node_id": node_id,
            "message_id": message_id,
            "mode": mode,
            "flags": flags,
        }
    if message_id == MSG_AXIS_REPORT:
        position = (data >> 32) & 0xFFFFFFFF
        velocity = data & 0xFFFFFFFF
        return {
            "node_id": node_id,
            "message_id": message_id,
            "position": bits_to_float(position),
            "velocity": bits_to_float(velocity),
        }
    if message_id == MSG_MOTOR_REPORT:
        current = (data >> 32) & 0xFFFFFFFF
        duty = data & 0xFFFFFFFF
        return {
            "node_id": node_id,
            "message_id": message_id,
            "current": bits_to_float(current),
            "duty": bits_to_float(duty),
        }
    return {"node_id": node_id, "message_id": message_id, "data": data}


def send_action_request(canbus, node_id, action_id):
    send_can_frame(canbus, build_frame_id(node_id, MSG_ACTION_REQUEST), action_id)


def arm(canbus, node_id):
    send_action_request(canbus, node_id, ACTION_ARM)


def disarm(canbus, node_id):
    send_action_request(canbus, node_id, ACTION_DISARM)


def float_to_bits(value):
    value_bits = struct.pack("f", value)
    return struct.unpack("I", value_bits)[0]


def bits_to_float(value):
    value_bits = struct.pack("I", value)
    return struct.unpack("f", value_bits)[0]


def send_motor_command(canbus, node_id, motor_command_type, command):
    data = ((motor_command_type & 0xFF) << 56) + (
        (float_to_bits(command) & 0xFFFFFFFF) << 24
    )
    send_can_frame(canbus, build_frame_id(node_id, MSG_MOTOR_COMMAND), data)


def send_duty_command(canbus, node_id, command):
    send_motor_command(canbus, node_id, 0, command)


def send_velocity_command(canbus, node_id, command):
    send_motor_command(canbus, node_id, 1, command)


def send_position_command(canbus, node_id, command):
    send_motor_command(canbus, node_id, 2, command)


def emergency_stop(canbus):
    send_can_frame(canbus, 0x001, 0)


# f = 0.5
# print(f)
# i = float_to_bits(f)
# print(hex(i))
# ff = bits_to_float(i)
# print(ff)
