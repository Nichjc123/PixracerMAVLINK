import os
from pymavlink import mavutil
import time

def send_command(master, cmd):
    """Send a single command tuple via the given pymavlink connection."""
    typ = cmd[0]

    if typ == "motor_test":
        _, m, pct, dur = cmd
        master.mav.command_long_send(
            1, 1,
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
            0,
            m,      # motor instance
            0,      # throttle type = percent
            pct,    # throttle percentage
            dur,    # duration (s)
            0,0,0
        )

    elif typ == "noop":
        # USER_1 is reserved and unimplemented
        master.mav.command_long_send(
            1, 1,
            mavutil.mavlink.MAV_CMD_USER_1, 0,
            0,0,0,0,0,0,0
        )

    elif typ == "set_servo":
        _, s, pwm = cmd
        master.mav.command_long_send(
            1, 1,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            s, pwm, 0,0,0,0,0
        )

    elif typ == "arm":
        master.mav.command_long_send(
            1, 1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,0,0,0,0,0,0
        )

    elif typ == "disarm":
        master.mav.command_long_send(
            1, 1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,0,0,0,0,0,0
        )

    elif typ == "set_mode":
        _, mode = cmd
        master.mav.set_mode_send(
            1, 1,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode
        )

    elif typ == "req_data":
        _, stream_id, rate = cmd
        master.mav.request_data_stream_send(
            1, 1,
            stream_id,
            rate,
            1    # start
        )

    elif typ == "heartbeat":
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,0,0
        )

    elif typ == "ping":
        seq = cmd[1]
        master.mav.ping_send(
            int(time.time()*1e6),
            seq, 1, 1
        )

    else:
        raise ValueError(f"Unknown command {cmd}")

def main():
    port = "/dev/cu.usbserial-AQ03MQKL"
    baud = 115200

    master = mavutil.mavlink_connection(port, baud=baud, mavlink_version=2)
    master.wait_heartbeat()
    print("Heartbeat received; starting per‑command fuzzing")

    commands = [
        ("motor_test",  1, 20, 0.5),      # MOTOR_TEST: spin motor 1 at 20% for 1 s
        ("motor_test",  1, 40, 0.5),      # MOTOR_TEST: spin motor 1 at 60% for 1 s
        ("motor_test",  2, 60, 0.5),      # MOTOR_TEST: spin motor 2 at 80% for 1 s
        ("set_servo",       5, 100),    # SET_SERVO: drive servo output 5 to 100 µs
        ("noop",),                      # does absolutely nothing
        ("arm",),                       # ARM: arm the vehicle (enable motors)
        ("disarm",),                    # DISARM: disarm the vehicle (disable motors)
        ("set_mode", mavutil.mavlink.MAV_MODE_GUIDED_ARMED),  # SET_MODE: switch to GUIDED mode
        ("req_data", mavutil.mavlink.MAV_DATA_STREAM_ALL, 20),# REQ_DATA_STREAM: request all data at 20 Hz
        ("heartbeat",),                 # HEARTBEAT: send a heartbeat (keep‑alive)
        ("ping",          1),           # PING: send a ping with seq=1 (harmless)
    ]

    current_command_index = 0

    for i in range(0, 110):
        send_command(master, commands[current_command_index])
        time.sleep(1)

    master.close()
    print("All commands tested. Connection closed")

if __name__ == "__main__":
    main()
