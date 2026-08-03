#!/usr/bin/env python3

#####################################################
##          OAK-D (Basalt VIO) to MAVLink          ##
##       plus FCU telemetry UDP forwarding          ##
#####################################################

import sys
sys.path.append("/usr/local/lib/")

import os
os.environ["MAVLINK20"] = "1"

import numpy as np
import transformations as tf
import math as m
import time
import argparse
import threading
import signal
import socket
import struct

from apscheduler.schedulers.background import BackgroundScheduler
from pymavlink import mavutil


def progress(string):
    print(string, file=sys.stdout)
    sys.stdout.flush()


#######################################
# Parameters
#######################################

DEFAULT_SOCK = "/tmp/basalt_vio_listener"

connection_string_default = '/dev/ttyAMA0'
connection_baudrate_default = 921600
connection_timeout_sec_default = 5

enable_msg_vision_position_estimate = True
vision_position_estimate_msg_hz_default = 30.0

enable_msg_vision_speed_estimate = True
vision_speed_estimate_msg_hz_default = 30.0

enable_update_tracking_confidence_to_gcs = True
update_tracking_confidence_to_gcs_hz_default = 1.0

enable_user_keyboard_input = False

enable_auto_set_ekf_home = False
home_lat = 1
home_lon = 103
home_alt = 10

scale_factor = 1.0

pose_data_confidence_level = ('FAILED', 'Low', 'Medium', 'High')
tracker_confidence = 3

lock = threading.Lock()
mavlink_thread_should_exit = False
exit_code = 1


#######################################
# Global variables
#######################################

linear_accel_cov = 0.01
angular_vel_cov = 0.01

V_aeroRef_aeroBody = None
current_confidence_level = 100.0
current_time_us = 0
raw_px = raw_py = raw_pz = 0.0
raw_roll = raw_pitch = raw_yaw = 0.0
vision_data_ready = False

prev_data = None
reset_counter = 1

main_loop_should_quit = False


#######################################
# Parse arguments
#######################################

parser = argparse.ArgumentParser(description='OAK-D VIO MAVLink Bridge with telemetry forwarding')

parser.add_argument('--connect', help="Vehicle connection target string.")
parser.add_argument('--baudrate', type=int, help="Vehicle connection baudrate.")
parser.add_argument('--scale_calib_enable', default=False, action='store_true')
parser.add_argument('--debug_enable', type=int)
parser.add_argument('--socket', default=DEFAULT_SOCK,
                    help=f"Unix datagram socket path. Default: {DEFAULT_SOCK}")

parser.add_argument(
    '--out',
    action='append',
    default=[],
    help=(
        "UDP telemetry output endpoint. Can be used multiple times. "
        "Examples: --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14556"
    )
)

parser.add_argument(
    '--streamrate',
    type=int,
    default=4,
    help="Requested MAVLink telemetry stream rate in Hz. Default: 4"
)

parser.add_argument(
    '--source-system',
    type=int,
    default=1,
    help="MAVLink source system for this companion computer. Default: 1"
)

parser.add_argument(
    '--source-component',
    type=int,
    default=93,
    help="MAVLink source component for this companion computer. Default: 93"
)

args = parser.parse_args()

connection_string = args.connect or connection_string_default
connection_baudrate = args.baudrate or connection_baudrate_default

vision_position_estimate_msg_hz = vision_position_estimate_msg_hz_default
vision_speed_estimate_msg_hz = vision_speed_estimate_msg_hz_default
scale_calib_enable = args.scale_calib_enable
debug_enable = args.debug_enable or 0
sock_path = args.socket

H_aeroRef_OAKRef = np.array([
    [-1, 0,  0, 0],
    [0,  1,  0, 0],
    [0,  0, -1, 0],
    [0,  0,  0, 1]
])
H_OAKBody_aeroBody = np.linalg.inv(H_aeroRef_OAKRef)

if debug_enable == 1:
    np.set_printoptions(precision=4, suppress=True)


#######################################
# UDP forwarding helpers
#######################################

def parse_udp_out(out_string):
    """
    Accepts:
      udp:127.0.0.1:14550
      127.0.0.1:14550
      udpout:127.0.0.1:14550

    Returns:
      (host, port)
    """
    s = out_string.strip()

    if s.startswith("udp:"):
        s = s[4:]
    elif s.startswith("udpout:"):
        s = s[7:]

    if ":" not in s:
        raise ValueError(f"Bad --out value '{out_string}'. Expected udp:HOST:PORT")

    host, port_str = s.rsplit(":", 1)
    return host, int(port_str)


class UDPForwarder:
    def __init__(self, out_strings):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.outputs = []

        for out_string in out_strings:
            host, port = parse_udp_out(out_string)
            self.outputs.append((host, port))

        if self.outputs:
            progress("INFO: UDP telemetry outputs:")
            for host, port in self.outputs:
                progress(f"INFO:   udp:{host}:{port}")

    def send(self, packet_bytes):
        for endpoint in self.outputs:
            try:
                self.sock.sendto(packet_bytes, endpoint)
            except Exception as e:
                progress(f"WARNING: Failed forwarding MAVLink packet to {endpoint}: {e}")

    def close(self):
        self.sock.close()


#######################################
# MAVLink functions
#######################################

def request_all_telemetry_streams(conn, rate_hz):
    """
    MAVProxy-like telemetry request:
    REQUEST_DATA_STREAM with MAV_DATA_STREAM_ALL.
    """
    if rate_hz < 0:
        progress("INFO: streamrate < 0, not requesting telemetry streams")
        return

    target_system = conn.target_system or 1
    target_component = conn.target_component or 1

    progress(
        f"INFO: Requesting MAV_DATA_STREAM_ALL at {rate_hz} Hz "
        f"from sysid={target_system}, compid={target_component}"
    )

    conn.mav.request_data_stream_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        rate_hz,
        1
    )


def mavlink_loop(conn, udp_forwarder):
    """
    Single MAVLink receive loop.

    Responsibilities:
      1. Send companion-computer heartbeats.
      2. Request all telemetry streams periodically.
      3. Read MAVLink packets from FCU.
      4. Forward raw FCU MAVLink packets to UDP outputs.
    """
    heartbeat_interval = 1.0
    stream_request_interval = 2.0

    last_heartbeat = 0.0
    last_stream_request = 0.0

    while not mavlink_thread_should_exit:
        now = time.time()

        if now - last_heartbeat >= heartbeat_interval:
            try:
                conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                    0,
                    0,
                    0
                )
            except Exception as e:
                progress(f"WARNING: Failed sending heartbeat: {e}")

            last_heartbeat = now

        if now - last_stream_request >= stream_request_interval:
            try:
                request_all_telemetry_streams(conn, args.streamrate)
            except Exception as e:
                progress(f"WARNING: Failed requesting telemetry streams: {e}")

            last_stream_request = now

        try:
            msg = conn.recv_match(blocking=True, timeout=0.2)
        except Exception as e:
            progress(f"WARNING: MAVLink receive error: {e}")
            continue

        if msg is None:
            continue

        msg_type = msg.get_type()
        if msg_type == "BAD_DATA":
            continue

        try:
            packet = msg.get_msgbuf()
            if packet:
                udp_forwarder.send(packet)
        except Exception as e:
            progress(f"WARNING: Failed forwarding MAVLink message {msg_type}: {e}")


def send_vision_position_estimate_message():
    global current_time_us, reset_counter
    global raw_px, raw_py, raw_pz, raw_roll, raw_pitch, raw_yaw, vision_data_ready

    with lock:
        if not vision_data_ready:
            return

        H_OAKRef_OAKBody = tf.euler_matrix(raw_yaw, raw_pitch, raw_roll, 'rzyx')
        H_OAKRef_OAKBody[0][3] = raw_px
        H_OAKRef_OAKBody[1][3] = raw_py
        H_OAKRef_OAKBody[2][3] = raw_pz

        H_aeroRef_aeroBody = H_aeroRef_OAKRef.dot(H_OAKRef_OAKBody).dot(H_OAKBody_aeroBody)

        X = H_aeroRef_aeroBody[0][3]
        Y = H_aeroRef_aeroBody[1][3]
        Z = H_aeroRef_aeroBody[2][3]

        YAW, PITCH, ROLL = tf.euler_from_matrix(H_aeroRef_aeroBody, 'rzyx')

        cov_pose = linear_accel_cov * pow(10, 3 - tracker_confidence)
        cov_twist = angular_vel_cov * pow(10, 1 - tracker_confidence)

        covariance = np.array([
            cov_pose, 0, 0, 0, 0, 0,
            cov_pose, 0, 0, 0, 0,
            cov_pose, 0, 0, 0,
            cov_twist, 0, 0,
            cov_twist, 0,
            cov_twist
        ])

        conn.mav.vision_position_estimate_send(
            current_time_us,
            X, Y, Z,
            ROLL, PITCH, YAW,
            covariance,
            reset_counter
        )


def send_vision_speed_estimate_message():
    global current_time_us, V_aeroRef_aeroBody, reset_counter

    with lock:
        if V_aeroRef_aeroBody is not None:
            cov_pose = linear_accel_cov * pow(10, 3 - tracker_confidence)

            covariance = np.array([
                cov_pose, 0, 0,
                0, cov_pose, 0,
                0, 0, cov_pose
            ])

            conn.mav.vision_speed_estimate_send(
                current_time_us,
                V_aeroRef_aeroBody[0][3],
                V_aeroRef_aeroBody[1][3],
                V_aeroRef_aeroBody[2][3],
                covariance,
                reset_counter
            )


def update_tracking_confidence_to_gcs():
    if update_tracking_confidence_to_gcs.prev_confidence_level != tracker_confidence:
        confidence_status_string = 'Tracking confidence: ' + pose_data_confidence_level[tracker_confidence]
        send_msg_to_gcs(confidence_status_string)
        update_tracking_confidence_to_gcs.prev_confidence_level = tracker_confidence


def send_msg_to_gcs(text_to_be_sent):
    text_msg = 'OAK-D: ' + text_to_be_sent
    conn.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text_msg.encode())
    progress("INFO: %s" % text_to_be_sent)


def set_default_global_origin():
    conn.mav.set_gps_global_origin_send(1, home_lat, home_lon, home_alt)


def set_default_home_position():
    conn.mav.set_home_position_send(
        1,
        home_lat,
        home_lon,
        home_alt,
        0,
        0,
        0,
        [1, 0, 0, 0],
        0,
        0,
        1
    )


def increment_reset_counter():
    global reset_counter

    if reset_counter >= 255:
        reset_counter = 1

    reset_counter += 1


#######################################
# Miscellaneous
#######################################

def user_input_monitor():
    global scale_factor

    while True:
        if scale_calib_enable is True:
            scale_factor = float(input("INFO: Type in new scale as float number\n"))
            progress("INFO: New scale is %s" % scale_factor)

        if enable_auto_set_ekf_home:
            send_msg_to_gcs('Set EKF home with default GPS location')
            set_default_global_origin()
            set_default_home_position()
            time.sleep(1)

        try:
            c = input()
            if c == "":
                send_msg_to_gcs('Set EKF home with default GPS location')
                set_default_global_origin()
                set_default_home_position()
        except IOError:
            pass


#######################################
# Socket Initialisation
#######################################

def init_socket(sock_path):
    try:
        os.unlink(sock_path)
    except FileNotFoundError:
        pass

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.bind(sock_path)
    sock.settimeout(0.5)
    return sock


#######################################
# Signal handlers
#######################################

def sigint_handler(sig, frame):
    global main_loop_should_quit
    main_loop_should_quit = True


def sigterm_handler(sig, frame):
    global main_loop_should_quit
    global exit_code
    main_loop_should_quit = True
    exit_code = 0


signal.signal(signal.SIGINT, sigint_handler)
signal.signal(signal.SIGTERM, sigterm_handler)


#######################################
# Main
#######################################

progress("INFO: Starting Vehicle communications")
progress(f"INFO: Connecting to {connection_string} at {connection_baudrate} baud")

conn = mavutil.mavlink_connection(
    connection_string,
    autoreconnect=True,
    source_system=args.source_system,
    source_component=args.source_component,
    baud=connection_baudrate,
    force_connected=True,
)

udp_forwarder = UDPForwarder(args.out)

progress("INFO: Waiting for FCU heartbeat...")
try:
    conn.wait_heartbeat(timeout=connection_timeout_sec_default)
    progress(
        f"INFO: Heartbeat received from sysid={conn.target_system}, "
        f"compid={conn.target_component}"
    )
except Exception:
    progress("WARNING: No heartbeat received before timeout; continuing anyway")

try:
    request_all_telemetry_streams(conn, args.streamrate)
except Exception as e:
    progress(f"WARNING: Initial telemetry request failed: {e}")

mavlink_thread = threading.Thread(
    target=mavlink_loop,
    args=(conn, udp_forwarder),
    daemon=True
)
mavlink_thread.start()

send_msg_to_gcs('Connecting to Unix Datagram socket...')
sock = init_socket(sock_path)
send_msg_to_gcs(f'Listening on {sock_path}')

sched = BackgroundScheduler()

if enable_msg_vision_position_estimate:
    sched.add_job(
        send_vision_position_estimate_message,
        'interval',
        seconds=1 / vision_position_estimate_msg_hz
    )

if enable_msg_vision_speed_estimate:
    sched.add_job(
        send_vision_speed_estimate_message,
        'interval',
        seconds=1 / vision_speed_estimate_msg_hz
    )

if enable_update_tracking_confidence_to_gcs:
    sched.add_job(
        update_tracking_confidence_to_gcs,
        'interval',
        seconds=1 / update_tracking_confidence_to_gcs_hz_default
    )
    update_tracking_confidence_to_gcs.prev_confidence_level = -1

if enable_user_keyboard_input:
    user_keyboard_input_thread = threading.Thread(target=user_input_monitor)
    user_keyboard_input_thread.daemon = True
    user_keyboard_input_thread.start()

sched.start()

send_msg_to_gcs('Sending vision messages to FCU')
packet_size = struct.calcsize("10f")

try:
    while not main_loop_should_quit:
        try:
            data, _ = sock.recvfrom(packet_size)
        except socket.timeout:
            progress("DEBUG: Socket timeout - waiting for OAK-D Basalt data...")
            continue

        if len(data) != packet_size:
            progress(f"DEBUG: Packet size mismatch! Expected {packet_size} bytes, got {len(data)} bytes.")
            continue

        qw, qx, qy, qz, px, py, pz, vx, vy, vz = struct.unpack("10f", data)

        with lock:
            current_time_us = int(round(time.time() * 1000000))

            raw_rpy = tf.euler_from_matrix(
                tf.quaternion_matrix([qw, qx, qy, qz]),
                'rzyx'
            )

            raw_px = px * scale_factor
            raw_py = py * scale_factor
            raw_pz = pz * scale_factor

            raw_yaw = raw_rpy[0]
            raw_pitch = raw_rpy[1]
            raw_roll = raw_rpy[2]

            vision_data_ready = True

            V_aeroRef_aeroBody = tf.quaternion_matrix([1, 0, 0, 0])
            V_aeroRef_aeroBody[0][3] = vx
            V_aeroRef_aeroBody[1][3] = vy
            V_aeroRef_aeroBody[2][3] = vz
            V_aeroRef_aeroBody = H_aeroRef_OAKRef.dot(V_aeroRef_aeroBody)

            curr_data_tuple = (px, py, pz, vx, vy, vz)

            if prev_data is not None:
                delta_translation = [
                    px - prev_data[0],
                    py - prev_data[1],
                    pz - prev_data[2]
                ]

                delta_velocity = [
                    vx - prev_data[3],
                    vy - prev_data[4],
                    vz - prev_data[5]
                ]

                position_displacement = np.linalg.norm(delta_translation)
                speed_delta = np.linalg.norm(delta_velocity)

                jump_threshold = 0.1
                jump_speed_threshold = 20.0

                if (position_displacement > jump_threshold) or (speed_delta > jump_speed_threshold):
                    send_msg_to_gcs('VISO jump detected')

                    if position_displacement > jump_threshold:
                        progress("Position jumped by: %s" % position_displacement)
                    elif speed_delta > jump_speed_threshold:
                        progress("Speed jumped by: %s" % speed_delta)

                    increment_reset_counter()

            prev_data = curr_data_tuple

            if debug_enable == 1:
                os.system('clear')
                progress("DEBUG: Raw RPY[deg]: {}".format(np.array(raw_rpy) * 180 / m.pi))
                progress("DEBUG: Raw pos xyz : {}".format(np.array([px, py, pz])))

except Exception as e:
    progress(e)

except:
    send_msg_to_gcs('ERROR IN SCRIPT')
    progress("Unexpected error: %s" % sys.exc_info()[0])

finally:
    progress('Closing the script...')

    mavlink_thread_should_exit = True

    try:
        mavlink_thread.join(timeout=2)
    except Exception:
        pass

    try:
        sched.shutdown(wait=False)
    except Exception:
        pass

    try:
        conn.close()
    except Exception:
        pass

    try:
        udp_forwarder.close()
    except Exception:
        pass

    try:
        sock.close()
    except Exception:
        pass

    try:
        os.unlink(sock_path)
    except FileNotFoundError:
        pass

    progress("INFO: Socket, UDP forwarder, and vehicle object closed.")
    sys.exit(exit_code)
