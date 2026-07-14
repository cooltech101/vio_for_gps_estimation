#!/usr/bin/env python3

#####################################################
##          OAK-D (Basalt VIO) to MAVLink          ##
#####################################################
# This script listens for OAK-D VIO packets via Unix Datagram socket
# and forwards them to ArduPilot via MAVLink.
#
# Install required packages: 
#   pip3 install transformations
#   pip3 install pymavlink
#   pip3 install apscheduler
#   pip3 install pyserial

import sys
sys.path.append("/usr/local/lib/")

# Set MAVLink protocol to 2.
import os
os.environ["MAVLINK20"] = "1"

# Import the libraries
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

# Replacement of the standard print() function to flush the output
def progress(string):
    print(string, file=sys.stdout)
    sys.stdout.flush()

#######################################
# Parameters
#######################################

# Unix Datagram socket setting
DEFAULT_SOCK = "/tmp/basalt_vio_listener"

# Default configurations for connection to the FCU
connection_string_default = '/dev/ttyAMA0'
connection_baudrate_default = 921600
connection_timeout_sec_default = 5

# https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
enable_msg_vision_position_estimate = True
vision_position_estimate_msg_hz_default = 30.0

# https://mavlink.io/en/messages/common.html#VISION_SPEED_ESTIMATE
enable_msg_vision_speed_estimate = True
vision_speed_estimate_msg_hz_default = 30.0

# https://mavlink.io/en/messages/common.html#STATUSTEXT
enable_update_tracking_confidence_to_gcs = True
update_tracking_confidence_to_gcs_hz_default = 1.0

# Monitor user's online input via keyboard, can only be used when runs from terminal
enable_user_keyboard_input = False

# Default global position for EKF home/ origin
enable_auto_set_ekf_home = False
home_lat = 1    # Somewhere random
home_lon = 103     # Somewhere random
home_alt = 10       # Somewhere random

# Global scale factor, position x y z will be scaled up/down by this factor
scale_factor = 1.0

# pose data confidence: 0x0 - Failed / 0x1 - Low / 0x2 - Medium / 0x3 - High 
pose_data_confidence_level = ('FAILED', 'Low', 'Medium', 'High')
tracker_confidence = 3 # Basalt doesn't output confidence currently, assume High

# lock for thread synchronization
lock = threading.Lock()
mavlink_thread_should_exit = False
exit_code = 1

#######################################
# Global variables
#######################################

linear_accel_cov = 0.01
angular_vel_cov  = 0.01

V_aeroRef_aeroBody = None
current_confidence_level = 100.0
current_time_us = 0
raw_px = raw_py = raw_pz = 0.0
raw_roll = raw_pitch = raw_yaw = 0.0
vision_data_ready = False


prev_data = None # Used for jump detection

reset_counter = 1

#######################################
# Parsing user' inputs
#######################################

parser = argparse.ArgumentParser(description='OAK-D VIO MAVLink Bridge')
parser.add_argument('--connect', help="Vehicle connection target string.")
parser.add_argument('--baudrate', type=float, help="Vehicle connection baudrate.")
parser.add_argument('--scale_calib_enable', default=False, action='store_true')
parser.add_argument('--debug_enable',type=int)
parser.add_argument('--socket', default=DEFAULT_SOCK, help=f"Unix datagram socket path. Default: {DEFAULT_SOCK}")

args = parser.parse_args()

connection_string = args.connect or connection_string_default
connection_baudrate = args.baudrate or connection_baudrate_default
vision_position_estimate_msg_hz = vision_position_estimate_msg_hz_default
vision_speed_estimate_msg_hz = vision_speed_estimate_msg_hz_default
scale_calib_enable = args.scale_calib_enable
debug_enable = args.debug_enable or 0
sock_path = args.socket

H_aeroRef_OAKRef = np.array([[0,1,0,0],[1,0,0,0],[0,0,-1,0],[0,0,0,1]])
H_OAKBody_aeroBody = np.linalg.inv(H_aeroRef_OAKRef)

if debug_enable == 1:
    np.set_printoptions(precision=4, suppress=True)

#######################################
# Functions - MAVLink
#######################################

def mavlink_loop(conn, callbacks):
    interesting_messages = list(callbacks.keys())
    while not mavlink_thread_should_exit:
        conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                                0, 0, 0)
        m = conn.recv_match(type=interesting_messages, timeout=1, blocking=True)
        if m is None:
            continue
        callbacks[m.get_type()](m)


def send_vision_position_estimate_message():
    global current_time_us, reset_counter
    global raw_px, raw_py, raw_pz, raw_roll, raw_pitch, raw_yaw, vision_data_ready
    
    with lock:
        if not vision_data_ready:
            return # Don't send anything until the first packet arrives

        # -------- CONSISTENT NED TRANSFORM (position + attitude) --------
        # H_aeroRef_OAKRef encodes the verified Basalt axes (bas_x=right,
        # bas_y=forward, bas_z=up). Rebuild the body rotation from the
        # already-decomposed Euler angles (exact inverse of euler_from_matrix
        # with the same 'rzyx' convention), attach the raw position as the
        # translation, then conjugate by H_aeroRef_OAKRef so position and
        # attitude go through the SAME frame transform as the velocity does.
        H_OAKRef_OAKBody = tf.euler_matrix(raw_yaw, raw_pitch, raw_roll, 'rzyx')
        H_OAKRef_OAKBody[0][3] = raw_px
        H_OAKRef_OAKBody[1][3] = raw_py
        H_OAKRef_OAKBody[2][3] = raw_pz

        H_aeroRef_aeroBody = H_aeroRef_OAKRef.dot(H_OAKRef_OAKBody).dot(H_OAKBody_aeroBody)

        X, Y, Z = H_aeroRef_aeroBody[0][3], H_aeroRef_aeroBody[1][3], H_aeroRef_aeroBody[2][3]
        YAW, PITCH, ROLL = tf.euler_from_matrix(H_aeroRef_aeroBody, 'rzyx')
        # ------------------------------------------------------------------
        
        cov_pose = linear_accel_cov * pow(10, 3 - tracker_confidence)
        cov_twist = angular_vel_cov * pow(10, 1 - tracker_confidence)
        covariance = np.array([cov_pose, 0, 0, 0, 0, 0,
                               cov_pose, 0, 0, 0, 0,
                               cov_pose, 0, 0, 0,
                               cov_twist, 0, 0,
                               cov_twist, 0,
                               cov_twist])

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
            cov_pose    = linear_accel_cov * pow(10, 3 - tracker_confidence)
            covariance  = np.array([cov_pose, 0, 0, 0, cov_pose, 0, 0, 0, cov_pose])
            
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
    conn.mav.set_home_position_send(1, home_lat, home_lon, home_alt, 0, 0, 0, [1, 0, 0, 0], 0, 0, 1)

def increment_reset_counter():
    global reset_counter
    if reset_counter >= 255: reset_counter = 1
    reset_counter += 1

#######################################
# Functions - Miscellaneous
#######################################

def user_input_monitor():
    global scale_factor
    while True:
        if scale_calib_enable == True:
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
        except IOError: pass

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
# Main code starts here
#######################################

progress("INFO: Starting Vehicle communications")
conn = mavutil.mavlink_connection(
    connection_string, autoreconnect = True, source_system = 1,
    source_component = 93, baud=connection_baudrate, force_connected=True,
)

mavlink_callbacks = {}
mavlink_thread = threading.Thread(target=mavlink_loop, args=(conn, mavlink_callbacks))
mavlink_thread.start()

send_msg_to_gcs('Connecting to Unix Datagram socket...')
sock = init_socket(sock_path)
send_msg_to_gcs(f'Listening on {sock_path}')

sched = BackgroundScheduler()

if enable_msg_vision_position_estimate:
    sched.add_job(send_vision_position_estimate_message, 'interval', seconds = 1/vision_position_estimate_msg_hz)


if enable_msg_vision_speed_estimate:
    sched.add_job(send_vision_speed_estimate_message, 'interval', seconds = 1/vision_speed_estimate_msg_hz)

if enable_update_tracking_confidence_to_gcs:
    sched.add_job(update_tracking_confidence_to_gcs, 'interval', seconds = 1/update_tracking_confidence_to_gcs_hz_default)
    update_tracking_confidence_to_gcs.prev_confidence_level = -1

if enable_user_keyboard_input:
    user_keyboard_input_thread = threading.Thread(target=user_input_monitor)
    user_keyboard_input_thread.daemon = True
    user_keyboard_input_thread.start()

sched.start()

main_loop_should_quit = False
def sigint_handler(sig, frame):
    global main_loop_should_quit
    main_loop_should_quit = True
signal.signal(signal.SIGINT, sigint_handler)

def sigterm_handler(sig, frame):
    global main_loop_should_quit
    main_loop_should_quit = True
    global exit_code
    exit_code = 0
signal.signal(signal.SIGTERM, sigterm_handler)

send_msg_to_gcs('Sending vision messages to FCU')
packet_size = struct.calcsize("10f")

try:
    while not main_loop_should_quit:
        try:
            data, _ = sock.recvfrom(packet_size)
        except socket.timeout:
            # This means the script is listening, but Basalt isn't sending anything
            progress("DEBUG: Socket timeout - waiting for OAK-D Basalt data...")
            continue
            
        if len(data) != packet_size:
            # This means data is arriving, but the byte size is wrong
            progress(f"DEBUG: Packet size mismatch! Expected {packet_size} bytes, got {len(data)} bytes.")
            continue

        # Unpack the Basalt packet: qw, qx, qy, qz, px, py, pz, vx, vy, vz
        qw, qx, qy, qz, px, py, pz, vx, vy, vz = struct.unpack("10f", data)

        
        with lock:
            current_time_us = int(round(time.time() * 1000000))


            # In transformations, Quaternions w+ix+jy+kz are represented as [w, x, y, z]!
            # Convert Basalt quaternion to Euler angles (rzyx)
            raw_rpy = tf.euler_from_matrix(tf.quaternion_matrix([qw, qx, qy, qz]), 'rzyx')

            # Update globals
            raw_px, raw_py, raw_pz = px * scale_factor, py * scale_factor, pz * scale_factor
            raw_yaw, raw_pitch, raw_roll = raw_rpy[0], raw_rpy[1], raw_rpy[2]

            vision_data_ready = True

            # Calculate GLOBAL XYZ speed
            V_aeroRef_aeroBody = tf.quaternion_matrix([1,0,0,0])
            V_aeroRef_aeroBody[0][3] = vx
            V_aeroRef_aeroBody[1][3] = vy
            V_aeroRef_aeroBody[2][3] = vz
            V_aeroRef_aeroBody = H_aeroRef_OAKRef.dot(V_aeroRef_aeroBody)

            # Detect jumps logic
            curr_data_tuple = (px, py, pz, vx, vy, vz)
            if prev_data != None:
                delta_translation = [px - prev_data[0], py - prev_data[1], pz - prev_data[2]]
                delta_velocity = [vx - prev_data[3], vy - prev_data[4], vz - prev_data[5]]
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
                progress("DEBUG: Raw RPY[deg]: {}".format( np.array(raw_rpy) * 180 / m.pi))
                progress("DEBUG: Raw pos xyz : {}".format( np.array( [px, py, pz])))

except Exception as e:
    progress(e)
except:
    send_msg_to_gcs('ERROR IN SCRIPT')  
    progress("Unexpected error: %s" % sys.exc_info()[0])
finally:
    progress('Closing the script...')
    mavlink_thread_should_exit = True
    mavlink_thread.join()
    conn.close()
    
    sock.close()
    try:
        os.unlink(sock_path)
    except FileNotFoundError:
        pass
        
    progress("INFO: Socket and vehicle object closed.")
    sys.exit(exit_code)