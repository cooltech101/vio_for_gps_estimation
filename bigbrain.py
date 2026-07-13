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

# Transformation to convert different camera orientations to NED convention. Replace camera_orientation_default for your configuration.
#   0: Forward, camera oriented standardly
#   1: Downfacing, camera top pointing towards the right
#   2: Forward, 45 degree tilted down
#   3: Downfacing, camera top pointing towards the back
camera_orientation_default = 0

# https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
enable_msg_vision_position_estimate = True
vision_position_estimate_msg_hz_default = 30.0

# https://mavlink.io/en/messages/ardupilotmega.html#VISION_POSITION_DELTA
enable_msg_vision_position_delta = False
vision_position_delta_msg_hz_default = 30.0

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

# In NED frame, offset from the IMU or the center of gravity to the camera's origin point
body_offset_enabled = 0
body_offset_x = 0  # In meters (m)
body_offset_y = 0  # In meters (m)
body_offset_z = 0  # In meters (m)

# Global scale factor, position x y z will be scaled up/down by this factor
scale_factor = 1.0

# Enable using yaw from compass to align north (zero degree is facing north)
compass_enabled = 0

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

H_aeroRef_aeroBody = None
V_aeroRef_aeroBody = None
heading_north_yaw = None
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
parser.add_argument('--vision_position_estimate_msg_hz', type=float)
parser.add_argument('--vision_position_delta_msg_hz', type=float)
parser.add_argument('--vision_speed_estimate_msg_hz', type=float)
parser.add_argument('--scale_calib_enable', default=False, action='store_true')
parser.add_argument('--camera_orientation', type=int)
parser.add_argument('--debug_enable',type=int)
parser.add_argument('--socket', default=DEFAULT_SOCK, help=f"Unix datagram socket path. Default: {DEFAULT_SOCK}")

args = parser.parse_args()

connection_string = args.connect or connection_string_default
connection_baudrate = args.baudrate or connection_baudrate_default
vision_position_estimate_msg_hz = args.vision_position_estimate_msg_hz or vision_position_estimate_msg_hz_default
vision_position_delta_msg_hz = args.vision_position_delta_msg_hz or vision_position_delta_msg_hz_default
vision_speed_estimate_msg_hz = args.vision_speed_estimate_msg_hz or vision_speed_estimate_msg_hz_default
scale_calib_enable = args.scale_calib_enable
camera_orientation = args.camera_orientation or camera_orientation_default
debug_enable = args.debug_enable or 0
sock_path = args.socket

H_aeroRef_OAKRef   = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
if camera_orientation == 0:     
    H_OAKbody_aeroBody = np.linalg.inv(H_aeroRef_OAKRef)
elif camera_orientation == 1:   
    H_OAKbody_aeroBody = np.array([[0,1,0,0],[1,0,0,0],[0,0,-1,0],[0,0,0,1]])
elif camera_orientation == 2:   
    H_OAKbody_aeroBody = (tf.euler_matrix(m.pi/4, 0, 0)).dot(np.linalg.inv(H_aeroRef_OAKRef))
elif camera_orientation == 3:   
    H_OAKbody_aeroBody = np.array([[-1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]])
else:                           
    H_OAKbody_aeroBody = np.linalg.inv(H_aeroRef_OAKRef)

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

        # -------- EXPLICIT MANUAL NED MAPPING --------
        X = raw_py       # bas_y increases forward -> NED X
        Y = raw_px       # bas_x increases right -> NED Y
        Z = -raw_pz      # bas_z increases up -> NED Z goes down
        
        YAW = -raw_yaw + (m.pi / 2)   # bas_yaw increases CCW -> NED Yaw CW
        PITCH = -raw_roll  
        ROLL  = raw_pitch  
        # ---------------------------------------------
        
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





# TEMPORARY
def send_test_vision_position_estimate_message():
    global current_time_us, reset_counter
    with lock:
        # Generate our own timestamp since we are skipping the socket listener
        current_time_us = int(round(time.time() * 1000000))
        
        # Hardcode x, y, z, and yaw to 10
        x, y, z = 10.0, 10.0, 10.0
        roll, pitch = 0.0, 0.0
        
        # NOTE: MAVLink expects radians. 
        # A value of 10 radians will wrap around, but for raw packet inspection, it works. 
        # If you want exactly 10 degrees to show up in the yaw field, use `m.radians(10)` instead.
        yaw = 10.0 
        
        # Create dummy covariance
        cov_pose    = linear_accel_cov * pow(10, 3 - tracker_confidence)
        cov_twist   = angular_vel_cov  * pow(10, 1 - tracker_confidence)
        covariance  = np.array([cov_pose, 0, 0, 0, 0, 0,
                                   cov_pose, 0, 0, 0, 0,
                                      cov_pose, 0, 0, 0,
                                        cov_twist, 0, 0,
                                           cov_twist, 0,
                                              cov_twist])

        # Force send the message
        conn.mav.vision_position_estimate_send(
            current_time_us,            
            x,   
            y,   
            z,   
            roll,	                
            pitch,	                
            yaw,	                
            covariance,                 
            reset_counter               
        )


def send_vision_position_delta_message():
    global current_time_us, current_confidence_level, H_aeroRef_aeroBody
    with lock:
        if H_aeroRef_aeroBody is not None:
            H_aeroRef_PrevAeroBody      = send_vision_position_delta_message.H_aeroRef_PrevAeroBody
            H_PrevAeroBody_CurrAeroBody = (np.linalg.inv(H_aeroRef_PrevAeroBody)).dot(H_aeroRef_aeroBody)

            delta_time_us    = current_time_us - send_vision_position_delta_message.prev_time_us
            delta_position_m = [H_PrevAeroBody_CurrAeroBody[0][3], H_PrevAeroBody_CurrAeroBody[1][3], H_PrevAeroBody_CurrAeroBody[2][3]]
            delta_angle_rad  = np.array( tf.euler_from_matrix(H_PrevAeroBody_CurrAeroBody, 'sxyz'))

            conn.mav.vision_position_delta_send(
                current_time_us, delta_time_us, delta_angle_rad, delta_position_m, current_confidence_level)

            send_vision_position_delta_message.H_aeroRef_PrevAeroBody = H_aeroRef_aeroBody
            send_vision_position_delta_message.prev_time_us = current_time_us

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

def att_msg_callback(value):
    global heading_north_yaw
    if heading_north_yaw is None:
        heading_north_yaw = value.yaw
        progress("INFO: Received first ATTITUDE message with heading yaw %.2f degrees" % m.degrees(heading_north_yaw))

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

mavlink_callbacks = {'ATTITUDE': att_msg_callback}
mavlink_thread = threading.Thread(target=mavlink_loop, args=(conn, mavlink_callbacks))
mavlink_thread.start()

send_msg_to_gcs('Connecting to Unix Datagram socket...')
sock = init_socket(sock_path)
send_msg_to_gcs(f'Listening on {sock_path}')

sched = BackgroundScheduler()

if enable_msg_vision_position_estimate:
    sched.add_job(send_vision_position_estimate_message, 'interval', seconds = 1/vision_position_estimate_msg_hz)


if enable_msg_vision_position_delta:
    sched.add_job(send_vision_position_delta_message, 'interval', seconds = 1/vision_position_delta_msg_hz)
    send_vision_position_delta_message.H_aeroRef_PrevAeroBody = tf.quaternion_matrix([1,0,0,0]) 
    send_vision_position_delta_message.prev_time_us = int(round(time.time() * 1000000))

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

if compass_enabled == 1:
    time.sleep(1)

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
            H_OAKRef_OAKbody = tf.quaternion_matrix([qw, qx, qy, qz]) 
            H_OAKRef_OAKbody[0][3] = px * scale_factor
            H_OAKRef_OAKbody[1][3] = py * scale_factor
            H_OAKRef_OAKbody[2][3] = pz * scale_factor


            # Convert Basalt quaternion to Euler angles (rzyx)
            raw_rpy = tf.euler_from_matrix(tf.quaternion_matrix([qw, qx, qy, qz]), 'rzyx')

            # Update globals
            raw_px, raw_py, raw_pz = px, py, pz
            raw_yaw, raw_pitch, raw_roll = raw_rpy[0], raw_rpy[1], raw_rpy[2] 
            
            vision_data_ready = True

            
            # Transform to aeronautic coordinates (body AND reference frame!)
            H_aeroRef_aeroBody = H_aeroRef_OAKRef.dot( H_OAKRef_OAKbody.dot( H_OAKbody_aeroBody))

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

            if body_offset_enabled == 1:
                H_body_camera = tf.euler_matrix(0, 0, 0, 'sxyz')
                H_body_camera[0][3] = body_offset_x
                H_body_camera[1][3] = body_offset_y
                H_body_camera[2][3] = body_offset_z
                H_camera_body = np.linalg.inv(H_body_camera)
                H_aeroRef_aeroBody = H_body_camera.dot(H_aeroRef_aeroBody.dot(H_camera_body))

            if compass_enabled == 1:
                H_aeroRef_aeroBody = H_aeroRef_aeroBody.dot( tf.euler_matrix(0, 0, heading_north_yaw, 'sxyz'))

            if debug_enable == 1:
                os.system('clear')
                progress("DEBUG: Raw RPY[deg]: {}".format( np.array( tf.euler_from_matrix( H_OAKRef_OAKbody, 'sxyz')) * 180 / m.pi))
                progress("DEBUG: NED RPY[deg]: {}".format( np.array( tf.euler_from_matrix( H_aeroRef_aeroBody, 'sxyz')) * 180 / m.pi))
                progress("DEBUG: Raw pos xyz : {}".format( np.array( [px, py, pz])))
                progress("DEBUG: NED pos xyz : {}".format( np.array( tf.translation_from_matrix( H_aeroRef_aeroBody))))

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
