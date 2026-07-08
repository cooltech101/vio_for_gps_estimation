from pymavlink import mavutil
import time

connection_string = "/dev/ttyACM0"
baud_rate = 115200

master = mavutil.mavlink_connection(connection_string, baud=baud_rate)

print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received")

# Request all streams at 20 Hz
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    20,
    1
)

last_gps_time = None

while True:
    msg = master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2)

    if msg is None:
        print("No GPS_RAW_INT")
        continue

    lat = msg.lat / 1e7
    lon = msg.lon / 1e7

    # msg.time_usec is the GPS message timestamp from PX4/GPS path
    duplicate = msg.time_usec == last_gps_time
    last_gps_time = msg.time_usec

    print(
        f"lat={lat:.7f}, lon={lon:.7f}, "
        f"time_usec={msg.time_usec}, duplicate={duplicate}"
    )
