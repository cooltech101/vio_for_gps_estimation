from pymavlink import mavutil
import math

CONNECTION_STRING = "/dev/ttyACM0"
BAUD_RATE = 115200
STREAM_RATE_HZ = 1


def connect_mavlink(connection_string, baud_rate):
    master = mavutil.mavlink_connection(
        connection_string,
        baud=baud_rate
    )

    print(f"Connecting to {connection_string}...")
    print("Waiting for heartbeat...")

    master.wait_heartbeat()

    print("Heartbeat received")
    print(f"System ID: {master.target_system}, Component ID: {master.target_component}")

    return master


def request_streams(master, rate_hz):
    """
    Request MAVLink data streams at the given rate.
    """
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        rate_hz,
        1
    )

    print(f"Requested MAVLink data streams at {rate_hz} Hz")


def request_yaw_data(master, rate_hz=20):
    """
    Request ATTITUDE messages at the given rate.

    ATTITUDE.yaw is already in radians.
    Range is usually -pi to +pi.
    """
    interval_us = int(1_000_000 / rate_hz)

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        interval_us,
        0,
        0,
        0,
        0,
        0
    )

    print(f"Requested ATTITUDE/yaw data at {rate_hz} Hz")


def get_gps_lat_lon(master, timeout=2):
    """
    Returns GPS latitude and longitude in decimal degrees.

    Returns:
        tuple: (lat, lon), or None if no valid GPS data received.
    """
    msg = master.recv_match(
        type="GPS_RAW_INT",
        blocking=True,
        timeout=timeout
    )

    if msg is None:
        return None

    if msg.fix_type < 2:
        return None

    lat = msg.lat / 1e7
    lon = msg.lon / 1e7

    return lat, lon


def get_yaw_rad(master, timeout=2):
    """
    Returns yaw in radians from ATTITUDE message.

    Returns:
        float: yaw in radians, or None if no ATTITUDE message received.
    """
    msg = master.recv_match(
        type="ATTITUDE",
        blocking=True,
        timeout=timeout
    )

    if msg is None:
        return None

    return msg.yaw


def main():
    master = connect_mavlink(CONNECTION_STRING, BAUD_RATE)

    # General stream request, useful because your setup responds to REQUEST_DATA_STREAM
    request_streams(master, STREAM_RATE_HZ)

    # Explicitly request ATTITUDE at 20 Hz for yaw
    request_yaw_data(master, STREAM_RATE_HZ)

    print("Reading GPS and yaw...")

    latest_lat = None
    latest_lon = None
    latest_yaw = None

    while True:
        msg = master.recv_match(
            type=["GPS_RAW_INT", "ATTITUDE"],
            blocking=True,
            timeout=2
        )

        if msg is None:
            print("No GPS/ATTITUDE message received")
            continue

        msg_type = msg.get_type()

        if msg_type == "GPS_RAW_INT":
            if msg.fix_type >= 2:
                latest_lat = msg.lat / 1e7
                latest_lon = msg.lon / 1e7

        elif msg_type == "ATTITUDE":
            latest_yaw = msg.yaw

        if latest_lat is not None and latest_lon is not None and latest_yaw is not None:
            print(
                f"lat={latest_lat:.7f}, "
                f"lon={latest_lon:.7f}, "
                f"yaw_rad={latest_yaw:.6f}"
            )


if __name__ == "__main__":
    main()
