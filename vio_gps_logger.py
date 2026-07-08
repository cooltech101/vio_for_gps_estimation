#!/usr/bin/env python3
"""
Listen for Basalt VIO pose packets and use initial GPS position to transpose local pose to GPS coordinates.
Read live GPS and yaw from flight controller via MAVLink, and log one CSV row per Basalt sample.
Resulting CSV can show relative performance of both positioning methods. 

For every Basalt packet received, the script logs:
    timestamp,
    basalt VIO latitude estimate,
    basalt VIO longitude estimate,
    basalt VIO x,
    basalt VIO y,
    GPS latitude,
    GPS longitude,
    flight-controller IMU yaw

GPS altitude and VIO z are ignored.

Basalt packet format:
    float[10] = qw qx qy qz px py pz vx vy vz
"""

import argparse
import csv
import math
import os
import signal
import socket
import struct
import threading
import time
import statistics

from pymavlink import mavutil
from geographiclib.geodesic import Geodesic


# -----------------------------
# Defaults
# -----------------------------

# change as necessary
DEFAULT_CONNECTION_STRING = "/dev/ttyACM0"
DEFAULT_BAUD_RATE = 115200

# set in basalt_vio.py
DEFAULT_SOCK = "/tmp/basalt_vio_listener"
# default csv filename
DEFAULT_CSV = "dataLog.csv"

DEFAULT_STREAM_RATE_HZ = 20
DEFAULT_ATTITUDE_RATE_HZ = 20

# Reference initialisation timing
REFERENCE_SETTLE_TIME_S = 3.0
REFERENCE_COLLECTION_TIME_S = 2.0

# fallback reference values for starting pose. script must be run with --use-fallback-reference flag for these to be used
FALLBACK_START_LAT = 0
FALLBACK_START_LON = 0
FALLBACK_START_YAW = 0

_geodesic = Geodesic.WGS84

running = True


# -----------------------------
# Shutdown handling
# -----------------------------

def sigint_handler(sig, frame):
    global running
    running = False


signal.signal(signal.SIGINT, sigint_handler)


# -----------------------------
# Helper functions
# -----------------------------

def median(values):
    if not values:
        return None
    return statistics.median(values)


def circular_mean_rad(angles):
    """
    Circular mean for angles in radians.

    This is safer than a normal median/mean for yaw because yaw wraps
    around at -pi/pi.
    """
    if not angles:
        return None

    sin_sum = sum(math.sin(a) for a in angles)
    cos_sum = sum(math.cos(a) for a in angles)

    return math.atan2(sin_sum, cos_sum)


# -----------------------------
# Shared MAVLink state
# -----------------------------

class LatestMavlinkState:
    def __init__(self):
        self.lock = threading.Lock()

        self.gps_time = None          # companion-computer time.time()
        self.gps_fc_time_usec = None  # GPS_RAW_INT.time_usec
        self.lat = None
        self.lon = None

        self.yaw_time = None          # companion-computer time.time()
        self.yaw_fc_time_boot_ms = None
        self.yaw_rad = None

        self.reference_set = False
        self.reference_lat = None
        self.reference_lon = None
        self.reference_yaw_rad = None

        # Reference initialisation state
        self.first_gps_time = None
        self.reference_collection_started = False
        self.reference_collection_start_time = None
        self.reference_gps_samples = []
        self.reference_yaw_samples = []
        self.ready_message_printed = False

    def update_gps(self, lat, lon, gps_fc_time_usec=None):
        now = time.time()

        with self.lock:
            self.lat = lat
            self.lon = lon
            self.gps_time = now
            self.gps_fc_time_usec = gps_fc_time_usec

            if self.first_gps_time is None:
                self.first_gps_time = now
                print(
                    "[mavlink] First valid GPS sample received. "
                    f"Waiting {REFERENCE_SETTLE_TIME_S:.1f} seconds before "
                    f"collecting reference samples..."
                )

            self._try_collect_reference_locked(now)

    def update_yaw(self, yaw_rad, yaw_fc_time_boot_ms=None):
        now = time.time()

        with self.lock:
            self.yaw_rad = yaw_rad
            self.yaw_time = now
            self.yaw_fc_time_boot_ms = yaw_fc_time_boot_ms

            self._try_collect_reference_locked(now)

    def _try_collect_reference_locked(self, now):
        """
        Reference initialisation logic:

        1. Wait for first GPS sample.
        2. Wait REFERENCE_SETTLE_TIME_S seconds.
        3. Collect GPS/yaw samples for REFERENCE_COLLECTION_TIME_S seconds.
        4. Set reference from median lat/lon and circular mean yaw.
        """
        if self.reference_set:
            return

        if self.first_gps_time is None:
            return

        elapsed_since_first_gps = now - self.first_gps_time

        if elapsed_since_first_gps < REFERENCE_SETTLE_TIME_S:
            return

        if not self.reference_collection_started:
            self.reference_collection_started = True
            self.reference_collection_start_time = now
            print(
                "[mavlink] Reference sample collection started. "
                f"Keep vehicle stationary for {REFERENCE_COLLECTION_TIME_S:.1f} seconds..."
            )

        collection_elapsed = now - self.reference_collection_start_time

        if collection_elapsed <= REFERENCE_COLLECTION_TIME_S:
            if self.lat is not None and self.lon is not None:
                self.reference_gps_samples.append((self.lat, self.lon))

            if self.yaw_rad is not None:
                self.reference_yaw_samples.append(self.yaw_rad)

            return

        if not self.reference_gps_samples:
            print("[mavlink-warning] Reference GPS collection finished but no GPS samples collected.")
            return

        if not self.reference_yaw_samples:
            print("[mavlink-warning] Reference yaw collection finished but no yaw samples collected.")
            return

        lat_samples = [sample[0] for sample in self.reference_gps_samples]
        lon_samples = [sample[1] for sample in self.reference_gps_samples]

        self.reference_lat = median(lat_samples)
        self.reference_lon = median(lon_samples)
        self.reference_yaw_rad = circular_mean_rad(self.reference_yaw_samples)

        self.reference_set = True

        print(
            "\n[mavlink] Reference initialisation complete:"
            f"\n    GPS samples: {len(self.reference_gps_samples)}"
            f"\n    Yaw samples: {len(self.reference_yaw_samples)}"
            f"\n    reference_lat={self.reference_lat:.10f}"
            f"\n    reference_lon={self.reference_lon:.10f}"
            f"\n    reference_yaw_rad={self.reference_yaw_rad:.6f}"
            "\n\n[READY] VIO global reference is set. You can start moving now.\n"
        )

    def snapshot(self):
        with self.lock:
            return {
                "gps_time": self.gps_time,
                "gps_fc_time_usec": self.gps_fc_time_usec,
                "lat": self.lat,
                "lon": self.lon,

                "yaw_time": self.yaw_time,
                "yaw_fc_time_boot_ms": self.yaw_fc_time_boot_ms,
                "yaw_rad": self.yaw_rad,

                "reference_set": self.reference_set,
                "reference_lat": self.reference_lat,
                "reference_lon": self.reference_lon,
                "reference_yaw_rad": self.reference_yaw_rad,

                "first_gps_time": self.first_gps_time,
                "reference_collection_started": self.reference_collection_started,
                "reference_collection_start_time": self.reference_collection_start_time,
                "reference_gps_sample_count": len(self.reference_gps_samples),
                "reference_yaw_sample_count": len(self.reference_yaw_samples),
            }


# -----------------------------
# MAVLink functions
# -----------------------------

def connect_mavlink(connection_string, baud_rate):
    master = mavutil.mavlink_connection(
        connection_string,
        baud=baud_rate
    )

    print(f"[mavlink] Connecting to {connection_string}...")
    print("[mavlink] Waiting for heartbeat...")

    master.wait_heartbeat()

    print("[mavlink] Heartbeat received")
    print(
        f"[mavlink] System ID: {master.target_system}, "
        f"Component ID: {master.target_component}"
    )

    return master


def request_streams(master, rate_hz):
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        rate_hz,
        1
    )

    print(f"[mavlink] Requested MAVLink data streams at {rate_hz} Hz")


def request_message_interval(master, message_id, rate_hz, label):
    interval_us = int(1_000_000 / rate_hz)

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        interval_us,
        0,
        0,
        0,
        0,
        0
    )

    print(f"[mavlink] Requested {label} at {rate_hz} Hz")


def mavlink_reader_thread(master, state: LatestMavlinkState):
    """
    Continuously read MAVLink messages and store latest GPS/yaw.

    CSV logging is driven only by Basalt samples.
    """
    global running

    print("[mavlink] Reader thread started")

    last_gps_fc_time_usec = None
    last_yaw_fc_time_boot_ms = None
    last_debug_print = 0

    while running:
        msg = master.recv_match(
            type=["GPS_RAW_INT", "ATTITUDE"],
            blocking=True,
            timeout=2
        )

        if msg is None:
            print("[mavlink] No GPS_RAW_INT/ATTITUDE")
            continue

        msg_type = msg.get_type()

        if msg_type == "GPS_RAW_INT":
            if msg.fix_type >= 2:
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                gps_fc_time_usec = msg.time_usec

                gps_duplicate = gps_fc_time_usec == last_gps_fc_time_usec
                last_gps_fc_time_usec = gps_fc_time_usec

                state.update_gps(
                    lat=lat,
                    lon=lon,
                    gps_fc_time_usec=gps_fc_time_usec
                )

                now = time.time()
                if now - last_debug_print > 1.0:
                    print(
                        "[mavlink-debug] "
                        f"GPS lat={lat:.7f}, lon={lon:.7f}, "
                        f"time_usec={gps_fc_time_usec}, "
                        f"duplicate={gps_duplicate}"
                    )
                    last_debug_print = now

        elif msg_type == "ATTITUDE":
            yaw_rad = msg.yaw
            yaw_fc_time_boot_ms = msg.time_boot_ms

            yaw_duplicate = yaw_fc_time_boot_ms == last_yaw_fc_time_boot_ms
            last_yaw_fc_time_boot_ms = yaw_fc_time_boot_ms

            state.update_yaw(
                yaw_rad=yaw_rad,
                yaw_fc_time_boot_ms=yaw_fc_time_boot_ms
            )

            now = time.time()
            if now - last_debug_print > 1.0:
                print(
                    "[mavlink-debug] "
                    f"ATTITUDE yaw={yaw_rad:.6f}, "
                    f"time_boot_ms={yaw_fc_time_boot_ms}, "
                    f"duplicate={yaw_duplicate}"
                )
                last_debug_print = now

    print("[mavlink] Reader thread stopped")


# -----------------------------
# CSV functions
# -----------------------------

def init_csv_logger(csv_path: str):
    file_exists = os.path.exists(csv_path)
    file_empty = (not file_exists) or os.path.getsize(csv_path) == 0

    csv_file = open(csv_path, mode="a", newline="")
    csv_writer = csv.writer(csv_file)

    if file_empty:
        csv_writer.writerow([
            "timestamp",
            "gps_timestamp",
            "yaw_timestamp",
            "fc_yaw_rad",
            "vio_x_m",
            "vio_y_m",
            "basalt_lat",
            "basalt_lon",
            "gps_lat",
            "gps_lon"        
        ])
        csv_file.flush()

    return csv_file, csv_writer


def write_csv_row(
    csv_writer,
    timestamp,
    gps_timestamp,
    yaw_timestamp,
    fc_yaw_rad,
    vio_x,
    vio_y,
    basalt_lat,
    basalt_lon,
    gps_lat,
    gps_lon
):
    csv_writer.writerow([
        f"{timestamp:.6f}",
        "" if gps_timestamp is None else f"{gps_timestamp:.6f}",
        "" if yaw_timestamp is None else f"{yaw_timestamp:.6f}",
        "" if fc_yaw_rad is None else f"{fc_yaw_rad:.6f}",
        f"{vio_x:.6f}",
        f"{vio_y:.6f}",
        f"{basalt_lat:.10f}",
        f"{basalt_lon:.10f}",
        "" if gps_lat is None else f"{gps_lat:.10f}",
        "" if gps_lon is None else f"{gps_lon:.10f}",
    ])


# -----------------------------
# Basalt coordinate conversion
# -----------------------------

def translate_relative_xy_to_latlon_geodesic(
    reference_lat: float,
    reference_lon: float,
    vio_x: float,
    vio_y: float,
    reference_yaw_rad: float
) -> tuple[float, float]:
    """
    Convert drone/body-relative x/y metre displacement to latitude/longitude.

    Assumed body-frame convention:
        +vio_x = forward from drone
        +vio_y = right from drone

    Assumed yaw convention:
        0 rad       = north
        pi / 2 rad  = east
        pi rad      = south
        -pi / 2 rad = west
    """

    distance_m = math.hypot(vio_x, vio_y)

    if distance_m == 0:
        return reference_lat, reference_lon

    heading_deg = math.degrees(reference_yaw_rad) % 360
    local_bearing_deg = math.degrees(math.atan2(vio_y, vio_x))
    global_bearing_deg = (heading_deg + local_bearing_deg) % 360

    result = _geodesic.Direct(
        reference_lat,
        reference_lon,
        global_bearing_deg,
        distance_m
    )

    return result["lat2"], result["lon2"]


# -----------------------------
# Main
# -----------------------------

def main():

    parser = argparse.ArgumentParser(
        description=(
            "Log Basalt VIO samples with corresponding latest GPS/yaw from "
            "a MAVLink flight controller."
        )
    )

    parser.add_argument(
        "--socket",
        default=DEFAULT_SOCK,
        help=f"Unix datagram socket path to bind to. Default: {DEFAULT_SOCK}",
    )

    parser.add_argument(
        "--csv",
        default=DEFAULT_CSV,
        help=f"CSV file path. Default: {DEFAULT_CSV}",
    )

    parser.add_argument(
        "--connection",
        default=DEFAULT_CONNECTION_STRING,
        help=f"MAVLink connection string. Default: {DEFAULT_CONNECTION_STRING}",
    )

    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD_RATE,
        help=f"MAVLink baud rate. Default: {DEFAULT_BAUD_RATE}",
    )

    parser.add_argument(
        "--stream-rate",
        type=int,
        default=DEFAULT_STREAM_RATE_HZ,
        help=f"General MAVLink stream rate in Hz. Default: {DEFAULT_STREAM_RATE_HZ}",
    )

    parser.add_argument(
        "--attitude-rate",
        type=int,
        default=DEFAULT_ATTITUDE_RATE_HZ,
        help=f"ATTITUDE/yaw message rate in Hz. Default: {DEFAULT_ATTITUDE_RATE_HZ}",
    )

    parser.add_argument(
        "--use-fallback-reference",
        action="store_true",
        help=(
            "Use hard-coded fallback start lat/lon/yaw until first valid "
            "GPS/yaw reference is received."
        ),
    )

    args = parser.parse_args()

    mav_state = LatestMavlinkState()

    master = connect_mavlink(args.connection, args.baud)

    request_streams(master, args.stream_rate)

    request_message_interval(
        master,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        args.attitude_rate,
        "ATTITUDE/yaw"
    )

    request_message_interval(
        master,
        mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
        args.stream_rate,
        "GPS_RAW_INT"
    )

    mav_thread = threading.Thread(
        target=mavlink_reader_thread,
        args=(master, mav_state),
        daemon=True
    )
    mav_thread.start()

    sock_path = args.socket
    csv_path = args.csv

    try:
        os.unlink(sock_path)
    except FileNotFoundError:
        pass

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.bind(sock_path)
    sock.settimeout(0.5)

    csv_file, csv_writer = init_csv_logger(csv_path)

    packet_size = struct.calcsize("10f")

    print(f"[basalt] Listening on {sock_path}")
    print(f"[logger] Logging CSV to {csv_path}")
    print("[logger] One CSV row will be written per Basalt sample")
    print("[logger] Keep the vehicle stationary until the READY message appears.")
    print("[logger] Press Ctrl+C to stop\n")

    try:
        while running:
            try:
                data, _ = sock.recvfrom(packet_size)
            except socket.timeout:
                continue

            if len(data) != packet_size:
                print(f"[warning] Invalid Basalt packet size: {len(data)} bytes")
                continue

            qw, qx, qy, qz, px, py, pz, vx, vy, vz = struct.unpack("10f", data)

            timestamp = time.time()

            snap = mav_state.snapshot()

            gps_lat = snap["lat"]
            gps_lon = snap["lon"]
            fc_yaw_rad = snap["yaw_rad"]
            gps_timestamp = snap["gps_time"]
            yaw_timestamp = snap["yaw_time"]

            if snap["reference_set"]:
                reference_lat = snap["reference_lat"]
                reference_lon = snap["reference_lon"]
                reference_yaw_rad = snap["reference_yaw_rad"]
            elif args.use_fallback_reference:
                reference_lat = FALLBACK_START_LAT
                reference_lon = FALLBACK_START_LON
                reference_yaw_rad = FALLBACK_START_YAW
            else:
                print(
                    "[logger] Basalt sample received, but GPS/yaw reference "
                    "is not ready yet. Keep vehicle stationary. Skipping sample."
                )
                continue

            # Original mapping from your script:
            #     vio_x = py
            #     vio_y = px
            # pz/VIO z is intentionally ignored.
            vio_x = py
            vio_y = px

            # use starting reference points to translate basalt local pose into gps coordinates
            basalt_lat, basalt_lon = translate_relative_xy_to_latlon_geodesic(
                reference_lat=reference_lat,
                reference_lon=reference_lon,
                vio_x=vio_x,
                vio_y=vio_y,
                reference_yaw_rad=reference_yaw_rad
            )

            # log to csv
            write_csv_row(
                csv_writer=csv_writer,
                timestamp=timestamp,
                gps_timestamp=gps_timestamp,
                yaw_timestamp=yaw_timestamp,
                fc_yaw_rad=fc_yaw_rad,
                vio_x=vio_x,
                vio_y=vio_y,
                basalt_lat=basalt_lat,
                basalt_lon=basalt_lon,
                gps_lat=gps_lat,
                gps_lon=gps_lon
            )

            csv_file.flush()

            print(
                f"basalt_lat={basalt_lat:+.7f}, "
                f"basalt_lon={basalt_lon:+.7f} | "
                f"vio_x={vio_x:+.4f}, "
                f"vio_y={vio_y:+.4f} | "
                f"gps_lat={gps_lat if gps_lat is not None else 'None'}, "
                f"gps_lon={gps_lon if gps_lon is not None else 'None'}, "
                f"yaw={fc_yaw_rad if fc_yaw_rad is not None else 'None'}"
            )

    finally:
        csv_file.close()
        sock.close()

        try:
            os.unlink(sock_path)
        except FileNotFoundError:
            pass

        print("\n[listener-logger] stopped")


if __name__ == "__main__":
    main()
