#!/usr/bin/env python3
"""
Listen for BasaltVIO pose packets on a Unix datagram socket and print them.

The sender is expected to send:
    float[10] = qw qx qy qz  px py pz  vx vy vz

Usage:
    python3 listen_basalt_vio.py

Optional:
    python3 listen_basalt_vio.py --socket /tmp/basalt_vio_listener
"""

import os
import socket
import struct
import argparse
import signal
import time
import math


from geographiclib.geodesic import Geodesic
_geodesic = Geodesic.WGS84


DEFAULT_SOCK = "/tmp/basalt_vio_listener"

running = True


def sigint_handler(sig, frame):
    global running
    running = False


signal.signal(signal.SIGINT, sigint_handler)



def translate_relative_xyz_to_latlonalt_geodesic(
    reference_lat: float,
    reference_lon: float,
    reference_alt: float,
    vio_x: float,
    vio_y: float,
    vio_z: float,
    reference_yaw_rad: float
) -> tuple[float, float, float]:
    """
    Convert drone-relative x/y metre displacement to lat/lon using GeographicLib.

    Drone-relative convention:
        +vio_x = forward from drone
        +vio_y = right from drone

    DroneKit yaw:
        reference_yaw_rad = vehicle.attitude.yaw, in radians

    Assumed yaw convention:
        0 rad       = facing north
        pi / 2 rad  = facing east
        pi rad      = facing south
        -pi / 2 rad = facing west

    GeographicLib bearing convention:
        0 deg  = north
        90 deg = east
    """

    distance_m = math.hypot(vio_x, vio_y)

    if distance_m == 0:
        return reference_lat, reference_lon

    heading_deg = math.degrees(reference_yaw_rad) % 360

    # Direction of the local/body-frame displacement relative to drone forward
    local_bearing_deg = math.degrees(math.atan2(vio_y, vio_x))

    # Convert local/body bearing to global bearing
    global_bearing_deg = (heading_deg + local_bearing_deg) % 360

    result = _geodesic.Direct(
        reference_lat,
        reference_lon,
        global_bearing_deg,
        distance_m
    )

    alt2 = reference_alt + vio_z

    return result["lat2"], result["lon2"], alt2



def main():
    parser = argparse.ArgumentParser(
        description="Listen for BasaltVIO location and pose packets."
    )
    parser.add_argument(
        "--socket",
        default=DEFAULT_SOCK,
        help=f"Unix datagram socket path to bind to. Default: {DEFAULT_SOCK}",
    )
    args = parser.parse_args()

    sock_path = args.socket

    try:
        os.unlink(sock_path)
    except FileNotFoundError:
        pass

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.bind(sock_path)
    sock.settimeout(0.5)

    print(f"[listener] Listening on {sock_path}")
    print("[listener] Waiting for packets...")
    print("[listener] Press Ctrl+C to stop\n")

    packet_size = struct.calcsize("10f")

    try:
        while running:
            try:
                data, _ = sock.recvfrom(packet_size)
            except socket.timeout:
                continue

            if len(data) != packet_size:
                print(f"[warning] Invalid packet size: {len(data)} bytes")
                continue

            qw, qx, qy, qz, px, py, pz, vx, vy, vz = struct.unpack("10f", data)

            timestamp = time.time()

            new_lat, new_lon, new_alt = translate_relative_xyz_to_latlonalt_geodesic(
                reference_lat=55.000,
                reference_lon=55.000,
                reference_alt=1,
                vio_x=py,
                vio_y=px,
                vio_z=pz,
                reference_yaw_rad=0
            )

            print(
                f"t={timestamp:.3f} | "
                f"pos [m]: x={px:+.4f}, y={py:+.4f}, z={pz:+.4f} | "
                f"quat: qw={qw:+.5f}, qx={qx:+.5f}, qy={qy:+.5f}, qz={qz:+.5f} | "
                f"vel [m/s]: vx={vx:+.4f}, vy={vy:+.4f}, vz={vz:+.4f}"
            )

            print (
                f"NEW LAT: {new_lat:+.7f} |"
                f"NEW LON: {new_lon:+.7f} |"
                f"NEW ALT: {new_alt:+.7f}"
            )

    finally:
        sock.close()
        try:
            os.unlink(sock_path)
        except FileNotFoundError:
            pass

        print("\n[listener] stopped")


if __name__ == "__main__":
    main()
