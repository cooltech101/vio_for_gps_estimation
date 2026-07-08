#!/usr/bin/env python3
"""
BasaltVIO alternative to the VINS-Fusion + feature_tracker stack.

    python3 basalt_vio.py [path/to/oak_d_w.yaml]
"""

import os
import sys
import struct
import socket
import time
import signal
import argparse

import cv2
import rerun as rr
import depthai as dai

# mavlink socket to pass data to listener script
MAVLINK_SOCK = "/tmp/basalt_vio_listener"   
LOCAL_SOCK   = "/tmp/basalt_vio"

CAM_W, CAM_H = 640, 400
CAM_FPS      = 20
IMU_RATE_HZ  = 200

running = True
def _sigint(sig, frame):
    global running
    running = False
signal.signal(signal.SIGINT, _sigint)


def load_imu_noise_from_yaml(yaml_path):
    try:
        fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
        acc_n = float(fs.getNode("acc_n").real())
        gyr_n = float(fs.getNode("gyr_n").real())
        fs.release()
        if acc_n > 0 and gyr_n > 0:
            print(f"[noise] acc_n={acc_n:.6f}  gyr_n={gyr_n:.6f} (from {yaml_path})")
            return acc_n, gyr_n
    except Exception as e:
        print(f"[noise] Could not read yaml ({e}), using Basalt defaults")
    return None, None


def main():
    parser = argparse.ArgumentParser(description="BasaltVIO. Use --rerun flag to activate visualiser")
    parser.add_argument("yaml_path", nargs="?", default=None, help="Path to oak_d_w.yaml.")
    parser.add_argument("--rerun", action="store_true", help="Enable rerun (rr) visualiser")
    args = parser.parse_args()

    yaml_path = args.yaml_path
    #yaml_path = sys.argv[1] if len(sys.argv) > 1 else None
    acc_n, gyr_n = load_imu_noise_from_yaml(yaml_path) if yaml_path else (None, None)
    

    # Unix datagram socket -> mavlink-udp-proxy
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        os.unlink(LOCAL_SOCK)
    except FileNotFoundError:
        pass
    sock.bind(LOCAL_SOCK)

    # Rerun viewer. optional with --rerun flag
    if args.rerun:
        rr.init("basalt_vio", spawn=True)
        rr.log("world", rr.ViewCoordinates.FLU, static=True)
        rr.log("world/ground", rr.Boxes3D(half_sizes=[[3.0, 3.0, 0.00001]]), static=True)


    trajectory = []
    fx = fy = None

    with dai.Pipeline() as p:
        left  = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=CAM_FPS)
        right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=CAM_FPS)
        imu   = p.create(dai.node.IMU)
        odom  = p.create(dai.node.BasaltVIO)

        imu.enableIMUSensor(
            [dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW],
            IMU_RATE_HZ
        )
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)

        if acc_n is not None:
            # BasaltVIO.cpp applies cwiseSqrt() to these values internally
            odom.setAccelNoiseStd([acc_n**2, acc_n**2, acc_n**2])
            odom.setGyroNoiseStd([gyr_n**2, gyr_n**2, gyr_n**2])

        left.requestOutput((CAM_W, CAM_H)).link(odom.left)
        right.requestOutput((CAM_W, CAM_H)).link(odom.right)
        imu.out.link(odom.imu)

        transform_q  = odom.transform.createOutputQueue(maxSize=10, blocking=False)
        passthrough_q = odom.passthrough.createOutputQueue(maxSize=4, blocking=False)

        p.start()
        print("[basalt_vio] running — Ctrl+Z to stop")

        while p.isRunning() and running:
            transform = transform_q.tryGet()
            img_frame = passthrough_q.tryGet()

            if transform is not None:
                t = transform.getTranslation()
                q = transform.getQuaternion()

                # Send to mavlink-udp-proxy: float[10] = qw qx qy qz  px py pz  vx vy vz
                msg = struct.pack('10f',
                    q.qw, q.qx, q.qy, q.qz,
                    t.x, t.y, t.z,
                    0.0, 0.0, 0.0)
                try:
                    sock.sendto(msg, MAVLINK_SOCK)
                except OSError:
                    pass

                # Rerun: camera pose + trajectory
                pos = [t.x, t.y, t.z]
                trajectory.append(pos)
                
                if args.rerun:
                    rr.log("world/camera", rr.Transform3D(
                        translation=pos,
                        rotation=rr.Quaternion(xyzw=[q.qx, q.qy, q.qz, q.qw])
                    ))
                    rr.log("world/trajectory", rr.LineStrips3D([trajectory]))

            if img_frame is not None:
                # Read intrinsics once from device
                if fx is None:
                    calib = p.getDefaultDevice().readCalibration()
                    intr = calib.getCameraIntrinsics(
                        dai.CameraBoardSocket.CAM_B, img_frame.getWidth(), img_frame.getHeight()
                    )
                    fx, fy = intr[0][0], intr[1][1]
                    if args.rerun:
                        rr.log("world/camera/image", rr.Pinhole(
                            resolution=[img_frame.getWidth(), img_frame.getHeight()],
                            focal_length=[fx, fy],
                            camera_xyz=rr.ViewCoordinates.FLU
                        ), static=True)

                img = img_frame.getCvFrame()
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                if args.rerun:
                    rr.log("world/camera/image/rgb", rr.Image(img))

            if transform is None and img_frame is None:
                time.sleep(0.001)

    sock.close()
    try:
        os.unlink(LOCAL_SOCK)
    except FileNotFoundError:
        pass
    print("[basalt_vio] stopped")


if __name__ == "__main__":
    main()
