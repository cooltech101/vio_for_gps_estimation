#!/usr/bin/env python3
"""
Combined BasaltVIO + Spatial Object Tracking pipeline.

Usage:

    python3 combined_vio_tracker.py [path/to/oak_d_w.yaml]

Optional rerun visualiser:

    python3 combined_vio_tracker.py [path/to/oak_d_w.yaml] --rerun

Quit:

    Press q in the tracker window
    or Ctrl+C in the terminal
"""

import os
import struct
import socket
import time
import signal
import argparse

import cv2
import depthai as dai
import rerun as rr


# ----------------------------
# Configuration
# ----------------------------

MAVLINK_SOCK = "/tmp/basalt_vio_listener"
LOCAL_SOCK = "/tmp/basalt_vio"

CAM_W, CAM_H = 640, 400
CAM_FPS = 20
IMU_RATE_HZ = 200

FULL_FRAME_TRACKING = False
USE_SPATIAL_ASSOCIATION = False

running = True


def _sigint(sig, frame):
    global running
    running = False


signal.signal(signal.SIGINT, _sigint)


# ----------------------------
# Helpers
# ----------------------------

def load_imu_noise_from_yaml(yaml_path):
    try:
        fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
        acc_n = float(fs.getNode("acc_n").real())
        gyr_n = float(fs.getNode("gyr_n").real())
        fs.release()

        if acc_n > 0 and gyr_n > 0:
            print(f"[noise] acc_n={acc_n:.6f}  gyr_n={gyr_n:.6f} from {yaml_path}")
            return acc_n, gyr_n

    except Exception as e:
        print(f"[noise] Could not read yaml: {e}")
        print("[noise] Using Basalt defaults")

    return None, None


def create_unix_socket():
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

    try:
        os.unlink(LOCAL_SOCK)
    except FileNotFoundError:
        pass

    sock.bind(LOCAL_SOCK)
    return sock


def cleanup_socket(sock):
    try:
        sock.close()
    except Exception:
        pass

    try:
        os.unlink(LOCAL_SOCK)
    except FileNotFoundError:
        pass


# ----------------------------
# Main
# ----------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Combined BasaltVIO and spatial object tracker"
    )
    parser.add_argument(
        "yaml_path",
        nargs="?",
        default=None,
        help="Path to oak_d_w.yaml"
    )
    parser.add_argument(
        "--rerun",
        action="store_true",
        help="Enable rerun visualiser"
    )
    args = parser.parse_args()

    acc_n, gyr_n = load_imu_noise_from_yaml(args.yaml_path) if args.yaml_path else (None, None)

    sock = create_unix_socket()

    if args.rerun:
        rr.init("basalt_vio_tracker", spawn=True)
        rr.log("world", rr.ViewCoordinates.FLU, static=True)
        rr.log(
            "world/ground",
            rr.Boxes3D(half_sizes=[[3.0, 3.0, 0.00001]]),
            static=True
        )

    trajectory = []
    fx = fy = None

    start_time = time.monotonic()
    counter = 0
    fps = 0.0
    color = (255, 255, 255)

    try:
        with dai.Pipeline() as pipeline:
            # ----------------------------
            # Cameras
            # ----------------------------

            cam_rgb = pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_A
            )

            mono_left = pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_B,
                sensorFps=CAM_FPS
            )

            mono_right = pipeline.create(dai.node.Camera).build(
                dai.CameraBoardSocket.CAM_C,
                sensorFps=CAM_FPS
            )

            left_output = mono_left.requestOutput((CAM_W, CAM_H))
            right_output = mono_right.requestOutput((CAM_W, CAM_H))

            # ----------------------------
            # IMU + Basalt VIO
            # ----------------------------

            imu = pipeline.create(dai.node.IMU)
            odom = pipeline.create(dai.node.BasaltVIO)

            imu.enableIMUSensor(
                [
                    dai.IMUSensor.ACCELEROMETER_RAW,
                    dai.IMUSensor.GYROSCOPE_RAW,
                ],
                IMU_RATE_HZ
            )
            imu.setBatchReportThreshold(1)
            imu.setMaxBatchReports(10)

            if acc_n is not None and gyr_n is not None:
                # BasaltVIO.cpp applies cwiseSqrt() internally
                odom.setAccelNoiseStd([acc_n ** 2, acc_n ** 2, acc_n ** 2])
                odom.setGyroNoiseStd([gyr_n ** 2, gyr_n ** 2, gyr_n ** 2])

            left_output.link(odom.left)
            right_output.link(odom.right)
            imu.out.link(odom.imu)

            transform_q = odom.transform.createOutputQueue(
                maxSize=10,
                blocking=False
            )

            vio_image_q = odom.passthrough.createOutputQueue(
                maxSize=4,
                blocking=False
            )

            # ----------------------------
            # Stereo depth for spatial NN
            # ----------------------------

            stereo = pipeline.create(dai.node.StereoDepth)

            # Same mono outputs are linked both to BasaltVIO and StereoDepth.
            # DepthAI outputs can usually be linked to multiple consumers.
            left_output.link(stereo.left)
            right_output.link(stereo.right)

            # ----------------------------
            # Spatial detection + tracker
            # ----------------------------

            spatial_detection_network = pipeline.create(
                dai.node.SpatialDetectionNetwork
            ).build(
                cam_rgb,
                stereo,
                "yolov6-nano"
            )

            object_tracker = pipeline.create(dai.node.ObjectTracker)

            spatial_detection_network.setConfidenceThreshold(0.6)
            spatial_detection_network.input.setBlocking(False)
            spatial_detection_network.setBoundingBoxScaleFactor(0.5)
            spatial_detection_network.setDepthLowerThreshold(100)
            spatial_detection_network.setDepthUpperThreshold(5000)

            label_map = spatial_detection_network.getClasses()

            object_tracker.setDetectionLabelsToTrack([0])  # person only
            object_tracker.setTrackerType(
                dai.TrackerType.SHORT_TERM_IMAGELESS
            )
            object_tracker.setTrackerIdAssignmentPolicy(
                dai.TrackerIdAssignmentPolicy.SMALLEST_ID
            )

            if USE_SPATIAL_ASSOCIATION:
                object_tracker.setSpatialAssociation(True)
                object_tracker.setSpatialAssociationWeight(0.5)
                object_tracker.setSpatialDistanceThreshold(1.5)
                object_tracker.setSpatialDepthAwareScale(0.1)

            if FULL_FRAME_TRACKING:
                cam_rgb.requestFullResolutionOutput().link(
                    object_tracker.inputTrackerFrame
                )
                object_tracker.inputTrackerFrame.setBlocking(False)
                object_tracker.inputTrackerFrame.setMaxSize(1)
            else:
                spatial_detection_network.passthrough.link(
                    object_tracker.inputTrackerFrame
                )

            spatial_detection_network.passthrough.link(
                object_tracker.inputDetectionFrame
            )
            spatial_detection_network.out.link(
                object_tracker.inputDetections
            )

            preview_q = object_tracker.passthroughTrackerFrame.createOutputQueue(
                maxSize=4,
                blocking=False
            )

            tracklets_q = object_tracker.out.createOutputQueue(
                maxSize=4,
                blocking=False
            )

            # ----------------------------
            # Start pipeline
            # ----------------------------

            pipeline.start()
            print("[combined] running")
            print("[combined] press q in tracker window or Ctrl+C to stop")

            while pipeline.isRunning() and running:
                did_work = False

                # ----------------------------
                # Basalt VIO output
                # ----------------------------

                transform = transform_q.tryGet()
                vio_img_frame = vio_image_q.tryGet()

                if transform is not None:
                    did_work = True

                    t = transform.getTranslation()
                    q = transform.getQuaternion()

                    # float[10] = qw qx qy qz px py pz vx vy vz
                    msg = struct.pack(
                        "10f",
                        q.qw, q.qx, q.qy, q.qz,
                        t.x, t.y, t.z,
                        0.0, 0.0, 0.0
                    )

                    try:
                        sock.sendto(msg, MAVLINK_SOCK)
                    except OSError:
                        pass

                    pos = [t.x, t.y, t.z]
                    trajectory.append(pos)

                    if args.rerun:
                        rr.log(
                            "world/camera",
                            rr.Transform3D(
                                translation=pos,
                                rotation=rr.Quaternion(
                                    xyzw=[q.qx, q.qy, q.qz, q.qw]
                                )
                            )
                        )
                        rr.log(
                            "world/trajectory",
                            rr.LineStrips3D([trajectory])
                        )

                if vio_img_frame is not None:
                    did_work = True

                    if fx is None:
                        calib = pipeline.getDefaultDevice().readCalibration()
                        intr = calib.getCameraIntrinsics(
                            dai.CameraBoardSocket.CAM_B,
                            vio_img_frame.getWidth(),
                            vio_img_frame.getHeight()
                        )
                        fx, fy = intr[0][0], intr[1][1]

                        if args.rerun:
                            rr.log(
                                "world/camera/image",
                                rr.Pinhole(
                                    resolution=[
                                        vio_img_frame.getWidth(),
                                        vio_img_frame.getHeight()
                                    ],
                                    focal_length=[fx, fy],
                                    camera_xyz=rr.ViewCoordinates.FLU
                                ),
                                static=True
                            )

                    if args.rerun:
                        vio_img = vio_img_frame.getCvFrame()
                        vio_img = cv2.cvtColor(vio_img, cv2.COLOR_BGR2RGB)
                        rr.log("world/camera/image/rgb", rr.Image(vio_img))

                # ----------------------------
                # Tracker output
                # ----------------------------

                img_frame = preview_q.tryGet()
                track = tracklets_q.tryGet()

                if img_frame is not None and track is not None:
                    did_work = True

                    counter += 1
                    current_time = time.monotonic()

                    if current_time - start_time > 1:
                        fps = counter / (current_time - start_time)
                        counter = 0
                        start_time = current_time

                    frame = img_frame.getCvFrame()
                    tracklets_data = track.tracklets

                    for trk in tracklets_data:
                        roi = trk.roi.denormalize(
                            frame.shape[1],
                            frame.shape[0]
                        )

                        x1 = int(roi.topLeft().x)
                        y1 = int(roi.topLeft().y)
                        x2 = int(roi.bottomRight().x)
                        y2 = int(roi.bottomRight().y)

                        try:
                            label = label_map[trk.label]
                        except Exception:
                            label = trk.label

                        cv2.putText(
                            frame,
                            str(label),
                            (x1 + 10, y1 + 20),
                            cv2.FONT_HERSHEY_TRIPLEX,
                            0.5,
                            255
                        )
                        cv2.putText(
                            frame,
                            f"ID: {trk.id}",
                            (x1 + 10, y1 + 35),
                            cv2.FONT_HERSHEY_TRIPLEX,
                            0.5,
                            255
                        )
                        cv2.putText(
                            frame,
                            trk.status.name,
                            (x1 + 10, y1 + 50),
                            cv2.FONT_HERSHEY_TRIPLEX,
                            0.5,
                            255
                        )

                        cv2.rectangle(
                            frame,
                            (x1, y1),
                            (x2, y2),
                            color,
                            cv2.FONT_HERSHEY_SIMPLEX
                        )

                        cv2.putText(
                            frame,
                            f"X: {int(trk.spatialCoordinates.x)} mm",
                            (x1 + 10, y1 + 65),
                            cv2.FONT_HERSHEY_TRIPLEX,
                            0.5,
                            255
                        )
                        cv2.putText(
                            frame,
                            f"Y: {int(trk.spatialCoordinates.y)} mm",
                            (x1 + 10, y1 + 80),
                            cv2.FONT_HERSHEY_TRIPLEX,
                            0.5,
                            255
                        )
                        cv2.putText(
                            frame,
                            f"Z: {int(trk.spatialCoordinates.z)} mm",
                            (x1 + 10, y1 + 95),
                            cv2.FONT_HERSHEY_TRIPLEX,
                            0.5,
                            255
                        )

                        if trk.velocity is not None and trk.speed is not None:
                            cv2.putText(
                                frame,
                                f"Velocity X: {trk.velocity.x:.2f} m/s",
                                (x1 + 10, y1 + 110),
                                cv2.FONT_HERSHEY_TRIPLEX,
                                0.5,
                                255
                            )
                            cv2.putText(
                                frame,
                                f"Velocity Y: {trk.velocity.y:.2f} m/s",
                                (x1 + 10, y1 + 125),
                                cv2.FONT_HERSHEY_TRIPLEX,
                                0.5,
                                255
                            )
                            cv2.putText(
                                frame,
                                f"Velocity Z: {trk.velocity.z:.2f} m/s",
                                (x1 + 10, y1 + 140),
                                cv2.FONT_HERSHEY_TRIPLEX,
                                0.5,
                                255
                            )
                            cv2.putText(
                                frame,
                                f"Speed: {trk.speed:.2f} m/s",
                                (x1 + 10, y1 + 155),
                                cv2.FONT_HERSHEY_TRIPLEX,
                                0.5,
                                255
                            )

                    cv2.putText(
                        frame,
                        f"NN fps: {fps:.2f}",
                        (2, frame.shape[0] - 4),
                        cv2.FONT_HERSHEY_TRIPLEX,
                        0.4,
                        color
                    )

                    cv2.imshow("tracker", frame)

                key = cv2.waitKey(1)
                if key == ord("q"):
                    break

                if not did_work:
                    time.sleep(0.001)

    finally:
        cleanup_socket(sock)
        cv2.destroyAllWindows()
        print("[combined] stopped")


if __name__ == "__main__":
    main()
