# Basalt VIO for GPS 2D Pose Estimation
Use OAK-D wide stereo camera to conduct Visual Inertial Odometry and simple object detection. Also tested with OAK-D Lite. Host controller used was the Raspberry Pi 5. Proper 5V 5A power supply is required to power the OAK-D and flight controller simultaneously from the Rasberry Pi's USB ports. VIO pose estimates can be sent to a flight controller via Mavlink and received by a laptop ground station running Mission Planner. 

## 1. Create new target directory and clone repo
```
mkdir basaltVio &&
cd basaltVio &&
git clone https://github.com/cooltech101/vio_for_gps_estimation.git .
```
## 2. Create new virtual environment and install packages
```
python3 -m venv venv &&
source venv/bin/activate &&
pip install -r requirements.txt
```
## 3. Install DepthAI
```
git clone https://github.com/luxonis/depthai-core.git && cd depthai-core
```
```
python3 examples/python/install_requirements.py && cd ..
```
## 4. Run Basalt VIO independently
Headless mode (default)
```
python3 basalt_vio.py
```
With rerun visualizer GUI
```
python3 basalt_vio.py --rerun
```
### Indoor Office Test
- Distance covered: 163.5m
- 3D VIO drift: 0.7153m (0.437%)
- ~20Hz average update rate

(Plots generated in Excel from coordinates logged in csv file)

<img src="vioPlots/indoor1.png" alt="Photo" width="45%">

## 5. Use Basalt VIO to estimate GPS pose (2D)
### Manual initialisation 
Run `basalt_vio.py`. Listen for VIO pose packets over Mavlink socket. Manually set starting lat lon and heading in lines 17-19 of `vio_translate_logger.py` which will be used to initialise VIO. Using initial pose, script converts local xy coordinates in metres into GPS lat lon coordinates in decimal degrees. Log estimated GPS pose entries to csv file of choice using --csv flag. No flight controller or GPS module needed.
```
python3 basalt_vio.py & python3 vio_translate_logger.py --csv newLog.csv
```
### Automatic initialisation (GPS signal required)
Connect the flight controller Debug port to the host controller via USB. Run `basalt_vio.py`. Upon execution, `vio_gps_logger.py` waits 3s for GPS readings to settle, then averages the next 2s of GPS readings to determine starting coordinates. This pose together with the compass yaw is used to initialise the VIO. Script listens for VIO pose packets over Mavlink socket. Log live GPS coordinates for each VIO-derived GPS estimate to a csv file of choice using --csv flag. Flight controller and GPS module needed. 
```
python3 basalt_vio.py & python3 vio_gps_logger.py --csv newLog.csv
```
Optional: use `run_gpsvio_logging.sh` to run `basalt_vio.py` and `vio_gps_logger.py` at the same time. Default output csv `dataLog.csv`. Shell script will cleanly terminate the processes upon interruption via Ctrl+C.
```
./run_gpsvio_logging.sh
```  
### Outdoor Test 1
Distance covered: 333m

<img src="vioPlots/outdoor2.png" alt="Photo" width="45%">

### Outdoor Test 2
Distance covered: 507m

<img src="vioPlots/outdoor1.png" alt="Photo" width="45%">

## 6. Run simple object tracking
YOLOv6-nano run on OAK-D with visualiser GUI.
```
python3 object_tracker.py
```
Run object tracking and VIO estimation simultaneously.
```
python3 vio_objtrack.py
```

N.B. if VIO was run earlier, object tracking will fail because old depthAI pipeline is still active. Reboot host controller, then run object tracking. 

## 7. Send VIO pose to Ardupilot flight controller over Mavlink
- Connect host controller UART to flight controller TELEM2 port
- Configure the relevant Ardupilot parameters as described here https://ardupilot.org/copter/docs/common-vio-tracking-camera.html#hardware-setup
- Ensure `SERIAL2_OPTIONS = 0`
- Set `VISO_TYPE = 1` (Mavlink)
- Open Mission Planner and connect to the flight controller over USB
- Right click on the map to set EKF origin (VIO 0,0,0) at desired point

Last, run Basalt VIO and forward visual position estimates to Mavlink.
```
python3 basalt_vio.py & python3 vio_to_mavlink.py 
```
Or more conveniently 
```
./run_viotomavlink.sh
```

Path traversed should be displayed as a purple track on Mission Planner. To view the xyz VIO pose received by Mission Planner, Ctrl+F -> Mavlink Inspector -> Vision Position Estimates. 

## 8. Automatically switch between GPS and VIO pose using Ardupilot's EKF3 (Cube Orange)
- As before, connect host controller UART to the flight controller TELEM2 port
- Store the automatic EKF source switching `gps_vio_autoswitch.lua` Lua script in the flight controller SD card in root/APM/scripts/
- Load the parameter file `gpsVIO_luaswitch_cubeOrange_params.param` which contains all the required parameter configurations in https://ardupilot.org/copter/docs/common-vio-tracking-camera.html#hardware-setup and https://ardupilot.org/copter/docs/common-non-gps-to-gps.html
- In the parameter file, GPS threshold `SCR_USER2 = 1.2` and VIO threshold `SCR_USER3 = 0.3`. Thresholds control how much to trust each EKF source. Adjust as needed.
- Open Mission Planner and connect to the flight controller with GPS disconnected
- Right click on the map to set EKF origin (VIO 0,0,0) at desired point
- Reconnect GPS

Last, run Basalt VIO and forward visual position estimates to Mavlink.
```
./run_viotomavlink.sh
```

N.B. Cube Black does not have sufficient EKF memory to execute this function

## 9. Debug scripts
Check if flight controller is receiving GPS coordinates and compass yaw by printing the values to terminal. 
```
python3 read_gps_yaw.py
```
To print GPS coordinates only
```
python3 read_gps.py
```




