# Basalt VIO for GPS 2D Pose Estimation
The code in this repo uses the OAK-D wide stereo camera to conduct Visual Inertial Odometry. 

## 1. Create new target directory and virtual environment
```
mkdir basaltVio &&
cd basaltVio &&
python3 -m venv venv &&
source venv/bin/activate
```
## 2. Clone this repo and install packages
```
git clone https://github.com/cooltech101/vio_for_gps_estimation && pip install -r vio_for_gps_estimation/requirements.txt
```
## 3. Install DepthAI
```
cd vio_for_gps_estimation && git clone https://github.com/luxonis/depthai-core.git && cd depthai-core
```
```
python3 examples/python/install_requirements.py
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
## 5. Use Basalt VIO to estimate GPS pose (2D)

Run `basalt_vio.py`. Listen for VIO pose packets over Mavlink socket. Manually set starting lat lon and heading in lines 17-19 and use these values to initialise VIO. Log estimated GPS pose entries to csv file of choice using --csv flag. No flight controller or GPS module needed. 
```
python3 basalt_vio.py & python3 vio_translate_logger.py --csv newLog.csv
```
Run `basalt_vio.py`. Upon startup, average first few GPS readings to determine starting pose. Use this pose as initial VIO offset. Listen for VIO pose packets over Mavlink socket. Log live GPS and heading for each VIO estimate and log entries to csv file of choice using --csv flag. Flight controller and GPS module needed. 
```
python3 basalt_vio.py & python3 vio_gps_logger.py --csv newLog.csv
```
Optional: use `startall.sh` to conveniently run `basalt_vio.py` and `vio_gps_logger.py` at the same time. Default output csv `dataLog.csv`. Shell script will cleanly terminate the processes upon interruption via Ctrl+C.
```
./startall.sh
```  
## 6. Run simple object tracking
YOLOv6-nano run on OAK-D with visualizer.
```
python3 object_tracker.py
```
Run object tracking and VIO estimation simultaneously.
```
python3 vio_objtrack.py
```
## 7. Helper scripts
Print received GPS coordinates and IMU yaw to terminal. Flight controller and GPS module needed. 
```
python3 read_gps_yaw.py
```
To print GPS coordinates only
```
python3 read_gps.py
```


