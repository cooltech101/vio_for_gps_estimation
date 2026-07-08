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
git clone https://github.com/cooltech101/vio_for_gps_estimation && pip install -r requirements.txt
```
## 3. Install DepthAI
```
git clone https://github.com/luxonis/depthai-core.git && cd depthai-core
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
Run simultaneously with `basalt_vio.py`
Listen for VIO pose packets over Mavlink socket. Estimate GPS pose based on starting lat lon and heading manually set in line 17-19. Log entries to csv file of choice using --csv flag. No flight controller needed.
```
python3 vio_translate_logger.py & python3 basalt_vio.py
```
Flight controller needed. Upon startup, average first few GPS readings to determine starting pose. Use this pose as initial VIO offset. Log live GPS and heading at each VIO estimate and log entries to csv file of choice using --csv flag.
```
python3 vio_gps_logger.py & python3 basalt_vio.py
```
## 6. Helper scripts
Print received GPS coordinates and IMU yaw to terminal. (Connect flight controller debug port to Raspi via USB first)
```
python3 read_gps_yaw.py
```
To print GPS coordinates only
```
python3 read_gps.py
```


