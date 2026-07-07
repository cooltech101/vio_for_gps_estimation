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
## 4. Run Basalt independently
### Headless mode (default)
```
python3 basalt_vio.py
```
### With rerun visualizer GUI
```
python3 basalt_vio.py --rerun
```
## 5. Helper scripts
### Print received GPS coordinates to terminal. (Connect flight controller debug port to Raspi via USB first)
```
python3 read_gps_yaw.py
```
