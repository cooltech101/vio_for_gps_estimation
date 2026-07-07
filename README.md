# Basalt VIO for GPS 2D Pose Estimation


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

## 4. To run Basalt
### Headless mode (default)
```
python3 basalt_vio.py
```
### With rerun visualizer
```
python3 basalt_vio.py --rerun
