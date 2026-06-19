# configs/romi_live.py
primary_device = "cuda:0"
dataset_name = "romi_ros2_stream"

camera_params = {
    "fx": 381.3, 
    "fy": 381.3, 
    "cx": 320.0,
    "cy": 240.0,
    "width": 640,
    "height": 480,
    "depth_scale": 1.0, 
}

mapping_iters = 60
tracking_iters = 40

# Enable the Native SplaTAM Live GUI
viz = True
