# Multi-Object-tracking-2D
Use Kalman Filter and Hungarian algorithm to do tracking work based on 2D Lidar dataset of KITTI dataset.

Author: Jia Yansong

Run the project: 
1. cd /yourpath/MOT2D
2. type'python main.py Car_test' in your terminal

The results are saved in /MOT2D/results/Car_test/track_result  with 28 different sequences with different numbers of frames.

If you want to visualize the results we get, just run the visualization.py and add your right path to results, and change the save_path where you want.

The gif.py is used for your .png of the results, and you can use it to create a beautiful gif which can makes tracking process more clearly.

The visualization gif is like this:

![0027.gif](https://github.com/JYS997760473/Multi-Object-tracking-2D/blob/main/0027.gif)

![0006.gif](https://github.com/JYS997760473/Multi-Object-tracking-2D/blob/main/0006.gif)

![0001.gif](https://github.com/JYS997760473/Multi-Object-tracking-2D/blob/main/GIF/0001.gif)

![0008.gif](https://github.com/JYS997760473/Multi-Object-tracking-2D/blob/main/GIF/0008.gif)

![0013.gif](https://github.com/JYS997760473/Multi-Object-tracking-2D/blob/main/GIF/0013.gif)

![](https://github.com/JYS997760473/Multi-Object-tracking-2D/blob/main/GIF/0005.gif)

## Docker setting

### Run

```bash
docker run --rm -it --privileged --net=host --ipc=host --gpus all -v /home/jiayansong/workspace/Multi-Object-tracking-2D:/home/venti/Multi-Object-tracking-2D -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -v $HOME/.Xauthority:/home/venti/.Xauthority -e XAUTHORITY=/home/venti/.Xauthority -e ROS_IP=127.0.0.1 -v /home/jiayansong/venti_shared_data:/home/venti/shared_data -v /home/jiayansong/venti_shared_data/nuscenes:/home/venti/nuscenes -v /home/jiayansong/venti_shared_data/nuScenes_rosbags:/home/venti/nuScenes_rosbags --name venti-mot venti-mot
```

### Password

`password`

## Launch ROS

```bash
roscore
cd <path-to-nuScenes_rosbags>
rosbag play <rosbag> 
```

## Launch Rviz

```bash
roscd detection_and_tracking
rviz -d launch/nuscenes.rviz
```
