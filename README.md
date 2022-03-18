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
