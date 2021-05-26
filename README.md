# Followbot
ROS package for a path following bot.</br>

The simulation was done in gazebo. </br>
The simulation environment contains paths of three colours.  
The script uses OpenCV for foreground extraction and uses hsv values for differentiating between colours.  
The control commands are then sent to _cmd_vel_.

The bot keeps on following the yellow path until it encounters a blue path.
![Screenshot Gazebo](https://user-images.githubusercontent.com/61026273/119612767-eaf02a80-be19-11eb-9f42-a95ea8df9f86.png)

## Simulation:
https://user-images.githubusercontent.com/61026273/119612106-23dbcf80-be19-11eb-9fbc-ca2e04370776.mp4

## Files:
The scripts folder contains:
>follower_blue.py: The bot keeps on following the yellow path until it encounters a blue path.</br>
>follower_green.py: The bot keeps on following the yellow path until it encounters a green path.</br>
>follower_red.py: The bot keeps on following the yellow path until it encounters a red path.</br>
