# My-Repository

base_img_inference-
    Contains the DL CNN to perform image classification based on the training on the CIFAR10 dataset
    
Q6-
    Contains the files for bilateralFilter through openCV
    
puCUDA_openCV_Grayscale_image-
    Contains the codes to convert image into grayscale through PyCUDA and OpenCV.
    
pthreads_prime_num.cpp-
    Using pthreads multi-threading to find the sum of all prime numbers between 1 to 1 million.

AuE893_project-

    Active files-

    Navigation & Robot environment-
    Please unzip the turtlebot3_gazebo.zip and run the turtlebot3_world_2.launch using the command-
    roslaunch turtlebot3_gazebo turtlebot3_world_2.launch

    roslocation.cpp -> provides the co-ordinates of the robot in the environment which then can be used to send as an input to the path planner.

    RRTPlanner.mlx-> matlab live script path planning with RRT*. Gives out an argument of N*2 matrix which is used for the controller.

    trialRRT_with_control_iterative.m -> RRT* path planner with a custom control logic using ROS connectivity through matlab script (alternative to simulink model) (didn't work as expected, needs to be refined in terms of logic)

    playpen_map -> slam mapping output map file

    GazeboDifferentialDriveControl.slx -> core simulink model developed having the purepuruit controller based upon inputs from pathPlanner algorithm providing linear and angular velocities to the robot.

    pathFollowingWithObstacleAvoidanceExample.slx -> simulink file having controller logic with obstacle detection and velocity modulation.
