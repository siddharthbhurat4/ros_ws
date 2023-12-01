# Demonstration of Indoor and Outdoor Localization

This repository focuses on implementing advanced filtering-based localization methods to seamlessly fuse data from various sensors such as wheel encoders, GPS, IMU, and cameras. The goal is to achieve highly accurate robot localization capabilities, suitable for both indoor and outdoor environments, especially for differential drive wheel mobile robots but can be extened to other configuration robots as well.

### Key Features:

* Integration of multiple sensors for robust localization.
* Utilization of Docker for streamlined deployment and reproducibility.
* Developed using ROS 2 for enhanced compatibility and flexibility.

### Demo:
The current demo showcases the localization system in action, visualized using Gazebo, a powerful physics-based simulation tool.

![](https://github.com/siddharthbhurat4/ros_ws/blob/humble/demo_gif.gif)

## How to Run Demo

### Prerequisites

You should already have Docker and VSCode with the remote containers plugin installed on your system.

* [docker](https://docs.docker.com/engine/install/)
* [vscode](https://code.visualstudio.com/)
* [vscode remote containers plugin](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### Clone this Repository

* Click on "code" and copy the link to repository
* Clone it in your local system

### Open it in vscode

* Now that you've cloned the repo onto your computer, you can open it in VSCode (File->Open Folder). 

* When you open it for the first time, you should see a little popup that asks you if you would like to open it in a container.  Say yes!

![template_vscode](https://user-images.githubusercontent.com/6098197/91332551-36898100-e781-11ea-9080-729964373719.png)

* If you don't see the pop-up, click on the little green square in the bottom left corner, which should bring up the container dialog

![template_vscode_bottom](https://user-images.githubusercontent.com/6098197/91332638-5d47b780-e781-11ea-9fb6-4d134dbfc464.png)

* In the dialog, select "Remote Containers: Reopen in container"

* VSCode will build the dockerfile inside of `.devcontainer` for you.  If you open a terminal inside VSCode (Terminal->New Terminal), you should see that your username has been changed to `ros`, and the bottom left green corner should say "Dev Container"

![template_container](https://user-images.githubusercontent.com/6098197/91332895-adbf1500-e781-11ea-8afc-7a22a5340d4a.png)

* Since you will be building this docker environment for the first time and there are some heavy files being installed, it will take few minutes for it to build

* Once the docker is built and container is ready you have to open terminal by clicking on terminal in top bar and selecting new terminal
* After the terminal opens at the bottom, you have to run the following commands
  
  `cd src`

  `colcon build`

  `source install/setup.bash`

  `ros2 launch final_demo final_demo.launch.py`

* On running these commands the gazebo window will show up on the screen where you can see the robot in gazebo town world along with 2 walls at the center which denote the indoor and outdoor environment.

![](https://github.com/siddharthbhurat4/ros_ws/blob/humble/gazebo_bot.png)

* The robot will follow a rectangular trajectory using a PID based controller
* It will transition through those 2 walls where the GPS signal will be dropped and robot will still localize itself using the rest of the sensors
* After it completes the rectangular trajectory , it will display the plots of the trajectory followed as a result of the estimated positions using localization algorithm
* In the plots you could see that the robot looses the gps signal when `x=5.0` and `-2.0 < Y < -8.0`. At these positions



## FAQ