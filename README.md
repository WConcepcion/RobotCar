# AutonomousRobot

Group 12 Project for Autonomous Robots

Pre-requisites:
Install Docker on the local machine.
Install docker-compose
Pull the repository to the local machine
git clone https://git.chalmers.se/courses/tme290/2022/group12/autonomousrobot.git

#Task 1: Create a robot behavior that drivers around the track as quickly as possible without hitting any cones. No other robots will drive in the track.

to run, navigate to the pulled folder using the CLI from the terminal. 

cd ~/autonomousrobots

xhost +

docker-compose -f kiwi-task-1.yml

#Task 2: Create a robot behavior that drives around the track as quickly as possible without hitting any cones, or any other slower driving robots. More than one robot will drive in the track at the same time. The distance to a robot should be kept relatively constant (to simply stop and wait is not acceptable). As the front distance sensor is quite narrow in it's field of view, the camera sensor is most likely needed in order to achieve good results.

to run, navigate to the pulled folder using the CLI from the terminal. 

cd ~/autonomousrobots

xhost +

docker-compose -f kiwi-task-2.yml
