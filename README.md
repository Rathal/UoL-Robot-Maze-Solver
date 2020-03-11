# RoboticsAssignment

Catkin_make the ~

sudo apt-get update && sudo apt-get upgrade && sudo apt-get install ros-kinetic-uol-cmp3103m
or sudo apt-get purge "*gazebo*"

roslaunch uol_turtlebot_simulator maze1.launch

proximity.py - to stop hitting things
follower.py - to head towards goals


#-Move metre at a time
#-Look Left
#  +Get Distance to wall
#-Look Right
#  +Get Distance to wall
#-Go Towards largest distance IF NO BLUE OR GREEN OR RED
#-Go until wall < 0.5m away



