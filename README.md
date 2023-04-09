# ENPM661-Proj3.2

#Running Part01: 

To run the python script for part one:
	1. launch the python file using VScode or any other similar program
	2. hit run to initiate the code
	3. input the user inputs in the terminal as directed by the code: please use theta in intervals of 30
		inputs are: 
			start_position x_coor (20-580): i.e: 25
			start_position y_coor (20-180): i.e: 45
			start_position theta (0-360): i.e: 30
			
			start_position x_coor (20-580): i.e: 575
			start_position y_coor (20-180): i.e: 45
	4. the code will run and solve for the path using the A* algorithim
	5. once the path is found, the visualization will pop up and demonstrate the path found and the branches it looked at along the way
	
#Running Part02: 
	1. Copy and paste the ROS package included in the part02 folder into your src folder of catkin_ws
	2. catkin_make the package in your terminal 
	3. run the following commands in the terminal:
		source /opt/ros/noetic/setup.bash
		source ~/catkin_ws/devel/setup.bash
	4. navigate to your catkin_ws folder in your command window
	5. run the package with the following line:
		roslaunch project_3 go.launch 
	6. the program will create the map and initiate the search algorithim
	7. once a solution is found, the bot will start to move towards the goal position
	8. after the both makes it to the goal, a 2D visualization of the path will also pop up
	9. to change the goal position, you will need to access the node: "turtlebot3_control" thats located in the ROS package under the nodes folder
	10. once you have accessed the python folder "turtlebot3_control", you can change the goal position in lines 514 and 515
	11. please make sure you select valid points, otherwise the code will not run and tell you points selected are not good
	
#libraries:
	libraries used in this project are: 
		rospy
		from geometry_msgs.msg import Twist
		heapq
		copy
		numpy
		math
		cv2
		from numpy import tan, deg2rad, rad2deg
		
#Team Members:
	Amro Narmouq
		directory ID: narmouq7
		UID: 115405011
	Timothy Sweeny
		directory ID:
		UID: 
		
#GitHub link: 
	https://github.com/timsweeney7/ENPM661-Proj3.2.git
	
#simulation link:
	https://drive.google.com/drive/folders/1bWFwyR8SvL4y13x5t6l8LUGf-4KxGvvU?usp=share_link
	
	
