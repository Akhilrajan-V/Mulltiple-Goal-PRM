# Multiple-Goal-PRM
This repository consists the source code and files for the Multiple Goal Positions Probabilistic RoadMap path planning algorithm, completed as part of ENPM 661 Path Planning for Autonomous Vehicles course at the University of Maryland, USA.

## Authors
1. Akhilrajan Vethirajan (UID: 117431773)
2. Vishaal Kanna Sivakumar (UID: 117764314)

## Required Libraries

- Numpy
- heapq
- Matplotlib
- Cv2
- 
## To visualize the planner in action 
![output](/Outputs/TSP_graph.gif)   ![output](/Outputs/Final_path.gif)

1. Open a new terminal and cd into the scripts sub-directory
2. Make sure the script PRM_vis.py  is executable

	> chmod +x PRM_vis.py

3. To run the file,

	> python3 PRM_vis.py 

## To Run Gazebo Simulation

1. Copy the package directory named multi_goal_prm inside the catkin package directory to your catkin workspace.
2. Build your catkin workspace.
3. To launch the package type 

	> roslaunch multi_goal_prm prm_sim.launch
