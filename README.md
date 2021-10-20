# a1
Repo for Advanced Group 1  

### Run the simulation  
```
roslaunch moveit_resources demo_gazebo.launch
  ```  
  Now u can plan and execute from rviz using the moveit plugin and see the arm moving in gazebo.
  
  To move the arm to a specific joint configuration navigate to the folder moveit_resources/scripts  
  Open up the file mymove.py and in line 32 group.go() pass the joint positions as a list.  
    
  Then open up a new terminal and type-
  ```
  roslaunch moveit_resources planning_context.launch
  ```  
  Now run the script
  ```
  rosrun moveit_resources mymove.py
  ```
