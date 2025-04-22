# std_msgs/Header header

# determine of trained agent should start balancng: X button
bool state 

#robot's pose
# axes 2, 3
float32 left_height 
float32 right_height

#robot's drive axes 0, 1 
geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular