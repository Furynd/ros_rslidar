## Lidar output data
Header header       ##header

uint8 flag  ## flag wether there are object or not in front of the vehicle
            ## 00 none, 01 left, 10 right

float32[] front_dist  ##front minimal distance
float32[] left_dist   ##left minimal distance
float32[] right_dist  ##right minimal distance

geometry_msgs/Point32[] frontPos
geometry_msgs/Point32[] leftPos
geometry_msgs/Point32[] rightPos