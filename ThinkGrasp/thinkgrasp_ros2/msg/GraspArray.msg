# Array of detected grasp poses
std_msgs/Header header
geometry_msgs/PoseArray grasp_poses   # All detected grasp poses
float64[] scores                      # Confidence scores for each grasp
int32 selected_index                  # Index of the selected/best grasp