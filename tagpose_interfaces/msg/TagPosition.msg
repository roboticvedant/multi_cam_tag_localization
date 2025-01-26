# Tag position relative to reference frame
std_msgs/Header header
string family           # Tag family
int32 id                # Tag ID
geometry_msgs/Pose pose # Tag pose in reference frame
# Additional fields for filtering and weighting
float32 decision_margin # Quality of the binary decoding process
float32 pose_error      # Object-space error of the pose estimation
