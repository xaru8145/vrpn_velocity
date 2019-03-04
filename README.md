vrpn_velocity

Node for taking VRPN position / pose estimates and computing the linear velocity. The current code filters the measurements and puts out the velocity of the vehicle in the VRPN frame. The transformation from VRPN frame to vehicle-1 frame is still being tested.

Input Topic: 
- Pose from VRPN (PoseStamped): /vrpn_client_ros/<vehicle_name>/pose

Output Topics:
- Unfiltered Velocity Estimate (TwistStamped): /vrpn_velocity/optitrack_frame/raw
- Low Pass Filtered Velocity Estimate (TwistStamped): /vrpn_velocity/optitrack_frame/filtered

