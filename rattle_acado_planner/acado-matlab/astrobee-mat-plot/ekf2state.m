function [state] = ekf2state(ekf_msg)
    %{ 
    Convert an ekf/msg to a standard TD state vector
    Outputs:
    x y z xd yd zd qx qy qz qw wx wy wz
    %}
    state = [ekf_msg.Pose.Position.X, ekf_msg.Pose.Position.Y, ekf_msg.Pose.Position.Z, ...
             ekf_msg.Velocity.X, ekf_msg.Velocity.Y, ekf_msg.Velocity.Z, ...
             ekf_msg.Pose.Orientation.X, ekf_msg.Pose.Orientation.Y, ekf_msg.Pose.Orientation.Z, ekf_msg.Pose.Orientation.W, ...
             ekf_msg.Omega.X, ekf_msg.Omega.Y, ekf_msg.Omega.Z];
end