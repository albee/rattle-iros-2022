%{
create and add ROS msgs
%}

function create_and_add_ros_msgs()
  rosgenmsg('/home/albee/workspaces/astrobee-ws-rattle/freeflyer-rattle/develop/rattle_msgs/rattle_msgs_matlab');

  addpath('/home/albee/workspaces/astrobee-ws-rattle/freeflyer-rattle/develop/rattle_msgs/rattle_msgs_matlab/matlab_msg_gen_ros1/glnxa64/install/m')
  savepath

  clear classes;
  rehash toolboxcache;
end

