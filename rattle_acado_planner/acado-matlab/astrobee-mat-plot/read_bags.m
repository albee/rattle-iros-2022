%{
Read rosbag data for Astrobee GNC and custom topics.
Uses MATLAB's rosbag interface.

rosbag_path: path to rosbag location, e.g. 'repos/data/bags'
topic_prefix: prefix to append to topic names, e.g. `/honey/`
bag_type: {'roam', 'reswarm'}

Keenan Albee, 2021.
%}

function [pbd] = read_bags(rosbag_path, topic_prefix, bag_type)  
    if bag_type == "reswarm"
     [pbd] = read_bags_reswarm(rosbag_path, topic_prefix);
    elseif bag_type == "roam"
      [pbd] = read_bags_roam(rosbag_path, topic_prefix);
    else
      disp("Invalid bag type!");
    end
end

