function [data, info] = reswarmPlannerStatus
%ReswarmPlannerStatus gives an empty data for reswarm_msgs/ReswarmPlannerStatus
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'reswarm_msgs/ReswarmPlannerStatus';
[data.Stamp, info.Stamp] = ros.internal.ros.messages.ros.time;
info.Stamp.MLdataType = 'struct';
[data.PlannerFinished, info.PlannerFinished] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.SentRobustMPC, info.SentRobustMPC] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.SentPID, info.SentPID] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'reswarm_msgs/ReswarmPlannerStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'stamp';
info.MatPath{2} = 'stamp.sec';
info.MatPath{3} = 'stamp.nsec';
info.MatPath{4} = 'planner_finished';
info.MatPath{5} = 'sent_robustMPC';
info.MatPath{6} = 'sent_PID';
