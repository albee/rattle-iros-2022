function [data, info] = reswarmTestNumber
%ReswarmTestNumber gives an empty data for reswarm_msgs/ReswarmTestNumber
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'reswarm_msgs/ReswarmTestNumber';
[data.Stamp, info.Stamp] = ros.internal.ros.messages.ros.time;
info.Stamp.MLdataType = 'struct';
[data.TestNumber, info.TestNumber] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Role, info.Role] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'reswarm_msgs/ReswarmTestNumber';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'stamp';
info.MatPath{2} = 'stamp.sec';
info.MatPath{3} = 'stamp.nsec';
info.MatPath{4} = 'test_number';
info.MatPath{5} = 'role';
