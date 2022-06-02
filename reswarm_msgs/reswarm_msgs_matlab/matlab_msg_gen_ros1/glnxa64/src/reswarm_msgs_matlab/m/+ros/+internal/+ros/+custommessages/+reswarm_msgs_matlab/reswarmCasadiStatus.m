function [data, info] = reswarmCasadiStatus
%ReswarmCasadiStatus gives an empty data for reswarm_msgs_matlab/ReswarmCasadiStatus
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'reswarm_msgs_matlab/ReswarmCasadiStatus';
[data.Stamp, info.Stamp] = ros.internal.ros.messages.ros.time;
info.Stamp.MLdataType = 'struct';
[data.CoordOk, info.CoordOk] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.MrpiFinished, info.MrpiFinished] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.TrajFinished, info.TrajFinished] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.ControlMode, info.ControlMode] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'reswarm_msgs_matlab/ReswarmCasadiStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'stamp';
info.MatPath{2} = 'stamp.sec';
info.MatPath{3} = 'stamp.nsec';
info.MatPath{4} = 'coord_ok';
info.MatPath{5} = 'mrpi_finished';
info.MatPath{6} = 'traj_finished';
info.MatPath{7} = 'control_mode';
