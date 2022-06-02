function [data, info] = reswarmStatusSecondary
%ReswarmStatusSecondary gives an empty data for reswarm_msgs_matlab/ReswarmStatusSecondary
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'reswarm_msgs_matlab/ReswarmStatusSecondary';
[data.Stamp, info.Stamp] = ros.internal.ros.messages.ros.time;
info.Stamp.MLdataType = 'struct';
[data.TestNumber, info.TestNumber] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.DefaultControl, info.DefaultControl] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.FlightMode, info.FlightMode] = ros.internal.ros.messages.ros.char('string',0);
[data.TestFinished, info.TestFinished] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.CoordOk, info.CoordOk] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.SolverStatus, info.SolverStatus] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.CostValue, info.CostValue] = ros.internal.ros.messages.ros.default_type('single',1);
[data.KktValue, info.KktValue] = ros.internal.ros.messages.ros.default_type('single',1);
[data.SolTime, info.SolTime] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'reswarm_msgs_matlab/ReswarmStatusSecondary';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'stamp';
info.MatPath{2} = 'stamp.sec';
info.MatPath{3} = 'stamp.nsec';
info.MatPath{4} = 'test_number';
info.MatPath{5} = 'default_control';
info.MatPath{6} = 'flight_mode';
info.MatPath{7} = 'test_finished';
info.MatPath{8} = 'coord_ok';
info.MatPath{9} = 'solver_status';
info.MatPath{10} = 'cost_value';
info.MatPath{11} = 'kkt_value';
info.MatPath{12} = 'sol_time';
