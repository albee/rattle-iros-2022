function [data, info] = reswarmUCBoundStatus
%ReswarmUCBoundStatus gives an empty data for reswarm_msgs_matlab/ReswarmUCBoundStatus
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'reswarm_msgs_matlab/ReswarmUCBoundStatus';
[data.Stamp, info.Stamp] = ros.internal.ros.messages.ros.time;
info.Stamp.MLdataType = 'struct';
[data.UcBoundFinished, info.UcBoundFinished] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.UnitTestComplete, info.UnitTestComplete] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'reswarm_msgs_matlab/ReswarmUCBoundStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'stamp';
info.MatPath{2} = 'stamp.sec';
info.MatPath{3} = 'stamp.nsec';
info.MatPath{4} = 'uc_bound_finished';
info.MatPath{5} = 'unit_test_complete';
