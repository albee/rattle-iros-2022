
classdef ReswarmMsgMRPI < ros.Message
    %ReswarmMsgMRPI MATLAB implementation of reswarm_msgs_matlab/ReswarmMsgMRPI
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'reswarm_msgs_matlab/ReswarmMsgMRPI' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '58a198290d3ee8c3c6acb609374259e3' % The MD5 Checksum of the message definition
        PropertyList = { 'K' 'Au' 'Bu' 'AZ' 'BZ' 'UsingFallbackMrpi' } % List of non-constant message properties
        ROSPropertyList = { 'K' 'Au' 'bu' 'AZ' 'bZ' 'using_fallback_mrpi' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Float64MultiArray' ...
            'ros.msggen.std_msgs.Float64MultiArray' ...
            'ros.msggen.std_msgs.Float64MultiArray' ...
            'ros.msggen.std_msgs.Float64MultiArray' ...
            'ros.msggen.std_msgs.Float64MultiArray' ...
            'ros.msggen.std_msgs.Bool' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        K
        Au
        Bu
        AZ
        BZ
        UsingFallbackMrpi
    end
    methods
        function set.K(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Float64MultiArray'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmMsgMRPI', 'K')
            obj.K = val;
        end
        function set.Au(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Float64MultiArray'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmMsgMRPI', 'Au')
            obj.Au = val;
        end
        function set.Bu(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Float64MultiArray'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmMsgMRPI', 'Bu')
            obj.Bu = val;
        end
        function set.AZ(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Float64MultiArray'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmMsgMRPI', 'AZ')
            obj.AZ = val;
        end
        function set.BZ(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Float64MultiArray'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmMsgMRPI', 'BZ')
            obj.BZ = val;
        end
        function set.UsingFallbackMrpi(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Bool'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmMsgMRPI', 'UsingFallbackMrpi')
            obj.UsingFallbackMrpi = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.reswarm_msgs_matlab.ReswarmMsgMRPI.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.reswarm_msgs_matlab.ReswarmMsgMRPI(strObj);
        end
    end
end
