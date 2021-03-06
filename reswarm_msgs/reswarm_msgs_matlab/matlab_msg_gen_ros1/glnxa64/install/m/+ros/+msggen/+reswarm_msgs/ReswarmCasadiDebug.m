
classdef ReswarmCasadiDebug < ros.Message
    %ReswarmCasadiDebug MATLAB implementation of reswarm_msgs/ReswarmCasadiDebug
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'reswarm_msgs/ReswarmCasadiDebug' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '71a2da2130cb8705dacf80ff5c94f1b9' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Wrench' 'U0Mpc' 'U0Dr' 'XNom' 'Accel' 'CasadiCompTime' 'TotalCompTime' 'ControlMode' 'StateMode' 'QPosFactor' 'QVelFactor' 'RFactor' 'QNPosFactor' 'QNVelFactor' 'QPosTubeFactor' 'QVelTubeFactor' 'RTubeFactor' 'QNPosTubeFactor' 'QNVelTubeFactor' 'QPosAncFactor' 'QVelAncFactor' 'RAncFactor' 'T' 'N' 'ControlDt' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'wrench' 'u0_mpc' 'u0_dr' 'x_nom' 'accel' 'casadi_comp_time' 'total_comp_time' 'control_mode' 'state_mode' 'Q_pos_factor' 'Q_vel_factor' 'R_factor' 'QN_pos_factor' 'QN_vel_factor' 'Q_pos_tube_factor' 'Q_vel_tube_factor' 'R_tube_factor' 'QN_pos_tube_factor' 'QN_vel_tube_factor' 'Q_pos_anc_factor' 'Q_vel_anc_factor' 'R_anc_factor' 'T' 'N' 'control_dt' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.geometry_msgs.Wrench' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.geometry_msgs.Point' ...
            'ros.msggen.std_msgs.Float64MultiArray' ...
            'ros.msggen.geometry_msgs.Vector3' ...
            'ros.msggen.std_msgs.Float64' ...
            'ros.msggen.std_msgs.Float64' ...
            'ros.msggen.std_msgs.String' ...
            'ros.msggen.std_msgs.String' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Wrench
        U0Mpc
        U0Dr
        XNom
        Accel
        CasadiCompTime
        TotalCompTime
        ControlMode
        StateMode
        QPosFactor
        QVelFactor
        RFactor
        QNPosFactor
        QNVelFactor
        QPosTubeFactor
        QVelTubeFactor
        RTubeFactor
        QNPosTubeFactor
        QNVelTubeFactor
        QPosAncFactor
        QVelAncFactor
        RAncFactor
        T
        N
        ControlDt
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'Header')
            obj.Header = val;
        end
        function set.Wrench(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Wrench'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'Wrench')
            obj.Wrench = val;
        end
        function set.U0Mpc(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'U0Mpc')
            obj.U0Mpc = val;
        end
        function set.U0Dr(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'U0Dr')
            obj.U0Dr = val;
        end
        function set.XNom(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Float64MultiArray'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'XNom')
            obj.XNom = val;
        end
        function set.Accel(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Vector3'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'Accel')
            obj.Accel = val;
        end
        function set.CasadiCompTime(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Float64'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'CasadiCompTime')
            obj.CasadiCompTime = val;
        end
        function set.TotalCompTime(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Float64'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'TotalCompTime')
            obj.TotalCompTime = val;
        end
        function set.ControlMode(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.String'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'ControlMode')
            obj.ControlMode = val;
        end
        function set.StateMode(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.String'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'StateMode')
            obj.StateMode = val;
        end
        function set.QPosFactor(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'QPosFactor');
            obj.QPosFactor = double(val);
        end
        function set.QVelFactor(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'QVelFactor');
            obj.QVelFactor = double(val);
        end
        function set.RFactor(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'RFactor');
            obj.RFactor = double(val);
        end
        function set.QNPosFactor(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'QNPosFactor');
            obj.QNPosFactor = double(val);
        end
        function set.QNVelFactor(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'QNVelFactor');
            obj.QNVelFactor = double(val);
        end
        function set.QPosTubeFactor(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'QPosTubeFactor');
            obj.QPosTubeFactor = double(val);
        end
        function set.QVelTubeFactor(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'QVelTubeFactor');
            obj.QVelTubeFactor = double(val);
        end
        function set.RTubeFactor(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'RTubeFactor');
            obj.RTubeFactor = double(val);
        end
        function set.QNPosTubeFactor(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'QNPosTubeFactor');
            obj.QNPosTubeFactor = double(val);
        end
        function set.QNVelTubeFactor(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'QNVelTubeFactor');
            obj.QNVelTubeFactor = double(val);
        end
        function set.QPosAncFactor(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'QPosAncFactor');
            obj.QPosAncFactor = double(val);
        end
        function set.QVelAncFactor(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'QVelAncFactor');
            obj.QVelAncFactor = double(val);
        end
        function set.RAncFactor(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'RAncFactor');
            obj.RAncFactor = double(val);
        end
        function set.T(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'T');
            obj.T = double(val);
        end
        function set.N(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'N');
            obj.N = int32(val);
        end
        function set.ControlDt(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ReswarmCasadiDebug', 'ControlDt');
            obj.ControlDt = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.reswarm_msgs.ReswarmCasadiDebug.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.reswarm_msgs.ReswarmCasadiDebug(strObj);
        end
    end
end
