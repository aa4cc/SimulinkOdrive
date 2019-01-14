classdef ODrive < matlab.System ...
        & coder.ExternalDependency ...
        & matlab.system.mixin.Propagates ...
        & matlab.system.mixin.CustomIcon
    % Leave Port name empty, if you want to use shared port between multiple blocks
    %
    % This template includes most, but not all, possible properties,
    % attributes, and methods that you can implement for a System object in
    % Simulink.
    %
    
    %#codegen
    properties
        % Public, tunable properties.
    end
    
    properties (Nontunable)
        Port = '/dev/ttyACM0'
        Autocalibration = 'Disabled (fail if not calibrated)'
    end

    properties(Nontunable, PositiveInteger)
        Baudrate = 115200
    end
    
    properties (Nontunable, Logical)
        EnableAxis0 = true; % Enable
        EnableAxis1 = true; % Enable
        EnableVbusOutput = false; % Bus voltage output
        EnableCurrent0Output = false; % Enable estimated current output
        EnablePosition0Output = false; % Enable estimated position output
        EnableVelocity0Output = false; % Enable estimated velocity output
        EnableCurrent1Output = false; % Enable estimated current output
        EnablePosition1Output = false; % Enable estimated position output
        EnableVelocity1Output = false; % Enable estimated velocity output
        UseIndex0 = true % Use index input of encoder
        UseIndex1 = true % Use index input of encoder
        ResetErrors0 = true; % Reset all error codes
        ResetErrors1 = true; % Reset all error codes
    end
    
    properties(Nontunable)
        ControlMode0 = 'Position' % Control Mode
        ControlMode1 = 'Position' % Control Mode
        
        MaxInputParameters = 20; % Maximum number of inputs
        MaxOutputParameters = 20; % Maximum number of outputs
    end
    
    properties(Constant, Hidden)
        ControlMode0Set = matlab.system.StringSet({'Position','Velocity','Current'})
        ControlMode1Set = matlab.system.StringSet({'Position','Velocity','Current'})
        AutocalibrationSet = matlab.system.StringSet({'Disabled (fail if not calibrated)','Autocalibrate','Autocalibrate and store'})
    end
    
    properties(Nontunable)
        VelocityLimit0 = 20*pi; % Velocity limit [rad/s]
        VelocityLimit1 = 20*pi; % Velocity limit [rad/s]
        CurrentLimit0 = 65; % Current limit [A]
        CurrentLimit1 = 65; % Current limit [A]
        
        CountsPerRotate0 = 8192; % Counts per rotate of encoder
        CountsPerRotate1 = 8192; % Counts per rotate of encoder
       
        Inputs = ''; % Aditional inputs
        Outputs = ''; % Aditional outputs
    end
    
    properties(Nontunable, Logical)
        enableVBusRead = false
    end
    
    properties (Access = private)
        inputParameters = {}
        outputParameters = {}
        inputNames = {}
        outputNames = {}        
        portFilePointer = 0;
    end
    
    methods
        % Constructor
        function obj = Coils(varargin)
            % Support name-value pair arguments when constructing the object.
            setProperties(obj,nargin,varargin{:});
            disp('constructor')
        end
    end
    
    methods (Access=protected)
        function setupImpl(obj) 
            obj.portFilePointer = int32(0);
            if isempty(coder.target)
                % Place simulation setup code here
            else
                [~, obj.inputParameters, ~] = obj.generateInputs();
                [~, obj.outputParameters, ~] = obj.generateOutputs();
                coder.cinclude('odrive.h');
                % Call C-function implementing device initialization
                obj.portFilePointer = coder.ceval('odrive_open_port', cstring(obj.Port), int32(obj.Baudrate));
                
                motor0_calibrated = false;
                motor1_calibrated = false;
                encoder0_ready = false;
                encoder1_ready = false;
                                
                if obj.EnableAxis0
                    if obj.ResetErrors0
                        % Reset all error codes
                        coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis0.motor.error'), int32(0));
                        coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis0.encoder.error'), int32(0));
                        coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis0.controller.error'), int32(0));
                        coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis0.error'), int32(0));
                    end
                    
                    coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis0.encoder.config.use_index'), int32(obj.UseIndex0));
                    coder.ceval('odrive_write_float', obj.portFilePointer, cstring('axis0.motor.config.current_lim'), obj.CurrentLimit0);
                    vel_limit = (obj.VelocityLimit0*obj.CountsPerRotate0)/(2*pi);
                    coder.ceval('odrive_write_float', obj.portFilePointer, cstring('axis0.controller.config.vel_limit'), vel_limit);
                    motor0_calibrated = coder.ceval('odrive_read_int', obj.portFilePointer, cstring('axis0.motor.is_calibrated'));
                    encoder0_ready = coder.ceval('odrive_read_int', obj.portFilePointer, cstring('axis0.encoder.is_ready'));
                end
                
                if obj.EnableAxis1
                    if obj.ResetErrors1
                        % Reset all error codes
                        coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis1.motor.error'), int32(0));
                        coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis1.encoder.error'), int32(0));
                        coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis1.controller.error'), int32(0));
                        coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis1.error'), int32(0));
                    end
                    coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis1.encoder.config.use_index'), int32(obj.UseIndex1));
                    coder.ceval('odrive_write_float', obj.portFilePointer, cstring('axis1.motor.config.current_lim'), obj.CurrentLimit1);
                    vel_limit = (obj.VelocityLimit1*obj.CountsPerRotate1)/(2*pi);
                    coder.ceval('odrive_write_float', obj.portFilePointer, cstring('axis1.controller.config.vel_limit'), vel_limit);
                    motor1_calibrated = coder.ceval('odrive_read_int', obj.portFilePointer, cstring('axis1.motor.is_calibrated'));
                    encoder1_ready = coder.ceval('odrive_read_int', obj.portFilePointer, cstring('axis1.encoder.is_ready'));
                end
                
                if(strcmp(obj.Autocalibration,'Disabled (fail if not calibrated)'))
                    if obj.EnableAxis0 && (~motor0_calibrated || ~encoder0_ready)
                        error("Axis 0 not ready, need calibration");                            
                    end
                    if obj.EnableAxis1 && (~motor1_calibrated || ~encoder1_ready)
                        error("Axis 1 not ready, need calibration");
                    end
                end
                if(strcmp(obj.Autocalibration,'Autocalibrate'))
                    if obj.EnableAxis0
                        coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis0.config.startup_motor_calibration'), ~motor0_calibrated);

                        if ~encoder0_ready
                            if obj.UseIndex0
                                encoder0_precalibrated = false;
                                encoder0_precalibrated = coder.ceval('odrive_read_int', obj.portFilePointer, cstring('axis0.encoder.config.pre_calibrated'));

                                coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis0.config.startup_encoder_index_search'), true);
                                coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis0.config.startup_encoder_offset_calibration'), ~encoder0_precalibrated);
                            else
                                coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis0.config.startup_encoder_index_search'), false);
                                coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis0.config.startup_encoder_offset_calibration'), true);
                            end
                        else
                            coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis0.config.startup_encoder_index_search'), false);
                            coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis0.config.startup_encoder_offset_calibration'), false);
                        end

                        coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis0.requested_state'), int32(2));
                    end
                    if obj.EnableAxis1
                        coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis1.config.startup_motor_calibration'), ~motor1_calibrated);

                        if ~encoder1_ready
                            if obj.UseIndex1
                                encoder1_precalibrated = false;
                                encoder1_precalibrated = coder.ceval('odrive_read_int', obj.portFilePointer, cstring('axis1.encoder.config.pre_calibrated'));

                                coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis1.config.startup_encoder_index_search'), true);
                                coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis1.config.startup_encoder_offset_calibration'), ~encoder1_precalibrated);
                            else
                                coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis1.config.startup_encoder_index_search'), false);
                                coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis1.config.startup_encoder_offset_calibration'), true);
                            end
                        else
                            coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis1.config.startup_encoder_index_search'), false);
                            coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis1.config.startup_encoder_offset_calibration'), false);
                        end

                        coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis1.requested_state'), int32(2));
                    end
                    
                    if obj.EnableAxis0
                        coder.ceval('odrive_wait_for_state', obj.portFilePointer, int32(0), int32(1), int32(100000), int32(0));
                    end
                    
                    if obj.EnableAxis1
                        coder.ceval('odrive_wait_for_state', obj.portFilePointer, int32(1), int32(1), int32(100000), int32(0)); 
                    end
                end
                
                if obj.EnableAxis0
                    switch(obj.ControlMode0)
                        case 'Position'
                            coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis0.controller.config.control_mode'), int32(3));
                        case 'Velocity'
                            coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis0.controller.config.control_mode'), int32(2));
                        case 'Current'
                            coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis0.controller.config.control_mode'), int32(1));       
                    end
                    coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis0.requested_state'), int32(8));
                end
                
                if obj.EnableAxis1
                    switch(obj.ControlMode1)
                        case 'Position'
                            coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis1.controller.config.control_mode'), int32(3));
                        case 'Velocity'
                            coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis1.controller.config.control_mode'), int32(2));
                        case 'Current'
                            coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis1.controller.config.control_mode'), int32(1));       
                    end
                    coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis1.requested_state'), int32(8));
                end
            end
        end
        
        function varargout = stepImpl(obj, varargin)
            if isempty(coder.target)
                % Place simulation output code here 
                for ind = 1:nargout
                    varargout{ind} = 625+ind;     
                end
            else
                for ind = 1:1
                    %coder.ceval('odrive_write_float', obj.portFilePointer, cstring('axis0.controller.pos_setpoint'), varargin{1});
                    coder.ceval('odrive_quick_write', obj.portFilePointer, int8('p'), int32(0), varargin{1});
                end
                for ind = 1:nargout
                    varargout{ind} = 0;
                    varargout{ind} = coder.ceval('odrive_read_float', obj.portFilePointer, cstring(obj.outputParameters{ind}));
                end
            end
        end
        
        function releaseImpl(obj)
            if isempty(coder.target)
                % Place simulation termination code here
            else
                % Call C-function implementing device termination
                if obj.EnableAxis0
                    coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis0.requested_state'), int32(1));
                end
                if obj.EnableAxis1
                    coder.ceval('odrive_write_int', obj.portFilePointer, cstring('axis1.requested_state'), int32(1));
                end
            end
        end
        
        function [names, parameters, count] = generateInputs(obj)
            count = 1;
            parameters = cell(1, obj.MaxInputParameters);
            names = cell(1, obj.MaxInputParameters);
            if obj.EnableAxis0
                names{count} = ['Axis 0 Reference ', lower(obj.ControlMode0)];
                parameters{count} = lower(obj.ControlMode0(1));
                count=count+1;
            end
            if obj.EnableAxis1
                names{count} = ['Axis 1 Reference ', lower(obj.ControlMode1)];
                parameters{count} = lower(obj.ControlMode1(1));
                count=count+1;
            end
            
            if ~isempty(obj.Inputs)
                for out = strsplit(obj.Inputs, ',')
                    parameters{count} = out{1};
                    names{count} = out{1};
                    count=count+1;
                end
            end
            
            if (count-1) > obj.MaxInputParameters
                error("Too many inputs, disable some or increase maximum inputs constant in ODrive module");
            end
            
            assert(length(names) == length(parameters));
            
            for ind = count:obj.MaxInputParameters
                names{ind} = '';
                parameters{ind} = '';
            end
            count = count - 1;
        end
        
        function [names, parameters, count] = generateOutputs(obj)
            count = 1;
            parameters = cell(1,obj.MaxOutputParameters);
            names = cell(1,obj.MaxOutputParameters);
            if obj.EnableAxis0
                if obj.EnableCurrent0Output
                    parameters{count} = 'axis0.motor.current_control.Iq_measured';
                    names{count} = 'Axis 0 Measured current [A]';
                    count=count+1;
                end

                if obj.EnablePosition0Output
                    parameters{count} = 'axis0.encoder.pos_estimate';
                    names{count} = 'Axis 0 Estimated position [rad]';
                    count=count+1;
                end

                if obj.EnableVelocity0Output
                    parameters{count} = 'axis0.encoder.vel_estimate';
                    names{count} = 'Axis 0 Estimated velocity [rad/s]';
                    count=count+1;
                end
            end
            if obj.EnableAxis1
                if obj.EnableCurrent1Output
                    parameters{count} = 'axis1.motor.current_control.Iq_measured';
                    names{count} = 'Axis 1 Measured current [A]';
                    count=count+1;
                end

                if obj.EnablePosition1Output
                    parameters{count} = 'axis1.encoder.pos_estimate';
                    names{count} = 'Axis 1 Estimated position [rad]';
                    count=count+1;
                end

                if obj.EnableVelocity1Output
                    parameters{count} = 'axis1.encoder.vel_estimate';
                    names{count} = 'Axis 1 Estimated position [rad/s]';
                    count=count+1;
                end
            end
            if obj.EnableVbusOutput
                parameters{count} = 'vbus_voltage';
                names{count} = 'Bus voltage [V]';
                count=count+1;
            end
            
            if ~isempty(obj.Outputs)
                for out = strsplit(obj.Outputs, ',')
                    parameters{count} = out{1};
                    names{count} = out{1};
                    count=count+1;
                end
            end
            
            if (count-1) > obj.MaxOutputParameters
                error("Too many outputs, disable some or increase maximum outputs constant in ODrive module");
            end
            
            for ind = count:obj.MaxOutputParameters
                names{ind} = '';
                parameters{ind} = '';
            end
            count = count - 1;
        end
        

    end
    
    methods (Access=protected)
        %% Define input properties
        function num = getNumInputsImpl(obj)
            [~, ~, num] = obj.generateInputs();
        end
        
        function num = getNumOutputsImpl(obj)
            [~, ~, num] = obj.generateOutputs();
        end
        
        function flag = isInputSizeLockedImpl(~,~)
            flag = true;
        end
        
        function varargout = isInputFixedSizeImpl(~,~)
            varargout{1} = true;
        end
        
        function flag = isInputComplexityLockedImpl(~,~)
            flag = true;
        end
        
        function validateInputsImpl(obj, varargin)
            if isempty(coder.target)
                % Run input validation only in Simulation                
            end
        end
        
        function validatePropertiesImpl(obj)
        end
        
        function icon = getIconImpl(obj)
            % Define a string as the icon for the System block in Simulink.
            if isempty(obj.Port) 
                icon = sprintf('ODrive\n');
            elseif any(startsWith(obj.Port, ["/dev/ttyACM", "/dev/ttyUSB"]))
                icon = sprintf('ODrive USB\nPort: %s', obj.Port);
            else
                icon = sprintf('ODrive\nPort: %s\n Baudrate: %d', obj.Port, obj.Baudrate);      
            end
        end

        function varargout = getInputNamesImpl(obj)
            % Return input port names for System block
            [varargout, ~, ~] = obj.generateInputs();
        end

        function varargout = getOutputNamesImpl(obj)
            % Return output port names for System block
            [varargout, ~, ~] = obj.generateOutputs();
        end

        function varargout = getOutputSizeImpl(obj)
            % Return size for each output port
            varargout = cell(1,nargout);
            for k = 1:nargout
                varargout{k} = [1 1];

                % Example: inherit size from first input port
                % varargout{k} = propagatedInputSize(obj,1);
            end
        end

        function varargout = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            varargout = cell(1,nargout);
            for k = 1:nargout
                varargout{k} = "double";

                % Example: inherit data type from first input port
                % varargout{k} = propagatedInputDataType(obj,1);
            end
        end

        function varargout = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            varargout = cell(1,nargout);
            for k = 1:nargout
                varargout{k} = false;

                % Example: inherit complexity from first input port
                % varargout{k} = propagatedInputComplexity(obj,1);
            end
        end

        function varargout = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            varargout = cell(1,nargout);
            for k = 1:nargout
                varargout{k} = true;

                % Example: inherit fixed-size status from first input port
                % varargout{k} = propagatedInputFixedSize(obj,1);
            end
        end
        
        
    end
    
    methods (Static, Access=protected)
        function header = getHeaderImpl
            % Define header panel for System block dialog
           header = matlab.system.display.Header(mfilename('class'), 'Title', ODrive.getDescriptiveName());
        end

        function groups = getPropertyGroupsImpl
            % Define property section(s) for System block dialog
            % group = matlab.system.display.Section(mfilename("class"));
           configGroup = matlab.system.display.Section(...
               'Title','ODrive configuration',...
               'PropertyList',{'Port','Baudrate', 'Autocalibration'});
                     
           axis0Group = matlab.system.display.SectionGroup(...
               'Title','Axis 0', ...
               'PropertyList',{'EnableAxis0','UseIndex0','ResetErrors0','ControlMode0','VelocityLimit0', 'CurrentLimit0', 'CountsPerRotate0', 'EnableCurrent0Output', 'EnablePosition0Output', 'EnableVelocity0Output'});
           
           axis1Group = matlab.system.display.SectionGroup(...
               'Title','Axis 1', ...
               'PropertyList',{'EnableAxis1','UseIndex1','ResetErrors1','ControlMode1','VelocityLimit1', 'CurrentLimit1', 'CountsPerRotate1', 'EnableCurrent1Output', 'EnablePosition1Output', 'EnableVelocity1Output'});
           
           inputsGroup = matlab.system.display.SectionGroup(...
               'Title','Inputs', ...
               'PropertyList',{'MaxInputParameters','Inputs'});
           
           outputsGroup = matlab.system.display.SectionGroup(...
               'Title','Outputs', ...
               'PropertyList',{'MaxOutputParameters','EnableVbusOutput','Outputs'});

           groups = [configGroup, axis0Group, axis1Group, inputsGroup, outputsGroup];
        end

        function simMode = getSimulateUsingImpl(~)
            simMode = 'Interpreted execution';
        end

        function flag = showSimulateUsingImpl
            % Return false if simulation mode hidden in System block dialog
            flag = false;
        end
    end
    
    methods (Static)
        function name = getDescriptiveName()
            name = 'ODrive';
        end
        
        function b = isSupportedContext(context)
            b = context.isCodeGenTarget('rtw');
        end
        
        function updateBuildInfo(buildInfo, context)
            if context.isCodeGenTarget('rtw')
                % Update buildInfo
                srcDir = fullfile(fileparts(mfilename('fullpath')),'src'); 
                includeDir = fullfile(fileparts(mfilename('fullpath')),'include');
                addIncludePaths(buildInfo,includeDir);
                % Use the following API's to add include files, sources and
                % linker flags
                addIncludeFiles(buildInfo,'odrive.h',includeDir);
                addSourceFiles(buildInfo,'odrive.c',srcDir);
                %addLinkObjects(buildInfo,'sourcelib.a',srcDir);
                %addCompileFlags(buildInfo,{'-D_DEBUG=1'});
                %addDefines(buildInfo,'MY_DEFINE_1')
            end
        end
    end
end

function output = cstring(input)
    output = [input 0];
end
