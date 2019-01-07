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
        Port = '/dev/ttyS0'
    end

    properties(Nontunable, PositiveInteger)
        Baudrate = 115200
    end
    
    properties (Nontunable, Logical)
        EnableAxis0 = true; % Enable
        EnableAxis1 = true; % Enable
        EnableVbusOutput = false; % Enable vbus_voltage output
        EnableCurrent0Output = false; % Enable estimated current output
        EnablePosition0Output = false; % Enable estimated position output
        EnableVelocity0Output = false; % Enable estimated velocity output
        EnableCurrent1Output = false; % Enable estimated current output
        EnablePosition1Output = false; % Enable estimated position output
        EnableVelocity1Output = false; % Enable estimated veloctity output
    end
    
    properties(Nontunable)
        ControlMode0 = 'Position Control' % Control Mode
        ControlMode1 = 'Position Control' % Control Mode
    end
    
    properties(Constant, Hidden)
        ControlMode0Set = matlab.system.StringSet({'Position Control','Velocity Control','Current Control'})
        ControlMode1Set = matlab.system.StringSet({'Position Control','Velocity Control','Current Control'})
    end
    
    properties(Nontunable)
        VelocityLimit0 = 3*pi; % Velocity limit [rad/s]
        VelocityLimit1 = 3*pi; % Velocity limit [rad/s]
        CurrentLimit0 = 100; % Current limit [A]
        CurrentLimit1 = 100; % Current limit [A]
        
        CountsPerRotate0 = 8192; % Counts per rotate of encoder
        CountsPerRotate1 = 8192; % Counts per rotate of encoder
       
        Inputs = ''; % Aditional inputs
        Outputs = ''; % Aditional outputs
    end
    
    properties(Nontunable, Logical)
        enableVBusRead = false
    end
    
    properties (Access = private)
        % Pre-computed constants.
    end
    
    methods
        % Constructor
        function obj = Coils(varargin)
            % Support name-value pair arguments when constructing the object.
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods (Access=protected)
        function setupImpl(obj) 
            if isempty(coder.target)
                % Place simulation setup code here
            else
                coder.cinclude('odrive.h');
                % Call C-function implementing device initialization
%                 if ~isempty(obj.Port)
%                     coder.ceval('magman_open', [obj.Port 0], int32(obj.Baud));
%                 end
            end
        end
        
        function varargout = stepImpl(obj,u)  
            if isempty(coder.target)
                % Place simulation output code here 
            else
                
            end
        end
        
        function releaseImpl(obj)
            if isempty(coder.target)
                % Place simulation termination code here
            else
                % Call C-function implementing device termination
            end
        end
        
        function inputs = getAllInputs(obj)
            n = 1;
            inputs = {};
            if obj.EnableAxis0
                mode = strsplit(obj.ControlMode0);
                inputs{n} = ['Axis0 Refernce ', lower(mode{1})];
                n = n+1;
            end
            if obj.EnableAxis1
                mode = strsplit(obj.ControlMode1);
                inputs{n} = ['Axis1 Refernce ', lower(mode{1})];
                n = n+1;
            end
            
            if ~isempty(obj.Inputs)
                for out = strsplit(obj.Inputs, ',')
                    inputs{n} = out{1};
                    n = n+1;
                end
            end
        end
        
        function outputs = getAllOutputs(obj)
            n = 1;
            outputs = {};
            
            if obj.EnableVbusOutput
                outputs{n} = 'vbus_voltage';
                n = n+1;
            end
            
            if ~isempty(obj.Outputs)
                for out = strsplit(obj.Outputs, ',')
                    outputs{n} = out{1};
                    n = n+1;
                end
            end
        end
    end
    
    methods (Access=protected)
        %% Define input properties
        function num = getNumInputsImpl(obj)
            num = length(obj.getAllInputs());
        end
        
        function num = getNumOutputsImpl(obj)
            num = length(obj.getAllOutputs());
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
        
        function validateInputsImpl(obj, Input)
            if isempty(coder.target)
                % Run input validation only in Simulation                
                validateattributes(Input,{'numeric'},{'size',[4,length(obj.Modules)]})
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
            varargout = getAllInputs(obj);
        end

        function varargout = getOutputNamesImpl(obj)
            % Return output port names for System block
            varargout = getAllOutputs(obj);
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
               'PropertyList',{'Port','Baudrate'});
                     
           axis0Group = matlab.system.display.SectionGroup(...
               'Title','Axis 0', ...
               'PropertyList',{'EnableAxis0','ControlMode0','VelocityLimit0', 'CurrentLimit0', 'CountsPerRotate0', 'EnableCurrent0Output', 'EnablePosition0Output', 'EnableVelocity0Output'});
           
           axis1Group = matlab.system.display.SectionGroup(...
               'Title','Axis 1', ...
               'PropertyList',{'EnableAxis1','ControlMode1','VelocityLimit1', 'CurrentLimit1', 'CountsPerRotate1', 'EnableCurrent1Output', 'EnablePosition1Output', 'EnableVelocity1Output'});
           
           inputsGroup = matlab.system.display.SectionGroup(...
               'Title','Inputs', ...
               'PropertyList',{'Inputs'});
           
           outputsGroup = matlab.system.display.SectionGroup(...
               'Title','Outputs', ...
               'PropertyList',{'EnableVbusOutput','Outputs'});

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
