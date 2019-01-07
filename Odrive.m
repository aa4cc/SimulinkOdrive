classdef Odrive < matlab.System ...
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
        Baud = 500000
        Modules = uint8(1:16)
        Mode = 'PWM';
    end
    
    properties (Nontunable, Logical)
        Trigger=false;
        % Collect meassured currents
        Currents=false;
        % Optimized comunication
        OptimizedComunication=false;
        % Green LED Off
        GreenLedOff=false;
    end

    properties(Constant, Hidden)
        ModeSet = matlab.system.StringSet({'PWM','Currents'});
        %BaudSet = matlab.system.StringSet({'230400','1500000'});
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
                coder.cinclude('magman.h');
                % Call C-function implementing device initialization
                if ~isempty(obj.Port)
                    coder.ceval('magman_open', [obj.Port 0], int32(obj.Baud));
                end
                coder.ceval('magman_zeros', obj.Trigger);
                coder.ceval('magman_on');
                if(obj.GreenLedOff)
                    coder.ceval('magman_led', 0, 0, 0, 0);
                end
            end
        end
        
        function varargout = stepImpl(obj,u)  
            if isempty(coder.target)
                % Place simulation output code here 
            else
                m = uint8(obj.Modules);
                % Call C-function implementing device output
                if strcmp(obj.Mode, 'PWM')
                    %void magman_setModules(uint8_t modules[], uint8_T len, real_T data[])
                    coder.ceval('magman_setModules', coder.rref(m), uint8(length(obj.Modules)), u, obj.Trigger, obj.OptimizedComunication)
                elseif strcmp(obj.Mode, 'Currents')
                    %void magman_setModulesCurrent(uint8_T modules[], uint8_T len, real_T current[],uint8_T trigger);
                    coder.ceval('magman_setModulesCurrents', coder.rref(m), uint8(length(obj.Modules)), u, obj.Trigger, obj.OptimizedComunication)
                end
                if obj.Currents
                    currents = zeros(4, length(obj.Modules));
                    %void magman_getCurrents(const uint8_T modules[], const uint8_T len, real_T currents[]);
                    coder.ceval('magman_getCurrents', coder.rref(m), uint8(length(obj.Modules)), coder.wref(currents));
                    
                    %int magman_getCurrentsFromModule(const uint8_T module, real_T currents[]);
                    %coder.ceval('magman_getCurrentsFromModule', uint8(22), coder.wref(currents));
                    varargout{1} = currents;
                end
            end
        end
        
        function releaseImpl(obj)
            if isempty(coder.target)
                % Place simulation termination code here
            else
                % Call C-function implementing device termination
                coder.ceval('magman_off')
                %coder.ceval('sink_terminate');
            end
        end
    end
    
    methods (Access=protected)
        %% Define input properties
        function num = getNumInputsImpl(~)
                num = 1;
        end
        
        function num = getNumOutputsImpl(obj)
            num = 0;
            if obj.Currents
                num = 1;
            end
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
            if ~isempty(obj.Port)
                icon = sprintf('MagMan coils\n%d modules\n Port: %s\n Baudrate: %d', length(obj.Modules), obj.Port, obj.Baud);
            else
                icon = sprintf('MagMan coils\n%d modules', length(obj.Modules));
            end
            
            if obj.Trigger
                icon = ['Triggered ' icon];
            end
            
            if obj.OptimizedComunication
                icon = [icon '\nOptimized comunication'];
            end
        end

        function name = getInputNamesImpl(obj)
            % Return input port names for System block
            name = obj.Mode;
        end

        function varargout = getOutputNamesImpl(obj)
            % Return output port names for System block
            if obj.Currents
                varargout{1} = 'Currents';
            end
        end
    end
    
    methods (Static, Access=protected)
        function header = getHeaderImpl
            % Define header panel for System block dialog
           header = matlab.system.display.Header(mfilename('class'), 'Title', Coils.getDescriptiveName());
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
            name = 'MagMan Coils';
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
                addIncludeFiles(buildInfo,'magman.h',includeDir);
                addSourceFiles(buildInfo,'magman.c',srcDir);
                addLinkFlags(buildInfo,{'-lwiringPi'});
                %addLinkObjects(buildInfo,'sourcelib.a',srcDir);
                %addCompileFlags(buildInfo,{'-D_DEBUG=1'});
                %addDefines(buildInfo,'MY_DEFINE_1')
            end
        end
    end
end
