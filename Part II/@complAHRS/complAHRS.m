classdef complAHRS< handle
    %DCM AHRS algorithm
    %
    
    %% Public properties
    properties (Access = public)
        SamplePeriod = 1/256; %For given data sets, change here for Part III
        DCM = eye(3);     % output DCM describing the inertial frame relative to body frame
        Kp_RP = 0;                   % algorithm proportional gain
        Ki_RP = 0;                   % algorithm integral gain
        Ki_Yaw =0;
        Kp_Yaw = 0;
        Euler=[0 0 0];
        Gyroest=[0 0 0];
        GRAVITY=1; %For given data sets
    end
    
    %% Private properties
    properties (Access = private)
        eInt = [0 0 0];             % integral error
        eProp = [0 0 0];             % proportional error
    end
    
    %% Public methods
    methods (Access = public)
        function obj = complAHRS(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'SamplePeriod'), obj.SamplePeriod = varargin{i+1};
                elseif  strcmp(varargin{i}, 'DCM'), obj.DCM = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Kp'), obj.Kp_RP = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Ki'), obj.Ki_RP = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Kp_Y'), obj.Kp_Yaw = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Ki_Y'), obj.Ki_Yaw = varargin{i+1};
                else error('Invalid argument');
                end
            end;
        end
        function obj = Update(obj, Gyroscope, Accelerometer, Magnetometer)
            A=obj.DCM; % short name local variable for readability
            roll=obj.Euler(1);
            pitch=obj.Euler(2);
            yaw=obj.Euler(3);%            
            
            %% Calculate the magnitude of the accelerometer vector
            Accel_magnitude = norm(Accelerometer)/obj.GRAVITY;
            % Normalise accelerometer measurement
            Accelerometer = Accelerometer / norm(Accelerometer);    % normalise magnitude
            % Normalise magnetometer measurement
            Magnetometer = Magnetometer / norm(Magnetometer);   % normalise magnitude
            
            %% Drift correction
            
            %*****Roll and Pitch***** FILL IN HERE
            eRollPitch=
            eP=
            eI=
            
            %*****Yaw***************
            % Gyro yaw drift correction from compass magnetic heading
            % Tilt compensated Magnetic filed X:
            mX = Magnetometer(1)*cos(pitch)+Magnetometer(2)*sin(roll)*sin(pitch)+Magnetometer(3)*cos(roll)*sin(pitch);
            % Tilt compensated Magnetic filed Y:
            mY= Magnetometer(2)*cos(roll)-Magnetometer(3)*sin(roll);
            % Magnetic Heading
            magHeading= atan2(-mY,mX);
            %Calculating Yaw error
            FILL IN HERE
            
            %% PI controller
            obj.eProp=obj.Kp_Yaw*eYaw+eP;
			  FILL IN HERE
            obj.eInt=obj.Ki_Yaw*eYaw*...;           
                    
            %% Update rotation matrix (DCM)
            FILL IN HERE
            %%Normalize
            FILL IN HERE
            
     
        end
        
        end
    end
