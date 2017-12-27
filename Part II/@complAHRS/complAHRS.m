classdef complAHRS< handle
    %DCM AHRS algorithm        %from natalie
    %% direction cosine matrix
    %% Public properties
    properties (Access = public)
        SamplePeriod = 1/256;
        DCM = eye(3);     % output DCM describing the inertial frame relative to body frame
        Euler=[0 0 0];
    end
    
    %% Private properties
    properties (Access = private)

    end
    
    %% Public methods
    methods (Access = public)
        function obj = dcmAHRS(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'SamplePeriod'), obj.SamplePeriod = varargin{i+1};
                elseif  strcmp(varargin{i}, 'DCM'), obj.DCM = varargin{i+1};
                else error('Invalid argument');
                end
            end
        end
                %% Update rotation matrix (DCM)   
        function obj = Update(obj, Gyroscope)
                
            %FILL IN HERE
            dcm = obj.DCM; % short name local variable for readability
              
            %Read gyro information
            
            omega(1)=Gyroscope(1);
            omega(2)=Gyroscope(2);
            omega(3)=Gyroscope(3);
            
            %Calculate rotation matrix time derivative A'=A+OMAdt
            
            OM = [0 omega(3) -omega(2);...
                -omega(3) 0 omega(1);...
                omega(2) -omega(1) 0];
            
            dcm_new = dcm + (OM*dcm*obj.SamplePeriod);
            
            %Update rotation matrix
            dcm = dcm_new;        %to natalie
            
            
           
      
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
            eRollPitch = cross(yaw,obj.GRAVITY);         %natalie
            eP =
            eI =
            
            %*****Yaw***************
            % Gyro yaw drift correction from compass magnetic heading
            % Tilt compensated Magnetic filed X:
            mX = Magnetometer(1)*cos(pitch)+Magnetometer(2)*sin(roll)*sin(pitch)+Magnetometer(3)*cos(roll)*sin(pitch);
            % Tilt compensated Magnetic filed Y:
            mY= Magnetometer(2)*cos(roll)-Magnetometer(3)*sin(roll);
            % Magnetic Heading
            magHeading= atan2(-mY,mX);
            %Calculating Yaw error
            eYGround = (roll*mY)-(pitch*mX);  %natalie
            eYaw = eYGround*yaw;        %natalie
            
            %% PI controller
            obj.eProp=obj.Kp_Yaw*eYaw+eP;
			  FILL IN HERE
            obj.eInt=obj.Ki_Yaw*eYaw*...;           
                    
            %% Update rotation matrix (DCM)
            FILL IN HERE
            %%Normalize
            %FILL IN HERE                    %from natalie
            u = dcm(1,:);
            v = dcm(2,:);
            w = dcm(3,:);
            
            E_rp = dot(u,v);
            
                if E_rp == 0
                    u_p = u;
                    v_p = v;
                else
                    u_p = u-(E_rp/2)*v;
                    v_p = v-(E_rp/2)*u;
                end
            
            
            w_p = cross(u,v);
            
            u_n = u_p/norm(u_p);
            v_n = v_p/norm(v_p);
            w_n = w_p/norm(w_p);
            
            dcm = [u_n; v_n; w_n];        %to natalie
     
        end
        
        end
    end
