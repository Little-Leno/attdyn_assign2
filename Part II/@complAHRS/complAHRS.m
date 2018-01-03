classdef complAHRS< handle

    %% Public properties
    properties (Access = public)
        SamplePeriod = 1/256; % For given data sets, change here for Part III.
        DCM = eye(3); % Output DCM describing the inertial frame relative to body frame.
        Kp_RP = 0; % Algorithm proportional gain.
        Ki_RP = 0; % Algorithm integral gain.
        Ki_Yaw = 0;
        Kp_Yaw = 0;
        Euler = [0 0 0];
        Gyroest = [0 0 0];
        eI_RP = 0;
        eI_Yaw = 0;
        GRAVITY=1; % For given data sets
    end
    
    %% Private properties
    properties (Access = private)
        eInt = [0 0 0]; % Integral error.
        eProp = [0 0 0]; % Proportional error.
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
            end
        end
        function obj = Update(obj, Gyroscope, Accelerometer, Magnetometer)
            A = obj.DCM; % Short name local variable.
            roll = obj.Euler(1);
            pitch = obj.Euler(2);
            %yaw = obj.Euler(3);            
            
            %% Calculate the magnitude of the accelerometer vector
            %Accel_magnitude = norm(Accelerometer)/obj.GRAVITY;
            % Normalise accelerometer measurement
            Accelerometer = Accelerometer / norm(Accelerometer); % normalise magnitude
            % Normalise magnetometer measurement
            Magnetometer = Magnetometer / norm(Magnetometer); % normalise magnitude
            
            %% Drift correction
            % Roll and Pitch.
            eRollPitch = cross(A(3,:), Accelerometer);  
            
            % Proportional error for pitchroll
            eP_RP = eRollPitch * obj.Kp_RP;
            
            % Integral error for pitchroll
            obj.eI_RP = obj.eI_RP + eRollPitch * obj.SamplePeriod;
            eI_RP = obj.eI_RP * obj.Ki_RP;
            
            % Yaw
            % Gyro yaw drift correction from compass magnetic heading
            % Tilt compensated Magnetic filed X:
            mX = Magnetometer(1) * cos(pitch) + Magnetometer(2) * sin(roll) * sin(pitch) + Magnetometer(3) * cos(roll) * sin(pitch);
            % Tilt compensated Magnetic filed Y:
            mY = Magnetometer(2) * cos(roll) - Magnetometer(3) * sin(roll);
            
            % Magnetic Heading
            %magHeading = atan2(-mY,mX);
            
            % Calculating Yaw error
            
            % Proportional error for yaw.
            eYaw = (- (A(1,1) * mY) + A(2,1) * mX) * A(3,:);
            eP_Yaw = eYaw * obj.Kp_Yaw;           
            
            % Integral error for yaw.
            eI_Yaw = (obj.eI_Yaw + eYaw * obj.SamplePeriod) * obj.Ki_Yaw; 
            
            %% PI controller
            eProp_Yaw = obj.Kp_Yaw * eYaw + eP_Yaw;
            eInt_Yaw = obj.Ki_Yaw * eYaw + eI_Yaw; 

            % Sum of the gyroscope measurement and all errors
            Gyroscope = Gyroscope + eP_RP + eI_RP + eProp_Yaw + eInt_Yaw;
            obj.Gyroest = Gyroscope;

            %% Update rotation matrix (DCM)   
            A = obj.DCM; % short name local variable for readability
              
            %Read gyro information
            omega(1) = Gyroscope(1);
            omega(2) = Gyroscope(2);
            omega(3) = Gyroscope(3);
            
            % Calculate rotation matrix time derivative A'=A+OMAdt
            OM = [0 omega(3) -omega(2);
                -omega(3) 0 omega(1);
                omega(2) -omega(1) 0];
            
            % Update rotation matrix
            A = A + (OM * A * obj.SamplePeriod);
   
            % Orthoganalise and Normalise rotation matrix
            u = A(1,:);
            v = A(2,:);
            %w = A(3,:);
            
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
            
            A = [u_n; v_n; w_n];
            
            obj.DCM = A;        
        
        end     
    end
end
