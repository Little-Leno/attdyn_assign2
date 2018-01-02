classdef complAHRS< handle
    %DCM AHRS algorithm
    
    %% Public properties
    properties (Access = public)
        SamplePeriod = 1/256; % For given data sets, change here for Part III
        DCM = eye(3);         % output DCM describing the inertial frame relative to body frame
        Kp_RP = 0;            % algorithm proportional gain
        Ki_RP = 0;            % algorithm integral gain
        Ki_Yaw =0;
        Kp_Yaw = 0;
        Euler=[0 0 0];
        Gyroest=[0 0 0];
        GRAVITY=1;            % For given data sets
    end
    
    %% Private properties
    properties (Access = private)
        eI_RP= [0 0 0];              % integral error
        eI_Yaw = [0 0 0];            % proportional error
    end
    
    %% Public methods
    methods (Access = public)
        function obj = complAHRS(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'SamplePeriod'), obj.SamplePeriod = varargin{i+1};
                    elseif  strcmp(varargin{i}, 'eProp'), obj.eProp = varargin{i+1};
                        elseif  strcmp(varargin{i}, 'eInt'), obj.eInt = varargin{i+1};
                elseif  strcmp(varargin{i}, 'DCM'), obj.DCM = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Kp_RP'), obj.Kp_RP = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Ki_RP'), obj.Ki_RP = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Kp_Yaw'), obj.Kp_Yaw = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Ki_Yaw'), obj.Ki_Yaw = varargin{i+1};
                else error('Invalid argument');
                end
            end;
        end
        function obj = Update(obj, Gyroscope, Accelerometer, Magnetometer)
            dcm = obj.DCM; % short name local variable for readability
            obj.Euler = rotMat2euler(dcm);
            roll = obj.Euler(1);
            pitch = obj.Euler(2);
            yaw = obj.Euler(3);%            
            
            %% Calculate the magnitude of the accelerometer vector
            Accel_magnitude = norm(Accelerometer)/obj.GRAVITY;
            % Normalise accelerometer measurement
            Accelerometer = Accelerometer / norm(Accelerometer);    % normalise magnitude
            % Normalise magnetometer measurement
            Magnetometer = Magnetometer / norm(Magnetometer);   % normalise magnitude
            
            %% Drift correction
            
            % Roll and Pitch
           
            eRollPitch=cross(dcm(3,:), Accelerometer);
            eP_RP = obj.Kp_RP * eRollPitch;
            
            obj.eI_RP = obj.eI_RP + eRollPitch * obj.SamplePeriod;% proportional error
            eI_RP = obj.Ki_RP * obj.eI_RP; % integral error
            
            % Yaw
          
            % Gyro yaw drift correction from compass magnetic heading
            % Tilt compensated Magnetic filed X:
            mX = Magnetometer(1)*cos(pitch)+Magnetometer(2)*sin(roll)*sin(pitch)+Magnetometer(3)*cos(roll)*sin(pitch);
            % Tilt compensated Magnetic filed Y:
            mY = Magnetometer(2)*cos(roll)-Magnetometer(3)*sin(roll);
            % Magnetic Heading
            magHeading = atan2(-mY,mX);
            
            % Calculating Yaw error
            eYaw = - (dcm(1,1) * mY) + dcm(2,1) * mX;
            eYaw = dcm(3,:) * eYaw;
            eP_Yaw = obj.Kp_Yaw * eYaw;
            obj.eI_Yaw = obj.eI_Yaw + eYaw * obj.SamplePeriod; % proportional error
            eI_Yaw = obj.Ki_Yaw * eYaw ;
            %% PI controller
            eProp_Yaw = obj.Kp_Yaw * eYaw + eP_Yaw;
            eInt_Yaw = obj.Ki_Yaw * eYaw + eI_Yaw;           
            
            eProp_RP = eP_RP; %obj.Kp_RP * eRollPitch + eP_RP;
            eInt_RP = eI_RP; %obj.Ki_RP * eRollPitch + eI_RP;
            
            % Sum of the gyroscope measurement and all errors
            Gyroscope = Gyroscope + eProp_RP + eInt_RP + eProp_Yaw + eInt_Yaw;
            obj.Gyroest = Gyroscope;
            
            %% Update rotation matrix (DCM)   
            dcm = obj.DCM;
            
            Omega = [
            0, Gyroscope(3), -Gyroscope(2);
            -Gyroscope(3), 0, Gyroscope(1);
            Gyroscope(2), -Gyroscope(1), 0];
        
            dcm = dcm + Omega * dcm * obj.SamplePeriod; 
            
            u = dcm(1, :);
            v = dcm(2, :);

            % Roll-Pitch Orthogonalization
            if dot(u, v) ~= 0
                epsilon = dot(u, v);
                u_perp = u - ((epsilon/2)*v); 
                v_perp = v - ((epsilon/2)*u);
                w_perp = cross(u_perp, v_perp);
                u_norm = u_perp / norm(u_perp);
                v_norm = v_perp / norm(v_perp);
                w_norm = w_perp / norm(w_perp);
            else
                w_perp = cross(u, v);
                u_norm = u / norm(u);
                v_norm = v / norm(v);
                w_norm = w_perp / norm(w_perp); % Yaw orthogonalization
            end       
            
            dcm(1, :) = u_norm;
            dcm(2, :) = v_norm;
            dcm(3, :) = w_norm;

            obj.DCM = dcm;
     
        end
  
        end
    end
