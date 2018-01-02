classdef complAHRS< handle

      
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
        eI_RP = 0;
        eI_Yaw = 0;
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
            
            %% Update rotation matrix (DCM)   
            
            
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
            dcm = dcm_new;
            
            %Orthoganalise and Normalise rotation matrix
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
            
            dcm = [u_n; v_n; w_n];
            
           % replace old dcm with updated
            
            obj.DCM = dcm;
            
            %get euler angles
            
            
                    
          
            
     
            
            %% Calculate the magnitude of the accelerometer vector
            Accel_magnitude = norm(Accelerometer)/obj.GRAVITY;
            % Normalise accelerometer measurement
            Accelerometer = Accelerometer / norm(Accelerometer);    % normalise magnitude
            % Normalise magnetometer measurement
            Magnetometer = Magnetometer / norm(Magnetometer);   % normalise magnitude
            
            %% Drift correction
            
            %*****Roll and Pitch***** FILL IN HERE
            eRollPitch = cross(A(3,:),Accelerometer);         
            eP_RP = eRollPitch*obj.Kp_RP;           % proportional error for pitchroll
            obj.eI_RP = eRollPitch*obj.SamplePeriod + obj.eI_RP;
            eI_RP = obj.eI_RP*obj.Ki_RP;        %integral error for pitchroll
            
            
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
            
            eP_Yaw = eYaw*obj.Kp_Yaw;           % proportional error for yaw
            obj.eI_Yaw = eYaw*obj.SamplePeriod + obj.eI_Yaw;
            eI_Yaw = obj.eI_Yaw*obj.Ki_Yaw;        %integral error for yaw
            
            
            %% PI controller
            %obj.eProp=obj.Kp_Yaw*eYaw+eP;
			%  FILL IN HERE
            %obj.eInt=obj.Ki_Yaw*eYaw*...;           
            
            eProp_RP = eP_RP; %obj.Kp_RP*eRollPitch + eP_RP;
            eInt_RP = eI_RP; %+ obj.Ki_RP*eRollPitch ;            
            
            eProp_Yaw = obj.Kp_Yaw*eYaw + eP_Yaw;
            eInt_Yaw = obj.Ki_Yaw*eYaw + eI_Yaw;           
            
            
            
            % Sum of the gyroscope measurement and all errors
            Gyroscope = Gyroscope + eProp_RP + eInt_RP + eProp_Yaw + eInt_Yaw;
            obj.Gyroest = Gyroscope;
     
        
        
        end
        
        end
    end
