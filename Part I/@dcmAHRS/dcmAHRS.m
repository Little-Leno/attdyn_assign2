classdef dcmAHRS< handle
    %DCM AHRS algorithm
    % direction cosine matrix
    
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
            end;
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
            
            %Calculate Euler angles in example1.m file
            
  
        
          obj.DCM = dcm;
            end
     
        end
end


        