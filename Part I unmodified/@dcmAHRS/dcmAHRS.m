classdef dcmAHRS< handle
    %DCM AHRS algorithm
    %
    
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
        function obj = Update(obj, Gyroscope)
            %% Update rotation matrix (DCM)   
            FILL IN HERE
          obj.DCM=???
            
     
        end
        
        end
    end
