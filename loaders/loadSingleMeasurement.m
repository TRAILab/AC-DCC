function [T_CW, T_CW_cov, target_points, pixels, gimbal_angles] = loadSingleMeasurement(filename)

%% This function reads a single measurement for a single camera from the 
% text file 

fid = fopen(filename);

target_points = []; % 3D target points in target frame
T_CW = []; % Transformation from Target to camera
T_CW_cov = [];
pixels = []; % pixel coordinates of target points
gimbal_angles = []; % gimbal angles

tline = fgetl(fid);
while ~strcmp('end:',tline)
    
    if strcmp(tline, 'T_CW:')
        readTCW();
    
    elseif strcmp(tline, 'T_CW_cov:')
        readTCWCov();
    
    elseif strcmp(tline, 'target_points_pix:')
        readTargetPointsPix();
    
    elseif strcmp(tline, 'gimbalangles:')
        readGimbalAngles();
    end
    tline = fgetl(fid);
end
    
    % Read the Transformation matrix from world to camera
    function readTCW()
        for i=1:4
            tline = fgetl(fid);
            a = sscanf(tline,'%f');
            T_CW(i,:) = a';
        end
        T_CW = tranValidate(T_CW);
    end

    % Reads the covariance matrix of the transformation
    function readTCWCov()
        for i=1:6
            tline = fgetl(fid);
            if isempty(tline)
                break;
            end
            a = sscanf(tline,'%f');
            T_CW_cov(i,:) = a';
        end
    end

    % Reads the points and pixels from the measurement file
    function readTargetPointsPix()
        tline = fgetl(fid);
        while ~strcmp(tline,'')
            a = sscanf(tline,'%f');
            target_points = [target_points; a(1:3)'];
            pixels = [pixels; a(4:5)'];
            tline = fgetl(fid);
        end
    end

    % Reads the gimbal angles
    function readGimbalAngles()
        tline = fgetl(fid);
        a = sscanf(tline,'%f');
        gimbal_angles = a';
    end

assert(size(target_points,1)==size(pixels,1), 'The length of target points and pixels should be the same')

end

