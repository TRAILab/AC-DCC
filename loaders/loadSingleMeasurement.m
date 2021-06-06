function [T_CW, T_CW_cov, target_points, pixels, gimbal_angles] = loadSingleMeasurement(filename)
fid = fopen( filename );

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
    
    % Read the Transformation matrix
    function readTCW()
        for i=1:4
            tline = fgetl(fid);
            a = sscanf(tline,'%f');
            T_CW(i,:) = a';
        end
        T_CW = tranValidate(T_CW);
    end

    % Read the Transformation matrix
    function readTCWCov()
        for i=1:6
            tline = fgetl(fid);
            a = sscanf(tline,'%f');
            T_CW_cov(i,:) = a';
        end
    end

    % Read the Transformation matrix
    function readTargetPointsPix()
        tline = fgetl(fid);
        while ~strcmp(tline,'')
            a = sscanf(tline,'%f');
            target_points = [target_points; a(1:3)'];
            pixels = [pixels; a(4:5)'];
            tline = fgetl(fid);
        end
    end

    % Read the Transformation matrix
    function readGimbalAngles()
        tline = fgetl(fid);
        a = sscanf(tline,'%f');
        gimbal_angles = a';
    end

assert(size(target_points,1)==size(pixels,1), 'The length of target points and pixels should be the same')

end

