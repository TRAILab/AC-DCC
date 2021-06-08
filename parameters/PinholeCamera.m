classdef PinholeCamera<handle
    
    properties
        sensor_name
        sensor_id
        model_type
        distortion_type
        width
        height
        fx
        fy
        cx
        cy
        k1
        k2
        k3
        p1
        p2
    end
    
    methods
        function obj = PinholeCamera(yaml_map)
            obj.sensor_name = yaml_map('sensor_name');
            obj.sensor_id = yaml_map('sensor_id');
            obj.model_type = yaml_map('model_type');
            obj.distortion_type = yaml_map('distortion_type');
            obj.width = str2double(yaml_map('width'));
            obj.height = str2double(yaml_map('height'));
            obj.fx = str2double(yaml_map('fx'));
            obj.fy = str2double(yaml_map('fy'));
            obj.cx = str2double(yaml_map('cx'));
            obj.cy = str2double(yaml_map('cy'));
            obj.k1 = str2double(yaml_map('k1'));
            obj.k2 = str2double(yaml_map('k2'));
            obj.k3 = str2double(yaml_map('k3'));
            obj.p1 = str2double(yaml_map('p1'));
            obj.p2 = str2double(yaml_map('p2'));
        end
        
        function pixels = project(obj, points)
            
            num_pts = size(points,1);
            
            x_dash = points(:,1)./points(:,3);
            y_dash = points(:,2)./points(:,3);
            
            r2 = x_dash.^2 + y_dash.^2;
            
            temp = ones(num_pts, 1) + obj.k1*r2 + obj.k2*(r2.^2) + obj.k3*(r2.^3);
            
            x_ddash = x_dash.*temp + 2*obj.p1*(x_dash.*y_dash) + obj.p2*(r2+2*(x_dash.^2));
            y_ddash = y_dash.*temp + obj.p1*(r2+2*(y_dash.^2)) + 2*obj.p2*(x_dash.*y_dash);
            
            pixels(:,1) = obj.fx*x_ddash + obj.cx*ones(num_pts,1);
            pixels(:,2) = obj.fy*y_ddash + obj.cy*ones(num_pts,1);
        end
        
        function ax = plotPixels(obj, pixels)
            if isempty(findobj('type','figure','name',obj.sensor_name))
                f = figure('Name',obj.sensor_name);   
            else
                f = figure(findobj('type','figure','name',obj.sensor_name).Number);    
            end
            clf; 
            set(f,'Visible','off');
            if(~isempty(pixels))
                scatter(pixels(:,1),-1*pixels(:,2),'filled','r');
                hold on;
                scatter(pixels(1,1),-1*pixels(1,2),'filled','g');
            end
            xlim([0 obj.width])
            ylim([-obj.height 0])
            title(obj.sensor_name)
            pbaspect([4 3 1])
            ax = gca;
        end
        
        function pixel_indices = getIndicesOnImage(obj, pixels, camera_points)
            xs = pixels(:,1);
            ys = pixels(:,2);
            pixel_indices = find(xs>0 & xs<obj.width & ys>0 & ys<obj.height & camera_points(:,3)>0);
        end
        
        function J = projectionJacobian(obj, point)
            q1 = point(1);
            q2 = point(2);
            q3 = point(3);
            J = [ (obj.fx*(7*obj.k3*q1^6 + 15*obj.k3*q1^4*q2^2 + 5*obj.k2*q1^4*q3^2 + 9*obj.k3*q1^2*q2^4 + 6*obj.k2*q1^2*q2^2*q3^2 + 3*obj.k1*q1^2*q3^4 + 6*obj.p2*q1*q3^5 + obj.k3*q2^6 + obj.k2*q2^4*q3^2 + obj.k1*q2^2*q3^4 + 2*obj.p1*q2*q3^5 + q3^6))/q3^7,                                                               obj.fx*((2*obj.p1*q1)/q3^2 + (2*obj.p2*q2)/q3^2 + (q1*((2*obj.k1*q2)/q3^2 + (4*obj.k2*q2*(q1^2 + q2^2))/q3^4 + (6*obj.k3*q2*(q1^2 + q2^2)^2)/q3^6))/q3), -(obj.fx*(7*obj.k3*q1^7 + 21*obj.k3*q1^5*q2^2 + 5*obj.k2*q1^5*q3^2 + 21*obj.k3*q1^3*q2^4 + 10*obj.k2*q1^3*q2^2*q3^2 + 3*obj.k1*q1^3*q3^4 + 6*obj.p2*q1^2*q3^5 + 7*obj.k3*q1*q2^6 + 5*obj.k2*q1*q2^4*q3^2 + 3*obj.k1*q1*q2^2*q3^4 + 4*obj.p1*q1*q2*q3^5 + q1*q3^6 + 2*obj.p2*q2^2*q3^5))/q3^8;
                obj.fy*((2*obj.p1*q1)/q3^2 + (2*obj.p2*q2)/q3^2 + (q2*((2*obj.k1*q1)/q3^2 + (4*obj.k2*q1*(q1^2 + q2^2))/q3^4 + (6*obj.k3*q1*(q1^2 + q2^2)^2)/q3^6))/q3), (obj.fy*(obj.k3*q1^6 + 9*obj.k3*q1^4*q2^2 + obj.k2*q1^4*q3^2 + 15*obj.k3*q1^2*q2^4 + 6*obj.k2*q1^2*q2^2*q3^2 + obj.k1*q1^2*q3^4 + 2*obj.p2*q1*q3^5 + 7*obj.k3*q2^6 + 5*obj.k2*q2^4*q3^2 + 3*obj.k1*q2^2*q3^4 + 6*obj.p1*q2*q3^5 + q3^6))/q3^7, -(obj.fy*(7*obj.k3*q1^6*q2 + 21*obj.k3*q1^4*q2^3 + 5*obj.k2*q1^4*q2*q3^2 + 21*obj.k3*q1^2*q2^5 + 10*obj.k2*q1^2*q2^3*q3^2 + 3*obj.k1*q1^2*q2*q3^4 + 2*obj.p1*q1^2*q3^5 + 4*obj.p2*q1*q2*q3^5 + 7*obj.k3*q2^7 + 5*obj.k2*q2^5*q3^2 + 3*obj.k1*q2^3*q3^4 + 6*obj.p1*q2^2*q3^5 + q2*q3^6))/q3^8];
        end
        
        function [p,J] = projectAndJacobian(obj,point)
            p = obj.project(point);
            J = obj.projectionJacobian(point);
        end
    end
    
end

