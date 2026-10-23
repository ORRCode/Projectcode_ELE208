%% PARAMETERAR

v = 0.3;                 % [m/s]

% Kameraets plassering i robotbasen sitt koordinatsystem
cam_x = 1.5;             % [m]
cam_y = 1.0;             % [m]
% Treng kanskje z og?

% Region of Interest (ROI) for å avgrense området me leser av
roi_x_min = 100;
roi_x_max = 500;

roi_y_min = 200;
roi_y_max = 400;

%% START KAMERA

cam = webcam;

objectFound = false;

%% OVERVÅKING

while ~objectFound

    img = snapshot(cam);

    gray = rgb2gray(img);

    BW = imbinarize(gray);

    % Avgrens søket til transportbeltet
    BW_roi = BW(roi_y_min:roi_y_max,...
                roi_x_min:roi_x_max);

    % Vis ROI for debugging
    imshow(BW_roi)
    drawnow

    stats = regionprops(BW_roi,...
        'Area',...
        'Centroid',...
        'Orientation');

    if ~isempty(stats)

        % Finn største objekt
        areas = [stats.Area];

        [maxArea,idx] = max(areas);

        % Ignorer små objekt/støy
        if maxArea > 1000

            x_pixel = stats(idx).Centroid(1) + roi_x_min;
            y_pixel = stats(idx).Centroid(2) + roi_y_min;

            theta = stats(idx).Orientation;

            t0 = tic;

            objectFound = true;

            fprintf('Objekt oppdaga!\n')

        end

    end

end

%% PIKSEL -> VERDENSKOORDINATAR
%
% PLACEHOLDER
%
% Seinare skal desse kome frå kamerakalibrering.
% Foreløpig brukar vi pikselverdiar direkte.

x_cam = x_pixel;
y_cam = y_pixel;

%% KAMERA -> ROBOTBASE

x_robot0 = x_cam + cam_x;
y_robot0 = y_cam + cam_y;

%% PREDIKERT PLUKKEPUNKT

elapsedTime = toc(t0);

x_pick = x_robot0 - v*elapsedTime;
y_pick = y_robot0;

theta_pick = theta;

%% RESULTAT

fprintf('\n')
fprintf('Plukkepunkt:\n')
fprintf('x = %.3f\n',x_pick)
fprintf('y = %.3f\n',y_pick)
fprintf('theta = %.2f deg\n',theta_pick)

%% ROS2
%
% publisher/send kjem her seinare
%
% msg.x = x_pick;
% msg.y = y_pick;
% msg.theta = theta_pick;
%
% send(pub,msg);
