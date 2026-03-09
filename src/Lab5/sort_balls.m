% In this file, write your "main" script for picking up the balls
% This should only be 10-15 lines long, the vast majority of the work
% should be in your Robot and ImageProcessor classes

% Ultimately it's up to you to decide how to organize this, but the
% "detect_balls" and "pick_up_ball" functions should do a lot of the work
% for you.

% @@@@@@@@@@@@@@
% YOUR CODE HERE
% @@@@@@@@@@@@@@
robot = Robot();
cd('/home/gordoc/rbe3001/RBE3001_A25_13-main/camera_calibration/team7photos')
img = ImageProcessor;
mask = img.generate_static_mask();
disp("before");
pause;
disp("after");

masked = uint8(double(img.camera.getImage()) .* double(mask));  % mask is 0/1

centroid = img.detect_centroids(masked);

ts_centroid = img.correct_centroids(centroid);


%robot.writeGripper(true);
%pause(1);
%robot.writeGripper(false);
%pause(1);
blocking_js_move(robot, [0 -45 35 20]);
pause(2);

while height(ts_centroid) > 0
    PICK_UP_BALL(robot, [(ts_centroid(1, 2) + 79) (ts_centroid(1, 1) - 106)], img.colors(1));
    
    blocking_js_move(robot, [0 -45 35 20], time = 1);
    pause(1);

    masked = uint8(double(img.camera.getImage()) .* double(mask));  % mask is 0/1

    centroid = img.detect_centroids(masked);

    ts_centroid = img.correct_centroids(centroid);
end
%blocking_js_move(robot, [0 -45 35 20]);