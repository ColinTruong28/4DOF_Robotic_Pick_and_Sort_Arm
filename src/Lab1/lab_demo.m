%% USE THIS FILE TO GET SIGN OFF 3

robot = Robot();  % Create a robot object to use
interpolate_time = 2.5; % TODO: choose a reasonable time to interpolate

%% Send the robot to its 0 position
% We don't need to collect data yet, as we're just putting the robot at its
% starting point

% YOUR CODE HERE
servo_jp(robot, 0);

%% Send the robot to an arbitrary position
% TODO: Choose a joint position

pos1 = [75 -45 20 35];

% TODO: Initialize an array full of 0s to store the positions
% See https://www.mathworks.com/help/matlab/ref/zeros.html
% We do this because allocating memory takes a lot of time, so we only want
% to do it once
incrementer = 1;

poses = zeros(800, 4);

% Initialize another array for the timestamps (or use the same array as the
% positions, your call)

times = zeros(800, 1);

% TODO: Call a function to move the robot
interpolate_jp(robot, pos1, 2);
% Collect data as the robot moves
tic;  % Start timer
while toc < interpolate_time
    % Read current joint positions (not velocities though!)
    temp = measure_js(robot, true, false);
    % Store the positions and timesetamps in an array 
    % Hint: use 'toc' to get the current elapsed time since tic
    
    poses(incrementer, :) = temp(1, :);
    times(incrementer) = toc;

    incrementer = incrementer + 1;
end
posesF = zeros(incrementer - 1, 4);
timesF = zeros(incrementer - 1, 1);

for i = 1:incrementer - 1
    posesF(i, :) = poses(i, :);
    timesF(i) = times(i);
end

%% Make your figure
% To use subfigures, you'll use the subplots feature of MATLAB.
% https://www.mathworks.com/help/matlab/ref/subplot.html

% In each subplot you create, you can use 'plot' to plot one joint value vs
% time
% https://www.mathworks.com/help/matlab/ref/plot.html

% Remember titles, labels, and units!

%% Calculate time step statistics
% MATLAB may have some functions for the requested statistics

fprintf('Time Step Data\nMean: %f; Median: %f; Max: %f; Min: %f', mean(timesF), median(timesF), max(timesF), min(timesF));

figure
plot(timesF, posesF(:,1),timesF, posesF(:, 2),'--',timesF, posesF(:, 3),'-p',timesF, posesF(:, 4),'g');
title("4DOF Arm Joint Positions Graph");
xlabel("Time (s)");
ylabel("Position (degrees)");