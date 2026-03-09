% Create a robot object
robot = Robot();

ternary = @(varargin) varargin{end - varargin{1}};
interpolate_time = 2.5;
% Define three joint positions to form the vertices of your triangle
% Make sure theta_1 = 0 for all of these configurations!
pos1 = [0 -90 45 45];
pos2 = [0 -45 45 90];
pos3 = [0 0 45 0];



poses1 = zeros(2000, 4);
times1 = zeros(2000, 1);
poses2 = zeros(2000, 4);
times2 = zeros(2000, 1);
poses3 = zeros(2000, 4);
times3 = zeros(2000, 1);

% Move your robot to the first vertex

robot.writeJoints(pos1);
pause(2);

poses = zeros(2000, 4);

% Initialize another array for the timestamps (or use the same array as the
% positions, your call)

times = zeros(2000, 1);

   incrementer = 1;


% Create a for loop that sends your robot to each vertex, recording the
% joint-space position the entire time
for i=1:3
    % See lab_demo.m from lab 1 for inspiration on how to move your robot
    % and collect data

% TODO: Call a function to move the robot
poses = zeros(2000, 4);
times = zeros(2000, 1);
incrementer = 1;
interpolate_jp(robot, ternary(i==1, pos2, ternary(i==2, pos3, pos1)), 2);
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
if (i == 1)
    poses2 = zeros(incrementer - 1, 4);
    times2 = zeros(incrementer - 1, 1);

    for j = 1:incrementer - 1
        poses2(j, :) = poses(j, :);
        times2(j) = times(j);
    end
elseif (i == 2)
    poses3 = zeros(incrementer - 1, 4);
    times3 = zeros(incrementer - 1, 1);

    for j = 1:incrementer - 1
        poses3(j, :) = poses(j, :);
        times3(j) = times(j);
    end
elseif (i == 3)
    poses1 = zeros(incrementer - 1, 4);
    times1 = zeros(incrementer - 1, 1);

    for j = 1:incrementer - 1
        poses1(j, :) = poses(j, :);
        times1(j) = times(j);
    end
end
end



% Loop through all collected data and convert it from joint-space to task
% space using your fk3001 function

task_space = zeros(height(poses1) + height(poses2) + height(poses3), 3);
poses = zeros(height(poses2) + height(poses3) + height(poses1), 4);

for i = 1:height(poses2)
    temp = fk3001(robot, poses2(i, :));
    poses(i, :) = poses2(i, :);
    task_space(i, :) = [temp(1, 4) temp(2, 4) temp(3, 4)];
end
for i = 1:height(poses3)
    temp = fk3001(robot, poses3(i, :));
    poses(i + height(poses2), :) = poses3(i, :);
    task_space(i + height(poses2), :) = [temp(1, 4) temp(2, 4) temp(3, 4)];
end
for i = 1:height(poses1)
    temp = fk3001(robot, poses1(i, :));
    poses(i + height(poses2) + height(poses3), :) = poses1(i, :);
    task_space(i + height(poses2) + height(poses3), :) = [temp(1, 4) temp(2, 4) temp(3, 4)];
end

% Plot the trajectory of your robot in x-y-z space using scatter3
% https://www.mathworks.com/help/matlab/ref/scatter3.html
figure
graph = scatter3(task_space(:, 1), task_space(:, 2), task_space(:, 3));
plot(graph);
% Plot the trajectory of your robot in theta2-theta3-theta-4 space using
% scatter3
% figure
% graph_j = scatter3(poses(:, 2), poses(:, 3), poses(:, 4), 'filled');
% plot(graph_j);
