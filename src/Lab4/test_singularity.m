% test singularity
robot = Robot();
tg = TrajGenerator();
% Move your robot to a point where Y=0, X > 150 and Z > 150
q_start = ik3001(robot, [0 150 230 0]);
robot.interpolate_jp(q_start, 2);
pause(2);

% Generate a trajectory that will move your robot in a straight line from
% your current (X, Y, Z) to the point (X, -Y, Z)
v0 = 0; vf = 0;
tf = 2;

Ax = zeros(1, 4);
Ay = zeros(1, 4);
Az = zeros(1, 4);
Ag = zeros(1, 4);

Ax(1, :) = tg.cubic_traj(tg, [0 tf 0 0 v0 vf]);
Ay(1, :) = tg.cubic_traj(tg, [0 tf 150 -150 v0 vf]);
Az(1, :) = tg.cubic_traj(tg, [0 tf 230 230 v0 vf]);
Ag(1, :) = tg.cubic_traj(tg, [0 tf 0 0 v0 vf]);

%% Execute the trajectory
travelTime = 10;
tic;

ts = linspace(0, tf, 300);
xj = zeros(1, 300);
yj = zeros(1, 300);
zj = zeros(1, 300);
gj = zeros(1, 300);

detj = zeros(1, 300);
yLog = zeros(1, 300);
while toc < travelTime
    % Evaluate your trajectory, and send your robot to its next setpoint
    
    % Record the joint position of your robot
    for k = 1:300
        t = ts(k);

        x = tg.eval_traj(Ax(1, :), t);
        y = tg.eval_traj(Ay(1, :), t);
        z = tg.eval_traj(Az(1, :), t); 
        g = tg.eval_traj(Ag(1, :), t);

        qcmd = ik3001(robot, [0 y 230 g]);
        
        J = jacob3001(robot, qcmd);

        Jp = J(1:3, :);

        detj(k) = det(Jp * Jp.');
        yLog(k) = -y;
        
        if (atSingularity(robot, qcmd, 0.1))
            disp('ERROR!!!! AT SINGULARITY!');
            break;
        else
            robot.interpolate_jp(qcmd, 20/300);
        end
    end
    % Record the determinant of the Jacobian
    % Trim unused elements if stopped early
    detj = detj(1:k);
    yLog = yLog(1:k);
    
    figure;
    plot(yLog, detj, 'LineWidth', 2);
    xlabel('Y Position');
    ylabel('Determinant of Jacobian');
    title('Jacobian Determinant vs Y Position');
    grid on;
    break;
    % Test if your robot is too close to a singularity, and stop if it is
end