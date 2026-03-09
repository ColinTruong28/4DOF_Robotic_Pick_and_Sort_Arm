tf_edge = 1.0;      
rate_hz = 50;       
dt = 1/rate_hz;     

% Triangle vertices in TASK SPACE [x y z gamma]
pos1 = [93.0814  0 208.0080   0];
pos2 = [50.4615  0  70.4067 -90];
pos3 = [206.0101 0  42.3166 -45];

v0 = 0; 
vf = 0;

%% ------------------- INIT ROBOT + TG -------------------
robot = Robot();
tg = TrajGenerator();

% Move to first vertex
q_start = ik3001(robot, pos1);
robot.interpolate_jp(q_start, 0.5);
pause(0.5);

%% ------------------- BUILD TRAJECTORY COEFFS -------------------
edges = {pos1, pos2;
         pos2, pos3;
         pos3, pos1};

Ax = zeros(3,4);
Ay = zeros(3,4);
Az = zeros(3,4);
Ag = zeros(3,4);

for e = 1:3
    p0 = edges{e,1};
    pf = edges{e,2};

    Ax(e,:) = tg.cubic_traj(tg, [0 tf_edge p0(1) pf(1) v0 vf]).';
    Ay(e,:) = tg.cubic_traj(tg, [0 tf_edge p0(2) pf(2) v0 vf]).';
    Az(e,:) = tg.cubic_traj(tg, [0 tf_edge p0(3) pf(3) v0 vf]).';
    Ag(e,:) = tg.cubic_traj(tg, [0 tf_edge p0(4) pf(4) v0 vf]).';
end

cubic_vel = @(A,t) (A(2) + 2*A(3).*t + 3*A(4).*t.^2);

%% ------------------- PREALLOC LOGS -------------------
maxSamp = ceil(3*tf_edge*rate_hz) + 50;

meas_q = zeros(maxSamp, 4);   % measured joints (deg)
meas_t = zeros(maxSamp, 1);   % global time (s)

x_tar  = zeros(maxSamp, 1);   % target pose (optional)
y_tar  = zeros(maxSamp, 1);
z_tar  = zeros(maxSamp, 1);

vx_tar = zeros(maxSamp, 1);   % target velocity
vy_tar = zeros(maxSamp, 1);
vz_tar = zeros(maxSamp, 1);

idx = 1;
t_global = 0;

%% ------------------- EXECUTE TRAJECTORY -------------------
for e = 1:3
    % Move to start of edge (helps reduce edge discontinuities)
    q_edge_start = ik3001(robot, edges{e,1});
    robot.interpolate_jp(q_edge_start, 0.4);
    pause(0.2);

    t0 = tic;
    nextTick = 0;

    while true
        t = toc(t0);
        if t > tf_edge
            break;
        end

        % Target pose at time t
        x = tg.eval_traj(Ax(e,:), t);
        y = tg.eval_traj(Ay(e,:), t);
        z = tg.eval_traj(Az(e,:), t);
        g = tg.eval_traj(Ag(e,:), t);

        % Target velocities at time t
        vx_t = cubic_vel(Ax(e,:), t);
        vy_t = cubic_vel(Ay(e,:), t);
        vz_t = cubic_vel(Az(e,:), t);

        % IK to joint space and command robot
        q_cmd = ik3001(robot, [x y z g]);
        robot.servo_jp(q_cmd);  % non-blocking, meant for servoing

        % Measure joints (deg) and log
        meas = robot.measure_js(true,false); % [1x4] deg
        if idx > size(meas_q,1)
            % grow logs if needed
            grow = 200;
            meas_q = [meas_q; zeros(grow,4)];
            meas_t = [meas_t; zeros(grow,1)];
            x_tar  = [x_tar;  zeros(grow,1)];
            y_tar  = [y_tar;  zeros(grow,1)];
            z_tar  = [z_tar;  zeros(grow,1)];
            vx_tar = [vx_tar; zeros(grow,1)];
            vy_tar = [vy_tar; zeros(grow,1)];
            vz_tar = [vz_tar; zeros(grow,1)];
        end

        meas_q(idx,:) = meas(1,:);
        meas_t(idx)   = t_global;

        x_tar(idx)    = x;
        y_tar(idx)    = y;
        z_tar(idx)    = z;

        vx_tar(idx)   = vx_t;
        vy_tar(idx)   = vy_t;
        vz_tar(idx)   = vz_t;

        idx = idx + 1;

        % Rate control
        nextTick = nextTick + dt;
        while toc(t0) < nextTick
            pause(0.001);
        end

        t_global = t_global + dt;
    end
end

% Trim logs
meas_q = meas_q(1:idx-1,:);
meas_t = meas_t(1:idx-1);

x_tar  = x_tar(1:idx-1);
y_tar  = y_tar(1:idx-1);
z_tar  = z_tar(1:idx-1);

vx_tar = vx_tar(1:idx-1);
vy_tar = vy_tar(1:idx-1);
vz_tar = vz_tar(1:idx-1);

%% ------------------- FK: JOINT -> TASK SPACE (MEASURED POS) -------------------
p_meas = zeros(size(meas_q,1), 3);  % measured xyz

for i = 1:size(meas_q,1)
    T = fk3001(robot, meas_q(i,:));     % uses measured joints (deg)
    p_meas(i,:) = [T(1,4), T(2,4), T(3,4)];
end

%% ------------------- MEASURED VELOCITY = FINITE DIFFERENCE OF FK -------------------
v_meas = zeros(size(p_meas));  % vx vy vz

for i = 2:size(p_meas,1)
    dt_i = meas_t(i) - meas_t(i-1);
    if dt_i <= 0
        dt_i = dt; % fallback
    end
    v_meas(i,:) = (p_meas(i,:) - p_meas(i-1,:)) / dt_i;
end

%% ------------------- PLOTS: MEASURED vs TARGET VELOCITIES -------------------
figure;
plot(meas_t, v_meas(:,1), 'LineWidth', 1.5); hold on;
plot(meas_t, vx_tar, '--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('v_x (mm/s)');
title('End Effector X Velocity vs Time (Measured vs Target)');
legend('measured','target'); grid on;

figure;
plot(meas_t, v_meas(:,2), 'LineWidth', 1.5); hold on;
plot(meas_t, vy_tar, '--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('v_y (mm/s)');
title('End Effector Y Velocity vs Time (Measured vs Target)');
legend('measured','target'); grid on;

figure;
plot(meas_t, v_meas(:,3), 'LineWidth', 1.5); hold on;
plot(meas_t, vz_tar, '--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('v_z (mm/s)');
title('End Effector Z Velocity vs Time (Measured vs Target)');
legend('measured','target'); grid on;

