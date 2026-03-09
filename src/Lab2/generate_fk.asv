robot = Robot(1);
%% Step 1: Define your symbolic DH table
% See sym_example.m for guidance on how to define symbolic variables
% Remember: you CAN freely mix symbolic variabes and plain-ol numbers.
syms theta1 theta2 theta3 theta4
%% Step 2: Pass your symbolic DH table into dh2fk to get your symbolic 
% FK matrix
% It's really that simple. MATLAB will leave everything unsolved
% Show this to an SA for SIGN-OFF #4

temp = [0 60 0 0; 0 36.326 0 -90; -79.38 0 sqrt((128^2)+(24^2)) 0; 79.38 0 124 0; 0 0 133.4 0];
disp(dh2fk(robot, temp));

temp1 = [0 60 0 0; theta1 36.326 0 -90; theta2-79.38 0 sqrt((128^2)+(24^2)) 0; theta3+79.38 0 124 0; theta4 0 133.4 0];
%disp(dh2fk(robot, temp1));

%% Step 3: Feed your symbolic FK matrix into 'matlabFunction' to turn it
% into a floating point precision function that runs fast.
T_sym = dh2fk(robot, temp1);

fkFunc = matlabFunction(T_sym);
% Write the fk_3001 function in Robot.m to complete sign-off #5
save('fk3001_function.mat', 'fkFunc');
disp(fk3001(robot, [0, 0, 0, 0]));
% Curiosity bonus (0 points): replicate the timeit experiment I did in
% sym_example.m to compare the matlabFunction FK function to just using
% subs to substitute the variables.

%% Lab 4 Breakoff

% --- FK symbolic ---
T_sym = dh2fk(robot, temp1);
p_end = T_sym(1:3,4);
R_end = T_sym(1:3,1:3);

% --- Linear part (correct) ---
Jv = [ diff(p_end, theta1), diff(p_end, theta2), diff(p_end, theta3), diff(p_end, theta4) ];  % 3x4

% --- Build z axes the way you already do (with fixed base row handled) ---
dh_fixed  = temp1(1,:);
dh_joints = temp1(2:5,:);

p = sym(zeros(3,5));
z = sym(zeros(3,5));

T_fixed = dh2fk(robot, dh_fixed);
p(:,1) = T_fixed(1:3,4);
z(:,1) = T_fixed(1:3,3);

for i = 1:4
    T0_i = dh2fk(robot, [dh_fixed; dh_joints(1:i,:)]);
    p(:,i+1) = T0_i(1:3,4);
    z(:,i+1) = T0_i(1:3,3);
end

% Two angular candidates
Jw_A = [z(:,1) z(:,2) z(:,3) z(:,4)]; % z0..z3
Jw_B = [z(:,2) z(:,3) z(:,4) z(:,5)]; % z1..z4

% ---- Compare against finite-difference angular Jacobian (BODY convention) ----
% omega_body = vee(R^T * Rdot)
eps_deg = 1e-3;

% pick a non-singular-ish test pose in DEGREES
qtest = [20 -30 40 10];

% numeric FK handle (yours is correct)
T0 = fk3001(robot, qtest);
R0 = T0(1:3,1:3);

Jw_fd = zeros(3,4);
for i = 1:4
    dq = zeros(1,4); dq(i) = eps_deg;
    T1 = fk3001(robot, qtest + dq);
    R1 = T1(1:3,1:3);

    Rdot = (R1 - R0)/eps_deg;         % per degree
    W = R0.' * Rdot;                  % body form

    % vee(W) for skew matrix
    Jw_fd(:,i) = [W(3,2); W(1,3); W(2,1)];
end

% Evaluate candidates at qtest
JwA_num = double(subs(Jw_A, [theta1 theta2 theta3 theta4], qtest));
JwB_num = double(subs(Jw_B, [theta1 theta2 theta3 theta4], qtest));

errA = norm(JwA_num - Jw_fd, 'fro');
errB = norm(JwB_num - Jw_fd, 'fro');

fprintf("errA (z(:,1:4)) = %.6f\n", errA);
fprintf("errB (z(:,2:5)) = %.6f\n", errB);

if errA <= errB
    Jw = Jw_A;
else
    Jw = Jw_B;
end

% ---- Final Jacobian (keep your current row order first: [v; w]) ----
J = [Jv; Jw];

jacobian = matlabFunction(J, 'Vars', {theta1,theta2,theta3,theta4});
save("jacobian_mat.mat","jacobian");