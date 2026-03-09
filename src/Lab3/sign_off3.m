tg = TrajGenerator();

t0 = 0; tf = 5;
q0 = 0; qf = 100;
v0 = 0; vf = 0;
acc0 = 0; accf = 0;

cubic = [t0 tf q0 qf v0 vf];
quinitic = [t0 tf q0 qf v0 vf acc0 accf];

a_c = tg.cubic_traj(tg, cubic);                 % 4x1
a_q = tg.quinitic_traj(tg, quinitic);   % 6x1

ts = linspace(t0, tf, 200);

qc = zeros(size(ts));
qq = zeros(size(ts));

for i = 1:length(ts)
    qc(i) = tg.eval_traj(a_c.', ts(i));   % pass as 1x4
    qq(i) = tg.eval_traj(a_q.', ts(i));   % pass as 1x6
end

figure;
plot(ts, qc, 'LineWidth', 2); hold on;
plot(ts, qq, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Position q(t)');
legend('Cubic', 'Quintic');
title('Cubic vs Quintic 1-D Trajectories');
