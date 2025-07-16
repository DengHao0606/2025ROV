% 定义推进器的相对位置 (单位：米)
r1 = [-98.7725,193.65,0];  % T1
r2 = [98.7725,193.65,0] ;  % T2
r3 = [-130,0,0];  % T3
r4 = [130.0,0,0];  % T4
r5 = [-98.7725,-193.65,0]; % T5
r6 = [98.7725,-193.65,0]; % T6

% 每个推进器的推力方向
f1 = [-sind(35), -cosd(35), 0];  % T1
f2 = [sind(35), -cosd(35), 0];  % T2
f3 = [0, 0, -1]; % T3
f4 = [0, 0, -1]; % T4
f5 = [-sind(35), cosd(35), 0];  % T5
f6 = [sind(35), cosd(35), 0]; % T6

% 定义推进器位置和推力方向的矩阵
positions = [r1; r2; r3; r4; r5; r6];
forces = [f1; f2; f3; f4; f5; f6];

% 初始化推力矩阵
T = zeros(6, 6);

% 填充推力矩阵
for i = 1:6
    % 前3行 (X, Y, Z)
    T(1:3, i) = forces(i, :)';
    
    % 计算并填充后3行 (τφ, τθ, τψ) 的力矩
    torque = calculate_torque(positions(i, :), forces(i, :));
    T(4:6, i) = torque';
end

% 输出推力矩阵
disp('推力矩阵 T:');
disp(T);

% 期望的运动向量 (Vdesired),对应x,y,z,Mx,MY,Mz
V_desired = [1000; 1000; 1000; 0;0; 10000];

% 计算推力矩阵的伪逆

T_inv = pinv(T);
disp('推力矩阵的逆:');
disp(T_inv)
% 计算每个推进器的推力
F_thrusters = T_inv * V_desired;
disp('每个推进器的推力 (N):');
disp(F_thrusters);


figure;
hold on;
grid on;
axis equal;

% 绘制坐标轴
quiver3(0, 0, 0, 1, 0, 0, 'r', 'LineWidth', 1); % X轴
quiver3(0, 0, 0, 0, 1, 0, 'g', 'LineWidth', 1); % Y轴
quiver3(0, 0, 0, 0, 0, 1, 'b', 'LineWidth', 1); % Z轴

% 绘制推进器位置和推力矢量
for i = 1:6
    % 推进器位置
    pos = positions(i, :);
    
    % 推进器推力方向和大小
    force = forces(i, :)*F_thrusters(i);
    
    % 绘制推进器位置
    plot3(pos(1), pos(2), pos(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    
    % 绘制推力矢量
    quiver3(pos(1), pos(2), pos(3), force(1), force(2), force(3), 'LineWidth', 1);
   
    % 标注推进器编号
    text(pos(1), pos(2), pos(3), sprintf(' 电机%d', i), 'FontSize', 12, 'Color', 'k', 'VerticalAlignment', 'bottom');
end

% 设置视角和标签
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('ROV 推进器位置与推力方向');
view(3);
text((r1(1)+r2(1))/2, (r1(2)+r2(2))/2, (r1(3)+r2(3))/2, sprintf(' ROV正面'), 'FontSize', 8, 'Color', 'k', 'VerticalAlignment', 'bottom');
text((r3(1)+r4(1))/2, (r3(2)+r4(2))/2, (r3(3)+r4(3))/2, sprintf(' ROV背面'), 'FontSize', 8, 'Color', 'k', 'VerticalAlignment', 'bottom');
hold off;
% 函数定义部分
function tau = calculate_torque(r, f)
    tau_phi = r(2) * f(3) - r(3) * f(2);  % 横滚力矩 (τφ)
    tau_theta = r(3) * f(1) - r(1) * f(3); % 俯仰力矩 (τθ)
    tau_psi = r(1) * f(2) - r(2) * f(1);  % 偏航力矩 (τψ)
    tau = [tau_phi, tau_theta, tau_psi];
end
