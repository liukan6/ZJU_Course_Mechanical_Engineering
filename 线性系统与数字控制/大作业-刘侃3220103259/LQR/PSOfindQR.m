%% 定义系统
clear; clc;
load("state_space.mat");
A_d = sysd.A;
B_d = sysd.B;
C_d = sysd.C;
D_d = sysd.D;

gbest_pos=[1e3, 1e3, 1e3, 1e3, 1e3, 1e3, 1e3];

while ((gbest_pos(1)==1e3) + (gbest_pos(2)==1e3) + (gbest_pos(3)==1e3))
    %% PSO参数设置
    n_particles = 20;   % 粒子数量
    max_iter = 15;      % 最大迭代次数
    dim = 7;            % 优化变量维度 (q1, q2, q3, v1, v2, v3, R)
    
    % 参数范围 [q1_min, q2_min, q3_min, v1_min, v2_min, v3_min, R_min; q1_max, q2_max, q3_max, v1_max, v2_max, v3_max, R_max]
    lb = [0.000, 0.000, 0.000, 0.00, 0.00, 0.00, 0.1];  % 下界
    ub = [1e3, 1e3, 1e3, 1e3, 1e3, 1e3, 1e3];    % 上界
    
    % PSO参数
    w = 0.5;            % 惯性权重
    c1 = 1.6;           % 个体学习因子
    c2 = 1.8;           % 社会学习因子
    
    % 初始化粒子群
    particles = zeros(n_particles, dim);
    velocities = zeros(n_particles, dim);
    pbest_pos = zeros(n_particles, dim);
    pbest_val = inf(n_particles, 1);
    gbest_val = inf;
    gbest_pos = zeros(1, dim);
    
    % 随机初始化粒子位置和速度
    for i = 1:n_particles
        particles(i, :) = lb + (ub - lb) .* rand(1, dim);
        velocities(i, :) = 0.1 * (ub - lb) .* rand(1, dim);
    end
    
    % 预分配适应度记录
    fitness_history = zeros(max_iter, 1);
    
    %% PSO主循环
    for iter = 1:max_iter
        for i = 1:n_particles
            % 提取当前粒子的Q和R参数
            q_params = particles(i, 1:3);
            v_params = particles(i, 4:6); % 提取速度状态权重
            R_param = particles(i, 7);
            
            % 构建Q矩阵
            Q = diag([q_params, v_params]); % 包含速度状态权重
            
            % 计算适应度
            fitness_val = calculate_fitness(A_d, B_d, C_d, D_d, Ts, Q, R_param);
            
            % 更新个体最优
            if fitness_val < pbest_val(i)
                pbest_val(i) = fitness_val;
                pbest_pos(i, :) = particles(i, :);
            end
            
            % 更新全局最优
            if fitness_val < gbest_val
                gbest_val = fitness_val;
                gbest_pos = particles(i, :);
            end
        end
        
        % 记录当前最佳适应度
        fitness_history(iter) = gbest_val;
        
        % 更新粒子速度和位置
        for i = 1:n_particles
            r1 = rand(1, dim);
            r2 = rand(1, dim);
            
            velocities(i, :) = w * velocities(i, :) + ...
                              c1 * r1 .* (pbest_pos(i, :) - particles(i, :)) + ...
                              c2 * r2 .* (gbest_pos - particles(i, :));
            
            particles(i, :) = particles(i, :) + velocities(i, :);
            
            % 边界处理
            particles(i, :) = max(particles(i, :), lb);
            particles(i, :) = min(particles(i, :), ub);
        end
        
        % 显示迭代信息
        fprintf('迭代 %d: 最佳适应度 = %.4f, 参数: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', ...
                iter, gbest_val, gbest_pos(1), gbest_pos(2), gbest_pos(3), gbest_pos(4), gbest_pos(5), gbest_pos(6), gbest_pos(7));
    end

end

%% 保存优化后的参数设计LQR控制器
R = gbest_pos(7);
Q = diag(round(gbest_pos(1:6),2));
save("LQRpara.mat","Q","R");


%% 适应度函数定义
function fitness = calculate_fitness(A_d, B_d, C_d, D_d, Ts, Q, R)
    try
        % 设计LQR控制器
        [K_d, ~, ~] = dlqr(A_d, B_d, Q, R);
        
        % 创建闭环系统
        sys_cl = ss(A_d - B_d*K_d, B_d, C_d, D_d, Ts);
        
        % 计算脉冲响应
        Tf = 10;
        t_sim = 0:Ts:Tf;
        [y, ~] = impulse(sys_cl, t_sim);
        
        % 计算性能指标 (ITSE - 积分时间平方误差)
        y1 = y(:,1);
        y2 = y(:,2);
        y3 = y(:,3);
        
        % 计算每个输出的ITSE
        itse1 = trapz(t_sim, t_sim'.*(y1.^2));
        itse2 = trapz(t_sim, t_sim'.*(y2.^2));
        itse3 = trapz(t_sim, t_sim'.*(y3.^2));
        
        % 总适应度值
        fitness = itse1 + 10*itse2 + 10*itse3;
        
        % 添加稳定性惩罚项
        poles = eig(A_d - B_d*K_d);
        if any(abs(poles) > 1)
            fitness = fitness + 1000; % 不稳定系统的惩罚
        end
        
    catch
        % 处理数值计算错误
        fitness = 1e10;
    end
end