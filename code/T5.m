function q5_five_uav_three_missiles
    % 无人机初始位置
    R0_U1 = [17800, 0, 1800];   
    R0_U2 = [12000, 1400, 1400]; 
    R0_U3 = [6000, -3000, 700];  
    R0_U4 = [11000, 2000, 1800]; 
    R0_U5 = [13000, -2000, 1300];
    uav_list = {R0_U1, R0_U2, R0_U3, R0_U4, R0_U5};
    
    % 导弹初始位置
    r0_M1 = [20000, 0, 2000]; 
    r0_M2 = [19000, 600, 2100]; 
    r0_M3 = [18000, -600, 1900];
    missile_list = {r0_M1, r0_M2, r0_M3};
    missile_names = {'M1', 'M2', 'M3'};
    
    target_base = [0, 200, 0];  
    g = 9.8;                    
    R = 10.0;                   
    v_M = 300;              
    v_c = 3.0;                  
    t_window = 20;              
    
    % 生成真目标采样点
    Q_pts = build_target_samples(36, 21);
    
    % IGA 参数 
    pop_size = 300;             % 种群大小
    gens = 100;                 % 迭代次数
    pc = 0.9;                   % 交叉概率
    pm = 0.3;                   % 变异概率
    dir_dev_range_deg = 20;     % 方向角偏差范围
    num_cx_points = 2;          % 交叉点数
    
    % 每架无人机最多3枚弹，每枚弹4个参数（θ, v, td, tb）
    num_bombs_per_uav = 3;      % 每架无人机烟幕弹数量
    num_params_per_bomb = 4;    % 每枚烟幕弹参数数量
    num_uavs = 5;               % 无人机数量
    total_vars = num_uavs * num_bombs_per_uav * num_params_per_bomb; % 总变量数
    
    % 初始化种群
    pop = init_population(pop_size, num_uavs, num_bombs_per_uav);
    best_fit = -1;              % 最佳适应度值
    best_sol = [];              % 最佳解决方案
    
    % 优化过程
    for gen = 1:gens
        fits = zeros(pop_size, 1); % 适应度值数组
        
        % 计算每个个体的适应度
        for i = 1:pop_size
            fits(i) = fitness_multi_missile(pop(i,:), uav_list, missile_list, ...
                target_base, g, R, v_M, v_c, t_window, Q_pts, num_bombs_per_uav);
        end
        
        % 更新最佳解
        [fmax, elite_idx] = max(fits);
        if fmax > best_fit
            best_fit = fmax;
            best_sol = pop(elite_idx,:);
            fprintf('Gen %d: 发现新最佳遮蔽时间=%.3f s\n', gen, best_fit);
        end
        
        new_pop = best_sol;  % 精英保留
        
        fmin = min(fits);
        while size(new_pop, 1) < pop_size
            % 选择父代
            idx = randi(pop_size, [1, 2]);
            p1 = pop(idx(1), :);
            p2 = pop(idx(2), :);
            
            % 自适应交叉概率
            if fmax ~= fmin
                fit1 = fitness_multi_missile(p1, uav_list, missile_list, ...
                    target_base, g, R, v_M, v_c, t_window, Q_pts, num_bombs_per_uav);
                fit2 = fitness_multi_missile(p2, uav_list, missile_list, ...
                    target_base, g, R, v_M, v_c, t_window, Q_pts, num_bombs_per_uav);
                pc_adapt = pc * (fmax - max(fit1, fit2)) / (fmax - fmin);
            else
                pc_adapt = pc;
            end
            
            % 交叉操作
            if rand < pc_adapt
                [c1, c2] = multi_point_crossover(p1, p2, num_cx_points);
            else
                c1 = p1;
                c2 = p2;
            end
            
            % 变异操作
            c1 = directed_mutation_multi(c1, pm, missile_list, target_base, ...
                dir_dev_range_deg, num_bombs_per_uav);
            c2 = directed_mutation_multi(c2, pm, missile_list, target_base, ...
                dir_dev_range_deg, num_bombs_per_uav);
            
            % 修复解
            c1 = repair_multi_missile(c1, missile_list, v_M, num_bombs_per_uav);
            c2 = repair_multi_missile(c2, missile_list, v_M, num_bombs_per_uav);
            
            % 添加到新种群
            new_pop = [new_pop; c1];
            if size(new_pop, 1) < pop_size
                new_pop = [new_pop; c2];
            end
        end
        pop = new_pop(1:pop_size, :);
        
        % 定期显示进度
        if mod(gen, 10) == 0
            fprintf('Gen %d: 当前最佳遮蔽时间=%.3f s\n', gen, best_fit);
        end
    end
    
    % 输出结果并保存到Excel
    output_results_to_excel(best_sol, best_fit, uav_list, missile_list, missile_names, ...
        target_base, g, R, v_M, v_c, t_window, Q_pts, num_bombs_per_uav);
end


function pop = init_population(pop_size, num_uavs, num_bombs_per_uav)
    % 初始化种群
    total_vars = num_uavs * num_bombs_per_uav * 4;
    pop = zeros(pop_size, total_vars);
    for i = 1:pop_size
        idx = 1;
        for uav = 1:num_uavs
            for bomb = 1:num_bombs_per_uav
                pop(i, idx) = rand() * 360;          
                pop(i, idx+1) = 70 + rand() * 70;   
                pop(i, idx+2) = rand() * 15;          
                pop(i, idx+3) = rand() * 15;        
                idx = idx + 4;
            end
        end
    end
end

function [c1, c2] = multi_point_crossover(p1, p2, num_points)
    % 多点交叉操作
    % 输入:
    %   p1, p2 - 父代个体
    %   num_points - 交叉点数
    % 输出:体
    %   c1, c2 - 子代个
    
    len = length(p1);
    cx_points = sort(randperm(len-2, num_points) + 1);
    
    c1 = p1;
    c2 = p2;
    swap = false;
    
    for i = 1:length(cx_points)+1
        if i == 1
            start_idx = 1;
            end_idx = cx_points(1)-1;
        elseif i == length(cx_points)+1
            start_idx = cx_points(end);
            end_idx = len;
        else
            start_idx = cx_points(i-1);
            end_idx = cx_points(i)-1;
        end
        
        if swap
            c1(start_idx:end_idx) = p2(start_idx:end_idx);
            c2(start_idx:end_idx) = p1(start_idx:end_idx);
        end
        swap = ~swap;
    end
end

function ind = directed_mutation_multi(ind, pm, missile_list, target_base, ...
    dev_range_deg, num_bombs_per_uav)
    % 定向变异操作
    % 输入:
    %   ind - 个体
    %   pm - 变异概率
    %   missile_list - 导弹列表
    %   target_base - 目标位置
    %   dev_range_deg - 方向角偏差范围
    %   num_bombs_per_uav - 每架无人机烟幕弹数量
    % 输出:
    %   ind - 变异后的个体
    
    num_uavs = length(missile_list);
    for uav = 1:num_uavs
        for bomb = 1:num_bombs_per_uav
            idx = ((uav-1)*num_bombs_per_uav + bomb - 1)*4 + 1;
            if rand < pm
                % 计算导弹方向角
                delta_x = target_base(1) - missile_list{uav}(1);
                delta_y = target_base(2) - missile_list{uav}(2);
                theta_target_rad = atan2(delta_y, delta_x);
                theta_target_deg = mod(rad2deg(theta_target_rad), 360);
                
                current_theta = ind(idx);
                delta = theta_target_deg - current_theta;
                delta = mod(delta + 180, 360) - 180;
                adjust = sign(delta) * min(abs(delta), dev_range_deg) * (rand - 0.5)*2;
                new_theta = current_theta + adjust;
                ind(idx) = mod(new_theta, 360);
            end
            % 变异其他参数
            for i = 1:3
                if rand < pm
                    if i == 1  % v
                        ind(idx+i) = max(70, min(140, ind(idx+i) + (rand*2-1)*10));
                    else       % td, tb
                        ind(idx+i) = max(0, ind(idx+i) + (rand*2-1)*1);
                    end
                end
            end
        end
    end
end

function ind = repair_multi_missile(ind, missile_list, v_M, num_bombs_per_uav)
    % 修复个体，确保参数在合理范围内
    % 输入:
    %   ind - 个体
    %   missile_list - 导弹列表
    %   v_M - 导弹速度
    %   num_bombs_per_uav - 每架无人机烟幕弹数量
    % 输出:
    %   ind - 修复后的个体
    
    num_uavs = length(missile_list);
    Tmaxs = zeros(1, num_uavs); % 各导弹最大飞行时间
    
    % 计算各导弹最大飞行时间
    for i = 1:num_uavs
        d = -missile_list{i};
        L = norm(d);
        Tmaxs(i) = L / v_M;
    end
    
    % 修复每个烟幕弹的参数
    for uav = 1:num_uavs
        Tmax = Tmaxs(uav);
        for bomb = 1:num_bombs_per_uav
            idx = ((uav-1)*num_bombs_per_uav + bomb - 1)*4 + 1;
            theta = mod(ind(idx), 360);
            v = min(max(ind(idx+1), 70), 140);
            td = max(0, ind(idx+2));
            tb = max(0, ind(idx+3));
            
            % 确保起爆时间在导弹到达之前
            if td + tb >= Tmax
                tb = max(0, Tmax - td - 0.1);
            end
            
            ind(idx:idx+3) = [theta, v, td, tb];
        end
    end
end

function val = fitness_multi_missile(ind, uav_list, missile_list, target_base, ...
    g, R, v_M, v_c, smoke_effective_time, Q_pts, num_bombs_per_uav)
    % 计算适应度函数（总有效遮蔽时间）
    % 输入:
    %   ind - 个体
    %   uav_list - 无人机列表
    %   missile_list - 导弹列表
    %   target_base - 目标位置
    %   g - 重力加速度
    %   R - 烟幕有效半径
    %   v_M - 导弹速度
    %   v_c - 烟幕下沉速度
    %   smoke_effective_time - 烟幕有效时间
    %   Q_pts - 目标采样点
    %   num_bombs_per_uav - 每架无人机烟幕弹数量
    % 输出:
    %   val - 总有效遮蔽时间
    
    num_uavs = length(uav_list);
    num_missiles = length(missile_list);
    S = cell(1, num_missiles);  % 为每个导弹存储烟幕信息
    
    % 为每个无人机和烟幕弹计算烟幕信息
    for uav = 1:num_uavs
        R0_U = uav_list{uav};  % 无人机初始位置
        for bomb = 1:num_bombs_per_uav
            idx = ((uav-1)*num_bombs_per_uav + bomb - 1)*4 + 1;
            theta_deg = ind(idx);
            v = ind(idx+1);
            td = ind(idx+2);
            tb = ind(idx+3);
            
            theta_rad = deg2rad(theta_deg);
            u_uav = [cos(theta_rad), sin(theta_rad), 0];
            
            % 计算投放点和起爆点
            r_d = R0_U + v * td * u_uav; % 投放点
            c0 = r_d + [0, 0, -0.5 * g * tb^2]; % 起爆点
            ton = td + tb; % 起爆时间
            
            % 为每个导弹计算烟幕有效时间
            for m = 1:num_missiles
                r0_M = missile_list{m}; % 导弹初始位置
                d = -r0_M;
                L = norm(d);
                Tmax = L / v_M; % 导弹最大飞行时间
                toff = min(ton + smoke_effective_time, Tmax); % 烟幕结束时间
                
                % 如果起爆时间在导弹到达之前，则记录烟幕信息
                if ton < Tmax
                    S{m} = [S{m}; c0, ton, toff, uav, bomb];
                end
            end
        end
    end
    
    % 计算总有效遮蔽时间
    total_cover = 0;
    for m = 1:num_missiles
        if isempty(S{m}), continue; end
        r0_M = missile_list{m};
        d = -r0_M;
        u_m = d / norm(d); % 导弹方向向量
        Tmax = norm(d) / v_M; % 导弹最大飞行时间
        
        events = []; % 事件点列表
        
        % 收集所有事件点（起爆、结束和遮蔽状态变化点）
        for i = 1:size(S{m},1)
            ton = S{m}(i,4);
            toff = S{m}(i,5);
            events = [events, max(0, ton), max(0, toff)];
            
            % 计算与目标采样点相关的事件点
            for j = 1:size(Q_pts,1)
                ev = line_sphere_events_in_time(r0_M, u_m, Q_pts(j,:), ...
                    S{m}(i,1:3), ton, toff, R, v_M, v_c);
                events = [events, ev];
            end
        end
        
        % 处理事件点
        events = unique(events);
        events = sort(events);
        if isempty(events)
            continue;
        end
        if events(1) > 0, events = [0, events]; end
        if events(end) < Tmax, events = [events, Tmax]; end
        
        % 计算每个时间段的遮蔽情况
        cover = 0;
        for k = 1:length(events)-1
            tmid = 0.5 * (events(k) + events(k+1)); % 时间段中点
            covered = false; % 是否被遮蔽
            
            % 计算导弹在tmid时刻的位置
            r_M = r0_M + v_M * tmid * u_m;
            
            % 检查每个烟幕弹是否在该时间段内有效遮蔽
            for i = 1:size(S{m},1)
                ton_i = S{m}(i,4);
                toff_i = S{m}(i,5);
                
                % 如果烟幕在tmid时刻有效
                if ton_i <= tmid && tmid <= toff_i
                    % 计算烟幕中心在tmid时刻的位置
                    C = S{m}(i,1:3) + [0,0,-v_c*(tmid-ton_i)];
                    
                    % 检查是否遮蔽导弹与目标的视线
                    for j = 1:size(Q_pts,1)
                        if segment_point_distance_sq(r_M, Q_pts(j,:), C) <= R^2
                            covered = true;
                            break;
                        end
                    end
                end
                if covered, break; end
            end
            
            % 如果被遮蔽，累加遮蔽时间
            if covered
                cover = cover + (events(k+1) - events(k));
            end
        end
        total_cover = total_cover + cover;
    end
    val = total_cover;
end

function output_results_to_excel(best_sol, best_fit, uav_list, missile_list, missile_names, ...
    target_base, g, R, v_M, v_c, smoke_effective_time, Q_pts, num_bombs_per_uav)
    % 输出结果到Excel文件
    % 输入:
    %   best_sol - 最佳解
    %   best_fit - 最佳适应度值
    %   uav_list - 无人机列表
    %   missile_list - 导弹列表
    %   missile_names - 导弹名称
    %   target_base - 目标位置
    %   g - 重力加速度
    %   R - 烟幕有效半径
    %   v_M - 导弹速度
    %   v_c - 烟幕下沉速度
    %   smoke_effective_time - 烟幕有效时间
    %   Q_pts - 目标采样点
    %   num_bombs_per_uav - 每架无人机烟幕弹数量
    
    [bomb_cover_times, bomb_missile_info] = compute_bomb_metrics(best_sol, uav_list, missile_list, ...
        target_base, g, R, v_M, v_c, smoke_effective_time, Q_pts, num_bombs_per_uav);
    
    result_data = cell(17, 12); 
    
    headers = {'无人机编号', '无人机运动方向', '无人机运动速度 (m/s)', '烟幕干扰弹编号', ...
               '烟幕干扰弹投放点的x坐标 (m)', '烟幕干扰弹投放点的y坐标 (m)', '烟幕干扰弹投放点的z坐标 (m)', ...
               '烟幕干扰弹起爆点的x坐标 (m)', '烟幕干扰弹起爆点的y坐标 (m)', '烟幕干扰弹起爆点的z坐标 (m)', ...
               '有效干扰时长 (s)', '干扰的导弹编号'};
    
    for i = 1:length(headers)
        result_data{1, i} = headers{i};
    end
    
    % 填充数据
    row_idx = 2;
    uav_names = {'FY1', 'FY2', 'FY3', 'FY4', 'FY5'};
    
    for uav = 1:5
        R0_U = uav_list{uav};
        for bomb = 1:num_bombs_per_uav
            idx = ((uav-1)*num_bombs_per_uav + bomb - 1)*4 + 1;
            theta_deg = best_sol(idx);
            v = best_sol(idx+1);
            td = best_sol(idx+2);
            tb = best_sol(idx+3);
            
            theta_rad = deg2rad(theta_deg);
            u_uav = [cos(theta_rad), sin(theta_rad), 0];
            
            % 计算投放点和起爆点坐标
            r_d = R0_U + v * td * u_uav;
            c0 = r_d + [0, 0, -0.5 * g * tb^2];
            
            % 填充数据到表格
            result_data{row_idx, 1} = uav_names{uav};   
            result_data{row_idx, 2} = theta_deg;        
            result_data{row_idx, 3} = v;                   
            result_data{row_idx, 4} = bomb;                 
            result_data{row_idx, 5} = r_d(1);                
            result_data{row_idx, 6} = r_d(2);          
            result_data{row_idx, 7} = r_d(3);          
            result_data{row_idx, 8} = c0(1);           
            result_data{row_idx, 9} = c0(2);            
            result_data{row_idx, 10} = c0(3);              
            result_data{row_idx, 11} = bomb_cover_times(uav, bomb);  
            result_data{row_idx, 12} = bomb_missile_info{uav, bomb};  
            
            row_idx = row_idx + 1;
        end
    end
    
    
    % 写入Excel文件
    filename = 'result3.xlsx';
    writecell(result_data, filename, 'Sheet', 'Sheet1');
    
    
    % 显示详细信息
    fprintf('\n各烟雾弹详细信息:\n');
    for uav = 1:5
        for bomb = 1:num_bombs_per_uav
            fprintf('%s-弹%d: 干扰时间=%.3f s, 干扰导弹=%s\n', ...
                uav_names{uav}, bomb, bomb_cover_times(uav, bomb), bomb_missile_info{uav, bomb});
        end
    end
end

function [bomb_cover_times, bomb_missile_info] = compute_bomb_metrics(ind, uav_list, missile_list, ...
    target_base, g, R, v_M, v_c, smoke_effective_time, Q_pts, num_bombs_per_uav)
    % 计算每个烟雾弹的有效干扰时间和干扰的导弹信息
    % 输入:
    %   ind - 个体
    %   uav_list - 无人机列表
    %   missile_list - 导弹列表
    %   target_base - 目标位置
    %   g - 重力加速度
    %   R - 烟幕有效半径
    %   v_M - 导弹速度
    %   v_c - 烟幕下沉速度
    %   smoke_effective_time - 烟幕有效时间
    %   Q_pts - 目标采样点
    %   num_bombs_per_uav - 每架无人机烟幕弹数量
    % 输出:
    %   bomb_cover_times - 每个烟雾弹的有效干扰时间矩阵
    %   bomb_missile_info - 每个烟雾弹干扰的导弹信息细胞数组
    
    num_uavs = length(uav_list);
    num_missiles = length(missile_list);
    bomb_cover_times = zeros(num_uavs, num_bombs_per_uav);
    bomb_missile_info = cell(num_uavs, num_bombs_per_uav);
    
    % 为每个导弹计算每个烟雾弹的贡献
    for m = 1:num_missiles
        r0_M = missile_list{m};
        d = -r0_M;
        u_m = d / norm(d);
        Tmax = norm(d) / v_M;
        
        % 收集该导弹的所有烟幕信息
        S = [];   % 烟幕信息矩阵
        bomb_info = []; % 烟幕弹信息矩阵
        
        for uav = 1:num_uavs
            R0_U = uav_list{uav};
            for bomb = 1:num_bombs_per_uav
                idx = ((uav-1)*num_bombs_per_uav + bomb - 1)*4 + 1;
                theta_deg = ind(idx);
                v = ind(idx+1);
                td = ind(idx+2);
                tb = ind(idx+3);
                
                theta_rad = deg2rad(theta_deg);
                u_uav = [cos(theta_rad), sin(theta_rad), 0];
                
                % 计算投放点和起爆点
                r_d = R0_U + v * td * u_uav;
                c0 = r_d + [0, 0, -0.5 * g * tb^2];
                ton = td + tb;
                toff = min(ton + smoke_effective_time, Tmax);
                
                % 如果起爆时间在导弹到达之前，则记录烟幕信息
                if ton < Tmax
                    S = [S; c0, ton, toff];
                    bomb_info = [bomb_info; uav, bomb];
                end
            end
        end
        
        if isempty(S), continue; end
        
        % 计算事件点
        events = [];
        for i = 1:size(S,1)
            ton = S(i,4);
            toff = S(i,5);
            events = [events, max(0, ton), max(0, toff)];
            
            % 计算与目标采样点相关的事件点
            for j = 1:size(Q_pts,1)
                ev = line_sphere_events_in_time(r0_M, u_m, Q_pts(j,:), ...
                    S(i,1:3), ton, toff, R, v_M, v_c);
                events = [events, ev];
            end
        end
        
        events = unique(events);
        events = sort(events);
        if isempty(events), continue; end
        if events(1) > 0, events = [0, events]; end
        if events(end) < Tmax, events = [events, Tmax]; end
        
        % 为每个时间段确定哪些烟雾弹起作用
        for k = 1:length(events)-1
            tmid = 0.5 * (events(k) + events(k+1));
            r_M = r0_M + v_M * tmid * u_m; % 导弹在tmid时刻的位置
            
            for i = 1:size(S,1)
                uav_idx = bomb_info(i,1);
                bomb_idx = bomb_info(i,2);
                ton_i = S(i,4);
                toff_i = S(i,5);
                
                % 如果烟幕在tmid时刻有效
                if ton_i <= tmid && tmid <= toff_i
                    % 计算烟幕中心在tmid时刻的位置
                    C = S(i,1:3) + [0,0,-v_c*(tmid-ton_i)];
                    
                    % 检查是否对该导弹有遮蔽效果
                    for j = 1:size(Q_pts,1)
                        if segment_point_distance_sq(r_M, Q_pts(j,:), C) <= R^2
                            % 该烟雾弹在这个时间段有贡献
                            time_contribution = events(k+1) - events(k);
                            bomb_cover_times(uav_idx, bomb_idx) = bomb_cover_times(uav_idx, bomb_idx) + time_contribution;
                            
                            % 记录干扰的导弹信息
                            if isempty(bomb_missile_info{uav_idx, bomb_idx})
                                bomb_missile_info{uav_idx, bomb_idx} = sprintf('M%d', m);
                            else
                                if ~contains(bomb_missile_info{uav_idx, bomb_idx}, sprintf('M%d', m))
                                    bomb_missile_info{uav_idx, bomb_idx} = [bomb_missile_info{uav_idx, bomb_idx} ', M' num2str(m)];
                                end
                            end
                            break;
                        end
                    end
                end
            end
        end
    end
    
    % 处理没有干扰任何导弹的情况
    for uav = 1:num_uavs
        for bomb = 1:num_bombs_per_uav
            if isempty(bomb_missile_info{uav, bomb})
                bomb_missile_info{uav, bomb} = '无';
            end
        end
    end
end

%% ===================== 几何辅助函数 =====================

function d2 = segment_point_distance_sq(P, Q, C)
    % 计算线段PQ到点C的最短距离平方
    % 输入:
    %   P, Q - 线段端点
    %   C - 点
    % 输出:
    %   d2 - 最短距离平方
    
    D = Q - P;
    denom = dot(D, D);
    if denom == 0
        d2 = dot(P - C, P - C);
        return;
    end
    u = dot(C - P, D) / denom;
    u_clamped = max(0, min(1, u));
    X = P + u_clamped * D;
    d2 = dot(X - C, X - C);
end

function samples = build_target_samples(n_theta, side_layers)
    % 构建目标采样点
    % 输入:
    %   n_theta - 角度采样数
    %   side_layers - 高度采样层数
    % 输出:
    %   samples - 采样点矩阵
    
    pts = [];
    r = 7;       % 圆柱半径
    y0 = 200;    % 圆柱中心y坐标
    height = 10; % 圆柱高度
    
    % 生成角度采样点
    thetas_deg = linspace(0, 360, n_theta+1);
    thetas_deg(end) = [];
    
    % 生成顶部和底部圆周点
    for th = thetas_deg
        theta_rad = deg2rad(th);
        x = r * cos(theta_rad);
        y = y0 + r * sin(theta_rad);
        pts = [pts; x, y, height]; % 顶部
        pts = [pts; x, y, 0];      % 底部
    end
    
    % 生成侧面采样点
    zs = linspace(0, height, side_layers);
    for th = thetas_deg
        theta_rad = deg2rad(th);
        x = r * cos(theta_rad);
        y = y0 + r * sin(theta_rad);
        for z = zs
            pts = [pts; x, y, z];
        end
    end
    samples = pts;
end

function events = line_sphere_events_in_time(r0_M, u_m, Qj, C0, ton, toff, R, ...
    v_M, v_c)
    % 计算线段与球体相交的时间事件
    % 输入:
    %   r0_M - 导弹初始位置
    %   u_m - 导弹方向向量
    %   Qj - 目标点
    %   C0 - 烟幕初始位置
    %   ton - 烟幕开始时间
    %   toff - 烟幕结束时间
    %   R - 烟幕有效半径
    %   v_M - 导弹速度
    %   v_c - 烟幕下沉速度
    % 输出:
    %   events - 事件点列表
    
    events = [];
    if toff <= ton, return; end
    
    % 定义距离函数
    f = @(t) segment_point_distance_sq(...
        r0_M + v_M * t * u_m, ... % 导弹位置
        Qj, ...                   % 目标点
        C0 + [0, 0, -v_c * (t - ton)] ... % 烟幕位置
    ) - R^2;
    
    N = 40;
    ts = linspace(ton, toff, N+1);
    vals = arrayfun(f, ts);
    
    % 检查是否整个区间都在球内或球外
    if all(vals <= 0)
        events = [ton, toff];
        return;
    elseif all(vals >= 0)
        return;
    end
    
    % 查找事件点
    for k = 1:N
        a = ts(k);
        b = ts(k+1);
        fa = vals(k);
        fb = vals(k+1);
        
        if abs(fa) < 1e-9
            events(end+1) = a;
        end
        
        if fa * fb <= 0
            lo = a;
            hi = b;
            for iter = 1:40
                mid = 0.5 * (lo + hi);
                fm = f(mid);
                if abs(fm) < 1e-9
                    lo = mid;
                    hi = mid;
                    break;
                end
                if fa * fm <= 0
                    hi = mid;
                    fb = fm;
                else
                    lo = mid;
                    fa = fm;
                end
            end
            root = 0.5 * (lo + hi);
            if root >= ton - 1e-9 && root <= toff + 1e-9
                events(end+1) = root;
            end
        end
    end
end

function deg = rad2deg(rad)
    % 弧度转角度
    deg = rad * 180 / pi;
end

function rad = deg2rad(deg)
    % 角度转弧度
    rad = deg * pi / 180;
end
