function t3final
    % 常量定义
    params_base.r0_M1 = [20000, 0, 2000];  
    params_base.R0_U1 = [17800, 0, 1800];    
    params_base.g = 9.8;                
    params_base.R = 10.0;                   
    
    % 真目标采样点
    samples = build_target_samples(24, 3);
    
    % IGA参数设置
    pop_size = 200;                    
    gens = 100;                        
    pc = 0.9;                         
    pm = 0.25;                         
    pm_max = 0.7;             
    pm_reset_val = 0.1;                  
    pm_high_gens = 7;                        
    pm_high_count = 0;                       
    immigrants_every = 20;                
    immigrants_frac = 0.1;                  
    stall_limit = 30;                        
    repeat_trigger = 3;                     
    
    % 初始化种群
    pop = init_population(pop_size);
    best_fit = -1;                     
    best_sol = [];                       
    best_stall = 0;                   
    repeat_count = 0;                   
    % 主循环
    for gen = 1:gens
        fits = zeros(pop_size, 1);
        for i = 1:pop_size
            fits(i) = fitness(pop(i, :), params_base, samples);
        end
        
        [fmax, elite_idx] = max(fits);
        favg = mean(fits);
        
        fit_var = var(fits);
        if fit_var < 1e-4
            fprintf('>>> 种群适应度方差 %.6g 小于阈值 1e-4，提前结束进化。\n', fit_var);
            best_fit = fmax;
            best_sol = pop(elite_idx, :);
            break;
        end

        % 更新最佳解和计数器
        if fmax > best_fit + 1e-9
            best_fit = fmax;
            best_sol = pop(elite_idx, :);
            best_stall = 0;
            repeat_count = 0;
        else
            best_stall = best_stall + 1;
            repeat_count = repeat_count + 1;
        end
        
        % 触发局部优化
        if repeat_count >= repeat_trigger
            fprintf('>>> 连续 %d 代无提升，触发跳出局部最优措施...\n', repeat_trigger);
            pm = min(pm * 2, pm_max);
            pm_high_count = 0;
            half = floor(pop_size / 2);
            pop = [best_sol; repmat(best_sol, half - 1, 1); init_population(pop_size - half)];
            
            % 扰动部分个体
            for i = 2:half
                pop(i, 3:8) = pop(i, 3:8) + (rand(1, 6) - 0.5) .* 0.6;
                pop(i, :) = repair(pop(i, :), params_base);
            end
            
            % 局部优化
            [best_sol, best_fit] = local_refine(best_sol, best_fit, params_base, samples);
            repeat_count = 0;
        end
        
        % 高变异率处理
        if pm > pm_reset_val
            pm_high_count = pm_high_count + 1;
            if pm_high_count >= pm_high_gens
                fprintf('>>> 高变异率已持续 %d 代，恢复到 %.2f\n', pm_high_gens, pm_reset_val);
                pm = pm_reset_val;
                pm_high_count = 0;
            end
        end
        
        % 创建新种群
        new_pop = best_sol;
        
        % 随机移民
        if immigrants_every > 0 && mod(gen, immigrants_every) == 0
            n_imm = max(1, round(pop_size * immigrants_frac));
            for ii = 1:n_imm
                new_pop = [new_pop; init_population(1)];
            end
        end
        
        % 选择与交叉
        while size(new_pop, 1) < pop_size
            idx = randi(pop_size, [1, 2]);
            p1 = pop(idx(1), :);
            p2 = pop(idx(2), :);
            f1 = fits(idx(1));
            f2 = fits(idx(2));
            fprime = max(f1, f2);
            
            % 自适应交叉概率
            if fmax ~= favg
                pc_adapt = pc - (pc - 0.6) * (fmax - fprime) / (fmax - favg);
            else
                pc_adapt = pc;
            end
            
            % 交叉操作
            if rand < pc_adapt
                cx_point = randi([2, length(p1) - 1]);
                c1 = [p1(1:cx_point), p2(cx_point + 1:end)];
                c2 = [p2(1:cx_point), p1(cx_point + 1:end)];
            else
                c1 = p1;
                c2 = p2;
            end
            
            % 变异操作
            for cc = 1:2
                if cc == 1
                    c = c1;
                else
                    c = c2;
                end
                
                f_ind = fitness(c, params_base, samples);
                
                % 自适应变异概率
                if fmax ~= favg
                    pm_adapt = pm - (pm - 0.05) * (fmax - f_ind) / (fmax - favg);
                else
                    pm_adapt = pm;
                end
                
                if rand < pm_adapt
                    % 参数扰动
                    for k = 3:8
                        c(k) = c(k) + (rand * 0.8 - 0.4);
                    end
                    c(1) = c(1) + (rand * 0.10 - 0.05);
                    c(2) = c(2) + (rand * 10 - 5);
                end
                
                c = repair(c, params_base);
                
                if cc == 1
                    c1 = c;
                else
                    c2 = c;
                end
                
                new_pop = [new_pop; c];
                if size(new_pop, 1) >= pop_size
                    break;
                end
            end
        end
        
        pop = new_pop(1:pop_size, :);
        
        % 停滞重启
        if best_stall >= stall_limit
            half = floor(pop_size / 2);
            pop = [best_sol; repmat(best_sol, half - 1, 1); init_population(pop_size - half)];
            
            for i = 2:half
                pop(i, 3:8) = pop(i, 3:8) + (rand(1, 6) - 0.5) .* 0.6;
                pop(i, :) = repair(pop(i, :), params_base);
            end
            
            best_stall = 0;
        end
        
        % 定期局部优化
        if mod(gen, 20) == 0
            [best_sol, best_fit] = local_refine(best_sol, best_fit, params_base, samples);
        end
        
        fprintf('Gen %d: best_fit=%.3f\n', gen, best_fit);
    end
    
    % 输出结果
    fprintf('\n=== 最优解参数 ===\n');
    fprintf('航向角 θ: %.6f rad\n', best_sol(1));
    fprintf('无人机速度 v: %.6f m/s\n', best_sol(2));
    fprintf('投放延时 td: %.6f, %.6f, %.6f s\n', best_sol(3), best_sol(4), best_sol(5));
    fprintf('起爆延时 tb: %.6f, %.6f, %.6f s\n', best_sol(6), best_sol(7), best_sol(8));
    fprintf('最优遮蔽时间: %.6f s\n', best_fit);
    
    % 保存结果到Excel
    save_results_to_excel(best_sol, best_fit, params_base, samples);
    
    % 可视化
    figure('Name', '导弹遮蔽状态时间序列', 'Position', [100, 100, 800, 400]);
    plot_missile_cover_status(best_sol, params_base, samples);
    saveas(gcf, 'missile_cover_status.png');
    
    function plot_missile_cover_status(ind, params_base, samples)
        params = params_base;
        params.theta = ind(1);
        params.v = ind(2);
        params.td = ind(3:5);
        params.tb = ind(6:8);
        
        [u_m, Tmax] = missile_dir_and_Tmax(params.r0_M1);
        u_uav = [cos(params.theta), sin(params.theta), 0];
        
        S = [];
        for i = 1:length(params.td)
            Pdrop = params.R0_U1 + params.v * params.td(i) * u_uav;
            C0 = smoke_center_on_ignite(Pdrop, params.tb(i), params.g);
            ton = params.td(i) + params.tb(i);
            toff = min(ton + 20.0, Tmax);
            
            if ton < Tmax
                S = [S; C0, ton, toff, i];
            end
        end
        
        time_resolution = 0.01;
        t_values = 0:time_resolution:10;
        cover_status = zeros(size(t_values));
        
        for i = 1:length(t_values)
            t = t_values(i);
            covered = false;
            
            for j = 1:size(S, 1)
                ton = S(j, 4);
                toff = S(j, 5);
                
                if t >= ton && t <= toff
                    Pm = Pm_of_t(params.r0_M1, u_m, t);
                    C = Ci_of_t(S(j, 1:3), ton, t);
                    
                    for k = 1:size(samples, 1)
                        if segment_point_distance_sq(Pm, samples(k, :), C) <= params.R^2
                            covered = true;
                            break;
                        end
                    end
                    
                    if covered
                        break;
                    end
                end
            end
            
            cover_status(i) = covered;
        end
        
        % 绘图
        stairs(t_values, cover_status, 'b-', 'LineWidth', 1.5);
        xlabel('时间 (s)');
        ylabel('遮蔽状态');
        title('导弹遮蔽状态时间序列 (0-10秒, 遮蔽:1, 未遮蔽:0)');
        grid on;
        ylim([-0.1 1.1]);
        xlim([0 10]);
        
        % 标记起爆时间
        colors = ['r', 'g', 'm'];
        for j = 1:size(S, 1)
            ton = S(j, 4);
            if ton >= 0 && ton <= 10
                line([ton ton], [-0.1 1.1], 'Color', colors(j), 'LineStyle', '--', 'LineWidth', 1);
                text(ton, 1.05, sprintf('弹%d起爆', S(j, 6)), 'Color', colors(j));
            end
        end
        
        total_cover_time = sum(cover_status) * time_resolution;
        text(0.02, 0.1, sprintf('0-10秒内遮蔽时间: %.2f s', total_cover_time), 'BackgroundColor', 'white');
    end
end

% 计算单弹遮蔽时间
function individual_cover_times = calculate_individual_cover_times(ind, params_base, samples)
    params = params_base;
    params.theta = ind(1);
    params.v = ind(2);
    params.td = ind(3:5);
    params.tb = ind(6:8);
    
    individual_cover_times = zeros(1, 3);
    
    for bomb_idx = 1:3
        temp_params = params;
        temp_params.td = params.td(bomb_idx);
        temp_params.tb = params.tb(bomb_idx);
        individual_cover_times(bomb_idx) = single_bomb_cover_time(temp_params, samples);
    end
end

% 单弹遮蔽时间计算
function cover_time = single_bomb_cover_time(params, samples)
    [u_m, Tmax] = missile_dir_and_Tmax(params.r0_M1);
    u_uav = [cos(params.theta), sin(params.theta), 0];
    
    % 烟幕弹信息
    Pdrop = params.R0_U1 + params.v * params.td * u_uav;
    C0 = smoke_center_on_ignite(Pdrop, params.tb, params.g);
    ton = params.td + params.tb;
    toff = min(ton + 20.0, Tmax);
    
    if ton >= Tmax
        cover_time = 0;
        return;
    end
    
    % 事件点计算
    events = [max(0, ton), max(0, toff)];
    
    for j = 1:size(samples, 1)
        ev = line_sphere_events_in_time(params.r0_M1, u_m, samples(j, :), C0, ton, toff, params.R);
        events = [events, ev];
    end
    
    events = unique(events);
    events = sort(events);
    
    if events(1) > 0
        events = [0, events];
    end
    
    if events(end) < Tmax
        events = [events, Tmax];
    end

    % 计算遮蔽时间
    cover_time = 0;
    for k = 1:length(events) - 1
        tmid = 0.5 * (events(k) + events(k + 1));
        covered = false;
        
        if ton <= tmid && tmid <= toff
            Pm = Pm_of_t(params.r0_M1, u_m, tmid);
            C = Ci_of_t(C0, ton, tmid);
            
            for j = 1:size(samples, 1)
                if segment_point_distance_sq(Pm, samples(j, :), C) <= params.R^2
                    covered = true;
                    break;
                end
            end
        end
        
        if covered
            cover_time = cover_time + (events(k + 1) - events(k));
        end
    end
end

% 保存结果到Excel
function save_results_to_excel(best_sol, best_fit, params_base, samples)
    individual_times = calculate_individual_cover_times(best_sol, params_base, samples);
    theta_deg = mod(rad2deg(best_sol(1)), 360);
    v_uav = best_sol(2);
    td = best_sol(3:5);
    tb = best_sol(6:8);
    u_uav = [cos(best_sol(1)), sin(best_sol(1)), 0];
    
    drop_points = zeros(3, 3);
    ignite_points = zeros(3, 3);
    
    for i = 1:3
        drop_points(i, :) = params_base.R0_U1 + v_uav * td(i) * u_uav;
        ignite_points(i, :) = drop_points(i, :) + [0, 0, -0.5 * params_base.g * tb(i)^2];
    end
    
    % 创建结果表
    result_data = cell(5, 10);
    
    % 表头
    result_data{1, 1} = '无人机运动方向 (°)';
    result_data{1, 2} = '无人机运动速度 (m/s)';
    result_data{1, 3} = '烟幕干扰弹编号';
    result_data{1, 4} = '烟幕干扰弹投放点的x坐标 (m)';
    result_data{1, 5} = '烟幕干扰弹投放点的y坐标 (m)';
    result_data{1, 6} = '烟幕干扰弹投放点的z坐标 (m)';
    result_data{1, 7} = '烟幕干扰弹起爆点的x坐标 (m)';
    result_data{1, 8} = '烟幕干扰弹起爆点的y坐标 (m)';
    result_data{1, 9} = '烟幕干扰弹起爆点的z坐标 (m)';
    result_data{1, 10} = '有效干扰时长 (s)';
    
    % 烟幕弹1数据
    result_data{2, 1} = theta_deg;
    result_data{2, 2} = v_uav;
    result_data{2, 3} = 1;
    result_data{2, 4} = drop_points(1, 1);
    result_data{2, 5} = drop_points(1, 2);
    result_data{2, 6} = drop_points(1, 3);
    result_data{2, 7} = ignite_points(1, 1);
    result_data{2, 8} = ignite_points(1, 2);
    result_data{2, 9} = ignite_points(1, 3);
    result_data{2, 10} = individual_times(1);
    
    % 烟幕弹2数据
    result_data{3, 1} = '';
    result_data{3, 2} = '';
    result_data{3, 3} = 2;
    result_data{3, 4} = drop_points(2, 1);
    result_data{3, 5} = drop_points(2, 2);
    result_data{3, 6} = drop_points(2, 3);
    result_data{3, 7} = ignite_points(2, 1);
    result_data{3, 8} = ignite_points(2, 2);
    result_data{3, 9} = ignite_points(2, 3);
    result_data{3, 10} = individual_times(2);
    
    % 烟幕弹3数据
    result_data{4, 1} = '';
    result_data{4, 2} = '';
    result_data{4, 3} = 3;
    result_data{4, 4} = drop_points(3, 1);
    result_data{4, 5} = drop_points(3, 2);
    result_data{4, 6} = drop_points(3, 3);
    result_data{4, 7} = ignite_points(3, 1);
    result_data{4, 8} = ignite_points(3, 2);
    result_data{4, 9} = ignite_points(3, 3);
    result_data{4, 10} = individual_times(3);
    
    % 备注
    result_data{5, 1} = '注：以x轴为正向，逆时针方向为正，取值0~360（度）。';
    
    % 写入Excel
    filename = 'result1.xlsx';
    writecell(result_data, filename, 'Sheet', 'Sheet1');
    fprintf('结果已成功保存到文件: %s\n', filename);
    fprintf('无人机运动方向: %.2f° (以x轴为正向，逆时针方向为正)\n', theta_deg);
    fprintf('无人机运动速度: %.2f m/s\n', v_uav);
    fprintf('烟幕弹1有效干扰时长: %.2f s\n', individual_times(1));
    fprintf('烟幕弹2有效干扰时长: %.2f s\n', individual_times(2));
    fprintf('烟幕弹3有效干扰时长: %.2f s\n', individual_times(3));
    fprintf('总有效干扰时长: %.2f s\n', best_fit);
end

% 弧度转角度
function deg = rad2deg(rad)
    deg = rad * 180 / pi;
end

% 初始化种群
function pop = init_population(pop_size)
    pop = zeros(pop_size, 8);
    for i = 1:pop_size
        pop(i, :) = [rand * 2 * pi, 70 + rand * 70, ...
                     rand * 5, 1 + rand * 5, 2 + rand * 5, ...
                     rand * 5, rand * 5, rand * 5];
    end
end

% 局部优化
function [best_sol, best_fit] = local_refine(best_sol, best_fit, params_base, samples)
    delta = [-0.4, 0, 0.4];
    for d3 = delta
        for d4 = delta
            for d5 = delta
                cc = best_sol;
                cc(3) = cc(3) + d3;
                cc(4) = cc(4) + d4;
                cc(5) = cc(5) + d5;
                cc = repair(cc, params_base);
                fcc = fitness(cc, params_base, samples);
                if fcc > best_fit
                    best_fit = fcc;
                    best_sol = cc;
                end
            end
        end
    end
end

% 修复解
function ind = repair(ind, params_base)
    theta = mod(ind(1), 2 * pi);
    v = min(max(ind(2), 70), 140);
    td1 = max(0, ind(3));
    td2 = ind(4);
    td3 = ind(5);
    
    % 确保投放时间间隔
    if td2 - td1 < 1
        td2 = td1 + 1;
    end
    
    if td3 - td2 < 1
        td3 = td2 + 1;
    end
    
    tb1 = max(0, ind(6));
    tb2 = max(0, ind(7));
    tb3 = max(0, ind(8));
    
    [~, Tmax] = missile_dir_and_Tmax(params_base.r0_M1);
    
    % 确保起爆时间有效
    if td3 + tb3 >= Tmax
        tb3 = max(0, Tmax - td3 - 0.01);
    end
    
    ind = [theta, v, td1, td2, td3, tb1, tb2, tb3];
end

% 适应度函数
function val = fitness(ind, params_base, samples)
    ind = repair(ind, params_base);
    params = params_base;
    params.theta = ind(1);
    params.v = ind(2);
    params.td = ind(3:5);
    params.tb = ind(6:8);
    val = cover_time_union(params, samples);
end

% 联合遮蔽时间计算
function total_cover = cover_time_union(params, samples)
    [u_m, Tmax] = missile_dir_and_Tmax(params.r0_M1);
    u_uav = [cos(params.theta), sin(params.theta), 0];
    
    % 烟幕弹信息
    S = [];
    for i = 1:length(params.td)
        Pdrop = params.R0_U1 + params.v * params.td(i) * u_uav;
        C0 = smoke_center_on_ignite(Pdrop, params.tb(i), params.g);
        ton = params.td(i) + params.tb(i);
        toff = min(ton + 20.0, Tmax);
        
        if ton < Tmax
            S = [S; C0, ton, toff];
        end
    end
    
    if isempty(S)
        total_cover = 0;
        return;
    end
    
    % 事件点计算
    events = [];
    for i = 1:size(S, 1)
        ton = S(i, 4);
        toff = S(i, 5);
        events = [events, max(0, ton), max(0, toff)];
        
        for j = 1:size(samples, 1)
            ev = line_sphere_events_in_time(params.r0_M1, u_m, samples(j, :), S(i, 1:3), ton, toff, params.R);
            events = [events, ev];
        end
    end
    
    events = unique(events);
    events = sort(events);
    
    if events(1) > 0
        events = [0, events];
    end
    
    if events(end) < Tmax
        events = [events, Tmax];
    end

    % 计算总遮蔽时间
    total_cover = 0;
    for k = 1:length(events) - 1
        tmid = 0.5 * (events(k) + events(k + 1));
        covered = false;
        
        for i = 1:size(S, 1)
            ton = S(i, 4);
            toff = S(i, 5);
            
            if ton <= tmid && tmid <= toff
                Pm = Pm_of_t(params.r0_M1, u_m, tmid);
                C = Ci_of_t(S(i, 1:3), ton, tmid);
                
                for j = 1:size(samples, 1)
                    if segment_point_distance_sq(Pm, samples(j, :), C) <= params.R^2
                        covered = true;
                        break;
                    end
                end
            end
            
            if covered
                break;
            end
        end
        
        if covered
            total_cover = total_cover + (events(k + 1) - events(k));
        end
    end
end

% ================ 几何计算函数 ================

% 导弹方向与最大时间
function [u, Tmax] = missile_dir_and_Tmax(Pm0)
    d = -Pm0;
    L = norm(d);
    u = d / L;
    Tmax = L / 300.0;
end

% 导弹位置计算
function Pm = Pm_of_t(Pm0, u, t)
    Pm = Pm0 + 300.0 * t * u;
end

% 烟幕起爆点计算
function C0 = smoke_center_on_ignite(Pdrop, tb, g)
    C0 = Pdrop + [0, 0, -0.5 * g * tb^2];
end

% 烟幕位置计算
function C = Ci_of_t(C0, ton, t)
    C = C0 + [0, 0, -3.0 * (t - ton)];
end

% 线段到点距离平方
function d2 = segment_point_distance_sq(P, Q, C)
    D = Q - P;
    denom = dot(D, D);
    
    if denom == 0
        d2 = dot(P - C, P - C);
        return;
    end
    
    u = dot(C - P, D) / denom;
    uc = max(0, min(1, u));
    X = P + uc * D;
    d2 = dot(X - C, X - C);
end

% 构建目标采样点
function samples = build_target_samples(n_theta, side_layers)
    pts = [];
    r = 7.0;
    y0 = 200.0;
    thetas = linspace(0, 2 * pi, n_theta + 1);
    thetas(end) = [];
    
    % 顶面和底面
    for th = thetas
        x = r * cos(th);
        y = y0 + r * sin(th);
        pts = [pts; x, y, 10.0];  % 顶面
        pts = [pts; x, y, 0.0];   % 底面
    end
    
    % 侧面
    zs = linspace(0, 10.0, side_layers);
    for th = thetas
        x = r * cos(th);
        y = y0 + r * sin(th);
        for z = zs
            pts = [pts; x, y, z];
        end
    end
    
    samples = pts;
end

% 线段与球体相交事件
function events = line_sphere_events_in_time(Pm0, u_m, Qj, C0, ton, toff, R)
    events = [];
    
    if toff <= ton
        return;
    end
    
    f = @(t) segment_point_distance_sq(Pm_of_t(Pm0, u_m, t), Qj, Ci_of_t(C0, ton, t)) - R^2;
    
    N = 40;
    ts = linspace(ton, toff, N + 1);
    vals = arrayfun(f, ts);
    
    for k = 1:N
        a = ts(k);
        b = ts(k + 1);
        fa = vals(k);
        fb = vals(k + 1);
        
        if fa == 0
            events(end + 1) = a;
        end
        
        if fa * fb <= 0
            lo = a;
            hi = b;
            
            for iter = 1:40
                mid = 0.5 * (lo + hi);
                fm = f(mid);
                
                if fm == 0
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
                events(end + 1) = root;
            end
        end
    end
end
