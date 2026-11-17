function t4final
    % 问题4
    clear; clc; close all;
    rng(1234, 'twister');

    % 常量定义
    g = 9.8;             
    R_cloud = 10;      
    cloud_lifetime = 20;   
    v_sink = 3;           
    v_M1 = 300;            
    
    % 初始位置
    r_M1_0 = [20000, 0, 2000];      
    T_base = [0, 200, 0];             
    cylinder_radius = 7;              
    cylinder_height = 10;          
    
    % 三架无人机初始位置
    r_FY1_0 = [17800, 0, 1800];      
    r_FY2_0 = [12000, 1400, 1400];  
    r_FY3_0 = [6000, -3000, 700];     

    % 导弹M1运动参数计算
    m_vec = -r_M1_0;             
    m_d = norm(m_vec);            
    m_dir = m_vec / m_d;              
    m_max_t = m_d / v_M1;        
    
    % 真目标圆柱表面采样点
    n_points = 20;  
    key_points = zeros(2*n_points + 2, 3);
    
    key_points(1, :) = T_base;
    key_points(2, :) = T_base + [0, 0, cylinder_height];
    
    for i = 1:n_points
        angle = 2*pi*(i-1)/n_points;
        key_points(2+i, :) = T_base + cylinder_radius*[cos(angle), sin(angle), 0];
    end
    
    for i = 1:n_points
        angle = 2*pi*(i-1)/n_points;
        key_points(2+n_points+i, :) = T_base + cylinder_radius*[cos(angle), sin(angle), 0] + [0, 0, cylinder_height];
    end

    % 遗传算法参数设置
    nvars = 12; 
    lb = [0.05, 80, 0.5, 0.3, 4, 70, 10, 5, 1.6, 80, 20, 5];    
    ub = [0.15, 120, 1, 0.6, 4.5, 90, 15, 9, 2.2, 120, 30, 8];  
    
    opts = optimoptions('ga', ...
        'PopulationSize', 50, ...
        'MaxGenerations', 30, ...
        'Display', 'iter', ...
        'UseParallel', false);
    
    % 优化
    [optimal_params, fval] = ga( ...
        @objective_function, ...
        nvars, [], [], [], [], lb, ub, ...
        [], opts);

    % 计算最大遮蔽时间
    max_shielding_time = -fval;
    fprintf('\n最大遮蔽时间: %.3f 秒\n', max_shielding_time);

    % 提取最优参数
    angle1 = optimal_params(1);
    speed1 = optimal_params(2);
    release_time1 = optimal_params(3);
    delay_time1 = optimal_params(4);

    angle2 = optimal_params(5);
    speed2 = optimal_params(6);
    release_time2 = optimal_params(7);
    delay_time2 = optimal_params(8);

    angle3 = optimal_params(9);
    speed3 = optimal_params(10);
    release_time3 = optimal_params(11);
    delay_time3 = optimal_params(12);

    % 输出结果
    fprintf('\n最优参数:\n');
    fprintf('FY1: 方向=%.2f°, 速度=%.1fm/s, 投放时间=%.2fs, 延迟时间=%.2fs\n', ...
        rad2deg(angle1), speed1, release_time1, delay_time1);
    fprintf('FY2: 方向=%.2f°, 速度=%.1fm/s, 投放时间=%.2fs, 延迟时间=%.2fs\n', ...
        rad2deg(angle2), speed2, release_time2, delay_time2);
    fprintf('FY3: 方向=%.2f°, 速度=%.1fm/s, 投放时间=%.2fs, 延迟时间=%.2fs\n', ...
        rad2deg(angle3), speed3, release_time3, delay_time3);

    % 计算投放点和起爆点坐标
    e_FY1 = [cos(angle1), sin(angle1), 0];
    e_FY2 = [cos(angle2), sin(angle2), 0];
    e_FY3 = [cos(angle3), sin(angle3), 0];

    release_pos1 = r_FY1_0 + speed1 * release_time1 * e_FY1;
    explosion_pos1 = release_pos1 + delay_time1 * speed1 * e_FY1 - 0.5 * g * delay_time1^2 * [0, 0, 1];

    release_pos2 = r_FY2_0 + speed2 * release_time2 * e_FY2;
    explosion_pos2 = release_pos2 + delay_time2 * speed2 * e_FY2 - 0.5 * g * delay_time2^2 * [0, 0, 1];

    release_pos3 = r_FY3_0 + speed3 * release_time3 * e_FY3;
    explosion_pos3 = release_pos3 + delay_time3 * speed3 * e_FY3 - 0.5 * g * delay_time3^2 * [0, 0, 1];

    fprintf('\n投放点和起爆点坐标:\n');
    fprintf('FY1投放点: (%.1f, %.1f, %.1f)\n', release_pos1);
    fprintf('FY1起爆点: (%.1f, %.1f, %.1f)\n', explosion_pos1);
    fprintf('FY2投放点: (%.1f, %.1f, %.1f)\n', release_pos2);
    fprintf('FY2起爆点: (%.1f, %.1f, %.1f)\n', explosion_pos2);
    fprintf('FY3投放点: (%.1f, %.1f, %.1f)\n', release_pos3);
    fprintf('FY3起爆点: (%.1f, %.1f, %.1f)\n', explosion_pos3);

    % 保存结果到Excel文件
    save_results_to_excel(angle1, speed1, release_pos1, explosion_pos1, ...
                         angle2, speed2, release_pos2, explosion_pos2, ...
                         angle3, speed3, release_pos3, explosion_pos3);

    % 目标函数
    function neg_dur = objective_function(x)
        % 提取参数
        angle1 = x(1);
        speed1 = x(2);
        release_time1 = x(3);
        delay_time1 = x(4);

        angle2 = x(5);
        speed2 = x(6);
        release_time2 = x(7);
        delay_time2 = x(8);

        angle3 = x(9);
        speed3 = x(10);
        release_time3 = x(11);
        delay_time3 = x(12);
        
        % 计算起爆时间
        explosion_time1 = release_time1 + delay_time1;
        explosion_time2 = release_time2 + delay_time2;
        explosion_time3 = release_time3 + delay_time3;
        
        % 检查时间约束
        if explosion_time1 + cloud_lifetime > m_max_t || ...
           explosion_time2 + cloud_lifetime > m_max_t || ...
           explosion_time3 + cloud_lifetime > m_max_t
            neg_dur = 0;
            return;
        end
        
        % 无人机运动方向
        e_FY1 = [cos(angle1), sin(angle1), 0];
        e_FY2 = [cos(angle2), sin(angle2), 0];
        e_FY3 = [cos(angle3), sin(angle3), 0];
        
        % 计算投放点和起爆点
        r_release1 = r_FY1_0 + speed1 * release_time1 * e_FY1;
        r_explosion1 = r_release1 + delay_time1 * speed1 * e_FY1 - 0.5 * g * delay_time1^2 * [0, 0, 1];
        
        r_release2 = r_FY2_0 + speed2 * release_time2 * e_FY2;
        r_explosion2 = r_release2 + delay_time2 * speed2 * e_FY2 - 0.5 * g * delay_time2^2 * [0, 0, 1];
        
        r_release3 = r_FY3_0 + speed3 * release_time3 * e_FY3;
        r_explosion3 = r_release3 + delay_time3 * speed3 * e_FY3 - 0.5 * g * delay_time3^2 * [0, 0, 1];
        
        % 时间步长
        dt = 0.1;
        t_start = min([explosion_time1, explosion_time2, explosion_time3]);
        t_end = max([explosion_time1, explosion_time2, explosion_time3]) + cloud_lifetime;
        t_range = t_start:dt:t_end;
        
        dur = 0;  % 遮蔽时长累计
        
        % 遍历时间区间
        for t = t_range
            % 导弹位置
            r_M1_t = r_M1_0 + m_dir * v_M1 * t;
            
            % 检查每个烟幕云团状态
            cloud_centers = [];
            
            if t >= explosion_time1 && t <= explosion_time1 + cloud_lifetime
                dt_sink1 = t - explosion_time1;
                cloud_center1 = r_explosion1 - [0, 0, v_sink * dt_sink1];
                cloud_centers = [cloud_centers; cloud_center1];
            end
            
            if t >= explosion_time2 && t <= explosion_time2 + cloud_lifetime
                dt_sink2 = t - explosion_time2;
                cloud_center2 = r_explosion2 - [0, 0, v_sink * dt_sink2];
                cloud_centers = [cloud_centers; cloud_center2];
            end
            
            if t >= explosion_time3 && t <= explosion_time3 + cloud_lifetime
                dt_sink3 = t - explosion_time3;
                cloud_center3 = r_explosion3 - [0, 0, v_sink * dt_sink3];
                cloud_centers = [cloud_centers; cloud_center3];
            end
            
            % 判断是否完全遮蔽
            if ~isempty(cloud_centers) && fully_occluded(r_M1_t, cloud_centers, R_cloud, key_points)
                dur = dur + dt;
            end
        end
        
        neg_dur = -dur;  % 负遮蔽时长
    end

    % 完全遮蔽判定
    function ok = fully_occluded(r_M1_t, cloud_centers, R, Q_pts)
        ok = true;
        for ii = 1:size(Q_pts, 1)
            if ~any_cloud_intersects(r_M1_t, cloud_centers, R, Q_pts(ii, :))
                ok = false;
                return;
            end
        end
    end

    % 检查是否有任一烟幕云团与线段相交
    function flag = any_cloud_intersects(r_M1_t, cloud_centers, R, Qj)
        flag = false;
        for k = 1:size(cloud_centers, 1)
            if segment_sphere_intersect(r_M1_t, cloud_centers(k, :), R, Qj)
                flag = true;
                return;
            end
        end
    end

    % 线段-球体相交判定
    function flag = segment_sphere_intersect(r_M1_t, cloud_center, R, Qj)
        D = Qj - r_M1_t;         
        Mv = r_M1_t - cloud_center;    
        a = dot(D, D);          
        b = 2 * dot(D, Mv);     
        c = dot(Mv, Mv) - R^2;  
        disc = b^2 - 4*a*c;      % 判别式
        
        if disc < 0  % 无交点
            flag = false;
            return;
        end
        
        % 求解交点参数
        sd = sqrt(disc);
        s1 = (-b + sd) / (2*a);
        s2 = (-b - sd) / (2*a);
        flag = (s1 >= 0 && s1 <= 1) || (s2 >= 0 && s2 <= 1);
    end

    % 保存结果到Excel
    function save_results_to_excel(angle1, speed1, release_pos1, explosion_pos1, ...
                                  angle2, speed2, release_pos2, explosion_pos2, ...
                                  angle3, speed3, release_pos3, explosion_pos3)
        % 创建数据表
        result_data = cell(5, 10);
        
        % 表头
        result_data{1, 1} = '无人机编号';
        result_data{1, 2} = '无人机运动方向';
        result_data{1, 3} = '无人机运动速度 (m/s)';
        result_data{1, 4} = '烟幕干扰弹投放点的x坐标 (m)';
        result_data{1, 5} = '烟幕干扰弹投放点的y坐标 (m)';
        result_data{1, 6} = '烟幕干扰弹投放点的z坐标 (m)';
        result_data{1, 7} = '烟幕干扰弹起爆点的x坐标 (m)';
        result_data{1, 8} = '烟幕干扰弹起爆点的y坐标 (m)';
        result_data{1, 9} = '烟幕干扰弹起爆点的z坐标 (m)';
        result_data{1, 10} = '有效干扰时长 (s)';
        
        % FY1数据
        result_data{2, 1} = 'FY1';
        result_data{2, 2} = rad2deg(angle1);
        result_data{2, 3} = speed1;
        result_data{2, 4} = release_pos1(1);
        result_data{2, 5} = release_pos1(2);
        result_data{2, 6} = release_pos1(3);
        result_data{2, 7} = explosion_pos1(1);
        result_data{2, 8} = explosion_pos1(2);
        result_data{2, 9} = explosion_pos1(3);
        result_data{2, 10} = 'N/A';
        
        % FY2数据
        result_data{3, 1} = 'FY2';
        result_data{3, 2} = rad2deg(angle2);
        result_data{3, 3} = speed2;
        result_data{3, 4} = release_pos2(1);
        result_data{3, 5} = release_pos2(2);
        result_data{3, 6} = release_pos2(3);
        result_data{3, 7} = explosion_pos2(1);
        result_data{3, 8} = explosion_pos2(2);
        result_data{3, 9} = explosion_pos2(3);
        result_data{3, 10} = 'N/A';
        
        % FY3数据
        result_data{4, 1} = 'FY3';
        result_data{4, 2} = rad2deg(angle3);
        result_data{4, 3} = speed3;
        result_data{4, 4} = release_pos3(1);
        result_data{4, 5} = release_pos3(2);
        result_data{4, 6} = release_pos3(3);
        result_data{4, 7} = explosion_pos3(1);
        result_data{4, 8} = explosion_pos3(2);
        result_data{4, 9} = explosion_pos3(3);
        result_data{4, 10} = 'N/A';
        
        % 写入Excel文件
        filename = 'result2.xlsx';
        writecell(result_data, filename, 'Sheet', 'Sheet1');
        
        fprintf('结果已成功保存到文件: %s\n', filename);
    end

    % 弧度转角度
    function deg = rad2deg(rad)
        deg = rad * 180 / pi;
    end
end
