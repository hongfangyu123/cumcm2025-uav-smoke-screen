function t2final
    % 问题2
    clear; clc; close all;
    rng(1234, 'twister');

    % 常量定义
    g               = 9.8;               
    R               = 10;               
    t_window        = 20;                
    v_c             = 3;                
    v_M1            = 300;               
    R0_U1           = [17800, 0, 1800];  
    r0_M1           = [20000, 0, 2000];  
    r_target        = 7;                
    h_target        = 10;               
    target_base     = [0, 200, 0];      
    
    % 导弹M1运动参数计算
    m_vec           = -r0_M1;            
    m_d             = norm(m_vec);       
    m_dir           = m_vec / m_d;      
    m_max_t         = m_d / v_M1;     
    
    % 真目标圆柱表面采样点
    N_ang = 36;  
    N_h = 21;   
    thetas = linspace(0,2*pi,N_ang+1); thetas(end) = []; 
    zs     = linspace(0,h_target,N_h);
    total_pts = N_ang*N_h + N_ang;
    Q_pts = zeros(total_pts,3); 
    idx = 1;
    for i = 1:N_ang
        for j = 1:N_h
            Q_pts(idx,:) = [ ...
                target_base(1) + r_target*cos(thetas(i)), ...
                target_base(2) + r_target*sin(thetas(i)), ...
                zs(j) ];
            idx = idx + 1;
        end
    end
    for i = 1:N_ang
        Q_pts(idx,:) = [ ...
            target_base(1) + r_target*cos(thetas(i)), ...
            target_base(2) + r_target*sin(thetas(i)), ...
            h_target ];
        idx = idx + 1;
    end
    
    % 遗传算法参数设置
    nvars = 4;  
    lb = [70,        0,             0,                 0            ]; 
    ub = [140,  2*pi, m_max_t - t_window, m_max_t]; 
    
    opts = optimoptions('ga', ...
        'PopulationSize',    70, ...
        'MaxGenerations',    90, ...
        'UseVectorized',     false, ...
        'Display',           'iter');
    
    [best_x, best_f] = ga( ...
        @(x)objective(x), ...
        nvars, [], [], [], [], lb, ub, ...
        @(x)nonlcon(x), ...
        opts);

    % 最小化遮蔽时长获得最优遮蔽时长
    best_duration = -best_f;  

    % 提取最优决策变量
    v_U1     = best_x(1);           
    theta_U1 = best_x(2);          
    t_d_11   = best_x(3);         
    t_e_11   = best_x(4);         
    
    % 计算投放点和起爆点坐标
    ux = cos(theta_U1); uy = sin(theta_U1);
    r_d_11 = R0_U1 + [ux, uy, 0]*v_U1*t_d_11; 
    c0_11_z = r_d_11(3) - 0.5*g*t_e_11^2;     
    c0_11 = [r_d_11(1), r_d_11(2), c0_11_z];  
    
    % 输出结果
    fprintf('\n===== 最优策略 =====\n');
    fprintf('飞行速度v= %.6f m/s\n',    v_U1);
    fprintf('飞行方向θ= %.6f°\n',      rad2deg(theta_U1));
    fprintf('投放延时t_drop = %.6f s\n',      t_d_11);
    fprintf('起爆延时t_burst= %.6f s\n',     t_e_11);
    fprintf('投放点坐标= (%.6f, %.6f, %.6f)\n', r_d_11(1), r_d_11(2), r_d_11(3));
    fprintf('起爆点坐标= (%.6f, %.6f, %.6f)\n', c0_11(1), c0_11(2), c0_11(3));
    fprintf('最大有效遮蔽时间= %.2f s\n',        best_duration);
    
    
    % 目标函数：计算负遮蔽时长
    function neg_dur = objective(x)
        % 决策变量：[无人机速度, 航向角, 投放时间, 起爆时间]
        v_U1     = x(1);
        theta_U1 = x(2);
        t_d_11   = x(3);
        t_e_11   = x(4);
        
        % 检查时间
        if t_d_11 < 0 || t_e_11 < 0 || ...
           t_d_11 + t_e_11 + t_window > m_max_t
            neg_dur = 0;
            return;
        end
        
        % 时间区间：烟幕起爆时刻到失效时刻
        t0 = t_d_11 + t_e_11;
        t_end = t0 + t_window;
        dt = 0.1;  % 时间步长
        dur = 0;   % 遮蔽时长累计
        
        % 无人机水平速度向量
        v_u = [cos(theta_U1), sin(theta_U1), 0] * v_U1;

        % 烟幕弹投放点位置
        r_d_11 = R0_U1 + v_u * t_d_11;
        z_off = -0.5 * g * t_e_11^2;
        
        % 遍历时间区间
        for t = t0:dt:t_end
            % 导弹M1在t时刻的位置
            r_M1_t = r0_M1 + m_dir * v_M1 * t;
            % 烟幕云团在t时刻的中心位置
            dt_sink = t - t0;
            c_11_t = r_d_11 + [0, 0, z_off] + [0, 0, -v_c * dt_sink];
            % 判断是否完全遮蔽
            if fully_occluded(r_M1_t, c_11_t, R, Q_pts)
                dur = dur + dt;
            end
        end
        
        neg_dur = -dur;  % 负时长
    end

    % 确保烟幕有效期在导弹到达前结束
    function [c, ceq] = nonlcon(x)
        t_d_11 = x(3);  
        t_e_11 = x(4);  
        c = (t_d_11 + t_e_11 + t_window) - m_max_t;
        ceq = [];  
    end

    % 完全遮蔽判定：检查导弹到所有目标采样点的线段是否被烟幕遮挡
    function ok = fully_occluded(r_M1_t, c_11_t, R, Q_pts)
        ok = true;  
        for ii = 1:size(Q_pts,1)
            if ~segment_sphere_intersect(r_M1_t, c_11_t, R, Q_pts(ii,:))
                ok = false;
                return;
            end
        end
    end

    % 线段-球体相交判定：检查导弹到目标点的线段是否与烟幕球体相交
    function flag = segment_sphere_intersect(r_M1_t, c_11_t, R, Qj)
        D = Qj - r_M1_t;         
        Mv = r_M1_t - c_11_t;    
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

end