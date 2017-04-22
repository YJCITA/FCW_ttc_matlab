%% 2017.03.13 修改程序增加对数据批处理的能力
% 修改模型 改变为匀加速模型  效果非常好
%% 2017.03.18 适应新的数据格式，增加批处理能力，统计正报和误报
% 以bounding box作为量测进行KF跟踪
%% 2017.03.20  这个版本效果还OK，考虑增加连续的横向速度估计和IMU判断颠簸情况，去掉误报
%% 2017.03.21 适应徐涵的数据结构
%% 2017.03.28 分析KF本身是否收敛（P)--未完成！！！！！！！！！
%  增加车速，查看是否有利于速度跟踪的收敛 用速度量测动态性会好（尤其是对于本车加减速比较剧烈的时候）
%  增加目标车辆横向距离和速度的估计--初步判断横向距离估计很不准
%% 2017.04.06 增加radar ttc
%% 2017.04.11 目前策略对误报已经有较好控制  对漏报（本质不一定是漏报的场景，有待讨论）
% 批量跑一下别的数据
clc
clear all
close all

%% 数据导入
% 遍历文件夹
% maindir = 'F:\数据\FCW\0321_vison_radar\nj';
% maindir = 'F:\数据\FCW\0321_vison_radar\radar';
% maindir = 'F:\数据\FCW\0321_vison_radar\acc_track'; % 测试KF跟踪性能
% maindir = 'F:\数据\FCW\0321_vison_radar\mobileye_FCW'; % mobileye fcw
% maindir = 'F:\数据\FCW\0321_vison_radar\radar\2016.11.22天津——秦皇岛'; % 一般数据集测试
maindir = '/home/yj/bak/data/fcw/C++_compare/fcw_new_data';


is_search_dir = 0; % 1:代表遍历main路径下的所有文件夹  0：只是找main下面的文件

if is_search_dir
    sub_l1_dir = dir( maindir );
    dir_NUM = length( sub_l1_dir );
else
    sub_l1_dir = [];
    dir_NUM = 1;
end
% 一级目录 
for l1_i = 1 : dir_NUM
%     if( isequal( sub_l1_dir(l1_i).name, '.' )|| isequal( sub_l1_dir(l1_i).name, '..')||  ~sub_l1_dir(l1_i).isdir)               % 如果不是目录则跳过
%         continue;
%     end
    for l2_i = 1 : 1
        if is_search_dir
            subdirpath = fullfile( maindir, sub_l1_dir(l1_i).name, '*.critical' ); %  遍历一级路径
        else
            subdirpath = fullfile( maindir, '*.critical' ); % 只跑当前路径下txt
        end
        
        txt_dir = dir( subdirpath );               % 子文件夹下找后缀为txt的文件
        for txt_i = 1:length(txt_dir)
            if is_search_dir
                vision_data_addr = fullfile( maindir, sub_l1_dir(l1_i).name, txt_dir(txt_i).name);
            else
                vision_data_addr = fullfile( maindir, txt_dir(txt_i).name ); % 只跑当前路径下txt
            end
            raw_log_addr = [vision_data_addr(1:end-8), 'log-speed.ini'];
%             raw_log_addr = [vision_data_addr(1:end-8), 'log.txt'];
            fid_vision_log = fopen(vision_data_addr, 'r');
            fid_raw_log = fopen(raw_log_addr, 'r'); % 原始的log
            if ~fid_vision_log || ~fid_raw_log
                break;
            end
            
           %% 初始化
            ipm_step = 1; % 步长
            is_first_read_timestamp = 1;
            time_start = 0;             
            speed_cur = 0;
            radar_data = [0  0 0 0]';
            radar_ttc = 0;
            % 量测数据
            struct_speed.data = 0;
            struct_speed.counter = 0;
            speed_update = 0; % 新的speed数据更新
            radar_range = 0;
            radar_range_vel = 0;          
            
            %% 滤波
            is_first_read_camera_data = 1;
            is_kf_init_ok = 0; % KF是否已经进行初始化
            is_first_radar_data = 1;
            save_i_index = 0; % 数据存储计数

            dv_filter = 0;
            is_first_vision_range = 1;
            fcw_state = 0;

            %% 主循环相关变量
            is_first_read_legal_data = 1;
            image_frame_id_pre = 0; % 上一次计算时候image frame index 用于计算dt
            image_frame_T = 5*60/7744; % 两帧图片之间的dt  一个视频5min，差不多是7744帧
            mobileye_fcw_state = 0; % mobileye fcw state  1：warning
            index_of_target_pre = 0; % 上一帧目标车辆的ID
            is_new_target = 0; % 是否是新目标车辆            
            fcw_counter = 0; % 满足触发条件计数
            %% IMU
            is_imu_match_camera = 0; % imu和camera是否匹配
            gyro_fliter = [0 0 0]';
            acc_fliter = [0 0 0]';
            vibe_gyro_fliter = [0 0 0]';
            vibe_acc_fliter = [0 0 0]';
            struct_gyro_d.data = [0, 0, 0]'; % 记录两帧之间采样到的imu数据，用于求平均
            struct_gyro_d.counter = 0;            
            %% ADRC
            is_ADRC_init_OK = 0;
            z_residual = [0 ,0]'; % range  range_horizon
          %% 主循环
            read_txt_line = 0; % 读取到文件的行数，如果是<100  则为空文件
            is_first_read_log = 1;
            tt_state = feof(fid_vision_log);
            while ~feof(fid_vision_log) && ~feof(fid_raw_log) % 读取log数据    
                lineData = fgetl(fid_vision_log);
                if lineData == -1
                    break;
                else
                    read_txt_line = read_txt_line + 1;
                end
                exp1 = ',';
                str_line_raw = regexp(lineData, exp1, 'split'); %以空格为特征分割字符串
                str_frame_index = str2num(str_line_raw{1,1}); % 每一幅图对应3帧 0:detect_vehicle_record    1:radar-recod  2：mobileye
                % 为了画图方便  获取初始时刻的时间
                if is_first_read_timestamp
                    % 用log的时间戳比较靠谱
                    lineData_raw_log = fgetl(fid_raw_log);
                    if lineData_raw_log == -1
                        break;
                    end
                    exp1 = ' ';
                    str_line_log_raw = regexp(lineData_raw_log, exp1, 'split'); %以空格为特征分割字符串
                    time_s = str2num(str_line_log_raw{1,1});
                    time_us = str2num(str_line_log_raw{1,2});
                    time_start = time_s + time_us *1e-6;
                    is_first_read_timestamp = 0;
                end

                % 读取mobileye 数据
                if(str_frame_index == 2)
                    mobileye_fcw_state = 1;
                end

                % 1: radar-recod      
                if(str_frame_index == 1)
                    speed_cur = str2num(str_line_raw{1,20+2});
                    radar_range = str2num(str_line_raw{1,19+2});
                    radar_range_vel = str2num(str_line_raw{1,16+2});
                    radar_range_acc = 0; %str2num(str_line_raw{1,14+2});
                    radar_horiz_dist = str2num(str_line_raw{1,18+2});
                    radar_data = [radar_range radar_range_vel radar_range_acc radar_horiz_dist]';                   
                    radar_ttc = str2num(str_line_raw{1,17+2});
                end
                
                % 0:   detect_vehicle_record 
                if(str_frame_index == 0)
                    % 采用 index:0 读取数据        
                    image_timestamp_cur = str2num(str_line_raw{1,2}) - time_start; 
                    image_frame_id = str2num(str_line_raw{1,4+2});
                    vison_range_raw = str2num(str_line_raw{1,10+2});
                    vision_horizon = str2num(str_line_raw{1,11+2});
                    index_of_target = str2num(str_line_raw{1,2+2});
                    ttc_raw = str2num(str_line_raw{1,14+2}); 
                    
                  %% 读取log.txt数据（车速+imu）
                     is_imu_match_camera = 0;
                     while ~is_imu_match_camera && ~feof(fid_raw_log)                       
                        % 第一次进来 先读取一个数据
                        while(is_first_read_log)
                            lineData_raw_log = fgetl(fid_raw_log);
                            exp1 = ' ';
                            str_line_log_raw = regexp(lineData_raw_log, exp1, 'split'); %以空格为特征分割字符串
                            time_s = str2num(str_line_log_raw{1,1});
                            time_us = str2num(str_line_log_raw{1,2});
                            time_log = time_s + time_us *1e-6 - time_start;
                            time_log_pre = time_log;
                            str_line_data_flag = str_line_log_raw{1,3};
                            % speed
                            if strcmp(str_line_data_flag, 'brake_signal')
                                is_first_read_log = 0;
                                speed_cur = str2num(str_line_log_raw{1, 24})/3.6;               
                            end
                        end
                        
                        % 判断上一个速度时间戳是否匹配上（因为speed更新比较慢，所以不能每次进来比较就先读取数据，很可能会一直匹配不上）
                        dt_imu_camera = time_log_pre - image_timestamp_cur;
                        if dt_imu_camera > -0.1
                            is_imu_match_camera = 1; % 数据已经匹配
                        else
                            lineData_raw_log = fgetl(fid_raw_log);
                            exp1 = ' ';
                            str_line_log_raw = regexp(lineData_raw_log, exp1, 'split'); %以空格为特征分割字符串
                            time_s = str2num(str_line_log_raw{1,1});
                            time_us = str2num(str_line_log_raw{1,2});
                            time_log = time_s + time_us *1e-6 - time_start;
                            time_log_pre = time_log;
                            str_line_data_flag = str_line_log_raw{1,3};
                            if strcmp(str_line_data_flag, 'brake_signal')
                                speed_cur = str2num(str_line_log_raw{1, 24})/3.6;                    
                            end
                        end                                     
                     end
                    
                    if(is_first_read_legal_data)
                        image_timestamp_pre = image_timestamp_cur;
                        is_first_read_legal_data = 0;
                    end 

                  %%  滤波          
                    if(~is_kf_init_ok)
                        acc_t = 0; % target vehicle
                        speed_t = speed_cur;
                        acc_s = 0; % host vehicle
                        speed_s = speed_cur;
                        vison_range_filter = vison_range_raw;

                        Xk = [vison_range_filter, speed_t, speed_s, acc_t, acc_s]';
                        Pk = diag([100, 100, 10, 20, 10]);
                        Q = diag([25,  20, 4, 3, 3]);
                        R = diag([10, 1]);   
                        
                        % 水平
                        acc_h = 0; % target vehicle
                        speed_h = 0;       
                        vison_range_h_filter = vision_horizon;
                        Xk_h = [vision_horizon, speed_h, acc_h]';
                        Pk_h = diag([5, 4, 4]);
                        Q_h = diag([1,  1, 0.5]);
                        R_h = diag([0.5]);  
                        is_kf_init_ok = 1;
                    end
                    
                    % 判断 target ID，来确认是否是一辆新的车
                    if (index_of_target_pre ~= index_of_target)
                        is_new_target = 1;
                    else
                        is_new_target = 0;                        
                    end
                    index_of_target_pre = index_of_target;                    
                    dt = image_timestamp_cur - image_timestamp_pre;
                    image_timestamp_pre = image_timestamp_cur; 
                    % 为了处理长时间没有有效前车的问题 dt太大则重置滤波
                    % 对于新目标，也要重置滤波
                    if(dt > 0.3 || is_new_target)
                        % dt >1 则重置
                        acc_t = 0; % target vehicle
                        speed_t = speed_cur;
                        acc_s = 0; % host vehicle
                        speed_s = speed_cur;
                        vison_range_filter = vison_range_raw;

                        Xk = [vison_range_filter, speed_t, speed_s, acc_t, acc_s]';
                        Pk = diag([100, 100, 10, 20, 10]);
                        Q = diag([25,  20, 4, 3, 3]);
                        R = diag([10, 1]); 
                        
                        % 水平
                        acc_h = 0; % target vehicle
                        speed_h = 0;   
                        vison_range_h_filter = vision_horizon;
                        Xk_h = [vision_horizon, speed_h, acc_h]';
                        Pk_h = diag([5, 4, 4]);
                        Q_h = diag([1,  1, 0.5]);
                        R_h = diag([0.5]);  
                        is_kf_init_ok = 1;
      
                    else
                        % 低通滤波处理 
                        filt_hz = 0.5; % 为了控制波动  基本0.7是极限（>1之后就会引入车辆波动导致的车速变化）
                        [ vison_range_filter ] = fun_LowpassFilter( vison_range_filter, vison_range_raw, dt, filt_hz ); 
                        
                        filt_hz = 0.2; 
                        [ vison_range_h_filter ] = fun_LowpassFilter( vison_range_h_filter, vision_horizon, dt, filt_hz ); 

                        % KF 滤波
                        % X = [d, vs, vt]' 车距，本车速度，目标车速
                        % z = [vision_range car_speed]                   
                        z = [vison_range_filter  speed_cur]';
                        F = [1  dt  -dt  0.5*dt^2 -0.5*dt^2;
                             0   1   0      dt       0;
                             0   0   1      0        dt;
                             0   0   0      1        0;
                             0   0   0      0        1;];
                        H = [ 1 0 0 0 0;
                              0 0 1 0 0 ];
                        [Xk, Pk, z_residual] = fun_KF_resdiual(Xk, Pk, z, Q, R, F, H);
                        
                        % 水平
                        z_h = [vison_range_h_filter]';
                        F_h = [1  dt   0.5*dt^2
                             0   1    dt  ;
                             0   0    1  ;];
                        H_h = [ 1 0 0 ];
                        [Xk_h, Pk_h, z_residual_h] = fun_KF_resdiual(Xk_h, Pk_h, z_h, Q_h, R_h, F_h, H_h);
                        
                    end
                    
                  %% FCW触发逻辑判断  
                    Vt = Xk(2);
                    Vs = Xk(3);
                    acc_t = Xk(4);
                    acc_s = Xk(5);                    
                    range_cur = Xk(1);
                    vel_relative = Vt - Vs;
                    acc_relative = acc_t - acc_s;
                    relative_data = [range_cur, vel_relative, acc_relative]';
                    
                    % 水平
                    horizon_range = Xk_h(1);
                    horizon_vel = Xk_h(2);
                    
                    % 计算ttc 
                    if abs(vel_relative) >0.2
                        ttc = abs(range_cur/vel_relative);
                    else
                        ttc = -1;
                    end
                    
                    % 考虑acc的ttc  考虑加速度的计算ttc,有问题，暂时不考虑
                    a = acc_relative;
                    V0 = vel_relative;
                    Z0 = range_cur;
                    tmp1 = V0^2 - 2*Z0*a;
                    if a ~= 0 && tmp1 > 0
                        ttc_acc = (-V0 + sqrt(V0^2 - 2*Z0*a))/a;
                    else
                        if abs(vel_relative) >0.2
                            ttc_acc = abs(range_cur/vel_relative);
                        else
                            ttc_acc = -1;
                        end
                    end
                    
                    % radar ttc
                    if abs(radar_range_vel) >0.2
                        ttc_radar = abs(radar_range/radar_range_vel);
                    else
                        ttc_radar = -1;
                    end
                    
                    % fcw 触发逻辑 ttc<4, 相对车距离<20, 车速>40/3.6, 水平距离<1.2, 累计满足5帧
                    % 初步判断mobileye告警的场景都是比较高速的
                    if ttc >0
                        tt_horizon = abs(horizon_range + horizon_vel*ttc);
                    else
                        tt_horizon = -1;    
                    end
                    ttc_threhold = 3.0;
                    if ttc<ttc_threhold  && ttc>0 && tt_horizon<3 && abs(horizon_range)<2 && acc_relative<0 && vel_relative<-1 && range_cur >= 10 && range_cur <= 50 && speed_cur>30/3.6%&& range_cur <= 55
                        fcw_counter = fcw_counter + 1;
                        if(fcw_counter >= 5)
                            fcw_state = 1;
                        else
                            fcw_state = 0;
                        end
                    else
                        fcw_counter = 0;
                        fcw_state = 0;
                    end
                    
                  %% 保存数据
                    time_cur = image_timestamp_cur;
                    save_i_index = save_i_index + 1;
%                     save_Xk(:, save_i_index) = [time_cur; Xk];
                    save_Xk(:, save_i_index) = [time_cur; relative_data];
                    save_Xk_h(:, save_i_index) = [time_cur; Xk_h];    
                    save_tt_horizon(:, save_i_index) = [time_cur; tt_horizon];   % 横向距离预测
                    save_vision_raw(:, save_i_index)= [time_cur; vison_range_raw];
                    save_ttc(:, save_i_index) = [time_cur; ttc ];
                    save_ttc_acc(:, save_i_index) = [time_cur; ttc_acc ];                    
                    save_ttc_raw(:, save_i_index) = [time_cur; ttc_raw ];
                    save_dt(:, save_i_index) = [time_cur; dt];
                    save_fcw_state_mobileye(:, save_i_index) = [time_cur; mobileye_fcw_state];
                    save_fcw_state(:, save_i_index) = [time_cur;fcw_state ];
%                     save_data_mobileye_tmp(:, save_i_index) = [time_cur; data_mobileye_tmp];    
                    save_index_of_target(:, save_i_index) = [time_cur; index_of_target];       
                    save_vision_horizon(:, save_i_index) = [time_cur; vision_horizon];   
                    save_speed_car(:, save_i_index) = [time_cur; speed_cur];        
                    save_radar_data(:, save_i_index) = [time_cur; radar_data];   %  [radar_range  radar_range_vel radar_range_acc radar_horiz_dist]';
                    save_radar_ttc(:, save_i_index) = [time_cur; radar_ttc];  
                    save_P(:, save_i_index) = [time_cur; sqrt(Pk(1,1)); sqrt(Pk(2,2)); sqrt(Pk(3,3))];
                    save_z_residual(:, save_i_index) = [time_cur;z_residual];   
                    % 清零
                    mobileye_fcw_state = 0;
                end        
            end
            
        % 大于100 才不是空文件
        if read_txt_line >10             
            h = figure();
%             figure('visible','off')   
            set(h,'outerposition',get(0,'screensize'));
            % 车距
            ax1 = subplot(4,1,1);
            plot(save_vision_raw(1,:), save_vision_raw(2,:), '.'); % vision range 量测
            hold on;
            plot(save_Xk(1,:), save_Xk(2,:), '.'); % range-estimate
            plot(save_radar_data(1,:), save_radar_data(2,:), '.'); % radar-range
            plot(save_speed_car(1,:), save_speed_car(2,:), '.'); % speed-car
            NUM = length(save_radar_data(1,:));
            plot(save_radar_data(1,:), ones(1,NUM)*55)
            grid on;
            legend({'vision-range-measure-raw','range-estimate', 'radar-range', 'speed-car', 'line-55m'},'Location','northeast','FontSize',10);
            legend('boxoff')
            vision_data_addr_t = vision_data_addr;
            str_name = sprintf('log文件: %s \n 车距 ', vision_data_addr_t);
            ylim([-1, 100])
            xlim([0, 350]);
            title(str_name);
            
            % 速度 & 加速度 &本车速度
            ax2 = subplot(4,1,2);
            plot(save_Xk(1,:), save_Xk(3, :), '.'); % vel estimation
            hold on;
            plot(save_Xk(1,:), save_Xk(4,:), '.'); % acc estimation
            plot(save_radar_data(1,:), -save_radar_data(3,:), '.'); % vel-radar
            plot(save_radar_data(1,:), -save_radar_data(4,:), '.'); % acc-radar
            grid on;
            legend({'vel-estimation','acc-estimation', 'vel-radar', 'acc-radar'},'Location','northeast','FontSize',10);
            legend('boxoff')
            xlim([0, 350]);
            title('速度&加速度')
            
            % 速度 & 加速度 &本车速度
            ax3 = subplot(4,1,3);
            hold on;
            
            plot(save_Xk(1,:), save_Xk_h(2,:), '.'); % vision horizon
            plot(save_Xk(1,:), save_Xk_h(3,:), '.'); % vision horizon vel
            plot(save_tt_horizon(1,:), save_tt_horizon(2,:), '.'); % ttc_horizon    
            NUM = length(save_tt_horizon(1,:));
            plot(save_tt_horizon(1,:), ones(1,NUM)*3)
            ylim([-5,5])
            grid on;
            legend({ 'vision-horizon*10', 'vision-horizon-vel*10', 'ttc-horizon*10','3'},'Location','northeast','FontSize',10);
            legend('boxoff')
            xlim([0, 350]);
            title('横向距离&本车速度')
            
            % ttc fcw
            ax4 = subplot(4,1,4);
            NUM1 = length(save_ttc);
            plot(save_ttc(1,:), save_ttc(2,:), '.'); % ttc
            hold on;
            plot(save_radar_ttc(1,:), save_radar_ttc(2,:), '.'); % ttc_radar      
            plot(save_fcw_state(1,:), save_fcw_state(2,:)*6); % fcw_minieye    
            plot(save_fcw_state(1,:), save_fcw_state_mobileye(2,:)*5); % fcw_mobileye              
            NUM = length(save_ttc(1,:));
            plot(save_ttc(1,:), ones(1,NUM)*ttc_threhold)
            grid on;
            legend({'ttc', 'ttc-radar', 'fcw-minieye','fcw-mobileye', 'ttc-3.0'},'Location','northeast','FontSize',10);
            legend('boxoff')
            str_name = sprintf('ttc&fcw');
            ylim([-1, 10]);
            xlim([0, 350]);
            title(str_name);
            linkaxes([ax1,ax2,ax3,ax4], 'x'); % 同步子图的坐标轴
            
            % 保存figure
%             filename=[vision_data_addr_t, '--KF.fig'];
            filename=[vision_data_addr_t, '--KF.png'];
            saveas(h,filename)
            close(h)   
            
           %% 分析KF
%             h = figure();
%             set(h,'outerposition',get(0,'screensize'));
%             % 车距
%             ax1 = subplot(2,1,1);
%             plot(save_vision_raw(1,:), save_vision_raw(2,:), '.'); % vision range 量测
%             hold on;
%             plot(save_Xk(1,:), save_Xk(2,:), '.'); % range-estimate
%             plot(save_radar_data(1,:), save_radar_data(2,:), '.'); % radar-range
%             grid on;
%             legend({'vision-range-measure-raw','range-estimate', 'radar-range'},'Location','northeast','FontSize',10);
%             legend('boxoff')
%             vision_data_addr_t = vision_data_addr;
%             str_name = sprintf('log文件: %s \n 车距 ', vision_data_addr_t);
%             title(str_name);
%             
%             ax2 = subplot(2,1,2);
%             plot(save_z_residual(1,:), save_z_residual(2, :), '.'); % z_resdiual
%             hold on;
%             plot(save_P(1,:), save_P(2,:), '.'); % range_P
%             plot(save_P(1,:), -save_P(2,:), '.'); % -range_P
%             grid on;
%             legend({'z-resdiual','range-P', 'range-P'},'Location','northeast','FontSize',10);
%             legend('boxoff')
%             title('残差');
%             linkaxes([ax1,ax2], 'x'); % 同步子图的坐标轴
%             
%             % 保存figure
%             filename=[vision_data_addr_t, '--residual.png'];
%             saveas(h,filename)
%             close(h)    
            
%             exp1 = '\';
%             str_line_raw = regexp(vision_data_addr_t, exp1, 'split'); %以空格为特征分割字符串
%             str_frame_index = str2num(str_line_raw{1,1}); 
%             filename=[str_line_raw{1,6}, '\', str_line_raw{1,7}, '--KF.png'];
%             saveas(h,filename)
%             close(h)  

            % 保存计算结果txt
        %     NUM = length(save_Xk);
        %     est_log_name = ['./data/est_result_new/', log_ID, '.log_KF_estimation.txt'];
        %     fp = fopen(est_log_name, 'wt');
        %     for i = 1:NUM
        %         fprintf(fp, '%d %f %f %f\n', save_Xk(1, i)/image_frame_T, save_Xk(2, i), save_Xk(3, i), save_Xk(4, i));
        %     end
        %     fclose(fp);
        % 
            end
            clear save_Xk  save_relative_v save_ttc save_vision_raw save_fcw_state save_data_mobileye_tmp...
                save_index_of_target save_vision_horizon save_dt save_speed_car save_ttc_raw save_fcw_state_mobileye...
                save_radar_data save_ADRC_data save_radar_ttc save_gyro_d_average save_acc_data save_vibe_acc_fliter save_vibe_gyro_fliter...
                save_P save_z_residual save_tt_horizon save_Xk_h save_ttc_acc
            fclose(fid_vision_log);
            fclose(fid_raw_log);
        end
    end
end

%% plot
% figure()
% plot(save_vision_raw(1,:), save_vision_raw(2,:)); % vision range 量测
% hold on;
% plot(save_Xk(1,:), save_Xk(2,:)); % range-estimate
% plot(save_radar(1,:), save_radar(2,:)); % 雷达
% grid on;
% legend('vision-range-measure-raw','range-estimate','radar-range');
% str_name = sprintf('车距 ');
% title(str_name);
% 
% % 速度
% figure()
% plot(save_relative_v(1,:), save_relative_v(2,:)); % vel estimation
% hold on;
% plot(save_radar(1,:), save_radar(3,:)); % 雷达测量相对速度
% % plot(save_dv_radar_filter(1,:), save_dv_radar_filter(2,:)); % 雷达数据直接微分的速度
% % plot(save_dv_filter(1,:), save_dv_filter(2,:)); % 直接微分后低通滤波的速度
% grid on;
% legend('vel-estimation', 'radar-vel', 'dv-雷达微分后低通', 'dv-微分后低通滤波');
% title('速度');
% 
% % 加速度
% figure()
% plot(save_Xk(1,:), save_Xk(4,:)); % acc estimation
% hold on;
% plot(save_radar(1,:), save_radar(4,:)); % 雷达测量acc
% grid on;
% legend('acc-estimation', 'radar-acc');
% title('加速度');
% 
% % ttc fcw
% figure()
% plot(save_ttc(1,:), save_ttc(2,:)); % ttc
% hold on;
% plot(save_fcw_state(1,:), save_fcw_state(2,:)*20); % fcw_mobileye
% NUM = length(save_ttc(1,:));
% plot(save_ttc(1,:), ones(1,NUM)*4)
% plot(save_ttc(1,:), ones(1,NUM)*-4)
% grid on;
% legend('ttc', 'fcw-mobileye');
% title('ttc&fcw');

