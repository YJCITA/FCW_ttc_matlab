%% 2017.03.13 �޸ĳ������Ӷ����������������
% �޸�ģ�� �ı�Ϊ�ȼ���ģ��  Ч���ǳ���
%% 2017.03.18 ��Ӧ�µ����ݸ�ʽ������������������ͳ����������
% ��bounding box��Ϊ�������KF����
%% 2017.03.20  ����汾Ч����OK���������������ĺ����ٶȹ��ƺ�IMU�жϵ��������ȥ����
%% 2017.03.21 ��Ӧ�캭�����ݽṹ
%% 2017.03.28 ����KF�����Ƿ�������P)--δ��ɣ�����������������
%  ���ӳ��٣��鿴�Ƿ��������ٶȸ��ٵ����� ���ٶ����⶯̬�Ի�ã������Ƕ��ڱ����Ӽ��ٱȽϾ��ҵ�ʱ��
%  ����Ŀ�공�����������ٶȵĹ���--�����жϺ��������ƺܲ�׼
%% 2017.04.06 ����radar ttc
clc
clear all
close all

%% ���ݵ���
% �����ļ���
% maindir = 'F:\����\FCW\0321_vison_radar\nj';
% maindir = 'F:\����\FCW\0321_vison_radar\radar';
maindir = 'F:\����\FCW\0321_vison_radar\mobileye_FCW'; % mobileye fcw
% maindir = 'F:\����\FCW\0321_vison_radar\acc_track'; % ����KF��������

sub_l1_dir  = dir( maindir );
% һ��Ŀ¼
for l1_i = 1 : length( sub_l1_dir )
    if( isequal( sub_l1_dir(l1_i).name, '.' )|| isequal( sub_l1_dir(l1_i).name, '..')||  ~sub_l1_dir(l1_i).isdir)               % �������Ŀ¼������
        continue;
    end
    for l2_i = 1 : 1
        subdirpath = fullfile( maindir, sub_l1_dir(l1_i).name, '*.critical' );
        txt_dir = dir( subdirpath );               % ���ļ������Һ�׺Ϊtxt���ļ�
        for txt_i = 1:length(txt_dir)
            vision_data_addr = fullfile( maindir, sub_l1_dir(l1_i).name, txt_dir(txt_i).name);
            raw_log_addr = [vision_data_addr(1:end-8), 'log-speed.ini'];
            fid_vision_log = fopen(vision_data_addr, 'r');
            fid_raw_log = fopen(raw_log_addr, 'r'); % ԭʼ��log
            if ~fid_vision_log || ~fid_raw_log
                break;
            end
            
           %% ��ʼ��
            ipm_step = 1; % ����
            is_first_read_timestamp = 1;
            time_start = 0;             
            speed_cur = 0;
            radar_data = [0  0 0 0]';
            radar_ttc = 0;
            % ��������
            struct_speed.data = 0;
            struct_speed.counter = 0;
            speed_update = 0; % �µ�speed���ݸ���
            radar_range = 0;
            radar_range_vel = 0;          
            
            %% �˲�
            is_first_read_camera_data = 1;
            is_kf_init_ok = 0; % KF�Ƿ��Ѿ����г�ʼ��
            is_first_radar_data = 1;
            save_i_index = 0; % ���ݴ洢����

            dv_filter = 0;
            is_first_vision_range = 1;
            fcw_state = 0;

            %% ��ѭ����ر���
            is_first_read_legal_data = 1;
            image_frame_id_pre = 0; % ��һ�μ���ʱ��image frame index ���ڼ���dt
            image_frame_T = 5*60/7744; % ��֡ͼƬ֮���dt  һ����Ƶ5min�������7744֡
            mobileye_fcw_state = 0; % mobileye fcw state  1��warning
            index_of_target_pre = 0; % ��һ֡Ŀ�공����ID
            is_new_target = 0; % �Ƿ�����Ŀ�공��            
            fcw_counter = 0; % ���㴥����������
            %% IMU
            is_imu_match_camera = 0; % imu��camera�Ƿ�ƥ��
            gyro_fliter = [0 0 0]';
            acc_fliter = [0 0 0]';
            vibe_gyro_fliter = [0 0 0]';
            vibe_acc_fliter = [0 0 0]';
            struct_gyro_d.data = [0, 0, 0]'; % ��¼��֮֡���������imu���ݣ�������ƽ��
            struct_gyro_d.counter = 0;            
            %% ADRC
            is_ADRC_init_OK = 0;
            z_residual = [0 ,0]'; % range  range_horizon
          %% ��ѭ��
            read_txt_line = 0; % ��ȡ���ļ��������������<100  ��Ϊ���ļ�
            is_first_read_log = 1;
            tt_state = feof(fid_vision_log);
            while ~feof(fid_vision_log) && ~feof(fid_raw_log) % ��ȡlog����    
                lineData = fgetl(fid_vision_log);
                if lineData == -1
                    break;
                else
                    read_txt_line = read_txt_line + 1;
                end
                exp1 = ',';
                str_line_raw = regexp(lineData, exp1, 'split'); %�Կո�Ϊ�����ָ��ַ���
                str_frame_index = str2num(str_line_raw{1,1}); % ÿһ��ͼ��Ӧ3֡ 0:detect_vehicle_record    1:radar-recod  2��mobileye
                % Ϊ�˻�ͼ����  ��ȡ��ʼʱ�̵�ʱ��
                if is_first_read_timestamp
                    % ��log��ʱ����ȽϿ���
                    lineData_raw_log = fgetl(fid_raw_log);
                    exp1 = ' ';
                    str_line_log_raw = regexp(lineData_raw_log, exp1, 'split'); %�Կո�Ϊ�����ָ��ַ���
                    time_s = str2num(str_line_log_raw{1,1});
                    time_us = str2num(str_line_log_raw{1,2});
                    time_start = time_s + time_us *1e-6;
                    is_first_read_timestamp = 0;
                end

                % ��ȡmobileye ����
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
                    % ���� index:0 ��ȡ����        
                    image_timestamp_cur = str2num(str_line_raw{1,2}) - time_start; 
                    image_frame_id = str2num(str_line_raw{1,4+2});
                    vison_range_raw = str2num(str_line_raw{1,10+2});
                    vision_horizon = str2num(str_line_raw{1,11+2});
                    index_of_target = str2num(str_line_raw{1,2+2});
                    ttc_raw = str2num(str_line_raw{1,14+2}); 
                    
                  %% ��ȡlog.txt���ݣ�����+imu��
                     is_imu_match_camera = 0;
                     while ~is_imu_match_camera && ~feof(fid_raw_log)                       
                        % ��һ�ν��� �ȶ�ȡһ������
                        while(is_first_read_log)
                            lineData_raw_log = fgetl(fid_raw_log);
                            exp1 = ' ';
                            str_line_log_raw = regexp(lineData_raw_log, exp1, 'split'); %�Կո�Ϊ�����ָ��ַ���
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

                        
                        % �ж���һ���ٶ�ʱ����Ƿ�ƥ���ϣ���Ϊspeed���±Ƚ��������Բ���ÿ�ν����ȽϾ��ȶ�ȡ���ݣ��ܿ��ܻ�һֱƥ�䲻�ϣ�
                        dt_imu_camera = time_log_pre - image_timestamp_cur;
                        if dt_imu_camera > -0.1
                            is_imu_match_camera = 1; % �����Ѿ�ƥ��
                        else
                            lineData_raw_log = fgetl(fid_raw_log);
                            exp1 = ' ';
                            str_line_log_raw = regexp(lineData_raw_log, exp1, 'split'); %�Կո�Ϊ�����ָ��ַ���
                            time_s = str2num(str_line_log_raw{1,1});
                            time_us = str2num(str_line_log_raw{1,2});
                            time_log = time_s + time_us *1e-6 - time_start;
                            time_log_pre = time_log;
                            str_line_data_flag = str_line_log_raw{1,3};
                            if strcmp(str_line_data_flag, 'brake_signal')
                                speed_cur = str2num(str_line_log_raw{1, 24})/3.6; 
                                % �ж�ʱ����Ƿ��Ѿ�ƥ����
                                dt_imu_camera = time_log - image_timestamp_cur;
                                if dt_imu_camera > -0.1
                                    is_imu_match_camera = 1; % �����Ѿ�ƥ��
                                end                            
                            end
                        end                                     
                     end
                    
                    if(is_first_read_legal_data)
                        image_timestamp_pre = image_timestamp_cur;
                        is_first_read_legal_data = 0;
                    end 

                  %%  �˲�          
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
                        
                        % ˮƽ
                        acc_h = 0; % target vehicle
                        speed_h = 0;       
                        vison_range_h_filter = vision_horizon;
                        Xk_h = [vision_horizon, speed_h, acc_h]';
                        Pk_h = diag([5, 4, 4]);
                        Q_h = diag([1,  1, 0.5]);
                        R_h = diag([0.5]);  
                        is_kf_init_ok = 1;
                    end
                    
                    % �ж� target ID����ȷ���Ƿ���һ���µĳ�
                    if (index_of_target_pre ~= index_of_target)
                        is_new_target = 1;
                    else
                        is_new_target = 0;                        
                    end
                    index_of_target_pre = index_of_target;                    
                    dt = image_timestamp_cur - image_timestamp_pre;
                    image_timestamp_pre = image_timestamp_cur; 
                    % Ϊ�˴���ʱ��û����Чǰ�������� dt̫���������˲�
                    % ������Ŀ�꣬ҲҪ�����˲�
                    if(dt > 0.3 || is_new_target)
                        % dt >1 ������
                        acc_t = 0; % target vehicle
                        speed_t = speed_cur;
                        acc_s = 0; % host vehicle
                        speed_s = speed_cur;
                        vison_range_filter = vison_range_raw;

                        Xk = [vison_range_filter, speed_t, speed_s, acc_t, acc_s]';
                        Pk = diag([100, 100, 10, 20, 10]);
                        Q = diag([25,  20, 4, 3, 3]);
                        R = diag([10, 1]); 
                        
                        % ˮƽ
                        acc_h = 0; % target vehicle
                        speed_h = 0;   
                        vison_range_h_filter = vision_horizon;
                        Xk_h = [vision_horizon, speed_h, acc_h]';
                        Pk_h = diag([5, 4, 4]);
                        Q_h = diag([1,  1, 0.5]);
                        R_h = diag([0.5]);  
                        is_kf_init_ok = 1;
      
                    else
                        % ��ͨ�˲����� 
                        filt_hz = 0.5; % Ϊ�˿��Ʋ���  ����0.7�Ǽ��ޣ�>1֮��ͻ����복���������µĳ��ٱ仯��
                        [ vison_range_filter ] = fun_LowpassFilter( vison_range_filter, vison_range_raw, dt, filt_hz ); 
                        
                        filt_hz = 0.2; 
                        [ vison_range_h_filter ] = fun_LowpassFilter( vison_range_h_filter, vision_horizon, dt, filt_hz ); 

                        % KF �˲�
                        % X = [d, vs, vt]' ���࣬�����ٶȣ�Ŀ�공��
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
                        
                        % ˮƽ
                        z_h = [vison_range_h_filter]';
                        F_h = [1  dt   0.5*dt^2
                             0   1    dt  ;
                             0   0    1  ;];
                        H_h = [ 1 0 0 ];
                        [Xk_h, Pk_h, z_residual_h] = fun_KF_resdiual(Xk_h, Pk_h, z_h, Q_h, R_h, F_h, H_h);
                        
                    end
                    
                  %% FCW�����߼��ж�  
                    Vt = Xk(2);
                    Vs = Xk(3);
                    acc_t = Xk(4);
                    acc_s = Xk(5);                    
                    range_cur = Xk(1);
                    vel_relative = Vt - Vs;
                    acc_relative = acc_t - acc_s;
                    relative_data = [range_cur, vel_relative, acc_relative]';
                    
                    % ˮƽ
                    horizon_range = Xk_h(1);
                    horizon_vel = Xk_h(2);
                    
                    % ����ttc 
                    if abs(vel_relative) >0.2
                        ttc = abs(range_cur/vel_relative);
                    else
                        ttc = -1;
                    end
                    
                    % radar ttc
                    if abs(radar_range_vel) >0.2
                        ttc_radar = abs(radar_range/radar_range_vel);
                    else
                        ttc_radar = -1;
                    end
                    
                    % fcw �����߼� ttc<4, ��Գ�����<20, ����>40/3.6, ˮƽ����<1.2, �ۼ�����5֡
                    % �����ж�mobileye�澯�ĳ������ǱȽϸ��ٵ�
                    if ttc >0
                        tt_horizon = abs(horizon_range + horizon_vel*ttc);
                    else
                        tt_horizon = -1;    
                    end
                    ttc_threhold = 3.0;
                    if(ttc<ttc_threhold  && ttc>0 && tt_horizon<3 && vel_relative<-1) && range_cur >= 11 && acc_relative < 0 && speed_cur>40/3.6%&& range_cur <= 55
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
                    
                  %% ��������
                    time_cur = image_timestamp_cur;
                    save_i_index = save_i_index + 1;
%                     save_Xk(:, save_i_index) = [time_cur; Xk];
                    save_Xk(:, save_i_index) = [time_cur; relative_data];
                    save_Xk_h(:, save_i_index) = [time_cur; Xk_h];    
                    save_tt_horizon(:, save_i_index) = [time_cur; tt_horizon];   % �������Ԥ��
                    save_vision_raw(:, save_i_index)= [time_cur; vison_range_raw];
                    save_ttc(:, save_i_index) = [time_cur; ttc ];
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
                    % ����
                    mobileye_fcw_state = 0;
                end        
            end
            
        % ����100 �Ų��ǿ��ļ�
        if read_txt_line >10             
            h = figure();
%             figure('visible','off')   
            set(h,'outerposition',get(0,'screensize'));
            % ����
            ax1 = subplot(4,1,1);
            plot(save_vision_raw(1,:), save_vision_raw(2,:), '.'); % vision range ����
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
            str_name = sprintf('log�ļ�: %s \n ���� ', vision_data_addr_t);
            ylim([-1, 100])
            xlim([0, 350]);
            title(str_name);
            
            % �ٶ� & ���ٶ� &�����ٶ�
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
            title('�ٶ�&���ٶ�')
            
            % �ٶ� & ���ٶ� &�����ٶ�
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
            title('�������&�����ٶ�')
            
            % ttc fcw
            ax4 = subplot(4,1,4);
            NUM1 = length(save_ttc);
            plot(save_ttc(1,:), save_ttc(2,:), '.'); % ttc
            hold on;
            plot(save_radar_ttc(1,:), save_radar_ttc(2,:), '.'); % ttc_radar      
            plot(save_fcw_state(1,:), save_fcw_state(2,:)*5); % fcw_minieye    
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
            linkaxes([ax1,ax2,ax3,ax4], 'x'); % ͬ����ͼ��������
            
            % ����figure
%             filename=[vision_data_addr_t, '--KF.fig'];
            filename=[vision_data_addr_t, '--KF.png'];
            saveas(h,filename)
            close(h)   
            
           %% ����KF
%             h = figure();
%             set(h,'outerposition',get(0,'screensize'));
%             % ����
%             ax1 = subplot(2,1,1);
%             plot(save_vision_raw(1,:), save_vision_raw(2,:), '.'); % vision range ����
%             hold on;
%             plot(save_Xk(1,:), save_Xk(2,:), '.'); % range-estimate
%             plot(save_radar_data(1,:), save_radar_data(2,:), '.'); % radar-range
%             grid on;
%             legend({'vision-range-measure-raw','range-estimate', 'radar-range'},'Location','northeast','FontSize',10);
%             legend('boxoff')
%             vision_data_addr_t = vision_data_addr;
%             str_name = sprintf('log�ļ�: %s \n ���� ', vision_data_addr_t);
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
%             title('�в�');
%             linkaxes([ax1,ax2], 'x'); % ͬ����ͼ��������
%             
%             % ����figure
%             filename=[vision_data_addr_t, '--residual.png'];
%             saveas(h,filename)
%             close(h)    
            
%             exp1 = '\';
%             str_line_raw = regexp(vision_data_addr_t, exp1, 'split'); %�Կո�Ϊ�����ָ��ַ���
%             str_frame_index = str2num(str_line_raw{1,1}); 
%             filename=[str_line_raw{1,6}, '\', str_line_raw{1,7}, '--KF.png'];
%             saveas(h,filename)
%             close(h)  

            % ���������txt
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
                save_P save_z_residual save_tt_horizon save_Xk_h
            fclose(fid_vision_log);
            fclose(fid_raw_log);
        end
    end
end

%% plot
% figure()
% plot(save_vision_raw(1,:), save_vision_raw(2,:)); % vision range ����
% hold on;
% plot(save_Xk(1,:), save_Xk(2,:)); % range-estimate
% plot(save_radar(1,:), save_radar(2,:)); % �״�
% grid on;
% legend('vision-range-measure-raw','range-estimate','radar-range');
% str_name = sprintf('���� ');
% title(str_name);
% 
% % �ٶ�
% figure()
% plot(save_relative_v(1,:), save_relative_v(2,:)); % vel estimation
% hold on;
% plot(save_radar(1,:), save_radar(3,:)); % �״��������ٶ�
% % plot(save_dv_radar_filter(1,:), save_dv_radar_filter(2,:)); % �״�����ֱ��΢�ֵ��ٶ�
% % plot(save_dv_filter(1,:), save_dv_filter(2,:)); % ֱ��΢�ֺ��ͨ�˲����ٶ�
% grid on;
% legend('vel-estimation', 'radar-vel', 'dv-�״�΢�ֺ��ͨ', 'dv-΢�ֺ��ͨ�˲�');
% title('�ٶ�');
% 
% % ���ٶ�
% figure()
% plot(save_Xk(1,:), save_Xk(4,:)); % acc estimation
% hold on;
% plot(save_radar(1,:), save_radar(4,:)); % �״����acc
% grid on;
% legend('acc-estimation', 'radar-acc');
% title('���ٶ�');
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

