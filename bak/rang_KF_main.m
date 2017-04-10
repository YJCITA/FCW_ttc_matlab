% ��bounding box��Ϊ�������KF����
% ���ڿ��Կ��Ǽ��뱾��speed

clc
clear all
close all

%% ���ݵ���
source_addr = 'F:/����/FCW/case1/log';
log_addr = [source_addr , '/39946.log.txt'];
vison_range_addr = [source_addr , '/39946.vision_dist'];
radar_addr = [source_addr , '/39946.log-radar.ini'];

fid_log = fopen(log_addr,'r');
fid_vison_range = fopen(vison_range_addr,'r');
fid_radar = fopen(radar_addr,'r');

%% ��ʼ��
ipm_index = 2200; % ����һ֡ͼƬ��ʼipm
ipm_step = 1; % ����

% ��������
struct_speed.data = 0;
struct_speed.counter = 0;

% �˲�
is_first_read_camera_data = 1;
is_kf_init_ok = 0; % KF�Ƿ��Ѿ����г�ʼ��
save_i_index = 0; % ���ݴ洢����

% ��vision range �� radar���ݲ�ѯ���ȶ�ȡһ��ֵ
% ��Ϊradar��camera���ݸ���Ƶ���ǽӽ����߸����ģ�����ÿ�β�ѯʱ�ȶ�ȡ�µ����ݣ������ǲ�ƥ��̶ȼӾ�
% Ӧ����ÿ�ν����ѯ��ʱ����ƥ����һ�������Ƿ����
vison_range_data = fgetl(fid_vison_range);
str_line_raw = regexp(vison_range_data,' ','split'); %
vison_range_cur = str2num(str_line_raw{1,2});
vison_range_raw = vison_range_cur;
is_first_vision_range = 1;

radar_data = fgetl(fid_radar);
str_line_raw = regexp(radar_data,' ','split'); %
radar_range_cur = str2num(str_line_raw{1,4});
radar_vel_cur = str2num(str_line_raw{1,6});
%% ��ѭ��
while ~feof(fid_log) % ��ȡlog����    
    lineData = fgetl(fid_log);
    str_line_raw = regexp(lineData,' ','split'); %�Կո�Ϊ�����ָ��ַ���
    time_s = str2num(str_line_raw{1,1});
    time_us = str2num(str_line_raw{1,2});
    time = time_s + time_us *1e-6;
    str_line_data_flag = str_line_raw(3);
    % Gsensor
    if  strcmp(str_line_data_flag, 'Gsensor')
        for i = 1:6
            imu_data_t(i, 1) = str2num(str_line_raw{1, i+3});
        end
        data_gensor_raw = [time; imu_data_t]; 
        
    % speed
    elseif strcmp(str_line_data_flag, 'brake_signal')
        speed_cur = str2num(str_line_raw{1, 24})/3.6; 
        struct_speed.data = struct_speed.data + speed_cur;
        struct_speed.counter =  struct_speed.counter + 1;
        
    % camera
    elseif strcmp(str_line_data_flag, 'cam_frame')                              
        % ��ȡ����
        t_s = str2num(str_line_raw{1, 1});
        t_us = str2num(str_line_raw{1, 2});
        image_timestamp = t_s + t_us*1e-6;
        mp4_file_name_log = str_line_raw{1, 4}; % mp4�ļ�·��
        length_tmp = length(mp4_file_name_log);
        mp4_file_name_log = mp4_file_name_log(length_tmp-22 : length_tmp-4); 
        
        if(is_first_read_camera_data)
            iamge_timestamp_pre = image_timestamp;
            time_start = image_timestamp;
            is_first_read_camera_data = 0;
        end
        dt_iamge = image_timestamp - iamge_timestamp_pre;
        iamge_timestamp_pre = image_timestamp;
                    
        image_index_str = str_line_raw{1, 5};
        image_index_num = str2num(image_index_str) + 1; % log��ͼ��index����Ǵ�0��ʼ

         % �ȶԵ�ǰͼ��index  ��Ϊ���ܿ�����step
        if  ipm_index == image_index_num  
            % �ٶȽ���ƽ��
            if struct_speed.counter > 0
                speed_average = struct_speed.data/struct_speed.counter;         
            end
            struct_speed.data = 0;
            struct_speed.counter = 0;
            
           %% ƥ��ʱ���  ��ȡbounding box��range
            is_vision_range_search_ok = 0;
            while (~feof(fid_vison_range) && ~is_vision_range_search_ok)                
                str_line_raw = regexp(vison_range_data,' ','split'); %�Կո�Ϊ�����ָ��ַ���
                time_vision = str2num(str_line_raw{1,1});
                dt1 = time_vision - image_timestamp;
                
                % ��ͨ�˲���ʼ��
                if(is_first_vision_range)
                    time_vision_range_pre = time_vision;
                    is_first_vision_range = 0;
                end
                if abs(dt1)<0.5 || dt1>0
                    if dt1 > 2
                        fprintf('vison_range search error!!\n ');
                    %                         return;
                    end 
                    is_vision_range_search_ok = 1;
                else
                    vison_range_data = fgetl(fid_vison_range);
                    % ��ͨ�˲�����                    
                    vison_range_raw =  str2num(str_line_raw{1,2});
%                     dt = time - time_vision_range_pre;
%                     time_vision_range_pre = time;
%                     filt_hz = 0.2;
%                     [ vison_range_cur ] = fun_LowpassFilter( vison_range_cur, vison_range_raw, dt, filt_hz );               
                end
            
               %% ƥ��ʱ���  ��ȡradar��range
                is_radar_search_ok = 0;
                while (~feof(fid_radar) && ~is_radar_search_ok)                
                    str_line_raw = regexp(radar_data,' ','split'); %�Կո�Ϊ�����ָ��ַ���
                    time = str2num(str_line_raw{1,1}) + str2num(str_line_raw{1,2})*1e-6;
                    dt1 = time - image_timestamp;
                    if abs(dt1)<0.5 || dt1>0
                        if dt1 > 2
                            fprintf('radar_data search error!!\n ');
                            return;
                        end 
                        radar_range_cur = str2num(str_line_raw{1,4});
                        radar_vel_cur = str2num(str_line_raw{1,6});
                        is_radar_search_ok = 1;
                    else
                        radar_data = fgetl(fid_radar);                   
                    end
                end                
            end
            
          %%  �˲�          
            if(~is_kf_init_ok)
                Xk = [vison_range_cur, speed_average, 0]';
                Pk = diag([10, 1, 5]);
                Q = diag([1,1,1]);
                R = diag([5, 1]);
                
                is_kf_init_ok = 1;
            end
            % ��ͨ�˲�����                    
            dt = time_vision - time_vision_range_pre;
            time_vision_range_pre = time_vision;
            filt_hz = 0.2;
            [ vison_range_cur ] = fun_LowpassFilter( vison_range_cur, vison_range_raw, dt, filt_hz ); 
                
            % X = [d, vs, vt]' ���࣬�����ٶȣ�Ŀ�공��
            % z = [vision_range car_speed]
            dt = dt_iamge;
            z = [vison_range_cur, speed_average]';
%             z = [radar_range_cur, speed_average]';  % ���״���Ϊ����
            F = [1  dt  -dt;
                 0   1  0;
                 0   0  1];
             H = [1 0 0;
                  0 1 0];
            [Xk_new, Pk_new] = fun_KF(Xk, Pk, z, Q, R, F, H);
            Xk = Xk_new;
            Pk = Pk_new;
            
            % ��������
            time_cur = image_timestamp-time_start;
            save_i_index = save_i_index+1;
            save_Xk(:, save_i_index) = [time_cur; Xk_new];
            save_vision_raw(:, save_i_index)= [time_cur; vison_range_raw];
            dv =  Xk_new(3) - Xk_new(2);% vt-vs 
            save_relative_v(:, save_i_index) = [time_cur; dv];
            save_z(:, save_i_index) = [time_cur; z];
            save_radar(:, save_i_index) = [time_cur; radar_range_cur; radar_vel_cur];
            
            ipm_index = ipm_index + ipm_step;
        end
     
    end
end

%% plot
% ������
figure()
plot(save_vision_raw(1,:), save_vision_raw(2,:)); % vision range ����
hold on;
plot(save_Xk(1,:), save_Xk(2,:)); % range estimation
plot(save_radar(1,:), save_radar(2,:)); % �״�
grid on;
legend('vision-range-measure-raw', 'range-estimation', 'radar-range');
str_name = sprintf('���� Q(1,1) %.2f R(1,1) %.2f ', Q(1,1),  R(1,1));
title(str_name);

% ������
figure()
hold on;
plot(save_Xk(1,:), save_Xk(2,:)); % range estimation
plot(save_radar(1,:), save_radar(2,:)); % �״�
grid on;
legend( 'range-estimation', 'radar-range');
str_name = sprintf('���� Q(1,1) %.2f R(1,1) %.2f ', Q(1,1),  R(1,1));
title(str_name);

% �ٶ�
figure()
plot(save_relative_v(1,:), save_relative_v(2,:)); % vel estimation
hold on;
plot(save_radar(1,:), -save_radar(3,:)); % �״��������ٶ�
grid on;
legend('vel-estimation', 'radar-vel');
title('�ٶ�');

% figure()
% plot(save_Xk(1,:), save_Xk(3,:)); % vel estimation
% hold on;
% plot(save_z(1,:), save_z(3,:)); %������������
% grid on;
% legend('vel-estimation', 'self-vel');
% title('�����ٶ�');

% figure()
% subplot(2,1,1)
% plot(vision_range_data(1,1:2802), vision_range_data(2,1:2802));
% 
% % subplot(2,1,2)
% hold on
% plot(vision_range_data(1,2803:end), vision_range_data(2,2803:end));
% title('����');