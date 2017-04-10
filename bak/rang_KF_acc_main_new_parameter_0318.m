%% 2017.03.13 �޸ĳ������Ӷ����������������
% �޸�ģ�� �ı�Ϊ�ȼ���ģ��  Ч���ǳ���
% ��bounding box��Ϊ�������KF����
clc
clear all
close all

is_save_results = 0; % �Ƿ񱣴���

%% ���ݵ���
log_id_vector = [14364, 14468, 14156, 10300, 15937, 13796, 1487, 12416, 8429, 9569, 11979, 11191, 11226, 15793, 10553];
NUM_log = length(log_id_vector);
for log_index = 1:NUM_log
    log_ID = num2str(log_id_vector(log_index));
    source_addr = 'F:/����/FCW/dist_cases';
    log_addr = [source_addr , '/', log_ID, '.log.txt'];
    vison_range_addr = [source_addr , '/', log_ID, '.vision_dist'];
    % source_addr = 'F:/����/FCW/case1/log';
    % log_addr = [source_addr , '/39946.log.txt'];
    % vison_range_addr = [source_addr , '/39946.vision_dist'];
    % radar_addr = [source_addr , '/39946.log-radar.ini'];

    fid_log = fopen(log_addr,'r');
    fid_vison_range = fopen(vison_range_addr,'r');

    %% ��ʼ��
    is_start_from_beginning = 1; % �Ƿ��log�е�һ֡��ʼ����
    ipm_index = 2200; % ����һ֡ͼƬ��ʼipm
    ipm_step = 1; % ����

    % ��������
    struct_speed.data = 0;
    struct_speed.counter = 0;
    speed_update = 0; % �µ�speed���ݸ���
    radar_range_cur = 0;
    radar_vel_cur = 0;
    % �˲�
    is_first_read_camera_data = 1;
    is_kf_init_ok = 0; % KF�Ƿ��Ѿ����г�ʼ��
    is_first_radar_data = 1;
    save_i_index = 0; % ���ݴ洢����

    dv_filter = 0;
    % ��vision range �� radar���ݲ�ѯ���ȶ�ȡһ��ֵ
    % ��Ϊradar��camera���ݸ���Ƶ���ǽӽ����߸����ģ�����ÿ�β�ѯʱ�ȶ�ȡ�µ����ݣ������ǲ�ƥ��̶ȼӾ�
    % Ӧ����ÿ�ν����ѯ��ʱ����ƥ����һ�������Ƿ����
    vison_range_data = fgetl(fid_vison_range);
    str_line_raw = regexp(vison_range_data,' ','split'); %
    vison_range_cur = str2num(str_line_raw{1,2});
    vison_range_raw = vison_range_cur;
    is_first_vision_range = 1;
    fcw_state = 0;

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
            speed_update = 1;        
        % fcw
        elseif strcmp(str_line_data_flag, 'sound_type')
            fcw_state = str2num(str_line_raw{1, 24}); 
        % radar
        elseif strcmp(str_line_data_flag, 'Id')
            radar_range_cur = str2num(str_line_raw{1,16});
            radar_vel_cur = str2num(str_line_raw{1,29});
            radar_acc_cur = str2num(str_line_raw{1,25});
            if is_first_radar_data
                radar_range_pre = radar_range_cur;
                dv_radar_filter = 0;
                radar_time_pre = time;
                is_first_radar_data = 0;
            end
            dt_radar = time - radar_time_pre;
            radar_time_pre = time;

            % ֱ�Ӿ���΢�ּ�������ٶ�
            if dt_radar ~= 0
                dv_radar = (radar_range_cur - radar_range_pre)/dt_radar;
                radar_range_pre = radar_range_cur;
                [ dv_radar_filter ] = fun_LowpassFilter( dv_radar_filter, dv_radar, dt_radar, 0.5 );  
            end
        % camera
        elseif strcmp(str_line_data_flag, 'cam_frame')                              
            % ��ȡ����
            t_s = str2num(str_line_raw{1, 1});
            t_us = str2num(str_line_raw{1, 2});
            image_timestamp = t_s + t_us*1e-6;
            mp4_file_name_log = str_line_raw{1, 4}; % mp4�ļ�·��
            length_tmp = length(mp4_file_name_log);
            mp4_file_name_log = mp4_file_name_log(length_tmp-22 : length_tmp-4); 
            image_index_str = str_line_raw{1, 5};
            image_index_num = str2num(image_index_str) + 1; % log��ͼ��index����Ǵ�0��ʼ

            if(is_first_read_camera_data)
                iamge_timestamp_pre = image_timestamp;
                time_start = image_timestamp;
                if is_start_from_beginning
                    ipm_index = image_index_num+3;
                end
                is_first_read_camera_data = 0;
            end        

             % �ȶԵ�ǰͼ��index  ��Ϊ���ܿ�����step  && speed����Ҫ�и���
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
                        end 
                        is_vision_range_search_ok = 1;
                    else
                        vison_range_data = fgetl(fid_vison_range);            
                        vison_range_raw =  str2num(str_line_raw{1,2});
                    end                         
                end

              %%  �˲�          
                if(~is_kf_init_ok)
                    acc = 0;
                    speed_relative = 0;
                    Xk = [vison_range_cur, speed_relative, acc]';
                    Pk = diag([2, 1, 1]);
                    Q = diag([5,2,2]);
                    R = diag([4]);      
%                     Pk = diag([10, 5, 4]);
%                     Q = diag([5,2,2]);
%                     R = diag([5]);  

                    vison_range_pre = vison_range_raw;
                    is_kf_init_ok = 1;
                end
                % ��ͨ�˲�����                    
                dt = time_vision - time_vision_range_pre;
                time_vision_range_pre = time_vision;
                filt_hz = 3; % Ϊ�˿��Ʋ���  ����0.7�Ǽ��ޣ�>1֮��ͻ����복���������µĳ��ٱ仯��
                [ vison_range_cur ] = fun_LowpassFilter( vison_range_cur, vison_range_raw, dt, filt_hz ); 

                % X = [d, vs, vt]' ���࣬�����ٶȣ�Ŀ�공��
                % z = [vision_range car_speed]
                dt_image = image_timestamp - iamge_timestamp_pre;
                iamge_timestamp_pre = image_timestamp; 
                z = [vison_range_cur]';
    %             z = [radar_range_cur, speed_average]';  % ���״���Ϊ����
                F = [1  dt_image  1/2*dt_image^2;
                     0   1  dt_image;
                     0   0  1];
                 H = [1 0 0;];
                [Xk_new, Pk_new] = fun_KF(Xk, Pk, z, Q, R, F, H);
                Xk = Xk_new;
                Pk = Pk_new;

                % ��������
                time_cur = image_timestamp - time_start;
                time_cur_file = image_timestamp;
                save_i_index = save_i_index+1;
                save_Xk_file(:, save_i_index) = [time_cur_file; Xk_new]; % ���ڱ����ļ�
                save_Xk(:, save_i_index) = [time_cur; Xk_new];
                save_vision_raw(:, save_i_index)= [time_cur; vison_range_raw];
                dv =  Xk_new(2);% vt-vs 
                % ����ttc
                if abs(dv) >0.2
                    ttc = Xk_new(1)/abs(dv);
                else
                    ttc = 0;
                end
                save_ttc(:, save_i_index) = [time_cur; ttc];
                % �״�ttc
                if abs(radar_vel_cur) >0.2
                    ttc_radar = radar_range_cur/abs(radar_vel_cur);
                else
                    ttc_radar = 0;
                end
                save_ttc_radar(:, save_i_index) = [time_cur; ttc_radar];
                
                save_relative_v(:, save_i_index) = [time_cur; dv];
                save_z(:, save_i_index) = [time_cur; z];
                save_radar(:, save_i_index) = [time_cur; radar_range_cur; radar_vel_cur; radar_acc_cur];
                save_car_speed(:, save_i_index) = [time_cur; speed_average];
                save_dv_radar_filter(:, save_i_index) = [time_cur; dv_radar_filter];
                save_dt_image(:, save_i_index) = [time_cur; dt_image];
                save_fcw_state(:, save_i_index) = [time_cur; fcw_state];
                % ֱ�Ӿ���΢�ּ�������ٶ�
                if dt_image ~= 0
                    dv_t = (vison_range_raw - vison_range_pre)/dt_image;
                    vison_range_pre = vison_range_raw;
                    [ dv_filter ] = fun_LowpassFilter( dv_filter, dv_t, dt_image, 3 ); 
                    save_dv_t(:, save_i_index) = [time_cur; dv_t];
                    save_dv_filter(:, save_i_index) = [time_cur; dv_filter];
                end
                ipm_index = ipm_index + ipm_step;
            end
        end
    end
    
%     figure('visible','off')    
    h = figure();
    % ����
    ax1 = subplot(3,1,1);
    plot(save_vision_raw(1,:), save_vision_raw(2,:), '.'); % vision range ����
    hold on;
    plot(save_Xk(1,:), save_Xk(2,:)); % range-estimate
    plot(save_radar(1,:), save_radar(2,:), '.'); % �״�
    grid on;
    legend({'vision-range-measure-raw','range-estimate','radar-range'},'Location','northeast','FontSize',7);
    legend('boxoff')
    log_addr_t = [ log_ID, '.log.txt'];
    str_name = sprintf('log�ļ�: %s \n ���� ', log_addr_t);
    title(str_name);
    
    % �ٶ� & �����ٶ�
    ax2 = subplot(3,1,2);
    plot(save_relative_v(1,:), save_relative_v(2,:), '.'); % vel estimation
    hold on;
    plot(save_radar(1,:), save_radar(3,:), '.'); % �״��������ٶ�
    plot(save_radar(1,:), save_radar(4,:), '.'); % �״����acc
    plot(save_car_speed(1,:), save_car_speed(2,:), '.'); % speed-car    
    grid on;
    legend({'vel-estimation', 'radar-vel', 'radar-acc', 'speed-car'},'Location','northeast','FontSize',10);
    legend('boxoff')
    title('�ٶ�');

    % ttc fcw
    ax3 = subplot(3,1,3);
    NUM1 = length(save_ttc);
    plot(save_ttc(1,:), save_ttc(2,:), '.'); % ttc
    hold on;
    plot(save_ttc_radar(1,:), save_ttc_radar(2,:), '.'); % ttc-radar    
    plot(save_fcw_state(1,:), save_fcw_state(2,:)*5); % fcw_mobileye
    NUM = length(save_ttc(1,:));
    plot(save_ttc(1,:), ones(1,NUM)*3)
    grid on;
    legend({'ttc', 'ttc-radar', 'fcw-mobileye','ttc = 3'},'Location','northeast','FontSize',10);
    legend('boxoff')
    str_name = sprintf('ttc&fcw');
    ylim([-1, 10]);
    title(str_name);
    linkaxes([ax1,ax2,ax3], 'x'); % ͬ����ͼ��������
    
    % ����figure
    if is_save_results
        filename=[log_addr_t, '--plot.png'];
        saveas(gcf,filename)
    %     close(gcf)    

        % ���������txt
        NUM = length(save_Xk_file);
        est_log_name = ['./data/est_result/', log_ID, '.log_KF_estimation.txt'];
        fp = fopen(est_log_name, 'wt');
        for i = 1:NUM
            fprintf(fp, '%f %f %f %f\n', save_Xk_file(1, i), save_Xk_file(2, i), save_Xk_file(3, i), save_Xk_file(4, i));
        end
        fclose(fp);
    end
    
     clear save_Xk  save_relative_v save_ttc save_vision_raw save_fcw_state save_data_mobileye_tmp...
            save_index_of_target save_vision_horizon save_dt_image save_speed_car save_ttc_raw save_fcw_state_mobileye...
            save_radar save_ttc_radar save_car_speed
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

