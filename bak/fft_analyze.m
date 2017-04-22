%% FCW ����Ƶ�ʷ���
% ���ȷ��������ݣ�����������ʻ������仯��Ƶ��Ӧ���ǲ������0.2~0.5hz���ң���������ٲ��Է������������ݣ�������
% ���Կ��Զ�bouding box�����������Ƚ��е�ͨ�˲���ȥ����Ƶ����

% �������ݽ����ز�����Ȼ�����FFTƵ�����
% save_z = [time, vision_range, vel_self]
close all
clear all
clc

%% ��������
vision_data = load('vision_range.mat');

%% imu������vision range�����Ա�
% �������ۣ�bounding box���������ݻᲨ�����ܿ��ܾ������ڳ����ĵ���Ӱ��ͼ��
source_addr = 'F:/����/FCW/case1/log';
% imu����
log_addr = [source_addr , '/39946.log-gsensor.ini'];
gsensor_data = load(log_addr)';
time_imu = gsensor_data(1,:) + gsensor_data(2,:)*1e-6;
data_imu_raw = [time_imu; gsensor_data(3:8, :)];
[ data_imu ] = fun_imu_data_trans( data_imu_raw );

% vision range����
log_addr = [source_addr , '/39946.vision_dist'];
vision_range_data = load(log_addr)';

time_start = min(vision_range_data(1,1), data_imu(1,1));

figure()
plot(data_imu(1,:)-time_start, data_imu(6,:)*100+30);
hold on;
grid on;
plot(vision_range_data(1,:)-time_start, vision_range_data(2,:));
legend('gyro-Y', 'vision-range');

% figure()
% hold on;
% grid on;
% plot(vision_range_data(1,:)-time_start, vision_range_data(2,:));
% legend('vision-range');

%% FFT Ƶ�����
% vision range
% �ز���
Fs = 25;  % ԭʼ���ݲ����25hz,���ֵ����Ӱ��Ƶ�׷���
vision_range_resample = resample(vision_range_data(2,:), vision_range_data(1, :), Fs);
fft_vision_range_sample = vision_range_resample;
fft_vision_range_sample = fft_vision_range_sample - mean(fft_vision_range_sample);
L = length(fft_vision_range_sample);
Y = fft(fft_vision_range_sample,L);%����fft�任
P2 = abs(Y/L);
P1 = P2(1 : L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;

figure();
plot(f,P1);% ��Ƶ��ͼ
grid on;
title('vision range ԭʼ����Ƶ������')

% imu
% �ز���
Fs = 100;  % ԭʼ���ݲ����25hz,���ֵ����Ӱ��Ƶ�׷���
data_resample = resample(data_imu(6,:), data_imu(1, :), Fs);
data_resample = data_resample - mean(data_resample);
L = length(data_resample);
Y = fft(data_resample,L);%����fft�任
P2 = abs(Y/L);
P1 = P2(1 : L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;

figure();
plot(f,P1);% ��Ƶ��ͼ
grid on;
title('imu-gyro-Y ԭʼ����Ƶ������')

%% ��ͨ�˲�����
% �Ƿ��ز������Ե�ͨ����ûӰ��
y = vision_range_data(2,1);
NUM = length(vision_range_data(2,:));
time_pre = vision_range_data(1,1);
for i = 1:NUM
    x_new =  vision_range_data(2,i);
    time_cur = vision_range_data(1,i);
    dt = time_cur - time_pre;
    time_pre = time_cur;
    filt_hz = 0.25;
    [ y ] = fun_LowpassFilter( y, x_new, dt, filt_hz );
    vision_new(:, i) = [time_cur; y];
    save_dt(1, i) = dt; 
end

figure();
plot(vision_range_data(1,:), vision_range_data(2,:)); % ԭʼvision range
hold on
plot(vision_new(1,:), vision_new(2,:)); % ��ͨ�˲���vision range
grid on;
legend('vision-range-raw', 'vision-range-filter')
str_name = sprintf('��ͨ��ֹƵ�ʣ� %.2f �˲�ǰ������  ', filt_hz);
title(str_name)


%% vision range ��ȥ��Ƶ���ݺ� �ڷ�����Ƶ������Ƶ��
% �ز���
Fs = 25;  % ԭʼ���ݲ����25hz,���ֵ����Ӱ��Ƶ�׷���
vision_data_remove_low_hz = vision_range_data(2,:) - vision_new(2,:);
vision_data_remove_low_hz = vision_data_remove_low_hz - mean(vision_data_remove_low_hz);
time_data = vision_new(1, :);
data_resample = resample(vision_data_remove_low_hz, time_data, Fs);
data_resample = data_resample - mean(data_resample);
L = length(data_resample);
Y = fft(data_resample,L);%����fft�任
P2 = abs(Y/L);
P1 = P2(1 : L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;

figure();
plot(f,P1);% ��Ƶ��ͼ
grid on;
title('vision range��ȥ��Ƶ���ݺ�Ƶ������')

%%  �Ա�imu������vision range����
figure()
plot(data_imu(1,:)-data_imu(1,1), data_imu(6,:));
grid on;
legend('gyro-Y')











