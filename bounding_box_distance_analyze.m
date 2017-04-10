% ��ȡbounding box������ԭʼ�������ݺ��״�����ݽ��жԱ�

clc
clear all
close all

%% ���ݵ���
source_addr = 'F:/����/FCW/case1/log';
% radar
data_addr = [source_addr , '/39946.log-radar.ini'];
data_t = load(data_addr)';
time_t = data_t(1,:) + data_t(2, :)*1e-6;
radar_data = [time_t; data_t(3:end, :)];

% bounding box range
data_addr = [source_addr , '/39946.vision_dist'];
vision_range_data = load(data_addr)';

% ͳһһ��ʱ����  ��0��ʼ
time_start = min(vision_range_data(1,1), radar_data(1,1));
radar_data(1,:) = radar_data(1,:) - time_start;
vision_range_data(1,:) = vision_range_data(1,:) - time_start;

%% plot
figure()
plot(radar_data(1,:), radar_data(2,:));
hold on;
plot(vision_range_data(1,:), vision_range_data(2,:));
grid on;
legend('radar', 'vision-raw');
title('����');

% figure()
% subplot(2,1,1)
% plot(vision_range_data(1,1:2802), vision_range_data(2,1:2802));
% 
% % subplot(2,1,2)
% hold on
% plot(vision_range_data(1,2803:end), vision_range_data(2,2803:end));
% title('����');