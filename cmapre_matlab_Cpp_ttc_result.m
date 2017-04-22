%% 2017.04.14 比较matlab和C++程序计算TTC结果

%%
clc
close all

kf_result = load('/home/yj/bak/data/fcw/C++_compare/fcw_new_data/kf_result.ini')';
diif_time = -0.74;
% ttc fcw
figure()
NUM1 = length(save_ttc);
plot(save_ttc(1,:), save_ttc(2,:), '.'); % ttc
hold on;
% plot(save_radar_ttc(1,:), save_radar_ttc(2,:), '.'); % ttc_radar      
% plot(save_fcw_state(1,:), save_fcw_state(2,:)*6); % fcw_minieye    
% plot(save_fcw_state(1,:), save_fcw_state_mobileye(2,:)*5); % fcw_mobileye              
NUM = length(save_ttc(1,:));
plot(save_ttc(1,:), ones(1,NUM)*ttc_threhold)
plot(kf_result(1,:)+diif_time, kf_result(2,:), '.'); % ttc_cpp  
grid on;
legend({'ttc', 'ttc-3.0', 'ttc-cpp'},'Location','northeast','FontSize',10);
legend('boxoff')
str_name = sprintf('ttc&fcw');
% ylim([-1, 10]);
% xlim([0, 350]);
title(str_name);

figure()
hold on;
plot(save_Xk(1,:), save_Xk_h(2,:), '.'); % vision horizon
plot(save_Xk(1,:), save_Xk_h(3,:), '.'); % vision horizon vel
plot(kf_result(1,:)+diif_time, kf_result(3,:), '.'); % vision horizon
plot(kf_result(1,:)+diif_time, kf_result(4,:), '.'); % vision horizon vel
grid on;
legend({ 'vision-horizon', 'vision-horizon-vel', 'vision-horizon-cpp', 'vision-horizon-vel-cpp'},'Location','northeast','FontSize',10);
legend('boxoff')
xlim([0, 350]);
title('横向距离')

figure()
hold on;
plot(save_Xk(1,:), save_Xk(2,:), '.'); % vision vertial
plot(save_Xk(1,:), save_Xk(3,:), '.'); % vision vertial vel
plot(kf_result(1,:)+diif_time, kf_result(5,:), '.'); % vision vertial
plot(kf_result(1,:)+diif_time, kf_result(6,:), '.'); % vision vertial vel
grid on;
legend({ 'vision-vertial', 'vision-vertial-vel', 'vision-vertial-cpp', 'vision-vertial-vel-cpp'},'Location','northeast','FontSize',10);
legend('boxoff')
xlim([0, 350]);
title('纵向距离')


% figure()
% ax1 = subplot(2,1,1)
% hold on;
% plot(save_Xk(1,:), save_Xk_h(2,:), '.'); % vision horizon
% plot(save_Xk(1,:), save_Xk_h(3,:), '.'); % vision horizon vel
% % plot(kf_result(1,:), kf_result(3,:), '.'); % vision horizon
% % plot(kf_result(1,:), kf_result(4,:), '.'); % vision horizon vel
% grid on;
% legend({ 'vision-horizon', 'vision-horizon-vel'},'Location','northeast','FontSize',10);
% legend('boxoff')
% 
% ax2 = subplot(2,1,2)
% hold on;
% plot(kf_result(1,:)+diif_time, kf_result(3,:), '.'); % vision horizon
% plot(kf_result(1,:)+diif_time, kf_result(4,:), '.'); % vision horizon vel
% grid on;
% legend({ 'vision-horizon-cpp', 'vision-horizon-vel-cpp'},'Location','northeast','FontSize',10);
% legend('boxoff')
% xlim([0, 350]);
% title('横向距离&本车速度')
% 
% linkaxes([ax1,ax2], 'x'); % 同步子图的坐标轴
%             
