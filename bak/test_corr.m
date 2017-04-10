% x = save_gyro_d_average(3, :)';
% y = save_vision_raw(2, :)';
x = save_gyro_d_average(3, :)';
y = save_gyro_d_average(3, :)';
r= corr(x, y, 'type' , 'Spearman');  