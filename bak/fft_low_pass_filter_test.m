%% FFT ����
% �����ֶ����ɺ��й̶�Ƶ�ʵ���ɢ���ݣ�����FFT�͵�ͨ�仯��֤
close all
clear all
clc

Fs = 20;
n=0:1/Fs:10;
A1=4;
A2=4;
f1=5;
f2=1;
data = A1*sin(f1*2*pi*n)+A2*sin(f2*2*pi*n);

%% low pass filter
NUM = length(data);
y = data(1);
for i = 1:NUM
    x_new = data(1,i);
    dt = 1/Fs;
    filt_hz = 1;
    [ y ] = fun_LowpassFilter( y, x_new, dt, filt_hz );
    data_new(1, i) = y;
end


%% FFT
% ԭʼ����
L = length(data);
Y = fft(data, L);%����fft�任
P2 = abs(Y/L);
P1 = P2(1 : L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;
figure();
plot(f,P1);%��Ƶ��ͼ
hold on;

% ��֮ͨ�������
L = length(data_new);
Y = fft(data_new, L);%����fft�任
P2 = abs(Y/L);
P1 = P2(1 : L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;

plot(f,P1);%��Ƶ��ͼ
grid on;
legend('raw', 'lowpass-filter');


% figure(1)
% Fs=100;
% subplot(3,1,1);
% t=0:1/Fs:1;
% plot(1000*t(1:50),data(1:50));
% xlabel('time(mm)')
% title('һԪʱ������ֱ��ͼ')
%  
% Y=fft(data,512);
% Pyy2=Y.*conj(Y)/512;
% f2=1000*(0:256)/512;
% subplot(3,1,2);
% plot(f2,Pyy2(1:257));
% title('��ɢ���ݵĸ���ҶƵ��ͼ')
% xlabel('Ƶ�ʣ�Hz��')