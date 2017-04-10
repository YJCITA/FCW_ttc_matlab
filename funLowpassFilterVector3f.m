% 1*3ʸ����ͨ�˲���
% y_pre����һ������ĵ�ͨ���ֵ
% x_new: �µ�����ֵ
% dt�� ǰ�����ε�ʱ����
% filt_hz����ͨ��ֹƵ��
function [ y ] = funLowpassFilterVector3f( y_pre, x_new, dt, filt_hz )

    if filt_hz == 0
        alpha = 1;
    else
        rc = 1/(2*pi*filt_hz);
        alpha = dt/(dt + rc);
    end    
    y = y_pre + alpha.*(x_new - y_pre);
end

