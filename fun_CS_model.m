% CS�˶�ģ��
function [Xk_new Q] = fun_CS_model(Xk, E_a, alpha, a_max, T)
% Xk = [x x' x''];
% alpha: ����Ƶ��
% E_a�� ���ٶ�ƽ��ֵ
% aX_max,aY_max : [-max, +max] �����С���ٶ�
    acc_cur = Xk(3); % ��ǰ���Ƶļ��ٶ�
    O33 = diag([ 0 0 0 ]);
    O31 = [ 0 0 0]';
%% Q:ϵͳ���������� ����
    q11 = (1-exp(-2*alpha*T)+2*alpha*T+(2*alpha^3*T^3)/3-2*alpha^2*T^2-4*alpha*T*exp(-alpha*T))/(2*alpha^4);
    q12 = (exp(-2*alpha*T)+1-2*exp(-alpha*T)+2*alpha*T*exp(-alpha*T)-2*alpha*T+alpha^2*T^2)/(2*alpha^3);
    q13 = (1-exp(-2*alpha*T)-2*alpha*T*exp(-alpha*T))/(2*alpha^2);
    q22 = (4*exp(-alpha*T)-3-exp(-2*alpha*T)+2*alpha*T)/(2*alpha^2);
    q23 = (exp(-2*alpha*T)+1-2*exp(-alpha*T))/(2*alpha);
    q33 = (1-exp(-2*alpha*T))/(2); 
      
    Q_tmp = [ q11 q12 q13   %  ?? �Ƿ����� subQ = a*subQ;????
             q12 q22 q23
             q13 q23 q33 ];

    if (acc_cur >= 0 ) % ���ݵ�ǰ���ٶȵ�����??Xk1??  �������ٶȷ���
        R = (4-pi)*(a_max(2) - acc_cur)^2/pi; % ax_max=[-ax_max, ax_max]X����������ٶ�
    else
        R = (4-pi)*(a_max(1) + acc_cur)^2/pi;
    end
    
    Q = 2*R*Q_tmp;
%% ״̬����
    
    Fai = [ 1  T  (-1+alpha*T+exp(-alpha*T))/(alpha^2);  % һ��ά��
               0  1  (1-exp(-alpha*T))/alpha;
               0  0  exp(-alpha*T) ];      

    U = [ (-T + alpha*T^2 + ( 1-exp(-alpha*T) )/alpha )/ alpha;  % һ��ά��
              T - ( 1-exp(-alpha*T) )/alpha;
               1 - exp(-alpha*T) ]; 
     
    Xk_new = Fai*Xk + U*E_a;  
end