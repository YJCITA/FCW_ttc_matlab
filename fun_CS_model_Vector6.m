% CS�˶�ģ�� ״̬������6ά��
function [Xk_new Q] = fun_CS_model_Vector6(Xk, E_a, alpha, a_max, T, F, U)
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
      
    subQ = [ q11 q12 q13   %  ?? �Ƿ����� subQ = a*subQ;????
             q12 q22 q23
             q13 q23 q33 ];
     
    if (Xk(3) >= 0 ) % ���ݵ�ǰ���ٶȵ�����??Xk1??  �������ٶȷ���
        Ra_x = (4-pi)*(a_max(2) - Xk(3))^2/pi; % ax_max=[-ax_max, ax_max]X����������ٶ�
    else
        Ra_x = (4-pi)*(a_max(1) + Xk(3))^2/pi;
    end
    
    if (Xk(6)>=0)
        Ra_y = (4-pi)*(a_max(2) - Xk(6))^2/pi;  
    else
        Ra_y = (4-pi)*(a_max(1) + Xk(6))^2/pi;
    end
    subQ_x = 2*Ra_x*subQ;
    subQ_y = 2*Ra_y*subQ;
    Q = [subQ_x   O33;
           O33   subQ_y ];
%% ״̬����
%     
%     Fai = [ 1  T  (-1+alpha*T+exp(-alpha*T))/(alpha^2);  % һ��ά��
%                0  1  (1-exp(-alpha*T))/alpha;
%                0  0  exp(-alpha*T) ];      
% 
%     U = [ (-T + alpha*T^2 + ( 1-exp(-alpha*T) )/alpha )/ alpha;  % һ��ά��
%               T - ( 1-exp(-alpha*T) )/alpha;
%                1 - exp(-alpha*T) ]; 
     
    Xk_new = F*Xk + U*E_a;  
end