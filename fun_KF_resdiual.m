% 一步EKF
function [Xk, Pk, z_residual] = fun_KF(Xk_1, Pk_1, z, Q, R, F, H)
    x_dimension = length(Xk_1);
    In = eye(x_dimension);
    
    Xk_predict = F*Xk_1;               % 1.状态一步预测    
    P_tmp = F*Pk_1*F';
    Pk_predict = F*Pk_1*F' + Q;        % 2.预测误差协方差阵
    S = H*Pk_predict*H' + R;           % 信息协方差阵
    Kk = Pk_predict*H'*(S^-1);          %3. 增益矩阵
    Z_predict = H*Xk_predict;    
    z_residual = z - Z_predict;
%     tt1 = Kk*residual;
    Xk = Xk_predict + Kk*z_residual;% 4.状态估计
    Pk = (In - Kk*H)*Pk_predict;     % 5.协方差估计
end