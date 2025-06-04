clc; clear;
T = 0.001;
t = 0 : T : 10;
%% constants
ep_p = 1; gamma_1 = 0.1; gamma_2 = 0.1; gamma_3 = 0.1; a_1 = 0.8; a_2 = 1.1;
%% variables
x_p = zeros(1,size(t,2));
x_p_dot = x_p; x_1 = x_p; x_2 = x_p; u = x_p; d = x_p;
d_hat = x_p; x_e = x_p; x_e_hat = x_p;
%% initialization
x_2(2) = 0;
%% loops
for i = 1 : size(t,2)
    past = x_e(i);
    d(i) = 0.3*sin(t(i));
    x_e(i) = x_2(i) - x_p(i);
    x_p(i+1) = (10*u(i) + ep_p*x_e(i))*T + x_p(i);
    x_e(i) = x_2(i) - x_p(i);
    x_e_dot(i) = (x_e(i) - x_e(i-1))/T;
    d_hat(i) = ep_p*x_e_hat(i) + x_e_dot(i);
    x_e_error = x_e(i) - x_e_hat(i);
    x_e_hat(i+1) = (x_e_dot(i) + gamma_1*x_e_error^a_1*sign(x_e_error)...
        + gamma_2*x_e_error^a_2*sign(x_e_error) + gamma_3*x_e_error)*T + x_e_hat(i);
end