clear
close all
clc
%% 
% One needs to run the previous code first, which is "task1.mlx", to get the 
% transfer function in between '/phi_0' and '/phi' (true roll angle) , and then 
% continue with the procedure here!

Y_v_nom = -0.1068;       % [1/s]         -> 4.26 %
Y_p_nom = 0;             % [m/(s rad)]   -> -
L_v_nom = 0;             % [rad s/m]     -> 1.83 %
L_p_nom = -2.6478;       % [1/s]         -> 2.01

Y_d_nom = -10.1647;      % [m/s^2]       -> 1.37 %
L_d_nom = 450.7085;      % [rad/s^2]     -> 0.81 %
%%
unc_Y_v = 3*4.26;
unc_L_p = 3*2.01;
unc_Y_d = 3*1.37;
unc_L_d = 3*0.81;

% Stability Derivatives
Y_v = ureal('Y_v',Y_v_nom,'Perc',unc_Y_v);
Y_p = Y_p_nom;
L_v = L_v_nom;
L_p = ureal('L_p',L_p_nom,'Perc',unc_L_p);

% Control Derivatives
Y_d = ureal('Y_d',Y_d_nom,'Perc',unc_Y_d);
L_d = ureal('L_d',L_d_nom,'Perc',unc_L_d);
%% 
% The new state-space matrices for second task should be created using the derivatives 
% above

g = 9.81;   % Gravity Constant
Ts = 0.01;  % Sampling Time

% State Space Model for Position Control
A = [L_p L_v 0; Y_p Y_v 0; 0 1 0];
B = [0 g 0]';
C = [0 1 0; 0 0 1];
D = [0 0]';
%%
% Uncertain Plant
sys_ld = ss(A,B,C,D)
sys_ld.u = '\phi';
sys_ld.y = {'v','y'};
sys_ld_d = c2d(tf(sys_ld),Ts,'tastin');

% Nominal Plant
sys_ld_nom = sys_ld.NominalValue
sys_ld_nom.u = '\phi';
sys_ld_nom.y = {'v','y'};
sys_ld_nom_d = c2d(tf(sys_ld_nom),Ts,'tastin');
%%
G_phi_v_nom = tf(sys_ld_nom(1));

figure
pzplot(sys_ld(1));
hold on
pzplot(sys_ld_nom(1));
grid on
legend('Uncertain model','Nominal model','location','east')

z_phi_v = tzero(sys_ld_nom(1))
p_phi_v = pole(sys_ld_nom(1))
%%
G_phi_y_nom = tf(sys_ld_nom(2));

figure
pzplot(sys_ld(2));
hold on
pzplot(sys_ld_nom(2));
grid on
legend('Uncertain model','Nominal model','location','east')

z_phi_y = tzero(sys_ld_nom(2))
p_phi_y = pole(sys_ld_nom(2))
%%
figure
bode(sys_ld)
hold on
bode(sys_ld_nom)
grid on
legend('Uncertain model','Nominal model','location','northwest')
%%
figure
step(sys_ld)
hold on
step(sys_ld_nom)
grid on
legend('Uncertain model','Nominal model','location','northwest')
%%
R_y = tunablePID('R_y','P',Ts);
R_y.u = 'e_y';
R_y.y = 'v_0';
%%
R_v = tunablePID2('R_v','PID',Ts);
R_v.c.Value = 0;
R_v.c.Free = false;
R_v.b.Value = 1;
R_v.b.Free = false;
R_v.Tf.Value = 0.01;
R_v.Tf.Free = false;
R_v.u = {'v_0','v'};
R_v.y = {'\phi_0'};
%%
SumOuter = sumblk('e_y = y_0 - y');
%%
load("F_phi_tf.mat");
F_phi_tf.u = {'\phi_0'};
F_phi_tf.y = {'\phi'};
F_phi_tf_d = c2d(F_phi_tf,Ts,'tastin');

CL_tun = connect(SumOuter,R_y,R_v,F_phi_tf_d,sys_ld_nom_d,'y_0',{'y','v'});
%%
omega_n = 2.5;       % [rad/s]
xi = 0.9;

s = tf('s');

F_2 = omega_n^2/(s^2 + 2*s*xi*omega_n + omega_n^2);
F_2_d = c2d(F_2,Ts,'tastin');
S_2 = 1-F_2;
S_2_d = c2d(S_2,Ts,'tastin');

figure
step(F_2_d)
grid on

stepinfo(F_2_d)
%%
figure
bodemag(F_2)
hold on
bodemag(S_2)
grid on
legend('$F\_2$','$S\_2$','interpreter','latex')
%% 
% $$W_p(s) = \frac{\frac{s}{M} + \omega_b}{s + A \omega_b}$$
% 
% The objective is that $1/W_p(s)$ is used to shape our sensitivity function 
% in order to have $S(s) \simeq S_2(s)$.
% 
% Where $M$ is related to the peak, $A$ is related to the value at steady state 
% and $\omega_b$ is related to the cross-over frequency and so to the settling 
% time of our response.

M =1.6;
A = 3e-2;
omega_b = 0.5*omega_n;

W_p = (s/M + omega_b)/(s + A*omega_b);
W_p2inv = makeweight(1e-3,8,1.3,0,1);
W_p2 = 1/W_p2inv;

W_p_d = c2d(W_p, 0.01,'tastin');
W_p2inv_d = c2d(W_p2inv, 0.01,'tastin');
W_p2_d = c2d(W_p2, 0.01,'tastin');

figure
bodemag(F_2)
hold on
bodemag(S_2)
grid on
bodemag(1/W_p)
bodemag(1/W_p2)
legend('$F\_2$','$S\_2$','$1/W\_p$','$1/W\_{p\_{2}}$','interpreter','latex', 'location','southeast')
%%
alpha = 2/deg2rad(40);
w_tau = 2.5;

W_q = alpha*(s+w_tau*1e-3)/(s+w_tau);

figure
bodemag(1/W_q)
grid on
%%
W_p.u = 'e_y';
W_p.y = 'z_1';

W_q.u = '\phi';
W_q.y = 'z_2';

W_p_d = c2d(tf(W_p),Ts,'tastin');
W_q_d = c2d(tf(W_q),Ts,'tastin');
%%
%We have to define our new closed-loop to include also the weights defined. For simplicity we have added three
% additional analysis points that will be useful to compute sensitivities of the tuned closed-loop

CL0 = connect(R_v, R_y, F_phi_tf_d, sys_ld_nom_d, SumOuter, W_p_d, W_q_d,...
    'y_0',{'z_1','z_2'},{'\phi','e_y','y'});
%%
opt = hinfstructOptions('Display','final','RandomStart',10)

[CL,gamma,info] = hinfstruct(CL0,opt);
%%
showTunable(CL)
%%
F = getIOTransfer(CL,'y_0','y');
F_c = d2c(tf(F),'tastin');

figure
bodemag(F_c)
grid on
%%
figure
step(F)
hold on
step(F_2_d)
grid on
legend('hinfstruct','second order', 'location','southeast')

stepinfo(F)
%%
S = getIOTransfer(CL,'y_0','e_y');
S_c = d2c(tf(S),'tastin');

figure
bodemag(S_c)
grid on
hold on
bodemag(1/W_p)
legend('$S$','$1/W\_p$','interpreter','latex', 'location','southeast')
%%
Q = getIOTransfer(CL,'y_0','\phi');
Q_c = d2c(tf(Q),'tastin');

figure
bodemag(Q_c)
grid on
hold on
bodemag(1/W_q)
legend('$Q$','$1/W\_q$','interpreter','latex', 'location','southeast')
%%
L = (1-S_c)/S_c;
%L_c = d2c(tf(L),'tastin');

figure
margin(L)
grid on
%%
t = linspace(0,6,10^4);
u = 0*(t<=1) + 1*(t>1 & t<= 3) - 1*(t>3 & t<= 5) + 0*(t>5);
u = rad2deg(u);

figure
lsim(Q_c,u,t);
hold on
grid on
%%
sys_ld_array = usample(sys_ld(2),60);
%%
[~,Info] = ucover(sys_ld_array,sys_ld_nom(2),3);

W = Info.W1;


figure
bodemag(W,'r')
hold on
bodemag((sys_ld_nom(2)-sys_ld_array)/sys_ld_nom(2),{10e-4,10e4})
grid on
%%
figure
bodemag(F_c,'r',1/W,'g')
grid on
legend('$F$','$1/W$','interpreter','latex', 'location','southeast')
%%
w = logspace(-5,5,500);

FW = bode(F_c*W,w);
SW = bode(S_c*W_p,w);

figure
semilogx(w, 20*log10(squeeze(FW)) + 20*log10(squeeze(SW)))
hold on
semilogx(w,20*log10(w./w))
grid on
legend('$|W_p S|+|W F|$','$1$','interpreter','latex', 'location','southeast')

% % % % % % %--------------------------- end of the script ---------------------------% % % % % % %