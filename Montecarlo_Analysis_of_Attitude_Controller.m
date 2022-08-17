clear 
close all 
clc
%%
% Defining PID controllers to be used in the analysis for task 1
R_phi_mc = tunablePID('R_phi_mc','P')
R_phi_mc.Kp.Value = 8.64;            

R_p_mc = tunablePID2('R_p','PID')
R_p_mc.Kp.Value = 0.399;             
R_p_mc.Ki.Value = 1.05;               
R_p_mc.Kd.Value = -4.38e-8;  
R_p_mc.c.Value = 0;
R_p_mc.c.Free = false;    
R_p_mc.b.Value = 1;
R_p_mc.b.Free = false;
R_p_mc.Tf.Value = 0.01;
R_p_mc.Tf.Free = false;

R_p_pid = tf(R_p_mc(1))
R_phi_pid = tf(R_phi_mc)
%%
% Defining PID controllers to be used in the analysis for task 2
R_y_mc = tunablePID('R_y_mc','P')
R_y_mc.Kp.Value = 1.02;           

R_v_mc = tunablePID2('R_v','PID')
R_v_mc.Kp.Value = 1.7;              
R_v_mc.Ki.Value = 0.588;               
R_v_mc.Kd.Value = 0.441;   
R_v_mc.c.Value = 0;
R_v_mc.c.Free = false;   
R_v_mc.b.Value = 1;
R_v_mc.b.Free = false;
R_v_mc.Tf.Value = 0.01;
R_v_mc.Tf.Free = false;

R_v_pid = tf(R_v_mc(1))
R_y_pid = tf(R_y_mc)

load("F_phi_tf.mat");
F_phi_tf.u = {'\phi_0'};
F_phi_tf.y = {'\phi'};
%%
g = 9.81;

integrator=tf(1,[1 0]);

N=500;

for n= 1:N
   n,    
   
    % Defining system derivatives using uncertainty ranges
    Y_v_mc(n) = normrnd(-0.1068,0.1068*3*4.26/100);  
    Y_p_mc(n) = normrnd(0.1192,0.1068*3*2.03/100);
    L_v_mc(n) = normrnd(-5.9755,5.9755*3*1.83/100);
    L_p_mc(n) = normrnd(-2.6478,2.6478*3*2.01/100);
    
    Y_d_mc(n) = normrnd(-10.1647,10.1647*3*1.37/100);
    L_d_mc(n) = normrnd(450.7085,450.7085*3*0.81/100);
    
    % For uncertain plant of position controller
    A_mc{n} = [L_p_mc(n) L_v_mc(n) 0; Y_p_mc(n) Y_v_mc(n) 0; 0 1 0];
    B_mc{n} = [0 g 0]';
    C_mc    = [0 1 0; 0 0 1];
    D_mc    = [0 0]';
    
    G_mc{n} = ss(A_mc{n},B_mc{n},C_mc,D_mc);
    G_mc_tf{n} = tf(G_mc{n});
    
    % For uncertain plant of attitude controller
    A_mc_1{n} = [Y_v_mc(n) Y_p_mc(n) g; L_v_mc(n) L_p_mc(n) 0; 0 1 0];
    B_mc_1{n} = [Y_d_mc(n) L_d_mc(n) 0]';
    C_mc_1    = [0 1 0; 0 0 1];
    D_mc_1    = [0 0]';
    
    G_mc_1{n} = ss(A_mc_1{n},B_mc_1{n},C_mc_1,D_mc_1);
    G_mc_tf_1{n} = tf(G_mc_1{n}); 
    
    % Defining the F and S of the inner loop for attitude controller
    F_1_1(n) = minreal((R_p_pid*G_mc_tf_1{n}(1))/(1+R_p_pid*G_mc_tf_1{n}(1)));
    S_1_1(n) = 1 - F_1_1(n);
    
    % Defining the F and S of the outer loop for attitude controller
    F_2_1(n) = minreal((integrator*F_1_1(n)*R_phi_pid)/(1+integrator*F_1_1(n)*R_phi_pid));
    S_2_1(n) = 1-F_2_1(n);
    
    % Defining the F and S of the inner loop
    F_1(n) = minreal((R_v_pid*F_2_1(n)*G_mc_tf{n}(1))/(1+R_v_pid*F_2_1(n)*G_mc_tf{n}(1)));
    S_1(n) = 1 - F_1(n);
    
    % Defining the F and S of the outer loop for poisiton controller
    F_2(n) = minreal((integrator*F_1(n)*R_y_pid)/(1+integrator*F_1(n)*R_y_pid));
    S_2(n) = 1-F_2(n);
    
    
    % Gain and Phase Margin
    [Gm(n),Pm(n)] = margin(minreal((integrator*F_1(n)*R_y_mc))); 
    
    t=(0:0.01:10);
    
    y=step(F_2(n),t);
    S_mc = stepinfo(y,t,1);

    % Settling Time and Overshoot
    Sett(n)=S_mc.SettlingTime;
    Over(n)=S_mc.Overshoot;
end

figure(6),hist(Pm,100), grid, title('Phase margin')

figure(7),hist(Gm,5000), grid, title('Gain margin')

figure(8),hist(Sett,100), grid, title('Settling time')

figure(9),hist(Over,100), grid, title('% Overshoot')