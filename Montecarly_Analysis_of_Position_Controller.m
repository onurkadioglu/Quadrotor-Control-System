clear 
close all 
clc
%%
% Defining PID controllers to be used in the Analysis
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
    
    A_mc{n} = [Y_v_mc(n) Y_p_mc(n) g; L_v_mc(n) L_p_mc(n) 0; 0 1 0];
    B_mc{n} = [Y_d_mc(n) L_d_mc(n) 0]';
    C_mc    = [0 1 0; 0 0 1];
    D_mc    = [0 0]';
    
    G_mc{n} = ss(A_mc{n},B_mc{n},C_mc,D_mc);
    G_mc_tf{n} = tf(G_mc{n}); 
    
    % Defining the F and S of the inner loop
    F_1(n) = minreal((R_p_pid*G_mc_tf{n}(1))/(1+R_p_pid*G_mc_tf{n}(1)));
    S_1(n) = 1 - F_1(n);
    
    % Defining the F and S of the outer loop
    F_2(n) = minreal((integrator*F_1(n)*R_phi_pid)/(1+integrator*F_1(n)*R_phi_pid));
    S_2(n) = 1-F_2(n);
    
    % Gain and Phase Margin
    [Gm(n),Pm(n)] = margin(minreal((integrator*F_1(n)*R_phi_pid))); 
    
    t=(0:0.01:10);
    
    y=step(F_2(n),t);
    S_mc = stepinfo(y,t,1);

    % Settling Time and Overshoot
    Sett(n)=S_mc.SettlingTime;
    Over(n)=S_mc.Overshoot;
end

figure(6),hist(Pm,100), grid, title('Phase margin')

figure(7),hist(Gm,500), grid, title('Gain margin')

figure(8),hist(Sett,100), grid, title('Settling time')

figure(9),hist(Over,100), grid, title('% Overshoot')