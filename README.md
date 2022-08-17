# Quadrotor-Control-System
In this repository, a cascade PID controller is designed for a quadrotor, to control the attitude and the position. This group project is an exam project for Aerospace Control Systems course of Politecnico di Milano in the 2021/2022 academic year. The base code of Robust_Attitude_Controller is derived from exercise session 4 of the same course by Giovaanni Gozzini. This readme file is also based on the same lecture.


In this project, we are asked to design a control system for a quadrotor;


- Designing a controller to control the attitude: “Attitude Control”
- Designing a controller to control the position: “Position Control”

Also, “Robust Stability Analysis” will be carried out with two different methods considering the parametric uncertainty;


- Model Uncertainty Approach
- Monte-Carlo Study


The system is discretized to be implementable on a quadrotor as a real-time system.


The below uncertain parameters will be used in the subsequent analysis; where the numerical values of the parameters are given by (𝑔=9.81).

Stability derivatives;
- 𝑌𝑣 = −0.1068 1/𝑠 (4.26%)
- 𝑌𝑝 = 0.1192 𝑚/𝑠.𝑟𝑎𝑑 (2.03%)
- 𝐿𝑣 = −5.9755 𝑟𝑎𝑑/𝑚.𝑠 (1.83%)
- 𝐿𝑝 = −2.6478 1/𝑠 (2.01%)

Control derivatives;
- 𝑌𝑑 = −10.1647 𝑚/𝑠2 (1.37%)
- 𝐿𝑑 = 450.7085 𝑟𝑎𝑑/𝑠2 (0.81%)

## Attitude Controller
The state space system / dynamic model obtained using model identification techniques for attitude control

![image](https://user-images.githubusercontent.com/48325841/185126439-14020ef0-dc56-4165-a126-f0e737fa6081.png)

Performance requirement: Response of φ to the variation of φ0 must be equivalent to second-order model with the natural frequency, and damping;

- wu ≥ 10 rad/s
- ζ   ≥ 0.9
			                 	
Control Effort Requirement: |δ| ≤ 5 for a doublet change in φ0 of ± 10 deg amplitude.

With everything being defined, using structured analysis in Mixed Sensitivity Synthesis gives us the opportunity to tune the controllers with the following structure;

![image](https://user-images.githubusercontent.com/48325841/185127786-485d5383-7e0b-4373-8697-95cc484b0b13.png)

Where  W1(s) = Wp(s), and W2(s) = Wq(s).


- Through comparing the step responses, we can see how well the requirements are being complied with;
 ![image](https://user-images.githubusercontent.com/48325841/185129506-e78ca358-8a26-4352-bdee-84b56ba41c1e.png)
- The sensitivity and its weight can be plotted to verify nominal performance.
![image](https://user-images.githubusercontent.com/48325841/185130538-495b1975-4607-4ef2-b8ac-2040b2c37745.png)
- The control weight defined plotted with the control sensitivity (left) , and control effort plot (right) may be seen in the plots below;
![image](https://user-images.githubusercontent.com/48325841/185131694-28f65702-87d3-441b-aba4-c471e61001ca.png)
![image](https://user-images.githubusercontent.com/48325841/185131738-79ce7411-d3d8-4463-9a7d-d32d43ec87bc.png)
- As a final step; phase and gain margins for the whole closed loop system can be seen in the following plot;
![image](https://user-images.githubusercontent.com/48325841/185132340-9c88b3e7-6ec9-44c9-9a78-3c2201dc3d06.png)

## Position Controller
The state space system / dynamic model obtained using model identification techniques for position control;

![image](https://user-images.githubusercontent.com/48325841/185135077-c3d8fdd9-1843-460d-be90-7cad193a80e4.png)

Let’s look at the requirements we are given before designing a controller;

Performance requirement: Response of y to the variation of y0 must be equivalent to second-order model with the natural frequency, and damping;

- ωu ≥ 2.5 rad/s
- ζ   ≥ 0.9
			                 	
Control Effort Requirement: |φ| ≤ 10 for a doublet change in y0 of ± 1 m  amplitude.

Since we could not satisfy the requirements with the given control effort limitation, we will loosen the weight to see if our system can achieve what is asked with different control and performance weights. 

![image](https://user-images.githubusercontent.com/48325841/185136788-174c004d-ee9c-409d-89bd-13c1b24f7688.png)

New Control Effort Requirement: |φ| ≤ 40 for a doublet change in y0 of ± 1 m  amplitude.

- Comparing the step responses

![image](https://user-images.githubusercontent.com/48325841/185137986-8d407ae6-7dae-45c6-a4eb-f8d39c3326af.png)

- Nominal performance

![image](https://user-images.githubusercontent.com/48325841/185139123-a56c1550-3007-4f1a-b15f-91af102d2ddd.png)

- Below at left, we see the control weight plotted together with the control sensitivity function and at right, we see the control effort plot.
![image](https://user-images.githubusercontent.com/48325841/185139990-f14570b4-16ea-4663-86a0-2d84c7bc6115.png)
![image](https://user-images.githubusercontent.com/48325841/185140048-809a722f-2171-41bc-ac73-adc0d000e81a.png)


- Bode diagram for the whole closed loop system can be seen below.
![image](https://user-images.githubusercontent.com/48325841/185140373-baf8c4e8-3ad1-428e-9745-0603a64eff10.png)















