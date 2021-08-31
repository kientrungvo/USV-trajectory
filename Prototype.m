clc;
% Thong so may bay
Ix = 5.2 * 10^(-4);
Iy = 5.1 * 10^(-4);
Iz = 10^(-3);
Ixy = 0;
Iyz = 0;
Izx = 0;

m = 0.175;
g = 9.81;

Condition = [0;0;0;0.1;0.1;0;0;0;0];

% ---STEADY STATE---
% CAMEs
P1 = 0;
Q1 = 0;
R1 = 0;
L1 = 0;
M1 = 0;
N1 = 0;

% IKEs
phi_1 = 0;
theta_1 = 0;
psi_1 = 0;
d_phi_1 = 0;
d_theta_1 = 0;
d_psi_1 = 0;

% CLMEs
U1 = 0;
V1 = 0;
W1 = 0;
Fx1 = 0;
Fy1 = 0;
Fz1 = -m*g;

% TEs
U_1 = 0;
V_1 = 0;
W_1 = 0;

% ------------------------------

% ---SMALL PERTURBATION---
% State-space coefficients
% CAMEs - PQR
A_CAME = [ 0 0 0 0 0 0 0 0 0;...
           0 0 0 0 0 0 0 0 0;...
           0 0 0 0 0 0 0 0 0];

A_CAME(1,1) = (-Q1*Izx) * (-Iz/(Ix*Iz-Izx^2))...
              + (Q1*(Iy-Ix)) * (-Izx/(Ix*Iz-Izx^2));
A_CAME(1,2) = ((-P1*Izx) + R1*(Iz-Iy)) * (-Iz/(Ix*Iz-Izx^2))...
              + (P1*(Iy-Ix) + R1*Izx) * (-Izx/(Ix*Iz-Izx^2));
A_CAME(1,3) = (Q1*(Iz-Iy)) * (-Iz/(Ix*Iz-Izx^2))...
              + (Q1*Izx) * (-Izx/(Ix*Iz-Izx^2));

A_CAME(2,3) = (P1*(Ix-Iz) - 2*R1*Izx)*(-1/Iy);
A_CAME(2,1) = (R1*(Ix-Iz) + 2*P1*Izx)*(-1/Iy);

A_CAME(3,2) = (P1*(Iy-Ix) + R1*Izx) * (-Ix/(Ix*Iz-Izx^2))...
              + ((-P1*Izx) + R1*(Iz-Iy)) * (-Izx/(Ix*Iz-Izx^2));
A_CAME(3,1) = (Q1*(Iy-Ix)) * (-Ix/(Ix*Iz-Izx^2))...
              + (-Q1*Izx) * (-Izx/(Ix*Iz-Izx^2));
A_CAME(3,3) = (Q1*Izx) * (-Ix/(Ix*Iz-Izx^2))...
              + (Q1*(Iz-Iy)) * (-Izx/(Ix*Iz-Izx^2));
%--------------          
B_CAME = [ 0 0 0 0;...
           0 0 0 0;...
           0 0 0 0];
B_CAME(1,2) = Iz/(Ix*Iz-Izx^2);
B_CAME(1,4) = Izx/(Ix*Iz-Izx^2);

B_CAME(3,4) = Ix/(Ix*Iz-Izx^2);
B_CAME(3,2) = Izx/(Ix*Iz-Izx^2);

B_CAME(2,3) = 1/Iy;
               
        
% CLMEs - UVW
A_CLME = [ 0 0 0 0 0 0 0 0 0;...
           0 0 0 0 0 0 0 0 0;...
           0 0 0 0 0 0 0 0 0];

A_CLME(1,9) = -Q1;
A_CLME(1,2) = -W1;
A_CLME(1,8) = R1;
A_CLME(1,3) = V1;
A_CLME(1,5) = -g*cos(theta_1);

A_CLME(2,3) = -U1;
A_CLME(2,7) = -R1;
A_CLME(2,9) = P1;
A_CLME(2,1) = W1;
A_CLME(2,5) = -g*sin(phi_1)*sin(theta_1);
A_CLME(2,4) = g*cos(phi_1)*cos(theta_1);

A_CLME(3,8) = -P1;
A_CLME(3,1) = -V1;
A_CLME(3,7) = Q1;
A_CLME(3,2) = U1;
A_CLME(3,5) = -g*cos(phi_1)*sin(theta_1);
A_CLME(3,4) = -g*sin(phi_1)*cos(theta_1);

%--------
B_CLME = [ 0 0 0 0;...
           0 0 0 0;...
           0 0 0 0];
B_CLME(3,1) = 1;


% IKEs - Euler - Phi,Theta,Psi
A_IKE = [  0 0 0 0 0 0 0 0 0;...
           0 0 0 0 0 0 0 0 0;...
           0 0 0 0 0 0 0 0 0];
% d_psi equation
A_IKE(3,4) = (-d_theta_1*sin(phi_1) ...
             + d_psi_1*cos(phi_1)*cos(theta_1)) * (-sin(phi_1)/cos(theta_1))...
             + (- d_psi_1*sin(phi_1)*cos(theta_1)...
             - d_theta_1*cos(phi_1)) * (-cos(phi_1)/cos(theta_1));
A_IKE(3,5) = (-d_psi_1*sin(phi_1)*sin(theta_1)) * (-sin(phi_1)/cos(theta_1))...
             + (-d_psi_1*cos(phi_1)*sin(theta_1)) * (-cos(phi_1)/cos(theta_1));
A_IKE(3,2) = sin(phi_1)/cos(theta_1);
A_IKE(3,3) = cos(phi_1)/cos(theta_1);

% d_theta euqation
A_IKE(2,4) = (-d_theta_1*sin(phi_1) ...
             + d_psi_1*cos(phi_1)*cos(theta_1)) * (-cos(phi_1))...
             + (- d_psi_1*sin(phi_1)*cos(theta_1)...
             - d_theta_1*cos(phi_1)) * (sin(phi_1));
A_IKE(2,5) = (-d_psi_1*sin(phi_1)*sin(theta_1)) * (-cos(phi_1))...
             + (-d_psi_1*cos(phi_1)*sin(theta_1)) * (sin(phi_1));
A_IKE(2,2) = cos(phi_1);
A_IKE(2,3) = -sin(phi_1);

% d_phi equation
A_IKE(1,4) = (  (-d_theta_1*sin(phi_1) ...
                + d_psi_1*cos(phi_1)*cos(theta_1)) * (-sin(phi_1)/cos(theta_1))...
                + (- d_psi_1*sin(phi_1)*cos(theta_1)...
                - d_theta_1*cos(phi_1)) * (-cos(phi_1)/cos(theta_1))...
             ) * sin(theta_1);
A_IKE(1,5) = (  (-d_psi_1*sin(phi_1)*sin(theta_1)) * (-sin(phi_1)/cos(theta_1))...
                + (-d_psi_1*cos(phi_1)*sin(theta_1)) * (-cos(phi_1)/cos(theta_1))...
             ) * sin(theta_1)...
             + d_psi_1*cos(theta_1);
A_IKE(1,2) = ( sin(phi_1)/cos(theta_1) ) * sin(theta_1);
A_IKE(1,3) = ( cos(phi_1)/cos(theta_1) ) * sin(theta_1);
A_IKE(1,1) = 1;


% B_IKE
B_IKE = [ 0 0 0 0;...
          0 0 0 0;...
          0 0 0 0];
      
      
% Final state-space matrices
A = [A_CAME; A_IKE; A_CLME];
B = [B_CAME; B_IKE; B_CLME];
C = eye(9);
D = zeros(9,4);

