% Brake Model for INCREASE of brake torque (equivalent to pressing brake pedal)

% PT2-Model Parameter for set-torque to measured acceleration:
K_brk = 27.5/563/25;
w0_brkinit = 5; % Frequency of first PT2 increase model used for blending
w0_brk = 10; % Frequency of second PT2 increase model used for blending
D_brk = 0.6;
brake_pres_exist_threshold = -0.1; % Brake threshold to switch delays during brake pressure build
delay_brake_inc = 0.08; % Delay in brake increase model when there is considerable pressure in system (sec)
delay_brake_inc_init = 0.3; % Delay in brake increase model when there is no considerable pressure in system (sec)
brakeBlendStrt = 0.02; % Deceleration by brake at which blending of pt2 models starts during increase phase
brakeBlendRange = 0.1; % Range of deceleration by brake in which blending of pt2 models happens during increase phase

% Brake Model for DECREASE of brake torque (equivalent to releasing brake pedal)

% PT1-Model Parameter for set-torque to measured torque:
K_brk_trq_dec = 1;
T_brk_trq_dec = 0.17;

% PT1-Model Parameter for measured torque to acceleration:
% If there are simulation problems when switching INC/DEC models, take 92.7% of K 
K_brk_accel_dec = 0.0021 * 0.927;
T_brk_accel_dec = 0.13;

%Brake torque distribution
Trq_Distrib_FL = 0.3;
Trq_Distrib_FR = 0.3;
Trq_Distrib_RL = 0.2;
Trq_Distrib_RR = 0.2;

% Acceleration to Torque conversion factor
accel_to_trq = 510; % approximately vehicle_mass * tire_radius

% SSM Functionality
Brk_SSM_Decel_Lim = -2; %m/s^2
Brk_SSM_Hold_Grd_Lim = -8; %m/s^3
Brk_SSM_Release_Grd_Lim = 8; %m/s^3

