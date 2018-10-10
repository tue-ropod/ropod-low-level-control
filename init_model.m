clearvars

%% ROS configuration

setenv('ROS_IP','localhost')
setenv('ROS_HOSTNAME','')
%% Smart wheel command specification
SW_COM1_ENABLE1         =  hex2dec('0001');             % 0x0001
SW_COM1_ENABLE2         =  hex2dec('0002');             % 0x0002
SW_COM1_MODE_TORQUE	    =  bitshift(hex2dec('0000'),2); % (0x0 << 2)
SW_COM1_MODE_DTORQUE	=  bitshift(hex2dec('0001'),2); % (0x1 << 2)
SW_COM1_MODE_VELOCITY	=  bitshift(hex2dec('0002'),2); % (0x2 << 2)
SW_COM1_MODE_DVELOCITY	=  bitshift(hex2dec('0003'),2); % (0x3 << 2)
SW_COM1_EMERGENCY1		=  hex2dec('0010');             %  0x0010	
SW_COM1_EMERGENCY2		=  hex2dec('0020');             %  0x0020
SW_COM1_USE_TS			=  hex2dec('8000');             %  0x8000



%% smart wheels limits and offset
run /home/cesar/Documents/ROPOD_LINUX/Matlabdocs/Global_Libraries/ropod_parameters/motor_parameters
sw_ini_enable   = 1;
max_sw_current  =  10; % [A]
sw_tau_2_curr = 1/motor_physical_parameters.torqueconstant.value;
max_sw_tau      = max_sw_current/sw_tau_2_curr;
max_hw_tau      = 7.0;
pivot_offs_sws  = [2.663 3.74064 5.877 3.875]; % Vector with pivot offsets

%% Ropod max limits

% This limits should be compatible with ID signal
max_ropod_vel_xy     =  1.0; % [m/s]
max_ropod_acc_xy     =  1.0; % [m/s^2]
max_ropod_vel_theta  =  1.6; % [rad/s]
max_ropod_acc_theta  =  1.6; % [rad/s^2]
max_ropod_sw_force   =  500; % [N]
max_ropod_sw_tau     =  250; % [Nm]



%% Initialze kinemtic-dynamic model and control parameters
Nwheels = 4; % Do not change this parameter. The simulink model would need to change as well
set_ropod_KinModparams;
%% Sample times and initialization
Ts = 0.001; % Controller 
Tsp = 1;
Ts_rost = 0.01;

Tinit = 5; % Time for initialization

%% Platform Vel controller

% load platform_vel_cntr_GLL.mat
dxr_cntr = tf(0,1,Ts); %shapeit_data.C_tf_z;
dyr_cntr = tf(0,1,Ts); %shapeit_data.C_tf_z;
dthetar_cntr = tf(0,1,Ts); %shapeit_data.C_tf_z;

K_gain_dxdy_cntr = 150;
I_fhz_dxdy_cntr = 0.25;
LL_wz_fhz_dxdy_cntr = 20; % Use the same value for not having the LL
LL_wp_fhz_dxdy_cntr = 20;
LPF_fhz_dxdy_cntr = 100;
FFxy_mass = 0*80;

K_gain_dtheta_cntr = 100;
I_fhz_dtheta_cntr = 0.3;
LL_wz_fhz_dtheta_cntr = 20; % Use the same value for not having the LL
LL_wp_fhz_dtheta_cntr = 20;
LPF_fhz_dtheta_cntr = 100;
FFtheta_intia = 0;


%% Wheel Vel controller
% No integrators at the wheel velocity level!;


K_gain_dvarphi_cntr = 0.05;
I_fhz_dvarphi_cntr = 0;
LL_wz_fhz_dvarphi_cntr = 2; % Use the same value for not having the LL
LL_wp_fhz_dvarphi_cntr = 10;
LPF_fhz_dvarphi_cntr = 50;

LPF_fhz_dvarphi_meas = 50;
LPF_fhz_ddelta_meas = 50;

%% Parameters prefix
param_prefix = "/ropod/ropod_4wheel_kinb_cntr_node/";







