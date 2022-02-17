clearvars

%% Smart wheel command specification
SW_COM1_ENABLE1         =  hex2dec('0001');             % 0x0001
SW_COM1_ENABLE2         =  hex2dec('0002');             % 0x0002
SW_COM1_MODE_TORQUE	    =  bitshift(hex2dec('0000'),2); % (0x0 << 2)
SW_COM1_MODE_DTORQUE	=  bitshift(hex2dec('0001'),2); % (0x1 << 2)
SW_COM1_MODE_VELOCITY	=  bitshift(hex2dec('0002'),2); % (0x2 << 2)
SW_COM1_MODE_DVELOCITY	=  bitshift(hex2dec('0003'),2); % (0x3 << 2)
SW_COM1_EMERGENCY1		=  hex2dec('0010');             %  0x0010	
SW_COM1_EMERGENCY2		=  hex2dec('0020');             %  0x0020
SW_COM1_ENABLESERVO     =  hex2dec('0400');             %  0x0400
SW_COM1_SERVOCLOSE      =  hex2dec('0800');             %  0x0800
SW_COM1_USE_TS			=  hex2dec('8000');             %  0x8000

%% Multi-ropod configuration and control
Nagents = 2;
AgentNumber = 1;

%% smart wheels limits and offset
run motor_parameters
sw_ini_enable   = 1;
max_sw_current  =  10; % [A]
sw_tau_2_current = 1/motor_physical_parameters.torqueconstant.value;
max_sw_tau      = max_sw_current/sw_tau_2_current;
max_hw_tau      = 7.0;

if AgentNumber == 1
    pivot_offs_sws  = [.23, 6.22, 0.03, 4.89]; % Vector with pivot offsets for ROPOD2     
    ropod_position_inflock = [1 0 0];
elseif AgentNumber == 2
%     pivot_offs_sws  = [ 0.00, 0.57, 0.00, 1.57]; % Vector with pivot offsets for ROPOD1
    pivot_offs_sws  = [0.0243    0.4689    0.3384    5.9050]; % Vector with pivot offsets for ROPOD_BRSU   
    ropod_position_inflock = [-1 0 3.1416];
end




%% Ropod max limits

% This limits should be compatible with ID signal
max_ropod_vel_xy     =  2.0; % [m/s]
max_ropod_acc_xy     =  2.0; % [m/s^2]
max_ropod_vel_theta  =  1.6; % [rad/s]
max_ropod_acc_theta  =  1.6; % [rad/s^2]
max_ropod_sw_force   =  500; % [N]
max_ropod_sw_tau     =  250; % [Nm]




%% Initialze kinemtic-dynamic model and control parameters
Nwheels = 4; % Do not change this parameter. The simulink model would need to change as well
set_ropod_KinModparams;


%% Multi-ropod control parameters

K_gain_dxdy_flock_cntr = 200;
I_fhz_dxdy_flock_cntr = 0.1;
LL_wz_fhz_dxdy_flock_cntr = 20; % Use the same value for not having the LL
LL_wp_fhz_dxdy_flock_cntr = 20;
LPF_fhz_dxdy_flock_cntr = 40;

K_gain_dtheta_flock_cntr = 100;
I_fhz_dtheta_flock_cntr = 0.1;
LL_wz_fhz_dtheta_flock_cntr = 20; % Use the same value for not having the LL
LL_wp_fhz_dtheta_flock_cntr = 20;
LPF_fhz_dtheta_flock_cntr = 40;

flock_FFxy_friction = 100;
flock_FFtheta_friction = 50;

max_flock_vel_xy     =  1.0; % [m/s]
max_flock_acc_xy     =  0.5; % [m/s^2]
max_flock_vel_theta  =  1.0; % [rad/s]
max_flock_acc_theta  =  0.5; % [rad/s^2]



%% Sample times and initialization
Ts = 0.001; % Controller 
Tsflck = 0.01; % Flock Control
Tsp = 1;
Ts_rost = 0.01;

Tinit = 2; % Time for initialization

%% Platform Vel controller

% load platform_vel_cntr_GLL.mat
dxr_cntr = tf(0,1,Ts); %shapeit_data.C_tf_z;
dyr_cntr = tf(0,1,Ts); %shapeit_data.C_tf_z;
dthetar_cntr = tf(0,1,Ts); %shapeit_data.C_tf_z;

K_gain_dxdy_cntr = 200;
I_fhz_dxdy_cntr = 0.25;
LL_wz_fhz_dxdy_cntr = 20; % Use the same value for not having the LL
LL_wp_fhz_dxdy_cntr = 20;
LPF_fhz_dxdy_cntr = 100;
FFxy_mass = 0*80;

K_gain_dtheta_cntr = 150;
I_fhz_dtheta_cntr = 0.3;
LL_wz_fhz_dtheta_cntr = 20; % Use the same value for not having the LL
LL_wp_fhz_dtheta_cntr = 20;
LPF_fhz_dtheta_cntr = 100;
FFtheta_intia = 0;


%% Wheel Vel controller
% No integrators at the wheel velocity level!;
% load wheel_vel_cntr_GLL.mat
% dvarphi_wheel_cntr = shapeit_data.C_tf_z;

K_gain_dvarphi_cntr = 0.05;
I_fhz_dvarphi_cntr = 0;
LL_wz_fhz_dvarphi_cntr = 2; % Use the same value for not having the LL
LL_wp_fhz_dvarphi_cntr = 10;
LPF_fhz_dvarphi_cntr = 50;

%% Pivot Vel controller
% No integrators at the pivot level!;

K_gain_ddelta_cntr = 0.1;
I_fhz_ddelta_cntr = 0;
LL_wz_fhz_ddelta_cntr = 2; % Use the same value for not having the LL
LL_wp_fhz_ddelta_cntr = 2;
LPF_fhz_ddelta_cntr = 50;

%% Measurement filter?
LPF_fhz_dvarphi_meas = 50;
LPF_fhz_ddelta_meas = 50;

%% SysID signal
load q_Traj



Tslot = (size(V_glb,1)-1)*Ts;
% Tinit = 0*Tslot; %The duration of the initialization is same as that of one period of the ID signal, to avoid switchiong to an abrupt reference.

qrefdot_sys_ID_num_sing = 1*[V_glb(:,1) 1*V_glb(:,2) 1*W_glb_rz ];
qrefdot_num = repmat(qrefdot_sys_ID_num_sing,[3 1]);
timesim = (0:1:size(qrefdot_num,1)-1).'*Ts;
qrefdot = [timesim qrefdot_num];
Tfinal = timesim(end);
%% trajectory checks

max_ropod_traj_vel_x = max(qrefdot_sys_ID_num_sing(:,1));
max_ropod_traj_vel_y = max(qrefdot_sys_ID_num_sing(:,2));
if (max_ropod_traj_vel_x>max_ropod_vel_xy) || (max_ropod_traj_vel_y>max_ropod_vel_xy)
    disp('Error: Trajectory xy velocity exceeds limits')
end

max_ropod_traj_acc_x = max(gradient(qrefdot_sys_ID_num_sing(:,1))/Ts);
max_ropod_traj_acc_y = max(gradient(qrefdot_sys_ID_num_sing(:,1))/Ts);
if (max_ropod_traj_acc_x>max_ropod_acc_xy) || (max_ropod_traj_acc_y>max_ropod_acc_xy)
    disp('Error: Trajectory xy acceleration exceeds limits')
end

max_ropod_traj_vel_theta = max(qrefdot_sys_ID_num_sing(:,3));
if (max_ropod_traj_vel_theta>max_ropod_vel_theta) 
    disp('Error: Trajectory theta velocity exceeds limits')
end

max_ropod_traj_acc_theta = max(gradient(qrefdot_sys_ID_num_sing(:,3))/Ts);
if (max_ropod_traj_acc_theta>max_ropod_acc_theta) 
    disp('Error: Trajectory theta acceleration exceeds limits')
end


%% ZMPC Bumper Parameters
load('RoPod_parameters.mat');
m_robot_sim = m_robot;
b_rx_sim = b_rx;
b_cx_sim = b_cx;
k_cx_sim = k_cx;
v_ini_sim = 0.7;

Ts_MPC = 0.01;

% High pass filter Fy signal
fq = 0.0001;
zelta = 1000;
hpfC_Bumper=tf([1 0 0],[1 (2*zelta*fq) fq^2]);
hpfD_Bumper = c2d(hpfC_Bumper,Ts_MPC,'zoh');

%% ZMPC parameters

% Mode 0: velocoty control

% Mode 1: Operation_mode
% state 2: Collision with object
ZMPCx_Fd_min_sp = 60;  % minimal force to enable ZMPC Fx
ZMPCx_Fd_max_sp = 100; % maximal interaction force witch collision Fx
ZMPCx_Fr_min_sp = -60; % minimal motor power Fx
ZMPCx_Fr_max_sp = 60;  % maximal motor power Fx
% state 3: Force from the side
ZMPCy_Fd_min_sp = 30;   % minimal force to enable ZMPC Fy (+/- 2N bound)
ZMPCy_Fd_max_sp = 70;   % maximal interaction force for sideways forces Fy
ZMPCy_Fr_min_sp = -120; % minimal motor power Fy
ZMPCy_Fr_max_sp = 120;  % maximal motor power Fy

% Mode 4: Connect to cart
ZMPCx_Fd_min_cart_sp = 20;   % minimal interaction force with cart Fx
ZMPCx_Fd_max_cart_sp = 130;  % maximal interaction force witch cart Fx
ZMPCx_Fr_min_cart_sp = -120; % minimal motor power Fx
ZMPCx_Fr_max_cart_sp = 120;  % maximal motor power Fx


%% ZMPC parameter checks

if (ZMPCx_Fd_min_sp >= ZMPCx_Fd_max_sp || ZMPCy_Fd_min_sp >= ZMPCy_Fd_max_sp || ZMPCx_Fd_min_cart_sp >= ZMPCx_Fd_max_cart_sp)
   disp('Error: ZMPC parameters incorrect')
end

if (ZMPCx_Fd_max_sp < 0 || ZMPCy_Fd_max_sp < 0 || ZMPCx_Fd_max_cart_sp < 0)
   disp('Error: ZMPC parameters incorrect')
end

if (ZMPCx_Fr_min_sp > 0 || ZMPCy_Fr_min_sp > 0 || ZMPCx_Fr_min_cart_sp > 0)
   disp('Error: ZMPC parameters incorrect')
end

if (ZMPCx_Fr_max_sp < 0 || ZMPCy_Fr_max_sp < 0 || ZMPCx_Fr_max_cart_sp < 0)
   disp('Error: ZMPC parameters incorrect')
end









