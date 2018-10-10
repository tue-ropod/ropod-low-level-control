
run /home/cesar/Documents/ROPOD_LINUX/Matlabdocs/Global_Libraries/ropod_parameters/ropod_parameters

ropod_kinmodel_param.r_w    = 0.5*wheel_physical_parameters.diameter.value*ones(2,Nwheels);
ropod_kinmodel_param.r_w    = ropod_kinmodel_param.r_w(:).';
ropod_kinmodel_param.s_w    = wheel_physical_parameters.caster_offset.value; % 
ropod_kinmodel_param.d_w    = wheel_physical_parameters.separation.value;
ropod_kinmodel_param.C_SW   = ropod_physical_parameters.wheel_distribution_4SW.value(:).';


ropod_dynmodel_param.Mr     = ropod_physical_parameters.ropod_mass.value;
ropod_dynmodel_param.Ir     = ropod_physical_parameters.ropod_base_inertia.value;
ropod_dynmodel_param.Mwma   = wheel_physical_parameters.middleaxis_mass.value;  
ropod_dynmodel_param.Iwma   = wheel_physical_parameters.casteraxis_inertia_zaxis.value;
ropod_dynmodel_param.Mwca   = wheel_physical_parameters.casteraxis_mass.value;   % The fact that the mass increases with caster offset is not taken into account.
ropod_dynmodel_param.Iwca   = wheel_physical_parameters.casteraxis_inertia_zaxis.value;
ropod_dynmodel_param.Mw     = wheel_physical_parameters.singlewheel_mass.value;     % Mass of an individual wheel
ropod_dynmodel_param.Iwz    = wheel_physical_parameters.singlewheel_inertia_zaxis.value;
ropod_dynmodel_param.Iwp    = wheel_physical_parameters.singlewheel_inertia_rolling.value;
ropod_dynmodel_param.platCoG = ropod_physical_parameters.ropod_cog.value;


ropod_dynmodel_param.Ddelta = 0.01*[1 1 1 1]*wheel_physical_parameters.smartwheel_viscous_friction_pivot.value;
ropod_dynmodel_param.Dvarphi = 0.01*[1 1 1 1 1 1 1 1]*wheel_physical_parameters.singlewheel_viscous_friction_rolling.value;
ropod_dynmodel_param.Fcdelta = 0.001*[1 1 1 1]*wheel_physical_parameters.smartwheel_coloumb_friction_pivot.value;
ropod_dynmodel_param.Fcvarphi = 0.001*[1 1 1 1 1 1 1 1]*wheel_physical_parameters.singlewheel_coloumb_friction_rolling.value;





ropod_kinmodel_pv = struct2array(ropod_kinmodel_param);
ropod_dynmodel_pv = struct2array(ropod_dynmodel_param);


