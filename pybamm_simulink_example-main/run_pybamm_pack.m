
clear all; clear classes; clear functions;
clc; tic;

% Python connection
try
    pe = pyenv; 
    if pe.Status == "NotLoaded"
        pe = pyenv( ...
            Version="C:\Users\Seunghun Jang\miniconda3\envs\pybamm-env\python.exe" ...
        );
    end
catch
    pe = pyenv(Version="C:\Users\Seunghun Jang\miniconda3\envs\pybamm-env\python.exe");
end

mod_dir = "C:\Users\Seunghun Jang\OneDrive\1. 진행중\[공동연구] Battery degradation 기반 에너지 최적화\pybamm_simulink_example-main\pybamm_simulink_example-main";
pyexe   = "C:\Users\Seunghun Jang\miniconda3\envs\pybamm-env\python.exe";
disp("======= ⚙️  PyBaMM/CasADi setup... ======");

% pybamm_setup.py implementation → CasADi objects regeneration
cmd = sprintf('"%s" pybamm_setup.py', pyexe);
[status, out] = system(cmd);
disp(out);

if status ~= 0
    error("pybamm_setup.py implementation failed (status=%d).", status);
end

disp("PyBaMM ( integrator/variables/y0_* ) regeneration completed");
addpath('C:\casadi-3.6.7-windows64-matlab2018b');

% Regenerate the casadi objects
delete(gcp('nocreate'))
disp([datestr(now, 'HH:MM:SS'),'  Model set up']);

% The simulink model name
mdl = 'pack_2s1p';

% Read in settings and calculate some derived parameters
param = table2struct(readtable('settings.csv'));
param.battery_volume = param.battery_height*pi*(param.battery_radius)^2;
param.battery_mass = param.battery_volume*param.rho;
param.cooled_surface_area = param.battery_height*2*pi*param.battery_radius;
param.t_init = param.reference_temperature;
param.I_app = param.I_mag * param.I_sign_init;

% set drive cycle
param.drive_cycle = 'sflip';
param.dt = 0.5;

% initial guesses for ocv and r_int
param.ocv_init = 4.0;
param.r_int_init = 0.1;

% Run the simulation
% N_sum = 1;
% in(1:N_sum) = Simulink.SimulationInput(mdl);
% disp([datestr(now, 'HH:MM:SS'),'  Start simulink']);
% parsimOut = sim(in, 'ShowProgress' ,true);
% disp([datestr(now, 'HH:MM:SS'),'  Finish simulink']);
% toc;

N_sum = 10;   
Tf = 300000;

if ~bdIsLoaded(mdl), load_system(mdl); end
set_param(mdl,'FastRestart','off');
set_param(mdl,'StopTime', num2str(Tf));   
set_param(mdl,'SimulationCommand','update');

open_system(mdl);
simOut = sim(mdl);

toc;