clear all
close all

%%
file_path = '';
filename = 'figure_eight_v1_a05_yaw025.csv';

% resolution
dt = 0.1;

% total time (should be greater than 10)
tend = 40.0; 

% true: no lateral overload, similar to quadrotors
% false: zero pitch and roll
opt.zero_lateral_overload   = true;  

% regulate terminal quaternion or not
opt.reset_terminal_att      = false; 

% plot trajectory
make_plot = true;

%% define distorted time variable

cd(fileparts(mfilename('fullpath')));
import casadi.*
t = SX.sym('t');
% Time distorsion for a smooth start
k = 1;
t0 = 5; 
t1 = tend-5.0;
ts = log(1+exp(k*(t-t0))) - log(1+exp(k*(t-t1)));
zvars = 1e-15*sin(ts); % zero variable, somehow '0' does not work

%% Define algebraic equations of trajectories

px = 2.5 * cos(0.25 * ts) + 0.2;
py = 2.0 * sin(0.5 * ts);
pz = zvars + 1;
psi = 0.25 * ts; % heading

%%

data = traj_generator_casadi(t, px, py, pz, psi, dt, tend, opt);
writetable(data, [file_path, filename]);

%%
if make_plot
    try
        close(h_figure_ref_traj)
        close(h_figure_ref_traj_3d)
    end
    
    time = data.t;
    acc = sqrt(data.a_lin_x.^2 + data.a_lin_y.^2 + data.a_lin_z.^2);
    vel = sqrt(data.v_x.^2 + data.v_y.^2 + data.v_z.^2);
    rate = sqrt(data.w_x.^2 + data.w_y.^2 + data.w_z.^2);


    h_figure_ref_traj = figure('Name', 'Reference Traj');
    subplot(3,1,1)
    plot(time, vel); ylabel('Velocity [m/s]')
    subplot(3,1,2);
    plot(time, acc); ylabel('acceleration [m/s^2]')
    subplot(3,1,3);
    plot(time, rate); ylabel('angular rate [rad/s]')

    h_figure_ref_traj_3d = figure('Name', 'Reference Traj 3d');
    plot3(data.p_x, data.p_y, data.p_z); grid on; axis equal;
    xlabel('x'); ylabel('y'); zlabel('z');
end
