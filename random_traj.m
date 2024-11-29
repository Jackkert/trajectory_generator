clear all
close all

%% Settings
file_path = 'trajectories/';

% Number of trajectories to generate
N = 20000;

% Resolution
dt = 0.1;

% Total time (should be greater than 10)
tend = 40.0; 

% True: no lateral overload, similar to quadrotors
% False: zero pitch and roll
opt.zero_lateral_overload   = true;  

% Regulate terminal quaternion or not
opt.reset_terminal_att      = false; 

% Plot trajectory
make_plot = false;

% Trajectory types
trajectory_types = {'figure8', 'ellipse'};
N_per_type = N / length(trajectory_types); % Equal number of each type

%% Create trajectories
cd(fileparts(mfilename('fullpath')));
import casadi.*

for i = 1:N
    % Alternate trajectory types
    traj_type = trajectory_types{mod(i-1, length(trajectory_types)) + 1};
    
    % Time variable
    t = SX.sym('t');
    
    % Time distortion for smooth start
    k = 1;
    t0 = 5;
    t1 = tend - 5.0;
    ts = log(1 + exp(k * (t - t0))) - log(1 + exp(k * (t - t1)));
    zvars = 1e-15 * sin(ts); % Zero variable

    %% Define algebraic equations of trajectories
    switch traj_type
        case 'figure8'
            % Parameters for figure-eight
            scale_x = 2 + rand(); % Randomize size
            scale_y = 1.5 + 0.5 * rand();
            yaw_rate = 0.2 + 0.1 * rand();
            time_scaling = (rand() + 1);
            px = scale_x * cos(yaw_rate * ts * time_scaling) + 0.2;
            py = scale_y * sin(2 * yaw_rate * ts * time_scaling);
            pz = zvars + 1; % Constant z-coordinate
            psi = yaw_rate * ts; % Heading
        
            % Randomize orientation in the xy-plane
            rotation_angle = 2 * pi * rand(); % Random rotation angle
            R = [cos(rotation_angle), -sin(rotation_angle); 
                 sin(rotation_angle), cos(rotation_angle)];
            rotated_coords = R * [px; py];
            px = rotated_coords(1, :);
            py = rotated_coords(2, :);

        case 'ellipse'
            % Parameters for ellipse
            semi_major = 2.5 + rand(); % Semi-major axis
            semi_minor = 1.0 + rand(); % Semi-minor axis
            rotation_angle = pi * rand(); % Random orientation
            R = [cos(rotation_angle), -sin(rotation_angle); sin(rotation_angle), cos(rotation_angle)];
            time_scaling = (rand() + 1)*2;
            xy = R * [semi_major * cos(0.2 * ts * time_scaling); semi_minor * sin(0.2 * ts * time_scaling)];
            px = xy(1, :);
            py = xy(2, :);
            pz = zvars + 1;
            psi = 0.2 * ts; % Heading
    end

    %% Generate trajectory
    data = traj_generator_casadi(t, px, py, pz, psi, dt, tend, opt);

    % Calculate maximum velocity, acceleration, and body rate
    max_velocity = max(sqrt(data.v_x.^2 + data.v_y.^2 + data.v_z.^2));
    max_acceleration = max(sqrt(data.a_lin_x.^2 + data.a_lin_y.^2 + data.a_lin_z.^2));
    max_body_rate = max(sqrt(data.w_x.^2 + data.w_y.^2 + data.w_z.^2));

    % Save trajectory to file
    filename = sprintf('%s_V%.2f_A%.2f_yaw%.2f_%05d.csv', ...
        traj_type, max_velocity, max_acceleration, max_body_rate, i);
    writetable(data, [file_path, filename]);

    %% Plot trajectory
    if make_plot
        time = data.t;
        acc = sqrt(data.a_lin_x.^2 + data.a_lin_y.^2 + data.a_lin_z.^2);
        vel = sqrt(data.v_x.^2 + data.v_y.^2 + data.v_z.^2);
        rate = sqrt(data.w_x.^2 + data.w_y.^2 + data.w_z.^2);

        h_figure_ref_traj = figure('Name', ['Reference Traj ', traj_type, ' #', num2str(i)]);
        subplot(3, 1, 1);
        plot(time, vel); ylabel('Velocity [m/s]');
        subplot(3, 1, 2);
        plot(time, acc); ylabel('Acceleration [m/s^2]');
        subplot(3, 1, 3);
        plot(time, rate); ylabel('Angular Rate [rad/s]');

        h_figure_ref_traj_3d = figure('Name', ['Reference Traj 3D ', traj_type, ' #', num2str(i)]);
        plot3(data.p_x, data.p_y, data.p_z); grid on; axis equal;
        xlabel('x'); ylabel('y'); zlabel('z');
        title(['3D Trajectory ', traj_type, ' #', num2str(i)]);
    end
end
