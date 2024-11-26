function data = traj_generator_casadi(t, px, py, pz, psi, dt, tend, opt)
%% define traj as function of x y z psi of t

import casadi.*

disp('Defining symbolics')

vx = jacobian(px, t);
vy = jacobian(py, t);
vz = jacobian(pz, t);
dpsi = jacobian(psi, t);

ax = jacobian(vx, t);
ay = jacobian(vy, t);
az = jacobian(vz, t);
ddpsi = jacobian(dpsi, t);

jx = jacobian(ax, t);
jy = jacobian(ay, t);
jz = jacobian(az, t);

%%
a = [ax; ay; az];
g = [0; 0; -9.81];

if opt.zero_lateral_overload
    e3 = a - g;
else
    e3 = -g;
end

T = norm(e3);
dT = jacobian(T, t);
e3 = e3 / norm(e3);
e1 = [cos(psi); sin(psi); 0];
e2 = cross(e3, e1);
e2 = e2 / norm(e2);
e1 = cross(e2, e3);

R = [e1, e2, e3];

j = [jx; jy; jz];
h = (j - dT*e3) / T;

wx = -dot(h, e2);
wy = dot(h, e1);
wz = dot([0; 0; 1], e3) * dpsi;

alphax = jacobian(wx, t);
alphay = jacobian(wy, t);
alphaz = jacobian(wz, t);

%% substitution.
time = 0: dt: tend; time = time';

% t = time';
fpx = Function('fun', {t}, {px});
fpy = Function('fun', {t}, {py});
fpz = Function('fun', {t}, {pz});

fvx = Function('fun', {t}, {vx});
fvy = Function('fun', {t}, {vy});
fvz = Function('fun', {t}, {vz});

fax = Function('fun', {t}, {ax});
fay = Function('fun', {t}, {ay});
faz = Function('fun', {t}, {az});

fR = Function('fun', {t}, {R});

fwx = Function('fun', {t}, {wx});
fwy = Function('fun', {t}, {wy});
fwz = Function('fun', {t}, {wz});

falphax = Function('fun', {t}, {alphax});
falphay = Function('fun', {t}, {alphay});
falphaz = Function('fun', {t}, {alphaz});

p_x = full(fpx(time));
p_y = full(fpy(time));
p_z = full(fpz(time));

v_x = full(fvx(time));
v_y = full(fvy(time));
v_z = full(fvz(time));

a_lin_x = full(fax(time));
a_lin_y = full(fay(time));
a_lin_z = full(faz(time));

w_x = full(fwx(time));
w_y = full(fwy(time));
w_z = full(fwz(time));

a_rot_x = full(falphax(time));
a_rot_y = full(falphay(time));
a_rot_z = full(falphaz(time));

N = length(time);
q_w = zeros(N, 1);
q_x = q_w; q_y = q_x; q_z = q_y;
for i = 1:N
%     rotm = double(subs(R, t(i)));
    rotm = full(fR(time(i)));
    quat = rotm2quat(rotm);
    q_w(i) = quat(1);
    q_x(i) = quat(2);
    q_y(i) = quat(3);
    q_z(i) = quat(4);
    if mod(i, 10) == 0
        fprintf("Generating quaternions: %.2f %% \n", i * 100/N);
    end
end

disp('Generating table')

% set the terminal velocities and accelerations to zero
v_x(end) = 0.0;
v_y(end) = 0.0;
v_z(end) = 0.0;
w_x(end) = 0.0;
w_y(end) = 0.0;
w_z(end) = 0.0;
a_lin_x(end) = 0.0;
a_lin_y(end) = 0.0;
a_lin_z(end) = 0.0;

if opt.reset_terminal_att
    q_w(end) = 1.0;
    q_x(end) = 0.0;
    q_y(end) = 0.0;
    q_z(end) = 0.0;
end

t = time;
data = table(t, p_x, p_y, p_z, q_w, q_x, q_y, q_z, v_x, v_y, v_z, ...
    w_x, w_y, w_z, a_lin_x, a_lin_y, a_lin_z, a_rot_x, a_rot_y, a_rot_z);
end



