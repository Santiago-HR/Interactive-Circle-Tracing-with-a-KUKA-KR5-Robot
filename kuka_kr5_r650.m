% KUKA KR5 R650
% Brynn Caldwell
% Santiago Henao Rojas
% Robotic Systems

clear; clc; close all;

%% ---------------- Robot model (your DH, meters) ------------------------
mm = 1e-3;

L(1) = Link('d', 335*mm, 'a',  75*mm, 'alpha', -pi/2, 'offset', 0, 'standard');
L(2) = Link('d',   0*mm, 'a', 270*mm, 'alpha',      0, 'offset', 0, 'standard');
L(3) = Link('d',   0*mm, 'a',  90*mm, 'alpha', +pi/2, 'offset', 0, 'standard');
L(4) = Link('d',-295*mm, 'a',   0*mm, 'alpha', -pi/2, 'offset', 0, 'standard');
L(5) = Link('d',   0*mm, 'a',   0*mm, 'alpha', +pi/2, 'offset', 0, 'standard');
L(6) = Link('d', -80*mm, 'a',   0*mm, 'alpha',     pi, 'offset', 0, 'standard');

BOT = SerialLink(L, 'name', 'KUKA_KR5_R650');
BOT.base = transl(0,0,0);
BOT.tool = transl(0,0,0); % TCP is a point (only tip touches)

% Joint limits (datasheet); used to clamp and seed IK
BOT.qlim = deg2rad([ ...
    -170  170;  % J1
    -190   45;  % J2
    -119  165;  % J3
    -190  190;  % J4
    -120  120;  % J5
    -358  358   % J6
]);

%% ---------------- Circle on a sphere (general case) --------------------
R     = 0.20;          % sphere radius (m)
alpha = deg2rad(30);   % circle half-angle (tilt on sphere)
d_tcp = 0.65;          % constant TCP distance from base (m)

aimdir = [0.60; 0.20; 0.35];
aimdir = aimdir / norm(aimdir); % sphere center direction

N = 360; % waypoints along the circle

% Generate target points p(:,k) and inward normals zdes(:,k)
[center, p, zdes, xtan, Tdes] = build_targets(R, alpha, d_tcp, aimdir, N);

%% ---------------- Scene / visualization --------------------------------
figure('Color',[0.1 0.1 0.1]); hold on; axis equal; grid on;

[xs,ys,zs] = sphere(64);
surf(R*xs + center(1), R*ys + center(2), R*zs + center(3), ...
    'FaceAlpha',0.22,'EdgeColor','none','FaceColor',[0 0.6 1]);

plot3(p(1,:),p(2,:),p(3,:),'r','LineWidth',2); % the red circle to trace

xlabel('X'); ylabel('Y'); zlabel('Z');
title('KR5: TCP tip locked to circle on sphere');

BOT.plot(zeros(1,6), ...
    'workspace',[-0.9 1.2 -0.9 1.2 -0.1 1.3], ...
    'scale',0.7,'noname');

legend({'Sphere','Target circle','Robot'}, ...
    'TextColor','w','Location','northoutside');

%% ---------------- Resolved-rate servo settings -------------------------
% Pass A: fast, smooth approach to the circle
COARSE.POS_TOL   = 2.0e-4;  % stop when position error < 0.20 mm
COARSE.TILT_TOL  = 0.15;    % and tilt error < 0.15 deg (Z toward inward normal)
COARSE.LAMBDA    = 0.08;    % DLS damping
COARSE.STEP0     = 0.16;    % initial integration step
COARSE.STEP_MIN  = 0.04;    % min step during backtracking
COARSE.QDOT_MAX  = 0.35;    % per-iteration speed cap (rad)
COARSE.K_NULL    = 0.06;    % nullspace smoothing toward previous pose
COARSE.MAX_INNER = 80;      % micro-iterations per waypoint

plot_every = 3;

% Pass B: tighten each waypoint
REF.POS_TOL   = 5.0e-5;  % 0.05 mm
REF.TILT_TOL  = 0.06;    % 0.06 deg
REF.LAMBDA    = 0.08;
REF.STEP0     = 0.12;
REF.STEP_MIN  = 0.02;
REF.QDOT_MAX  = 0.25;
REF.K_NULL    = 0.06;
REF.MAX_INNER = 300;

%% ---------------- Good first pose using ikcon --------------------------
Td0 = SE3(Tdes(:,:,1));                 % desired SE3 for first point (roll free)
q0  = multistart_ikcon(BOT, Td0, 50);    % robust seed within limits
q0  = clamp(q0, BOT.qlim);

% Tighten first waypoint with coarse tolerances (stabilizes the start)
q0 = lock_to_point(BOT, q0, p(:,1), zdes(:,1), q0, ...
    COARSE.POS_TOL, COARSE.TILT_TOL, COARSE.LAMBDA, ...
    COARSE.STEP0, COARSE.STEP_MIN, ...
    COARSE.QDOT_MAX, COARSE.K_NULL, COARSE.MAX_INNER);

% TCP marker (yellow dot = actual contact point)
Tk  = BOT.fkine(q0).T;
pt  = Tk(1:3,4);
hTip = plot3(pt(1),pt(2),pt(3),'yo','MarkerSize',6,'MarkerFaceColor','y');

%% ---------------- PASS A: coarse (fast & smooth) -----------------------
q_coarse = zeros(N,6);
q_coarse(1,:) = q0;

for k = 2:N
    qref = q_coarse(k-1,:);
    qk = lock_to_point(BOT, qref, p(:,k), zdes(:,k), qref, ...
        COARSE.POS_TOL, COARSE.TILT_TOL, COARSE.LAMBDA, ...
        COARSE.STEP0, COARSE.STEP_MIN, ...
        COARSE.QDOT_MAX, COARSE.K_NULL, COARSE.MAX_INNER);

    q_coarse(k,:) = qk;

    % lightweight live preview to keep it responsive
    if mod(k,plot_every) == 0 || k == N
        Tk = BOT.fkine(qk).T;
        pt = Tk(1:3,4);

        set(hTip,'XData',pt(1),'YData',pt(2),'ZData',pt(3));
        BOT.plot(qk, ...
            'workspace',[-0.9 1.2 -0.9 1.2 -0.1 1.3], ...
            'trail',{'r',2},'scale',0.7,'noname');

        drawnow;
    end
end

%% ---------------- PASS B: refine (off-screen, tight) -------------------
q_ref = q_coarse;

for k = 1:N
    qseed = q_ref(k,:);
    q_ref(k,:) = lock_to_point(BOT, qseed, p(:,k), zdes(:,k), qseed, ...
        REF.POS_TOL, REF.TILT_TOL, REF.LAMBDA, ...
        REF.STEP0, REF.STEP_MIN, ...
        REF.QDOT_MAX, REF.K_NULL, REF.MAX_INNER);
end

%% ---------------- Final animation of refined path ----------------------
for k = 1:N
    Tk = BOT.fkine(q_ref(k,:)).T;
    pt = Tk(1:3,4);

    set(hTip,'XData',pt(1),'YData',pt(2),'ZData',pt(3));
    BOT.plot(q_ref(k,:), ...
        'workspace',[-0.9 1.2 -0.9 1.2 -0.1 1.3], ...
        'trail',{'r',2},'scale',0.7,'noname');

    drawnow;
end

ptcp = zeros(3,N);
ztcp = zeros(3,N);

for k = 1:N
    Tk = BOT.fkine(q_ref(k,:)).T;
    ptcp(:,k) = Tk(1:3,4); % final TCP positions
    ztcp(:,k) = Tk(1:3,3); % final TCP +Z axes
end

plot3(ptcp(1,:),ptcp(2,:),ptcp(3,:),'c--','LineWidth',1.4);

pos_err = vecnorm(ptcp - p, 2, 1);                         % to red circle
rad_err = abs(vecnorm(ptcp - center(:), 2, 1) - R);        % to sphere surface
zin     = -(ptcp - center(:)) ./ R;                        % inward normals
tilt    = acos(max(-1,min(1,sum(ztcp.*zin,1)))) * 180/pi;

fprintf('Refined path:\n');
fprintf(' pos-to-trace error: mean %.3f mm | max %.3f mm\n', ...
    1e3*mean(pos_err), 1e3*max(pos_err));
fprintf(' sphere radius err : mean %.3f mm | max %.3f mm\n', ...
    1e3*mean(rad_err), 1e3*max(rad_err));
fprintf(' normal alignment : mean %.3f° | max %.3f°\n', ...
    mean(tilt), max(tilt));

view(135,25);

%% ======================= helper functions ==============================
function q = multistart_ikcon(BOT, Td, tries)
% Try several random seeds within limits; keep the best ikcon result
best    = [];
besterr = inf;

for s = 1:tries
    qseed = rand_in_limits(BOT.qlim);
    try
        [qk, err] = BOT.ikcon(Td, qseed);
    catch
        qk  = [];
        err = inf;
    end

    if ~isempty(qk) && all(isfinite(qk)) && err < besterr
        best    = qk(:).';
        besterr = err;
    end
end

if isempty(best)
    error('ikcon failed to find a feasible first pose');
end

q = best;
end

function qrow = clamp(qrow, qlim)
% Clamp a 1x6 row to joint limits
qrow = min(max(qrow, qlim(:,1)'), qlim(:,2)');
end

function q = rand_in_limits(qlim)
lo = qlim(:,1)';
hi = qlim(:,2)';
q  = lo + rand(1,6).*(hi - lo);
end

% DLS tilt-only servo (roll free) with nullspace smoothing and backtracking
function qnext = lock_to_point(BOT, qstart, p_target, z_inward, q_ref, ...
    pos_tol, tilt_tol, lambda, step0, step_min, ...
    qdot_max, k_null, itmax)

% Weights bias the task to align TCP +Z with inward normal slightly before position
w_pos  = 1.0;
w_tilt = 2.0;

qk = qstart(:).';

unit = @(v) v / max(1e-12, norm(v));

make_uv = @(z) ( ...
    (abs(dot(z,[1;0;0])) < 0.9) .* ...
    [unit(cross(z,[1;0;0])), ...
     unit(cross(z, unit(cross(z,[1;0;0]))))] + ...
    (abs(dot(z,[1;0;0])) >= 0.9) .* ...
    [unit(cross(z,[0;1;0])), ...
     unit(cross(z, unit(cross(z,[0;1;0]))))] );

[pos_e, tilt_e] = cur_err(BOT, qk, p_target, z_inward);
step = step0;

for t = 1:itmax %#ok<NASGU>
    Tnow = BOT.fkine(qk).T;
    pnow = Tnow(1:3,4);
    znow = Tnow(1:3,3);

    e_pos  = p_target - pnow;         % 3x1 position error
    e_axis = cross(znow, z_inward);   % axis to rotate znow→z_inward

    UV     = make_uv(znow);           % 3x2 basis znow⟂
    e_tilt = UV.' * e_axis;           % 2x1 tilt error (no roll)

    e = [w_pos*e_pos; w_tilt*e_tilt]; % weighted task (5x1)

    J     = BOT.jacob0(qk);
    Jtilt = UV.' * J(4:6,:);
    Jact  = [w_pos*J(1:3,:); w_tilt*Jtilt];

    JJt   = Jact*Jact.';
    Jpinv = Jact.' * ((JJt + (lambda^2)*eye(5)) \ eye(5));

    qdot = Jpinv * e;

    % keep motion smooth by pulling toward previous pose in nullspace
    Nmat = eye(6) - Jpinv*Jact;
    qdot = qdot + k_null * (Nmat * (q_ref(:) - qk(:)));

    % speed limit + backtracking step control
    qdot = qdot * min(1, qdot_max / max(1e-12, norm(qdot)));

    improved = false;
    for tries = 1:6
        qtrial = clamp(qk + step*qdot.', BOT.qlim);
        [pos_e_new, tilt_e_new] = cur_err(BOT, qtrial, p_target, z_inward);

        if (pos_e_new <= pos_e && tilt_e_new <= tilt_e) || step <= step_min
            qk     = qtrial;
            pos_e  = pos_e_new;
            tilt_e = tilt_e_new;
            improved = true;
            break;
        else
            step = 0.5 * step;
        end
    end

    if ~improved
        qk = qtrial; %#ok<NASGU>
    end

    if pos_e <= pos_tol && tilt_e <= tilt_tol
        break;
    end
end

qnext = qk;
end

function [pos_e, tilt_e] = cur_err(BOT, q, p_target, z_inward)
% Position error (m) and tilt error (deg) for TCP +Z vs inward normal
T    = BOT.fkine(q).T;
pnow = T(1:3,4);
znow = T(1:3,3);

pos_e  = norm(p_target - pnow);
tilt_e = acos(max(-1,min(1, dot(znow, z_inward)))) * 180/pi;
end

function [center, p, zdes, xtan, Tdes] = build_targets(R, alpha, d, aim_dir, N)
% Build the small circle on the sphere and the inward normals at each point
aim_dir = aim_dir / norm(aim_dir);

under = d^2 - (R*sin(alpha))^2;
if under <= 0
    alpha = asin(0.95*d/R);
    under = d^2 - (R*sin(alpha))^2;
end

c = R*cos(alpha) + sqrt(under);
center = (c*aim_dir).';

% local basis on sphere for circle generation
n = -center(:) / norm(center);

w = [0;0;1];
if abs(dot(n,w)) > 0.97
    w = [0;1;0];
end

e1 = cross(n,w); e1 = e1 / norm(e1);
e2 = cross(n,e1);

theta = linspace(0,2*pi,N);
p    = zeros(3,N);
zdes = zeros(3,N);
xtan = zeros(3,N);

for k = 1:N
    th = theta(k);

    p(:,k) = center' + R*( cos(alpha)*n + sin(alpha) * ( ...
        e1*cos(th) + e2*sin(th) ) );

    zdes(:,k) = - (p(:,k) - center') / R; % inward normal at p_k

    dpdth = R*sin(alpha) * (-e1*sin(th) + e2*cos(th));
    xtan(:,k) = dpdth / max(1e-9, norm(dpdth)); % tangent (used for frame viz)
end

% frames (for optional visualization; roll not enforced)
Tdes = repmat(eye(4),1,1,N);
for k = 1:N
    yk = cross(zdes(:,k), xtan(:,k)); yk = yk / norm(yk);
    xk = cross(yk, zdes(:,k));

    Tdes(:,:,k) = [xk, yk, zdes(:,k), p(:,k); 0 0 0 1];
end
end
