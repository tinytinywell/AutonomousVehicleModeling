function [sys, x0, str, ts] = CLFCBF(t, x, u, flag, t_hw, a_max, a_min, fixed_safety_distance)
%% default 
switch flag
    case 0
        [sys, x0, str, ts] = mdlInitializeSizes;
      
    case 2
        sys = mdlUpdates(t, x, u);
    
    case 3
        sys = mdlOutputs(t, x, u, t_hw, a_max, a_min, fixed_safety_distance);
    
    case {1, 4, 9}
        sys = [];
    
    otherwise
        error(['unhandled flag = ', num2str(flag)]);
end

function [sys, x0, str, ts] = mdlInitializeSizes
%% S-Function initialization
% initial the system by a struct "size"
sizes = simsizes;
% 0 continuous state
sizes.NumContStates  = 0;
% 3 discrete states: xr, vh, vr
sizes.NumDiscStates  = 3;
% 1 S-Function output: acceleration command to the ego vehicle
% the sizes.NumOutputs must be consistent with the output number in the
% "mdlOutputs" function
sizes.NumOutputs     = 1;
% 3 S-Function inputs: relative distance, relative speed, ego speed
sizes.NumInputs      = 3;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
% intial value of x
x0 = [0; 20; 100];
str = [];
ts = [0.02 0];

function sys = mdlUpdates(t, x, u)
%% S-Function system state update
sys = [];

function sys = mdlOutputs(t, x, u, t_hw, a_max, a_min, fixed_safety_distance)
%% output of S-Function 
% here define the CLF/CBF and optimize the quadratic programming.
%% states
% relative distance
x_r = u(1);
% relative speed
v_r = u(2);
% ego vehicle speed
v_h = u(3);
% xr_des = fixed_safety_distance + vh*t_hw - 0.5*vr^2/amin;
%% control lyapunov function
% set speed of ego vehicle, when there is no leading vehicle or relative
% distance is greater than safety following distance
v_set = 24;
clf = (v_h - v_set)^2;
LfV = 0;
LgV = 2 * (v_h - v_set);
%% control barrier function
cbf = x_r - t_hw * v_h + 0.5 * v_r^2 / a_min - fixed_safety_distance;
Lfh = v_r;
Lgh = -t_hw - v_r / a_min;
%% cost function
% J = (u - u_ref)^T * H * (u - u_ref) + rho * ep^2, ep is slack variable
% H: weight of control input part in cost function
H = 1;
% rho: weight of slack variable part in cost function
rho = 2e-2;
% cost: J = [u ep] * [H 0; 0 rho] * [u ep]^T + [-u_ref * H 0] * [u ep]^T
% u_ref: reference of control input, the source can be another controller,
% here I set it to 0
u_ref = 0;
f = [-u_ref * H 0];
H = [H 0; 0 rho];
%% constraints
lambda = 1.5;
gamma = 0.5;
% constraints of quadratic programming problem:
% LfV + LgV * u + lambda * V <= ep -> LgV * u <= ep - LfV + lambda * V
% Lfh + Lgh * u + gamma * h >= 0 -> -Lgh * u <= gamma * h + Lfh
% Omega * U <= T, with U = [u; ep] 
Omega = [LgV -1; -Lgh 0];
T = [-lambda * clf - LfV; gamma * cbf + Lfh];
% optimization
options = optimset('Algorithm', 'active-set');
% Inequality constraint
A_ine = Omega;
b_ine = T;
% No equality constraint.
Aep = [];
Bep=[];
% lower and upper bound
ep_min = 0;
ep_max = 1000;
lower_bound = [a_min; ep_min]; 
upper_bound=[a_max; ep_max];
ini = zeros(2, 1);
[acc, ~] = quadprog(H, f', A_ine, b_ine, Aep, Bep, lower_bound, upper_bound, ini, options);
% just apply the first control command to the vehicle model
U_1 = acc(1);
% The quadprog still outputs when the programming is infeasible, so U
% should be limited so that it won't be out of the boundary
if U_1 > a_max
    U_1 = a_max;
elseif U_1 < a_min
    U_1 = a_min;
end
% output of S-Function, it must be consistent with the "size.NumOutputs" 
% in the "mdlInitializeSizes" function
sys = U_1;
