function [sys, x0, str, ts] = MPC_quadprog(t, x, u, flag, Np, t_hw, a_max, a_min, fixed_safety_distance)
%% default 
switch flag
    case 0
        [sys, x0, str, ts] = mdlInitializeSizes(Np);
    
    case 2
        sys = mdlUpdates(t, x, u);
    
    case 3
        sys = mdlOutputs(t, x, u, Np, t_hw, a_max, a_min, fixed_safety_distance);
    
    case {1,4,9}
        sys = [];
    
    otherwise
        error(['unhandled flag = ', num2str(flag)]);
end
end

function [sys, x0, str, ts] = mdlInitializeSizes(Np)
%% S-Function initialization
% initial the system by a struct "size"
sizes = simsizes;
% 0 continuous state
sizes.NumContStates  = 0;
% 3 discrete states: xr(k), vh(k), vr(k)
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
% intial value of x, instead of using x here, I use x_k which is defined in 
% the following
x0 = [0; 0; 0];
% Control input of system: U_1, it is the 1st component of control input
% sequence U
global U_1 acc x_k;
x_k = [0; 0; 0];
U_1 = 0;
acc = zeros(Np + 1, 1);
str = [];
ts = [0.5 0];
end

function sys = mdlUpdates(t, x, u)
%% S-Function system state update
sys = x;
end

function sys = mdlOutputs(t, x, u, Np, t_hw, a_max, a_min, fixed_safety_distance)
%% output of S-Function 
% here define the system matrices, estimate the state and optimize the
% quadratic programming.
global U_1 acc x_k;
% sample time
Ts = 0.5;
%% discrete state-space model: A, B, C, S, Z
% state space model: x(k + 1|k) = A * x(k) + B * u(k) + S * d(k)
% x(k) = [xr(k); vh(k); vr(k)]
% xr: relative distance, vr: relative speed, vh: ego vehicle speed
% system matrix
A = [1 Ts 0;
     0 1  0;
     0 0  1];
% input matrix
B = [-0.5 * Ts^2; -Ts; Ts];
% disturbance matrix
S = [0.5 * Ts^2; Ts; 0];
% y(k)=[e(k); vr(k)]
% spacing error: e(k) = xr(k) - xr,des(k)
% desired following distance:xr,des(k) = xr0 + vh(k) * t_hw
% xr0: fixed safety distance
% y(k) = C * x(k) - Z
% observation matrix  
% t_hw: desired headway time
C = [1 0  -t_hw;
     0 1   0   ];
Z = [fixed_safety_distance; 0];
%% output error vector: A_bar, B_bar, S_bar, Z_bar
% Y = A_bar * x(k) + B_bar * U + S_bar * D - Z_bar
% Y = [y(k + 1|k) - yr(k + 1|k), ..., y(k + Np|k) - yr(k + Np|k)]
% yr(k + i|k) = [0; 0]
% note: dim_A_1: size(A, 1); dim_A_2: size(A, 2)
% A_bar = [C * A; ...; C * A^Np]
% size: (Np * dim_C_1) x dim_A_2
% Np: prediction horizon
A_bar = cell(Np, 1);
A_bar{1, 1} = C * A;
for i = 2 : Np
    A_bar{i, 1} = A_bar{i - 1, 1} * A;
end
A_bar = cell2mat(A_bar);
% B_bar = [C * B                0                   0                      ... 0;
%          C * A * B            C * B               0                      ... 0;
%          ...
%          C * A^(Np - 1) * B   C*A^(Np - 2) * B    C * A^(Np - 3) * B     ... C * B]
% size: (Np * dim_C_1) x (Np * dim_B_2)
n_C = size(C, 1);
B_bar = cell(Np, Np);
for i = 1 : Np
    for j = 1 : Np
        if j > i
            % upper triangular values are 0
            B_bar{i, j} = zeros(n_C, 1);
        else
            % only assigne lower triangular values
            B_bar{i, j} = C * A^(i - j) * B;
         end
    end
end
B_bar = cell2mat(B_bar);
% S_bar = [C * S                0                     0                      ... 0;
%          C * A * S            C * S                 0                      ... 0;
%          ...
%          C * A^(Np - 1) * S   C * A^(Np - 2) * S    C * A^(Np - 3) * S     ... C * S]
% size: (Np * dim_C_1) x (Np * dim_S_2)
S_bar = cell(Np, Np);
for i = 1 : Np
    for j = 1 : Np
        if j > i
            % upper triangular values are 0
            S_bar{i, j} = zeros(n_C, 1);
        else
            % only assigne lower triangular values
            S_bar{i, j} = C * A^(i - j) * S;
        end
    end
end
S_bar = cell2mat(S_bar);
% Z_bar = [Z_1; ...; Z_Np]
% size: dim_Z_1 * (Np x dim_Z_2) 
Z_bar = cell(Np, 1);
for i = 1 : Np
    Z_bar{i, 1} = Z;
end
Z_bar = cell2mat(Z_bar);
%% cost: Q_bar, R_bar
% cost function: 
% J = ∑(i = 1 to Np)(y(k + i|k) - yr(k + i|k))' * Q * (y(k + i|k) - yr(k + i|k)) + 
%     ∑(i = 0 to Np - 1)(u(k + i|k))' * R * (u(k + i|k)) +
%     ∑(i = 0 to Np - 2)(u(k + i + 1|k) - u(k + i|k))' * Ru * (u(k + i + 1|k) - u(k + i|k))
% matrix form: J = Y'* Q_bar * Y + U' * R_bar * U + U' * G'* Ru_bar * G * U
% U = [u(k|k); ...; u(k + Np - 1|k)]
% G = 1/Ts*[-1 1 0 ... 0; 0 -1 1 0 ... 0; ...; 0 ... -1 1]
% Optional: 
% Q = [15 0;
%      0  7];
Q = [10 0;
     0  7];
% optional: R = 20;
R = 50;
% Q_bar = diag(Q), Q: weight matrix for output error part
% size: (dim_Q_1 * Np) x (Np * dim_Q_2) 
Q_bar = cell(Np, Np);
% R_bar = diag(R), R: weight matrix for control input part
% size: (dim_R_1 * Np) x (Np * dim_R_2) 
R_bar = cell(Np, Np);
n_R = size(R, 1);
for i = 1 : Np
    for j = 1 : Np
        if i == j
            % only assigned diagonal values
            Q_bar{i, i} = Q;
            R_bar{i, i} = R;
        else
            % otherwises 0
            Q_bar{i, j} = zeros(n_C, n_C);
            R_bar{i, j} = zeros(n_R, n_R);
        end
    end
end
Q_bar = cell2mat(Q_bar);
R_bar = cell2mat(R_bar);
%% cost: G, Ru and Ru_bar
% G = 1 / Ts * [-1  1 0 0 ...  0  0;
%                0 -1 1 0 ...  0  0;
%                ...
%                0  0 0 0 ... -1  1]
% becuase of the differentiation, the size of G is (Np - 1) x Np
G = zeros(Np - 1, Np);
for i = 1 : Np - 1
    G(i, i) = -1;
    G(i, i + 1) = 1;
end
G = 1 / Ts * G;
% Ru_bar = diag(Ru)
% size: (dim_Ru_1 * (Np - 1)) x ((Np - 1) * dim_Ru_2)
% Ru, weight for u(k + i + 1|k) - u(k + i|k) part, or jerk part in cost function
% optional: Ru = 50;
Ru = 70;
Ru_bar = cell(Np - 1, Np - 1);
for i = 1 : Np - 1
    for j = 1 : Np - 1
        if i == j
            % only assign diagonal values
            Ru_bar{i, j} = Ru;
        else
            % otherwise 0
            Ru_bar{i, j} = 0;
        end
    end
end
Ru_bar = cell2mat(Ru_bar);
%% current measured states and relative speed at last time step
vr_last_k = x_k(2);
% Current measured states
x_r = u(1);
v_r = u(2);
v_h = u(3);
x_k = [x_r; v_r; v_h];
%% cost: f^T, H, f_bar^T, H_bar
% J = f_T * U + U' * H * U + p * ep^2
% U = [u(k|k); ...; u(k + Np - 1|k)]; ep: slack variable
% leader vehicle acceleration as disturbance and is assumed to be constant
% in the next Np time step
d = (v_r - vr_last_k) / Ts + U_1;
D = d * ones(Np, 1);
% f^T
f_T = 2 * (x_k' * A_bar' * Q_bar * B_bar - Z_bar' * Q_bar * B_bar + D' * S_bar' * Q_bar * B_bar);
% f_bar^T
f_bar_T = [f_T 0];
% H
H = 2 * (B_bar' * Q_bar * B_bar + R_bar + G' * Ru_bar * G);
% H may be an asymmetric matrix due to rounding
H = (H + H') / 2;
% p: weight for slack variable, optional: p = 20
p = 10;
% H_bar
H_bar = [H             zeros(Np, 1);
         zeros(1, Np)  p          ];
%% Constraints: L, M, N
% [xr(k); vh(k)] = L * x(k), L = [1 0 0; 0 0 1]
L = [1 0 0;
     0 0 1];
% M = [minimal_relative_distance; vmin] <= L * x(k + 1)
% L * x(k + 1) = L * A * x(k) + L * B * u(k) + L * S * d(k)
% => -L * B * u(k) <= -M + L * A * x(k) + L * S * d(k)
% maximum and minimum of ego vehicle speed
v_max = 24;
v_min = 0;
minimal_relative_distance = 10;
M = [minimal_relative_distance; v_min];
% N = [maximal_relative_distance; vmax] >= L * x(k + 1)
% L * B * u(k) <=  N - L * A * x(k) - L * S * d(k)
N = [10000; v_max];
% delta_U = [u(k + 1) - u(k); ...; u(k + Np - 1) - u(k + Np - 2)] = GU
% GU <= Jmax
% GU >= Jmin
%% Constraints: L_bar, Ae_bar, Be_bar, Se_bar
% L_bar = diag(L), size: (Np * dim_L_1) x (dim_L_2 * Np)
n_L1 = size(L, 1);
n_L2 = size(L, 2);
L_bar = cell(Np, Np);
for i = 1 : Np
    for j = 1 : Np
        if i == j
            % assign only diagonal values
            L_bar{i, j} = L;
        else
            % otherwise 0
            L_bar{i, j} = zeros(n_L1, n_L2);
        end
    end
end
L_bar = cell2mat(L_bar);
% Ae_bar = [A; ...; A^Np], size: (Np * dim_A_1) x dim_A_2
Ae_bar = cell(Np, 1);
Ae_bar{1, 1} = A;
for i = 2 : Np
    Ae_bar{i, 1} = Ae_bar{i - 1, 1} * A;
end
Ae_bar = cell2mat(Ae_bar); 
% Be_bar = [B               0                0               ... 0;
%           A * B           B                0               ... 0;          
%           ...
%           A^(Np - 1) * B  A^(Np - 2) * B   A^(Np - 3) * B  ... B]
% size: (Np * dim_B_1) x (dim_B_2 x Np)
n_B = size(B, 1);
Be_bar = cell(Np, Np);
for i = 1 : Np
    for j = 1 : Np
        if j > i
            Be_bar{i, j} = zeros(n_B, 1);
        else
            Be_bar{i, j} = A^(i - j) * B;
        end
    end
end
Be_bar = cell2mat(Be_bar);
% Se_bar = [S               0                0               ... 0;
%           A * S           S                0               ... 0;          
%           ...
%           A^(Np - 1) * S  A^(Np - 2) * S   A^(Np - 3) * S  ... S]
% size: (Np * dim_S_1) x (dim_S_2 * Np)
Se_bar = cell(Np, Np);
n_S = size(S, 1);
for i = 1 : Np
    for j = 1 : Np
        if j > i
            % upper triangular values are 0
            Se_bar{i, j} = zeros(n_S, 1);
        else
            % only assigne lower triangular values
            Se_bar{i, j} = A^(i - j) * S;
        end
    end
end
Se_bar = cell2mat(Se_bar);
%% Constraints: Omega_bar, T
% Omega * U <= T
% Omega = [L_bar * Be_bar; -L_bar * Be_bar; G; -G]
% with slack variable ep, Omega_bar * U_bar <= T, and U_bar = [U; ep]
% Omega_bar
n_N = size(N, 1);
Omega_bar = cell(4, 1);
Omega_bar{1, 1} = [ L_bar * Be_bar -ones(n_N * Np, 1)];
Omega_bar{2, 1} = [-L_bar * Be_bar  zeros(n_N * Np, 1)];
Omega_bar{3, 1} = [ G -ones(Np - 1, 1)];
Omega_bar{4, 1} = [-G  ones(Np - 1, 1)];
Omega_bar = cell2mat(Omega_bar);
% N_bar = [N; ...; N]
% M_bar = [M; ...; M]
N_bar = cell(Np, 1);
M_bar = cell(Np, 1);
for i = 1 : Np
    N_bar{i, 1} = N;
    M_bar{i, 1} = M;
end
N_bar = cell2mat(N_bar);
M_bar = cell2mat(M_bar);
% T = [ N_bar - L_bar * Ae_bar * xe(k) - L_bar * Se_bar * D;
%      -M_bar + L_bar * Ae_bar * xe(k) + L_bar * Se_bar * D;
%       Jmax;
%       Jmin]
% lower bound, upper bound of jerk
j_max =  2.5;
j_min = -2.5;
T = cell(4, 1);
T{1, 1} =  N_bar - L_bar * Ae_bar * x_k - L_bar * Se_bar * D;
T{2, 1} = -M_bar + L_bar * Ae_bar * x_k + L_bar * Se_bar * D;
T{3, 1} =  j_max * ones(Np - 1, 1);
T{4, 1} = -j_min * ones(Np - 1, 1);
T = cell2mat(T);
%% optimization
% Inequality constraint
A_ine = Omega_bar;
b_ine = T;
% No equality constraint, since the state-space equation is already
% incorperated in the derivation of matrix form cost function
A_ep = [];
B_ep = [];
% lower bound, upper bound.
U_max = a_max * ones(Np, 1);
U_min = a_min * ones(Np, 1);
ep_min = 0;
ep_max = 10;
lower_bound = [U_min; ep_min]; 
upper_bound = [U_max; ep_max];
% initial point is chosen to zero because of convex problem, we can always
% find a global minimum, and it might be faster when we choose another
ini = acc;
% min x'Hx + f'x, when use quadprog, the input is f not f'
% use 'active-set' algorithm
options = optimset('Algorithm','active-set');
[acc, ~] = quadprog(H_bar, f_bar_T', A_ine, b_ine, A_ep, B_ep, lower_bound, upper_bound, ini, options);
% only apply the first control command to the vehicle model
U_1 = acc(1);
% The quadprog still outputs when the programming is infeasible, so U
% should be limited so that it won't be out of the boundary
if U_1 > a_max
    U_1 = a_max;
elseif U_1 < a_min
    U_1 = a_min;
end
% outputs of S-Function, it must be consistent with the "size.NumOutputs" 
% in the "mdlInitializeSizes" function
sys = U_1;
end