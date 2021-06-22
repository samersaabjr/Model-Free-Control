% A quadruple tank process with non-minimum phase setting
% Model-free control
clear
close all

Ts = 1e-1; % Sample period
tf = 2000; % Duration of run time

t = 0:Ts:tf; % Run time

% Reference output trajectories
t1 = [0:Ts:tf/2-Ts];
t2 = [tf/2:Ts:tf];
yd1 = ones(1,length(t));
yd21 = zeros(1,length(t1));
yd22 = 2*ones(1,length(t2));

yd2 = [yd21 yd22];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%SYSTEM
n = 4;
q = 2; % Number of inputs
p = 2; % Number of outputs

% Construction of the continuous state-space model: Ac, Bc, Cc
% Nominal Model
A1 = 28;
A3 = A1;
A2 = 32;
A4 = A2;

kc = 0.5;
g = 981;
k1 = 3.14;
k2 = 3.29;
alfa1 = 0.43;
alfa2 = 0.34;
T1 = 63;
T2 = 91;
T3 = 39;
T4 = 56;
Ac = [
    -1/T1 0 A3/(A1*T3) 0
    0 -1/T2 0 A4/(A2*T4)
    0 0 -1/T3 0
    0 0 0 -1/T4];

Bc = [
    alfa1*k1/A1 0
    0   alfa2*k2/A2
    0  (1-alfa2)/A3
    (1-alfa1)/A4 0
    ];

Cc = [
    kc 0 0 0
    0 kc 0 0
    ];
% Discretization of state space (A,B,C)
sysc = ss(Ac, Bc, Cc, zeros(p,q));
sys = c2d(sysc,Ts);
[A,B,C,D] = ssdata(sys);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initial conditions
X = zeros(4,1);
U = [0 0]';

y1(1) = 0;
y2(1) = 0;

Y = C*X;
Yp = Y;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Options for adding measurement noise to the outputs
disp(' ');
noise = input('To add measurement noise enter 1 otherwise enter 0 ');
disp(' ');
if noise == 0, 
    sigR = 0; 
elseif noise == 1, sigR = 0.05; 
else
    error('Options are either 1 or 0 ');
end
Noise = sigR*randn(2,length(t)); % Generation of measurement noise
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Options for limiting the maximum absolute value of the control signal
Constraint = input('To add a maximum absolute limit to the control signal enter 1 otherwise enter 0 ');
disp(' ');
if Constraint == 1
    thres = input('Enter a positive value not less than 3 for the maximum limit for the control e.g. 5 ');
    disp(' ');
    if thres <= 3, error('Maximum limit must be no less than 3');end
elseif abs(Constraint) > 0
    error('Options are either 1 or 0 ');
end

% Implementing the closed-loop control system

for k = 1:length(t)-1
% Model-free Controller     
    if noise == 0
        KP = 500*eye(2);
    elseif noise == 1
        KP = 500*eye(2)/k^0.2;
    end
    gamma = 1.1; % gamma = 1 + Ts
    if k == 1
        % Evaluate errors 
        E = [yd1(k) yd2(k) ]'-[y1(k) y2(k) ]' + Noise(:,k);
        Ep = [yd1(k+1) yd2(k+1)]'-Y + Noise(:,k+1);
        
        U = U + KP*(gamma*Ep - E); % Model free controller
           
        % Truncate control signal so that its max magnitude does not exceed
        % thres
        if Constraint == 1
            if abs(U(1)) > thres; U(1) = sign(U(1))*thres;end
            if abs(U(2)) > thres; U(2) = sign(U(2))*thres;end
        end  
        
        % State space
        X = A*X + B*U; 
        Yp = C*X;
        Y = Yp;
        
    else
        % Evaluate errors
        E = [yd1(k) yd2(k) ]'- Y + Noise(:,k);
        Ep = [yd1(k+1) yd2(k+1) ]'-Yp + Noise(:,k+1);
        
        U = U + KP*(gamma*Ep - E);
        
        
        if Constraint == 1
            if abs(U(1)) > thres; U(1) = sign(U(1))*thres;end
            if abs(U(2)) > thres; U(2) = sign(U(2))*thres;end
        end
        Ym = Y;
        Y = Yp;
        X = A*X + B*U;
        Yp = C*X;
        
    end
    
    y1(k+1) = Yp(1);
    y2(k+1) = Yp(2);
 
    u1(k) = U(1);
    u2(k) = U(2);
    

end
err1 = yd1-y1;
err2 = yd2-y2;
disp('Standard deviation of output errors for the first 2,000 sec, entire range and last 2,000 sec')
[std(err1(1:2000)) std(err1) std(err1(18000:20000));std(err2(1:2000)) std(err2) std(err2(18000:20000))]

disp('Standard deviation of output errors for the first 1,000 sec, entire range and last 1,000 sec')
[std(err1(1:1000)) std(err1) std(err1(19000:20000));std(err2(1:1000)) std(err2) std(err2(19000:20000))]

figure
subplot(2,1,1),plot(t(1:length(u1)),u1,'k'),grid,legend('Control signal u_1(k)')
subplot(2,1,2),plot(t(1:length(u1)),u2,'k'),grid,legend('Control signal u_2(k)')
xlabel('time (sec)')

figure
subplot(2,1,1),plot(t,y1,'r'),hold,plot(t,yd1),legend('Output y_1(k)','Reference y_1^r^e^f(k)')
subplot(2,1,2),plot(t,y2,'r'),hold,plot(t,yd2),,legend('Output y_2(k)','Reference y_2^r^e^f(k)')
xlabel('time (sec)')
