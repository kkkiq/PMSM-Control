%% Instituto Federal Fluminense
%  Signals and Systems Lab - 2022.2
%  Week 04 - Control design of PMSM motor problem
%  Student: Kaique Guimaraes Cerqueira
clear, clc, close all
% X = [id,iq,w]
% U = [Vd,Vq]
% N.L.D.E. = pmsm_comparison.slx
% -----------Code Structs--------------------------------------------------
% simParam:           ->        Parameters required to simulate pmsm_nl.slx
%        Model      string      Model name
%     xInitial      array(3)    States' initial onditions
%     stopTime      real        Simulation time (seconds)
%      maxStep      real        Max step made by numerical integration
%        Input      array(3)    Input vector + timebase in last element
% 
% opPoint:            ->        States/Inputs after trim point being chosen
%         xDot      array(3)    State derivatives operation point
%            x      array(3)    States (Id,Iq,w) operation point
%        Input      array(2)    Input (Vd,Vq) operation point
% 
% linear:             ->        Linearized matrices
%            A      matrix(3x3) Linearized states' dynamics
%            B      matrix(3x2) Linearized actuators' dynamics
%            C      matrix(3x3) Linearized sensors' dynamics
%            D      matrix(3x2) Linearized direct transmission's dynamics
%       System      ss          State-space linearized system
%       States      matrix(nx3) States' dynamic footprint
% 
% aug:                ->        State-space augmented control model
%        poles      array(5)    Closed-loop desired poles
%            K      matrix(2x5) State feedback Gains Matrix
%            A      matrix(5x5) Augmented states' matrix
%            B      matrix(5x2) Augmented actuators' matrix

%% Parameters:
Rs = 1.3; % Ohms
Ld = 0.9; % mH
Lq = 0.9; % mH
J = 0.1; % Nms^2
b = 0.01; % Nms
lambdaM = 0.14; % Wb
N = 18;

%% Control Parameters
% Load trim and linearization data
load('linearSystem.mat')
% Considering we have access to all measures:
linear.C = eye(3);
% Load simulink parameters
simParam.Model = 'pmsm_l.slx';
simParam.maxStep = 2e-5;
simParam.xInitial = opPoint.x; % -> not used
simParam.stopTime = 4;
timeInput = 0:simParam.maxStep:simParam.stopTime-simParam.maxStep;

%% Set transfer functions (Bode design method)
% The system has 2 inputs (Vd,Vq) and 2 outputs (Id,Iq), making it a 2x2 
% transference matrix. We will only try to control the main diagonal TFs
[num,den] = ss2tf(linear.A,linear.B,linear.C,linear.D,1);
Id_Vd = tf(num(1,:),den)

[num,den] = ss2tf(linear.A,linear.B,linear.C,linear.D,2);
Iq_Vq = tf(num(2,:),den)
%% Defining requirements
% FIRST TRY: LOOSER REQUIREMENTS
% 0.6 < zeta < 1
zeta = 0.9;
% Steady state error (Ess) < 5%
% Rise time (Tr) < 0.1s
% Settling time (Ts) < 0.5s             Ts = 4/sigma        sigma = zeta*wn
sigma = 4/0.9;
wn = sigma/zeta;
% Second-order standard system
[~, den] = ord2(wn,zeta);
% Closed-loop dominant poles
poles = roots(den);
clear num den
%% Control architecture 
% State Feedback with state augmentation:
% Define augmented system: x = [id iq w Iid Iiq] -> Id, Iq and its integrals
Iid = [1 0 0]; % Id's Integral state
Iiq = [0 1 0]; % Iq's Integral state
aug.A = [[linear.A; Iid; Iiq],zeros(5,2)]; % States Augmented matrix
aug.B = [linear.B; zeros(2)]; % Inputs Augmented matrix

clear Iid Iiq
%% Controller type - PI
% With that, the steady-state error goes to 0 (Ess < 5%)

%% Defining controller gains
% [x] Pole placement and state augmentation: Ogata's Inverted Pendulum
%       - Example - pg. 697 use place() command
% [ ] Use bode plot and discover gains using limit
%       - Use PI Controller for each transfer function
poleDistance = 20; % Non-dominant poles are 5x more distant from imag axis
poleLocation = real(poles(1)); % Non-dominant poles location
% Closed-loop desired poles
aug.poles = [poles', poleLocation*poleDistance,...
            poleLocation*poleDistance+0.1, poleLocation*poleDistance-0.1];
aug.K = place(aug.A, aug.B, aug.poles); % use as SIMO

%% Step response simulation
chooseStepState = input('1-Id   2-Iq   3-Omega\nChoose the parameter to excitate: ');
if chooseStepState == 1
    %--------------------- Step input (Id) ----------------------------------
    % Linear frame:
    % simParam.Input = [timeInput' (timeInput'*0)+opPoint.x(1)*0.5 timeInput'*0 timeInput'*0];
    % Non-linear frame:
    simParam.Input = [timeInput', (timeInput'*0)+opPoint.x(1)*1.5,... 
                                  (timeInput'*0)+opPoint.x(2)*1,...
                                  (timeInput'*0)+opPoint.x(3)*1];
elseif chooseStepState == 2
    %--------------------- Step input (Iq) ----------------------------------
    % Linear frame:
    % simParam.Input = [timeInput' timeInput'*0 (timeInput'*0)+opPoint.x(2)*0.1 timeInput'*0]; 
    % Non-linear frame:
    simParam.Input = [timeInput', (timeInput'*0)+opPoint.x(1)*1,... 
                                  (timeInput'*0)+opPoint.x(2)*1.1,...
                                  (timeInput'*0)+opPoint.x(3)*1];
elseif chooseStepState == 3
%     Step Input: from 10rad/s to 20rad/s operation point (experimental)
    load("trim.mat");
    operationPoint = operationPoint(2,:);
    opPoint.Input = operationPoint(1:2);
    simParam.Input = [timeInput', (timeInput'*0)+operationPoint(3),...
                                  (timeInput'*0)+operationPoint(4),... 
                                  (timeInput'*0)+opPoint.x(3)];
end
clear operationPoint omega cost
% Simulate non-linear control model
simParam.Model = 'pmsm_comparison.slx';
data = sim(simParam.Model);

%% Step response analysis
% Transient response Key Performance Indicators' (KPI's) 
kpi.riseTime = opPoint.x*0;
kpi.overshoot = opPoint.x*0;
kpi.settlingTime = opPoint.x*0;
kpi.ssError = opPoint.x*0;
kpi.mse = opPoint.x*0;

for state = 1:numel(opPoint.x)
% Rise time (to 100% of reference value):
    i = 1;
    while data.states(i,state) < simParam.Input(end,state+1)
        i = i+1;
    end
    kpi.riseTime(state) = data.tout(i);
% Overshoot (%):
    kpi.overshoot(state) = ((max(data.states(:,state)) - simParam.Input(end,state+1))...
                    / simParam.Input(end,state+1)) * 100;
% Settling time (2% criteria) -> Is it 2% of step or 2% of total range?:
    i = numel(data.tout);
    ub = simParam.Input(end,state+1)*0.98;
    lb = simParam.Input(end,state+1)*1.02;
    while ((data.states(i,state) > ub) && ...
           (data.states(i,state) < lb))
        if i > 1
            i = i-1;
        else
            break
        end
    end
    kpi.settlingTime(state) = data.tout(i);
% Steady-State error:
    kpi.ssError(state) = data.states(end,state) - simParam.Input(end,state+1);
% MSE
    kpi.mse(state) = immse(simParam.Input(:,state+1), data.states(1:end-1,state));
end

% Control signal-related KPI's
kpi.uMax = opPoint.Input*0;
kpi.uInt = opPoint.Input*0;
for input = 1:numel(opPoint.Input)
% Control signal integral
    kpi.uInt(input) = sum(abs(data.control(:,input)));
% Maximum control input(module)
    kpi.uMax(input) = max(abs(data.control(:,input)));
end