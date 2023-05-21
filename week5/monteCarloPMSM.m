%% Instituto Federal Fluminense
%  Signals and Systems Lab - 2022.2
%  Week 05(1) - Monte Carlo simulation with PMSM - Generating data
%  Student: Kaique Guimaraes Cerqueira
clear, clc, close all
% -----------Code Structs--------------------------------------------------
% simParam:           ->        Parameters required to simulate model
%        Model      string      Model name (monteCarloPMSMsim.slx)
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
% aug:                ->        State-space augmented control model
%        poles      array(5)    Closed-loop desired poles
%            K      matrix(2x5) State feedback Gains Matrix
%            A      matrix(5x5) Augmented states' matrix
%            B      matrix(5x2) Augmented actuators' matrix
% noise:              ->        Disturbances and Noise parameters
%          var      array(2)    Noise(1) and disturbances(2) variance
%% Load structures (opPoint and aug)
load('controllerGains.mat');
simParam.Model = 'monteCarloPMSMsim.slx';
simParam.maxStep = 2e-5;
simParam.xInitial = opPoint.x;
simParam.stopTime = 4;
timeInput = 0:simParam.maxStep:simParam.stopTime;
% Step in Iq == Step in velocity
simParam.Input = [timeInput', (timeInput'*0)+opPoint.x(1)*1,... 
                                  (timeInput'*0)+opPoint.x(2)*1.1,...
                                  (timeInput'*0)+opPoint.x(3)*1];
% Number of experiments
numExp = 1000;

%% Model Parameters (disturbed)
uncertainty = 0.025; % 2.5% of value
% Parameters with uncertainties
Rs_mean = 1.3; % [Ohms]
rs_amplitude = Rs_mean*uncertainty;
noise.param.Rs = Rs_mean + (rand(numExp,1)*rs_amplitude - rs_amplitude/2); 
Ld_mean = 0.9; % [mH]
ld_amplitude = Ld_mean*uncertainty;
noise.param.Ld = Ld_mean + (rand(numExp,1)*ld_amplitude - ld_amplitude/2); 
% Lq_mean = 0.9; % [mH]
% lq_amplitude = Lq_mean*uncertainty;
% noise.param.Lq = Lq_mean + (rand(numExp,1)*lq_amplitude - lq_amplitude/2);
noise.param.Lq = noise.param.Ld;
J_mean = 0.1; % [Nms^2]
j_amplitude = J_mean*uncertainty;
noise.param.J = J_mean + (rand(numExp,1)*j_amplitude - j_amplitude/2); 
b_mean = 0.01; % [Nms]
b_amplitude = b_mean*uncertainty;
noise.param.B = b_mean + (rand(numExp,1)*b_amplitude - b_amplitude/2); 
lambdaM_mean = 0.14; % Flux linkage [Wb]
lambdam_amp = lambdaM_mean*uncertainty;
noise.param.lambdaM = lambdaM_mean + (rand(numExp,1)*lambdam_amp - lambdam_amp/2);
N = 18; % Number of turns

% Variance vectors
% Noise (on measurements)
var_noise = [10 10 100]*10e-11;
noise.seed.noise = randi(10000,[numExp 1]);
% Disturbance (on inputs)
var_disturbance = [48 48]*10e-9;
noise.seed.disturbance = randi(10000,[numExp 1]);
%% Simulate
% load('monteCarloData.mat') % Re-generate data (Onedrive trash bin)
for i = 1:numExp
    Rs = noise.param.Rs(i);
    assignin('base',"Rs",Rs);
    Ld = noise.param.Ld(i);
    assignin('base',"Ld",Ld);
    Lq = noise.param.Lq(i);
    assignin('base',"Lq",Lq);
    J = noise.param.J(i);
    assignin('base',"J",J);
    B = noise.param.B(i);
    assignin('base',"B",B);
    lambdaM = noise.param.lambdaM(i);
    assignin('base',"lambdaM",lambdaM);
    simParam.seed(1) = noise.seed.noise(i);
    simParam.seed(2) = noise.seed.disturbance(i);
    assignin('base',"simParam",simParam);
    warning off
    data(i) = sim(simParam.Model) %#ok<NOPTS,SAGROW> 
    warning on

end

%% Save simulation data
% save('monteCarloData.mat','data','-v7.3')

%% Plot Generated data
% States' plot
figure()
subplot(3,1,1)
for j = 1:numExp
    plot(data(j).tout,data(j).states(:,1))
    hold on
end
subplot(3,1,2)
for j = 1:numExp
    plot(data(j).tout,data(j).states(:,2))
    hold on
end
subplot(3,1,3)
for j = 1:numExp
    plot(data(j).tout,data(j).states(:,3))
    hold on
end

% Control's plot
figure()
for j = 1:numExp
    plot(data(j).tout,data(j).control)
    hold on
end