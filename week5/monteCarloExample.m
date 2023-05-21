%% Instituto Federal Fluminense
%  Signals and Systems Lab - 2022.2
%  Week 05 - Monte Carlo simulation example
%  Student: Kaique Guimaraes Cerqueira
clear, clc, close all
% We will define a simple first-order plant, feedback-controlled by a
% proportional controller, altering its parameters and adding noise to the
% system, in order to test its robustness.
% The first-order system is described as:
%              1
%   G(S) = -----------
%            S + b
%% Parameters
numExp = 10;
% Plant params and disturbances magnitude
b = rand(numExp,1)*0.4 - 0.2;
sigma.n = 0.001;
seed.n = randi(10000, [numExp 1]);
sigma.d = 0.0025;
seed.d = randi(10000, [numExp 1]);

% Simulation Parameters
simParam.Model = 'monteCarloTest.slx';
simParam.maxStep = 1e-3;
simParam.stopTime = 20;
timeInput = 0:simParam.maxStep:simParam.stopTime;
simParam.Input = [timeInput' (timeInput'*0)+1];

%% Simulate step response
overshoot = zeros(numExp,1);
riseTime = zeros(numExp,1);
settlingTime = zeros(numExp,1);
ssError = zeros(numExp,1);
mse = zeros(numExp,1);
uMax = zeros(numExp,1);
uInt = zeros(numExp,1);

for i = 1:numExp
    simParam.b = b(i);
    simParam.seedNoise = seed.n(i);
    simParam.seedDist = seed.d(i);
    
    assignin('base',"simParam",simParam);
    data = sim(simParam.Model);
    plot(data.tout, data.y)
    [overshoot(i), riseTime(i), settlingTime(i), ssError(i), mse(i), uMax(i), uInt(i)] = ...
        computeKPI(timeInput',data.y,data.control, simParam.Input(:,2));
    hold on
end





