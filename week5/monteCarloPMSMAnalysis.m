%% Instituto Federal Fluminense
%  Signals and Systems Lab - 2022.2
%  Week 05(2) - Monte Carlo simulation with PMSM - Analyzing data
%  Student: Kaique Guimaraes Cerqueira
% clear, clc, close all
% -----------Code Structs--------------------------------------------------
% data:               ->        Monte Carlo Simulation data
%      control      matrix(nx2) Feedback control signals
%       states      matrix(nx3) states measurements
%         tout      array(n)    time vector at simulation

%% Load simulation data
load("monteCarloSimData.mat")
load('controllerGains.mat','opPoint')

% Setpoints
id.setpoint = opPoint.x(1);
% Step in Iq == Step in velocity -> 10% step
iq.setpoint = opPoint.x(2)*1.1;
omega.setpoint = opPoint.x(3)*1.1;
clear opPoint
%% Calculate KPIs
for i = 1:numel(data)
%     KPIs for Id, Vd:
    [id.overshoot(i), id.riseTime(i), id.settlingTime(i), id.ssError(i), id.mse(i), vd.uMax(i), vd.uInt(i)] =...
    computeKPI(data(i).tout, data(i).states(:,1), data(i).control(:,1), id.setpoint);
%     KPIs for Iq, Vq:
    [iq.overshoot(i), iq.riseTime(i), iq.settlingTime(i), iq.ssError(i), iq.mse(i), vq.uMax(i), vq.uInt(i)] =...
    computeKPI(data(i).tout, data(i).states(:,2), data(i).control(:,2), iq.setpoint);
%     KPIs for Omega:
    [omega.overshoot(i), omega.riseTime(i), omega.settlingTime(i), omega.ssError(i), omega.mse(i), ~, ~] =...
    computeKPI(data(i).tout, data(i).states(:,3), 0, omega.setpoint);
end

%% Generate Histogram
[counts, centers] = hist(omega.ssError, round(sqrt(numel(omega.ssError))));
% bar(centers, counts)
% Normalize data
countsn = counts./trapz(centers,counts)
bar(centers, countsn)
%% Fit Curve to data
% Gaussian Generic curve
gauss = @(x) a1*exp(-((x-b1)/c1)^2);

figure()
subplot(3,1,1)
for j = 1:numel(data)
    plot(data(j).tout,data(j).states(:,1))
    hold on
end
subplot(3,1,2)
for j = 1:numel(data)
    plot(data(j).tout,data(j).states(:,2))
    hold on
end
subplot(3,1,3)
for j = 1:numel(data)
    plot(data(j).tout,data(j).states(:,3))
    hold on
end

figure()
subplot(2,1,1)
for i = 1:numel(data)
    plot(data(i).tout,data(i).control(:,1))
    hold on
end
subplot(2,1,2)
for i = 1:numel(data)
    plot(data(i).tout,data(i).control(:,2))
    hold on
end
%% Generate statistics related to curve

%% Parallel Axis Plot