%% Instituto Federal Fluminense
%  Signals and Systems Lab - 2022.2
%  Week 03 - Trim and Linearization of PMSM motor model
%  Student: Kaique Guimaraes Cerqueira
clear, clc, close all
% X = [id,iq,w,theta]
% U = [Vd,Vq]
% Y = [id,iq]
% N.L.D.E. = pmsm_nl.slx
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
% disturbance:        ->        Numerical Linearization matrices and params
%    Magnitude      real        Disturb magnitude
%       States      array(3)    Operation point offset at states' array
%        Input      array(2)    Operation point offset at inputs' array
%            A      matrix(3x3) A matrix' state derivatives computed
%          opX      cell(2x1)   Set of X initial conditions w/ disturbance
%            B      matrix(3x2) B matrix' state derivatives computed
%      opInput      cell(2x1)   Set of Input initial conditions w/ dist.
% 
% linear:             ->        Linearized matrices
%            A      matrix(3x3) Linearized states' dynamics
%            B      matrix(3x2) Linearized actuators' dynamics
%            C      matrix(3x3) Linearized sensors' dynamics
%            D      matrix(3x2) Linearized direct transmission's dynamics
%       System      ss          State-space linearized system
%       States      matrix(nx3) States' dynamic footprint
% 
%% Parameters:
Rs = 1.3; % Ohms
Ld = 0.9; % mH
Lq = 0.9; % mH
J = 0.1; % Nms^2
b = 0.01; % Nms
lambdaM = 0.14; % Wb
N = 18;

%% Initial Conditions and Simulation parameters
simParam.Model = 'pmsm_nl.slx';
simParam.xInitial = zeros(1,3);
simParam.xInitial(3) = 20; % X3op = omega
simParam.stopTime = 0;
simParam.maxStep = 0.0001;

%% Operation point: Optimization problem
% Find X1, X2, U1, U2 when X1_dot, X2_dot, X3_dot = 0; X4_dot = X3op
% ub = [48 48 0.001 10]/2;
% lb = -ub;
% firstGuess = [rand(1,2)*48-24 rand(1,2)*10-5];
% options = optimoptions("fmincon","Display","iter-detailed");
% [operationPoint,cost] = fmincon(@(x) nonLinearPMSMCost(x,simParam), firstGuess, [], [], [], [], lb, ub,[],options)

% Cheat: load trim point at trimPoint.mat
load("trim.mat", "operationPoint")
operationPoint = operationPoint(2,:);

% Create opPoint struct
opPoint.Input = [operationPoint(1) operationPoint(2)];
opPoint.x = [operationPoint(3) operationPoint(4) simParam.xInitial(3)];
% Preparing to simulate
simParam.Input = [0 opPoint.Input];
simParam.xInitial = opPoint.x;
simParam.stopTime = 10;
simParam.maxStep = 0.0001;
assignin('base',"simParam",simParam);
data = sim(simParam.Model);
opPoint.xDot = data.stateDerivative(1,:);
opPoint.x = data.state(1,:);
plot(data.tout, data.state)

clear lb firstGuess Aeq beq options operationPoint data
%% Linearization : Numerical analysis
simParam.stopTime = 0;
% Determine disturbances vector
% Question:
%       how to choose the scale of the disturbance? is it proportional to
%       opPoint amplitude? or bounds amplitude?
disturbance.Magnitude = 0.000001;
disturbance.States = opPoint.x *disturbance.Magnitude;
% disturbance.States = [ub(3) ub(4) opPoint.x(3)] *disturbance.Magnitude;
disturbance.Input = opPoint.Input * disturbance.Magnitude;
% disturbance.Input = [ub(1) ub(2)] * disturbance.Magnitude;

disturbance.A = zeros(numel(disturbance.States)); %#ok<MNUML>
% Creating disturbance cell with set of disturbed states
disturbance.opX = cell(numel(disturbance.States), 1);
for row = 1:numel(disturbance.States)
        disturbance.opX{row} = opPoint.x;
    for col = 1:numel(disturbance.States)
        if col == row
            disturbance.opX{row}(col) = disturbance.opX{row}(col) + disturbance.States(col);
        end
    end
end

% Simulation of disturbances in A matrix:
for row = 1:numel(disturbance.States)
    [stateDisturbed, ~] = linearizePMSMSim(opPoint.Input, disturbance.opX{row}, simParam);
    disturbance.A(:,row) = stateDisturbed';
end

% Numerical Linearization
linear.A = disturbance.A*0;
for row = 1:numel(disturbance.States)
    for col = 1:numel(disturbance.States)
        linear.A(row,col) = (disturbance.A(row,col) - opPoint.xDot(row)) / (disturbance.opX{col}(col) - opPoint.x(col));
    end
end

%=============================Do with B matrix===========================
disturbance.B = zeros(numel(disturbance.States), numel(disturbance.Input));
% Creating disturbance cell with set of disturbed inputs
disturbance.opInput = cell(numel(disturbance.Input),1);
for row = 1:numel(disturbance.Input)
    disturbance.opInput{row} = opPoint.Input;
    for col = 1:numel(disturbance.Input)
        if row == col
            disturbance.opInput{row}(col) = disturbance.opInput{row}(col) + disturbance.Input(col);
        end
    end
end

% Simulation of disturbances in B matrix:
for col = 1:numel(disturbance.Input)
    [stateDisturbed, ~] = linearizePMSMSim(disturbance.opInput{col}, opPoint.x, simParam);
    disturbance.B(:,col) = stateDisturbed;
end

% Numerical Linearization
linear.B = disturbance.B*0;
for row = 1:numel(disturbance.States)
    for col = 1:numel(disturbance.Input)
        linear.B(row,col) = (disturbance.B(row,col) - opPoint.xDot(row)) / (disturbance.opInput{col}(col) - opPoint.Input(col));
    end
end

% Create state-space linear system
linear.C = [1 0 0;
            0 1 0;
            0 0 0];
linear.D = [0 0; 0 0; 0 0];
linear.System = ss(linear.A, linear.B, linear.C, linear.D);

%% Excite both systems (linear and non-linear) and compare
simParam.maxStep = 5e-6;
simParam.stopTime = 3;
timeInput = 0:simParam.maxStep:simParam.stopTime-simParam.maxStep;

% Step in Vq (after 1s)
U = [timeInput*0; (timeInput>0.2)*opPoint.Input(2)*0.5]';
simParam.Input = [timeInput', U+opPoint.Input];
assignin('base',"simParam", simParam)
% Non-linear system simulation
data = sim(simParam.Model);
% Linear system simulation
linear.States = lsim(linear.System, U, timeInput);
linear.States = linear.States + opPoint.x;

figure()
subplot(3,1,1)
    plot(data.tout, data.state(:,3))
    title(['Step in V_q after 200ms, U = [ 0 ',num2str(U(end,2)),' ]'])
    hold on
    plot(timeInput, linear.States(:,3))
    legend('omega_{nl}','omega_{lin}')
    grid minor
subplot(3,1,2)
    plot(data.tout, data.state(:,1))
    hold on
    plot(timeInput, linear.States(:,1))
    legend('Id_{nl}','Id_{lin}')
    grid minor
subplot(3,1,3)
    plot(data.tout, data.state(:,2))
    hold on
    plot(timeInput, linear.States(:,2))
    legend('Iq_{nl}','Iq_{lin}')
    grid minor

% Step in Vd (after 1s)
U = [(timeInput>0.2)*opPoint.Input(1)*0.5; timeInput*0]';
simParam.Input = [timeInput', U+opPoint.Input];
assignin('base',"simParam", simParam)
% Non-linear system simulation
data = sim(simParam.Model);
% Linear system simulation
linear.States = lsim(linear.System, U, timeInput);
linear.States = linear.States + opPoint.x;

figure()
subplot(3,1,1)
    plot(data.tout, data.state(:,3))
    title(['Step in V_d after 200ms, U = [ ',num2str(U(end,1)),' 0 ]'])
    hold on
    plot(timeInput, linear.States(:,3))
    legend('omega_{nl}','omega_{lin}')
    grid minor
subplot(3,1,2)
    plot(data.tout, data.state(:,1))
    hold on
    plot(timeInput, linear.States(:,1))
    legend('Id_{nl}','Id_{lin}')
    grid minor
subplot(3,1,3)
    plot(data.tout, data.state(:,2))
    hold on
    plot(timeInput, linear.States(:,2))
    legend('Iq_{nl}','Iq_{lin}')
    grid minor

%% Numerical analysis
% do mean-squared error (immse)
fprintf('Id Mean-Squared error: %e\n', immse(data.state(1:end-2,1), linear.States(:,1)))
fprintf('Iq Mean-Squared error: %e\n', immse(data.state(1:end-2,2), linear.States(:,2)))
