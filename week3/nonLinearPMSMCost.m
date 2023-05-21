function J = nonLinearPMSMCost(X, simParam)
%   Cost function used to search and define trim point
% simParam:           ->        Parameters required to simulate pmsm_nl.slx
%        X:           ->        [U1, U2, X1_i, X2_i]
%   Steps:
%       [1] Rearrange data parameters and input
%       [2] Assign parameters into simulation
%       [3] Simulate model
%       [4] Arrange state derivative output
%       [5] Cost function - goal is to minimize

%       [1]
%       Input -> [time, data]
    simParam.Input = [zeros(numel(X(1)),1) X(1) X(2)];
    simParam.xInitial(1) = X(3);
    simParam.xInitial(2) = X(4);
%       [2]
    assignin('base',"simParam",simParam)
%       [3]
    dataSim = sim(simParam.Model);
%       [4]
    X1_dot = dataSim.stateDerivative(end,1);
    X2_dot = dataSim.stateDerivative(end,2);
    X3_dot = dataSim.stateDerivative(end,3);
%     X4_dot = dataSim.stateDerivative(:,4); % Not used

% DEBUG:
%     fprintf('id_dot = %2.2f\t',X1_dot);
%     fprintf('iq_dot = %2.2f\t',X2_dot);
%     fprintf('omega_dot = %2.2f\n',X3_dot);

%       [5]
    J = X1_dot^2 + X2_dot^2 + X3_dot^2;
end