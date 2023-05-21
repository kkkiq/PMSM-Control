function [X1, X2] = nonLinearExampleModel(u, t, a, b, X1_i, X2_i)
% State-space simulation using Euler1. System used:
% X_1_dot = X_2
% X_2_dot = a * u^2 + b * sign(X_2) * X_2^2
% Remarks:
%   - Parameters a, b = 1
    X1 = t'*0;
    X2 = t'*0;
    n = numel(t) - 1;
    Ts = t(2) - t(1); % Sampling time
%     Initialization
    X1(1) = X1_i;
    X2(1) = X2_i;
%     Solve ODEs using Euler1
    for i = 2:n
        X1(i) = X1(i-1) + (X2(i-1) * Ts);
        X2(i) = X2(i-1) + (a*u(i)*u(i) + b*sign(X2(i-1))*(X2(i-1)*X2(i-1))) * Ts;
    end
%     Gambiarra: últimos valores estavam zerados (será que não passou pela iteração?)
    X1(end) = X1(end-1);
    X2(end) = X2(end-1);
end