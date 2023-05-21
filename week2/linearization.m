%% Instituto Federal Fluminense
%  Laboratório de Controle e Sinais - 2022.2
%  Semana 02 - Linearização do modelo através do método numérico
%  Aluno: Kaique Guimarães Cerqueira
clear, clc, close all
%% Script de linearizacao numérica do sistema não linear.
% X_1_dot = X_2
% X_2_dot = a * u^2 + b * sign(X_2) * X_2^2
% [ ]Adota-se os parâmetros a e b como unitários (a=b=1).
% [x]Adota-se os parâmetros a e b de mesmo módulo (a=1, b=-1).
a = 1;
b = -1;
% Ponto de operacao utilizado:
x_1op = input('Digite o valor de X1 no ponto de operacao:');
xdot_1op = input('Digite o valor da derivada de X1 no ponto de operacao:');
x_2op = input('Digite o valor de X2 no ponto de operacao:');
xdot_2op = input('Digite o valor da derivada de X2 no ponto de operacao:');
U_op = input('Digite o valor de U no ponto de operacao:');

% =============dist.: percentual do range max de perturbacao==========
disturbance = input('Digite a ordem de grandeza da perturbacao a ser usada: 10e-');
xd_1 = x_1op*(1 + 10^(-disturbance));
xd_2 = x_2op + 10^(-disturbance);
Ud = 10^(-disturbance);

% Disturbance calculation:
X2 = @(x2,u) a*u*u + b*sign(x2)*x2*x2;

% Implementar As matrizes A e B da linearização numérica
% ==================Implementar genérico=======================
a11 = (x_2op - xdot_1op)/(xd_1-x_1op);
a12 = (xd_2 - xdot_1op)/(xd_2-x_2op);
a21 = (X2(x_2op,U_op) - xdot_2op)/(xd_1-x_1op);
a22 = (X2(xd_2,U_op) - xdot_2op)/(xd_2-x_2op);
A = [a11,a12;a21,a22];

b11 = (x_2op - xdot_1op)/(Ud-U_op);
b21 = (X2(x_2op,Ud) - xdot_2op)/(Ud-U_op);
B = [b11;b21];

% Linear system
linSys = ss(A,B,eye(2),[0;0])

%% Step response
% Non-linear:
entrada = input('Digite a amplitude do degrau:');
t = 0:0.00001:10;
u = (t>1) * entrada;
%=================Resolver no Simulink==========================
[X1_nlin, X2_nlin] = nonLinearExampleModel(u, t, a, b, x_1op, x_2op);

% Linear:
y = lsim(linSys, u, t);
X1_lin = y(:,1) + x_1op;
X2_lin = y(:,2) + x_2op;

figure()
    subplot(2,1,1)
        plot(t, X1_nlin, t, X1_lin)
        title(['Step response: U = ', num2str(entrada)])
        axis tight
        legend('Non-Linear', 'Linear')
        xlabel('Time (s)')
    subplot(2,1,2)
        plot(t, X2_nlin, t, X2_lin)
        axis tight
        legend('Non-Linear', 'Linear')
        xlabel('Time (s)')

%% Data analysis
emq_X1 = sum((X1_lin-X1_nlin).^2) / numel(X1_lin)
emq_X2 = sum((X2_lin-X2_nlin).^2) / numel(X2_lin)








