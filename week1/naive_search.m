%% Instituto Federal Fluminense
%  Laboratório de Controle e Sinais - 2022.2
%  Semana 01 - Busca do ponto de operação
%  Aluno: Kaique Guimarães Cerqueira

%% Dado o sistema não Linear:
% X_1_dot = X_2
% X_2_dot = a * u^2 + b * sign(X_2) * X_2^2
% Adota-se os parâmetros a e b como unitários.
% Busque o ponto de operação nos 4 casos a seguir:
% -------------------Dúvidas---------------------------------
% O que seria o resultado esperado? X_1 e X_2 nos pontos de operação?
% Como visualizar o resultado? no plano em espaços de estado?
%% a) X_2_op = 5
clear, clc, close all
% X_1_dot_op = 5
% X_2_dot_op = 1*u^2 + 1*sign(X_2)*X_2^2 -> naive search

u_trial = 0.1:0.1:10;
X_2_trial = u_trial;
output = [];
X_2_dot_trim = [];

for X_2 = X_2_trial
    for u = u_trial
        X_2_dot_op = u*u + sign(X_2)*X_2*X_2;

        if (abs(X_2_dot_op-5) <= 1e-5) % Vínculo para o critério ser atendido
            X_2_dot_trim = [X_2_dot_trim X_2_dot_op];
            output = [output; u X_2];
        end
    end
end
X_2_dot_trim

[X_plt, Y_plt] = meshgrid(u_trial,X_2_trial);
figure()
    plot(X_plt,Y_plt, '.b')
    hold on
    plot(output(:,1), output(:,2), '.r', 'MarkerSize', 12)
    xlabel("Uop")
    ylabel("X_{2}op")

%% b) X_1_op = 10, logo:
clear, clc, close all
% X_1_dot_op = 0 -> X_2 = 0
% X_2_dot_op = 1*u^2 + 1*0*0^2 -> naive search
u_trial = 0:0.1:10;
X_2_trial = u_trial;
output = [];
X_2_dot_trim = [];

for X_2 = X_2_trial
    for u = u_trial
        X_2_dot_op = u*u + sign(X_2)*X_2*X_2;

        if (abs(X_2_dot_op-0) <= 1e-5) % Vínculo para o critério ser atendido
            X_2_dot_trim = [X_2_dot_trim X_2_dot_op];
            output = [output; u X_2];
        end
    end
end
X_2_dot_trim

[X_plt, Y_plt] = meshgrid(u_trial,X_2_trial);
figure()
    plot(X_plt,Y_plt, '.b')
    hold on
    plot(output(:,1), output(:,2), '.r', 'MarkerSize', 12)
    xlabel("Uop")
    ylabel("X_{2}op")

%% c) X_1_op = 3*t + 5, logo:
clear, clc, close all
% X_1_dot_op = 3 -> X_2 = 3
% X_2_dot_op = 1*u^2 + 1*1*3^2 -> naive search
u_trial = 0:0.1:10;
X_2_trial = u_trial;
output = [];
X_2_dot_trim = [];

for X_2 = X_2_trial
    for u = u_trial
        X_2_dot_op = u*u + sign(X_2)*X_2*X_2;

        if (abs(X_2_dot_op-3) <= 1e-5) % Vínculo para o critério ser atendido
            X_2_dot_trim = [X_2_dot_trim X_2_dot_op];
            output = [output; u X_2];
        end
    end
end
X_2_dot_trim

[X_plt, Y_plt] = meshgrid(u_trial,X_2_trial);
figure()
    plot(X_plt,Y_plt, '.b')
    hold on
    plot(output(:,1), output(:,2), '.r', 'MarkerSize', 12)
    xlabel("Uop")
    ylabel("X_{2}op")

%% d) U_op = 4, X_2_dot_op = 0, logo:
clear, clc, close all
% X_1_dot_op = 0 -> X_2 = 0
% X_2_dot_op = 1*u^2 + 1*0*0^2 -> naive search (NÃO ENCONTRADO)
u_trial = 0:0.1:10;
X_2_trial = u_trial;
output = [];
X_2_dot_trim = [];

for X_2 = X_2_trial
    for u = u_trial
        X_2_dot_op = u*u + sign(X_2)*X_2*X_2;

        if ((abs(u-4) <= 1e-4) && (abs(X_2_dot_op) <= 1e-4)) % Vínculo para o critério ser atendido
            X_2_dot_trim = [X_2_dot_trim X_2_dot_op];
            output = [output; u X_2];
        end
    end
end
X_2_dot_trim

[X_plt, Y_plt] = meshgrid(u_trial,X_2_trial);
figure()
    plot(X_plt,Y_plt, '.b')
    hold on
    plot(output(:,1), output(:,2), '.r', 'MarkerSize', 12)
    xlabel("Uop")
    ylabel("X_{2}op")






