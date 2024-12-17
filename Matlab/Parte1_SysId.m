close all;
clc;
clear;
warning off

%% Aquisicao dos dados
filePath = 'C:\Users\leocs\OneDrive\Documentos\MATLAB\SEL0328\';  %Colocar caminho dos arquivos .csv

ch1Table = readtable('F0000CH1.csv'); %Corrente
ch2Table = readtable([filePath 'F0000CH2.csv']); %Velocidade
currentAtten = table2array(ch1Table(15,2));
velocityAtten = table2array(ch2Table(15,2));

%Parâmetros iniciais
Ktg = 0.1504; %Valor da constante que relaciona a tensão de saída do tacogerador e a velocidade do eixo.
Rext = 0.47; %Resitor utilizado para medir corrente
Vstep = 12.28; %Degrau de tensão aplicado na entrada.

Ts = table2array(ch1Table(2,2)); %Sample Time, presente no arquivo .csv importado do osciloscópio.

currentValue =  table2array(ch2Table(1:end,5));
currentTime =  table2array(ch2Table(1:end,4));
velocityValue =  table2array(ch1Table(1:end,5));
velocityTime = currentTime;

current = currentAtten*(currentValue/Rext); %A
velocity = velocityAtten*(velocityValue/Ktg); %rad/s

%Tratando tamanho dos vetores para o degrau
Ia = current(973:end);
Omega = velocity(973:end);
Va  = [zeros(0,1);Vstep*ones(length(Omega),1)];
%Adequa tempo às amostras
time = linspace(Ts,Ts*size(Omega,1),size(Omega,1));
time = time';

Wmed=[time Omega];      %Velocidade medida do degrau rad/s
Vamed=[time Va];      %Tensao de armadura Volts
Iamed=[time Ia];      %Corrente de armadura Volts

%% Ident
warning off
%ident velocidade 
motor = iddata;
motor.Tstart = 0;
motor.Ts = Ts;
motor.InputData = Va;
motor.OutputData = Omega;

velocidade1polo = tfest(motor,1,0);
velocidade2polos = tfest(motor,2,0);

[y1, t] = step(time,Vstep*velocidade1polo);
[y2, t] = step(time,Vstep*velocidade2polos);

figure
plot(t,y1);
hold on
plot(t,y2);
plot(time,Omega, Color=[0.4940 0.1840 0.5560]);
xlabel('Tempo (s)')
ylabel('$\omega_m$ (rad/s)')
xlim([0 0.15])
ylim([0 140])
legend('Modelo de 1ª ordem: 91.77%','Modelo de 2ª ordem: 91.94%', 'Experimental','Location','SouthEast')

saveas(gcf, 'ident-velocidade.eps', 'epsc')

%ident corrente
motor.InputData = Va;
motor.OutputData = Ia;

corrente1polo = tfest(motor,1,1);
corrente2polos = tfest(motor,2,1);

[y3, t] = step(time,Vstep*corrente1polo);
[y4, t] = step(time,Vstep*corrente2polos);

figure
plot(t,y3)
hold on
plot(t,y4)
plot(time,Ia, Color=[0.4940 0.1840 0.5560]);
xlim([0 0.15])
ylim([0 4])
xlabel('Tempo (s)')
ylabel('$I_a$ (A)')
legend('Modelo de 1ª ordem: 64.91%','Modelo de 2ª ordem: 94.11%', 'Experimental','Location','NorthEast')

saveas(gcf, 'ident-corrente.eps', 'epsc')

% funções de transferência obtidas
G1 = tf(velocidade1polo);
G2 = tf(velocidade2polos);
G3 = tf(corrente1polo);
G4 = tf(corrente2polos);

save G1
save G2
save G3
save G4

warning on
%% Parameters estimation 

%Chute inicial dos parâmetros do motor
Ra = 0.1501; % Resistência de armadura [Ohm]
La = 0.000150;  % Indutância de armadura [H]
K = 0.087;  % Kt=Ke para este motor: Constante de torque [N-m/A] ou Constante f.c.e.m [V-s/rad]
J = 0.00050804; % Momento de inércia [N-m-s^2/rad]
B = 0.0006; % Coeficiente de atrito viscoso [N-m-s/rad]
Kt = K;
Ke = K;
s = tf('s');
Gm = @(Kt,Ke,Ra,La,J,B,s) Kt/(La*J*s^2+(Ra*J+La*B)*s+(Ra*B+Ke*Kt));
Gm0 = Gm(Kt,Ke,Ra,La,J,B,s);

run('SimulacaoMotorCC_r2016a');

%% Valores obtidos
Ra = 2.8731;
La = 0.0047;
B = 2.3689e-04;
K = 0.0870;
J = 5.5388e-05;
Kt = K;
Ke = K;
run('SimulacaoMotorCC_r2016a');
%% 
GmEstimated = Gm(Kt,Ke,Ra,La,J,B,s);
save GmEstimated

%% Resultados Parameter Estimation

t1 = Iam_out.Time;
y1 = Iam_out.Data(:, 1); 
y2 = Iam_out.Data(:, 2); 

figure
plot(t1, y1)
hold on
plot(t1, y2)
xlabel('Tempo (s)')
ylabel('$I_a$ (A)')
legend('Parameter Estimation', 'Experimental', 'Location', 'NorthEast')
ylim([0 4])
saveas(gcf, 'parameter-corrente.eps', 'epsc') 

t1 = Vel_out1.Time;
y3 = Vel_out1.Data(:, 1);  
y4 = Vel_out1.Data(:, 2);  

figure
plot(t1, y3)
hold on
plot(t1, y4)
xlabel('Tempo (s)')
ylabel('$\omega_m$ (rad/s)')
legend('Parameter Estimation', 'Experimental', 'Location', 'SouthEast')
saveas(gcf, 'parameter-velocidade.eps', 'epsc')  

%% Comparação dos modelos Ident, Parameter Estimation e experimentais

[yid1,tid1] = step(Vstep*G4);

figure
plot(t1, y1) 
hold on
plot(t1, y2)
plot(tid1,yid1)
xlabel('Tempo (s)')
ylabel('$I_a$ (A)')
legend('Parameter Estimation', 'Experimental', 'System Identification')
ylim([0 4])
saveas(gcf, 'comparison-corrente.eps', 'epsc') 

[yid2,tid2] = step(Vstep*G2);

figure
plot(t1, y3) 
hold on
plot(t1, y4)
plot(tid2,yid2)
xlabel('Tempo (s)')
ylabel('$\omega_m$ (rad/s)')
legend('Parameter Estimation', 'Experimental', 'System Identification', 'Location', 'SouthEast')
saveas(gcf, 'comparison-velocidade.eps', 'epsc') 
%% Comparação com os resultados utilizando o Power Systems
figure
plot(t1, y1,LineWidth=1)
hold on
plot(t1, y2,LineWidth=1)
plot(tid1,yid1,LineWidth=1)
plot(t,Ia,LineWidth=1)
xlabel('Tempo (s)')
ylabel('$I_a$ (A)')
legend('Parameter Estimation', 'Experimental','System Identification', ...
    'Power Systems', 'Location', 'NorthEast')
ylim([0 4])
xlim([0 0.15])
saveas(gcf, 'powersystems-corrente.eps', 'epsc') 

figure
plot(t1, y3,LineWidth=1)
hold on
plot(t1, y4,LineWidth=1)
plot(tid2,yid2,LineWidth=1)
plot(t,wm,LineWidth=1)
xlim([0 0.15])
ylim([0 140])
xlabel('Tempo (s)')
ylabel('$\omega_m$ (rad/s)')
legend('Parameter Estimation', 'Experimental', 'System Identification' ,...
    'Power Systems', 'Location', 'SouthEast')
saveas(gcf,'powersystems-velocidade.eps', 'epsc')
%% Resultados utilizando u=1.5V (50%)

T = 1/4e3;
u = ones(size(saw1));

figure
plot(t,u)
hold on
plot(t,saw1)
plot(t,pulse1)
xlim([0 3*T])
legend('u','Dente de Serra','Saída do PWM')
xlabel('Tempo (s)')
ylabel('Tensão (V)')
saveas(gcf,'powersystems-pwm.eps', 'epsc')
