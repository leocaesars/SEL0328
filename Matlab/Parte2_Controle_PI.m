%%
clear all
close all
clc

load GmEstimated.mat
%% definicao dos valores de Mp e ts
zeta = 0.6;
Mp = exp((-pi*zeta)/sqrt(1-zeta^2)); % Mp = 9.48%
ts = 0.5*stepinfo(GmEstimated).SettlingTime; % 50% do ts da função estimada. ts = 0.0709/2 = 0.0355
Ka = 3.63;
%% inicializacao do rltool
F = tf(100,1);
G = tf(Ka*GmEstimated);
H = tf(1,1);
%%
rltool

%% funcao de transf do controlador PI
Ctf = tf(C)
save Ctf

%% tf em malha fechada do sistema com o controlador PI

Gma = Ctf*G;
Gmf = feedback(Gma,H);
%%
%infos para entrada degrau
stepinfo(Gmf)

%plot para entrada degrau para referencia de 100 rad/s
[y,t] = step(100*Gmf);
figure()
plot(t,y)
xlabel('Tempo (s)', FontSize=20)
xlim([0 0.14])
ylabel('$\omega_m$', FontSize=20)

saveas(gcf, 'degrauPI.eps', 'epsc')

%% simulacao do sistema em malha fechada com o controlador PI
%run(MotorCC_chopperPWM)

figure()
plot(tout,wm)
xlabel('Tempo (s)', FontSize=20);
ylabel('$\omega_m ~(rad/s)$', FontSize=20)

saveas(gcf, 'simulacao-PI.eps', 'epsc')

figure()
subplot(1,2,1)
plot(tout,u1)
xlabel('Tempo (s)', FontSize=20)
ylabel('Sinal de controle $u(s)$', FontSize=20)

subplot(1,2,2)
plot(tout,erro)
xlabel('Tempo (s)', FontSize=20)
ylabel('Erro $e(s)$', FontSize=20)

saveas(gcf, 'erro-controle.eps', 'epsc')

%% discretizacao do controlador PI

Cd = c2d(Ctf,1e-3,'zoh')
Gd = c2d(Gmf, 1e-3, 'zoh')

[y, t] = step(100*Gmf);
[y1, t1] = step(100*Gd);

figure
plot(t,y, LineWidth=1);
hold on
stairs(t1,y1,LineWidth=1);
xlabel('Tempo (s)', FontSize=20)
ylabel('$\omega_m ~(rad/s)$', FontSize=20)
legend('Contínuo', 'Discreto', FontSize=20)
xlim([0 0.14])

saveas(gcf, 'c2d.eps', 'epsc')

%eq a diferencas
%u(k) = u(k-1)+0.01273e(k)-0.01126e(k-1)

%% Dados coletados

filePath = 'C:\Users\lucas\OneDrive\Documentos\Codes_MATLAB\Lab_Sistemas_Lin';
Medidas = readtable('Medidas.csv');

tempo = linspace(0,(9981-2388)*1e-3,9981-2387);

duty = table2array(Medidas(1:end,2));
duty = duty(2388:end)';

errotab = table2array(Medidas(1:end,3));
errotab = errotab(2388:end)';

ref = table2array(Medidas(1:end,4));
ref = ref(2388:end)';

u = table2array(Medidas(1:end,5));
u = u(2388:end)';

velocidade = table2array(Medidas(1:end,6));
velocidade = velocidade(2388:end)';

%% Gráficos

%media movel utilizando os 10 ultimos pontos para suavizar o sinal
e0_filtro = movmean(errotab, 10);
duty_filtro = movmean(duty, 10);
ref_filtro = movmean(ref,10);
u0_filtro = movmean(u, 10);
vel_filtro = movmean(velocidade, 10);

figure()
plot(tempo,vel_filtro,'Color',"#7E2F8E");
hold on
plot(tout,wm)
ylabel('$\omega (rad/s)$', Fontsize=20)
xlabel('Tempo(s)',Fontsize=20)
legend('Experimental', 'Simulação','Location','Southeast',Fontsize=20)
xlim([0 0.5])
saveas(gcf, 'vel_comp.eps', 'epsc');

figure()
plot(tout,u1)
hold on
plot(tempo,u0_filtro);
xlabel('Tempo (s)', FontSize=20)
ylabel('$u(t)$', FontSize=20)
xlim([0 0.5])
legend('Simulação', 'Experimental', 'Location','southeast',FontSize=20)
saveas(gcf,'u_comp.eps','epsc')

figure()
plot(tout,erro)
hold on
plot(tempo,e0_filtro);
xlabel('Tempo (s)', FontSize=20)
ylabel('$e(t)$', FontSize=20)
xlim([0 0.5])
legend('Simulação', 'Experimental', 'Location','northeast',FontSize=20)
saveas(gcf,'e_comp.eps','epsc')




