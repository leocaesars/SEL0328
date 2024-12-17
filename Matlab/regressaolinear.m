% Dados
x = [2.955 2.738 2.465 2.195 1.88];
y = [10.420, 9.980, 9.110, 8.11, 6.9];

linearModel = @(a, x) a*x;

% Estimativa inicial do coeficiente a
a0 = 1;

% Ajuste dos parâmetros usando lsqcurvefit
a_est = lsqcurvefit(linearModel, a0, x, y);

x1 = linspace(0, 4, 50);
y_fit = linearModel(a_est, x1);

% Gráfico
figure;
scatter(x, y, 'filled', 'DisplayName', 'Dados'); % Dados originais
hold on;
plot(x1, y_fit, 'DisplayName', 'Ajuste linear');
legend('Dados', 'Ajuste Linear', FontSize=20);
xlabel('$u(t)$', FontSize=20);
ylabel('$V_m$', FontSize=20);

disp('Parâmetro ajustado (a):');
disp(a_est);

saveas(gcf,'ajustelinear.eps', 'epsc')

Ka = a_est;
save Ka
