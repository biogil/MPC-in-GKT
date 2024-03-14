close;
pid_data = dlmread('output_pid.txt');
mpc_data = dlmread('output_mpc.txt');
pid_ConT = pid_data(:,1);
pid_ConP = pid_data(:,2);
pid_P = pid_data(:,3);
mpc_ConT = mpc_data(:,1);
mpc_ConP = mpc_data(:,2);
mpc_P = mpc_data(:,3);
subplot(311);
plot(1:50, pid_ConT(1:50),'--k', 'linewidth',2);
hold on;
set(0,'defaultfigurecolor','w');
set(gca,'XMinorTick',true)
set(gca,'YMinorTick',true)
grid on     % 添加网格线
ax = gca;   % 将当前坐标区实例化
ax.GridLineStyle = '--';    % 设置网格线样式
ax.GridColor = [0.5,0.5,0.5];   % 设置颜色
ax.GridAlpha = 0.5; % 设置透明度
plot(1:50, mpc_ConT(1:50),'-k', 'linewidth',2);
legend("PID", "MPC",'FontSize', 15);
xlabel("进口总温",'FontSize', 20);
ylabel("K",'FontSize', 20);
subplot(312);
plot(1:50, pid_ConP(1:50),'--k', 'linewidth',2);
hold on;set(gca,'XMinorTick',true)
set(gca,'YMinorTick',true)
grid on     % 添加网格线
ax = gca;   % 将当前坐标区实例化
ax.GridLineStyle = '--';    % 设置网格线样式
ax.GridColor = [0.5,0.5,0.5];   % 设置颜色
ax.GridAlpha = 0.5; % 设置透明度
plot(1:50, mpc_ConP(1:50),'-k', 'linewidth',2);
xlabel("进口总压",'FontSize', 20);
ylabel("Pa",'FontSize', 20);
subplot(313);
plot(1:50, pid_P(1:50),'--k', 'linewidth',2);
hold on;
set(gca,'XMinorTick',true)
set(gca,'YMinorTick',true)
grid on     % 添加网格线
ax = gca;   % 将当前坐标区实例化
ax.GridLineStyle = '--';    % 设置网格线样式
ax.GridColor = [0.5,0.5,0.5];   % 设置颜色
ax.GridAlpha = 0.5; % 设置透明度
plot(1:50, mpc_P(1:50),'-k', 'linewidth',2);
xlabel("排气静压",'FontSize', 20);
ylabel("Pa",'FontSize', 20);


