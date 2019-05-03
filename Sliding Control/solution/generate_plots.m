
clear; clc;
figure;

sim('sliding_model'); % --------------------------------------------------

subplot(2,2,1);
plot(u(:,1),u(:,2),'k-','LineWidth',2);
grid on;
xlabel('time (s)');
ylabel('control input');
axis([0 4 -4 6])

subplot(2,2,2);
plot(e(:,1),e(:,2),'k-','LineWidth',2);
grid on;
xlabel('time (s)');
ylabel('tracking error');
axis([0 4 -5e-5 5e-5]);

sim('sliding_model_smooth'); % -------------------------------------------

subplot(2,2,3);
plot(u(:,1),u(:,2),'k-','LineWidth',2);
grid on;
xlabel('time (s)');
ylabel('control input');
axis([0 4 -4 6])

subplot(2,2,4);
plot(e(:,1),e(:,2),'k-','LineWidth',2);
grid on;
xlabel('time (s)');
ylabel('tracking error');
axis([0 4 -4e-3 4e-3]);

