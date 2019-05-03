% ------------------------------------------------------------------------
% University of Maryland, College Park
% ENME 808B - Applied Nonlinear Controls
% Homework #8
% Due November 12, 2018
%
% David A Smart
% November 7, 2018
% ------------------------------------------------------------------------
%% Problem Statement
%
% 2. [Example 8.3] 
%
% Consider the 1st-order system 
% y_dot = ?a_p*y + b_p*u = y + 3*u 
% [a_p = -1, b_p = 3]
%
% and its reference model 
% y_m_dot = -a_m*y_m + b_m*r = ?4*y_m + 4*r
% [a_m = 4, b_m = 4]
% 
% Simulate the system with the MRAC law:
% u = a_r_hat(t)*r + a_y_hat(t)*y
% a_r_hat_dot = ?sign(b_p)*gamma*e*r = ?2*e*r
% a_y_hat_dot = ?sign(b_p)*gamma*e*y = ?2*e*y
%
% Choose a_r_hat(0) = a_y_hat(0) = 0 as well as y(0) = y_m(0)=0.
% (1) r(t) = 4
% (2) r(t) = 4*sin(3*t)
%
% Plot the time responses of y(t) and y_m(t) 
% as well as a_r_hat(t) and a_y_hat(t) 
% in conjunction with a_r_star and a_y_star 
% [a_r_star = b_m / b_p]
% [a_y_star = (-a_m + a_p) / b_p]
% to assess the quality of on-line parameter estimation.
% 
% ------------------------------------------------------------------------
%% clean up
close all
clear
clc

% ------------------------------------------------------------------------
%% initial conditions
% time
t(1)    = 0;

% parameter estimates
a_r_hat_dot(1) = 0;
a_y_hat_dot(1) = 0;
a_r_hat(1) = 0;
a_y_hat(1) = 0;

% state
y_dot(1) = 0;
y_m_dot(1) = 0;
y(1) = 0;
y_m(1) = 0;

% error
e(1) = 0;

% input & control
r(1) = 0;
u(1) = 0;

%% optimal parameters
% a_p = -1, b_p = 3
% a_m = 4, b_m = 4
% a_r_star = b_m / b_p;
% a_y_star = (-a_m + a_p) / b_p;
a_r_star = 4/3;
a_y_star = -5/3;

% ------------------------------------------------------------------------
%% time-loop

% simulation time-step
dt  = 1/6000;

for i = 2:(10/dt)
    
% time update
t(i) = t(i-1) + dt;
    
% update system dynamics
y_dot(i) = y(i-1) + 3*u(i-1);
y(i) = y(i-1) + y_dot(i)*dt;

% update reference model 
% r(i) = 4;
r(i) = 4*sin(3*t(i));
y_m_dot(i) = -4*y_m(i-1) + 4*r(i);
y_m(i) = y_m(i-1) + y_m_dot(i)*dt;

% update error
e(i) = y(i) - y_m(i);

% update estimated parameters
a_r_hat_dot(i) = -2*e(i)*r(i);
a_r_hat(i) = a_r_hat(i-1) + a_r_hat_dot(i)*dt;
a_y_hat_dot(i) = -2*e(i)*y(i);
a_y_hat(i) = a_y_hat(i-1) + a_y_hat_dot(i)*dt;

% update control input
u(i) = a_r_hat(i)*r(i) + a_y_hat(i)*y(i);  
    
end

%% eval final parameter offsets
a_r_hat(end) - a_r_star
a_y_hat(end) - a_y_star

% -------------------------------------------------------------------------
%% Display

figure(1)
set(gcf,'Units','normalized','Position',[0 0.2 1 0.5]); % large

subplot(1,3,1);
hold on
plot(t, y,'r', 'LineWidth',2);
plot(t, y_m,'--k', 'LineWidth',2);
xlabel('time (s)')
ylabel('y')
title('System Responce: y')
legend('y', 'y-hat','Location','Best')
grid on

subplot(1,3,2);
hold on
plot(t, a_r_hat,'r', 'LineWidth',2);
plot(t, ones(size(t,2),1)*a_r_star,'--k', 'LineWidth',2);
xlabel('time (s)')
ylabel('a_r')
title('Parameter Estimation: a_r')
legend('a_r-hat', 'a_r','Location','Best')
grid on

subplot(1,3,3);
hold on
plot(t, a_y_hat,'r', 'LineWidth',2);
plot(t, ones(size(t,2),1)*a_y_star,'--k', 'LineWidth',2);
xlabel('time (s)')
ylabel('a_y')
title('Parameter Estimation: a_y')
legend('a_y-hat', 'a_y','Location','Best')
grid on

%% ------------------------------------------------------------------------