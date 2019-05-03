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
% Consider the nonlinear system:
%
%   m(x_dotdot) = u
%   where m is an unknown parameter
%
% An adaptive control law for this system is given by: 
%
% u = m_hat(v)
% m_hat_dot = -gamma*v*s
%
% where 
% gamma > 0
% v = x_m_dotdot - 2*lambda*x_tilda_dot - lambda^2*x_tilda
% s = x_tilda_dot + lambda*x_tilda
%
% and xm is the ideal response as the output of the following reference model: 
%
% x_m_dotdot + lambda_1*x_m_dot + lambda_2*x_m = lambda_2*r(t)
%
% where lambda_1, lambda_2 > 0
%
% For the sake of simulation, let m = 2
% (1) Simulate the adaptive control system when     r(t) = 0        and     x(0) = 0.5
% (2) Simulate the adaptive control system when     r(t) = sin(4t)  and     x(0) = 0
% ------------------------------------------------------------------------
%% clean up
close all
clear
clc

% ------------------------------------------------------------------------
%% setup

% true paramater value
m   = 2;

% estimater parameters > 0
gamma       = 1;

% controller parameters > 0
lambda      = 8;

% refrence model paramaters > 0
RT          = 1/4;                                  % rise time
POS         = 0.01;                                 % percent overshoot
zeta        = -log(POS)/sqrt(pi^2 + log(POS)^2);    % damping ratio
lambda_2    = round((1.8/RT)^2,1);                  % "stifness"
lambda_1    = round(2*sqrt(lambda_2)*zeta,1);       % "damping"

% simulation time-step
dt  = 1/1000;

%% initial conditions
% time
t(1)    = 0;
tt = 0;

% state
x(1)            = 0;
x_dot(1)        = 0;
x_dotdot(1)     = 0;

% desired trajectory
noise(1)        = 0;
r(1)            = 0;
x_m_dotdot(1)   = 0;
x_m_dot(1)      = 0;
x_m(1)          = 0;

% error
x_tilda(1)          = 0;
x_tilda_dot(1)      = 0;
x_tilda_dotdot(1)   = 0;

% control input
s(1) = 0;
v(1) = 0;
u(1) = 0;

% parameter estimate
m_hat_dot(1)    = 1e-6;
m_hat(1)        = m_hat_dot(1)*dt;
 
% ------------------------------------------------------------------------
%% time-loop
for i = 2:(3/dt)
    
    % time
    t(i)    = t(i-1) + dt;

    % state
    x_dotdot(i) = u(i-1)/m(i-1);
    x_dot(i)    = x_dot(i-1) + x_dotdot(i)*dt;
    x(i)        = x(i-1) + x_dot(i)*dt;
    noise(i)    = x(i)*rand(1)*0.005*(-1)^randi(2);
    x(i)        = x(i) + noise(i); % changed from adding to r(t) after speaking with proffessor about real world application

    % desired trajectory
    r(i)            = sin(4*t(i));
    x_m_dotdot(i)   = lambda_2*r(i) - lambda_1*x_m_dot(i-1) - lambda_2*x_m(i-1);
    x_m_dot(i)      = x_m_dot(i-1) + x_m_dotdot(i)*dt;
    x_m(i)          = x_m(i-1) + x_m_dot(i)*dt;

    % error
    x_tilda(i)      = x(i) - x_m(i);
    x_tilda_dot(i)  = (x_tilda(i) - x_tilda(i-1))/dt;

    % control input
    s(i)    = x_tilda_dot(i) + lambda*x_tilda(i);
    v(i)    = x_m_dotdot(i) - 2*lambda*x_tilda_dot(i) - lambda^2*x_tilda(i);
    u(i)    = m_hat(i-1)*v(i);
    
%     % parameter estimate
    m_hat_dot(i)    = -gamma*v(i)*s(i);
    m_hat(i)        = m_hat(i-1) + m_hat_dot(i)*dt;
    
    % change m
    if abs(t(i) - 6) <= dt
        m(i) = 10;
    elseif t(i) > 12 && t(i) < 18
        m(i) = m(i-1) - 1*dt;
    elseif t(i) > 24 && t(i) < 30
        tt = tt + (pi)*dt/2;
        m(i) = 4 + cos(tt); 
    else
        m(i) = m(i-1);
    end

end
% -------------------------------------------------------------------------
%% Display
figure(1)
set(gcf,'Units','normalized','Position',[0.2 0.2 0.7 0.5]); % large

subplot(1,2,2);
hold on
plot(t, m_hat,'r', 'LineWidth', 2);
plot(t, m, '--k', 'LineWidth', 2);
xlabel('time (s)')
ylabel('m-hat (kg)')
title('Parameter Estimate')
legend('m-hat','m')
grid on

subplot(1,2,1);
hold on
plot(t, x, 'b', 'LineWidth', 2);
plot(t, x_m,'--k', 'LineWidth', 2);
xlabel('time (s)')
ylabel('x (m)')
title('Tracking Performance')
legend('m','x_m')
grid on

figure(3);
hold on
plot(t, noise, 'c', 'LineWidth', 2);
plot(t, zeros(length(t)),'--k', 'LineWidth', 2);
xlabel('time (s)')
ylabel('noise (m)')
grid on
%% ------------------------------------------------------------------------