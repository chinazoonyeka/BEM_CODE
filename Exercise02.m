% -----------------------------
% Script: Compares a wind step simulation of the SLOW model with self-made 
% BEM to a FAST simulation
% Exercise 02 of Master Course "Introduction to Wind Turbine Aerodynamics"
% ------------
% Task:
% Implement the BEM code into Aerodynamics_BEM and then run it by changing
% Parameter.AeroMode from 'AD' to 'BEM'. Results should be similar to FAST.
% ------------
% Modified: 
% - David Schlipf on 03-May-2020:
%   adjust NREL5MWParameterSLOW and IC for better agreement 
% - David Schlipf on 17-May-2019:
%   * Preprocessing for SLOW
%   * Add Actuator Disk (AD) simulation via Euler forward as an example
%   * Simulate FAST only if not done before
%   * Read FAST results with textscan instead of importdata (more robust)
% ------------
% Created: 
% David Schlipf on 05-May-2019
% ----------------------------------

clearvars;clc;

%% Preprocessing SLOW
% Define parameter (turbine and controller)
Parameter                       = NREL5MWParameterSLOW;  
Parameter.VSC.k                 = 2.3323;   % Variable Speed Controller parameter for Reagion 2 from FBNREL_discon.in

% Aerodynamics
Parameter.AeroMode              = 'BEM'; 	% 'AD' (already implemented) or 'BEM' (needs to be implemented)
Parameter.AD                    = load('PowerAndThrustCoefficientRegion2NREL5MW','c_P','c_T','lambda'); % Actuator Disc Parameter Region 2
Parameter.BEM                   = ReadAirfoilsNREL5MW;

% Define simulation details (same as FAST)
dt          = 0.0125;           % time step                 [s]
TMax        = 60;           	% maximum time              [s]
t           = 0:dt:TMax;        % time vector
n           = length(t);        % number of simulation steps

% Define disturbance (wind signal)
v_0         = interp1([0 30 30+dt 120],[8 8 9 9],t);       % rotor effective wind speed at every simulation step

%% Processing SLOW
% initial values (from FAST simulation)
x           = NaN(n,3);         % allocation of state vector
x(1,1)      = 9.196/60*2*pi;    % rotor speed               [rad/s]
x(1,2)      = 0.199;            % tower top position        [m]
x(1,3)      = 0.000;            % tower top velocity        [m/s]

% simulation using Euler forward
tic
for k = 1:1:n-1
    x_dot_k = SLOW(x(k,:),v_0(k),Parameter);
    x(k+1,:)= x(k,:)+dt*x_dot_k;
end
TimeRatio   = TMax/toc;

%% PostProcessing SLOW
Omega       = x(:,1);         	% rotor speed is 1st state
x_T         = x(:,2);          	% tower top position is 2nd state

%% Processing FAST
OutputFile  = 'WindStep_08.out';
if ~exist(OutputFile,'file') % only run FAST if out file does not exist
    dos('FAST_Win32.exe WindStep_08.fst');
end

%% PostProcessing FAST
fid         = fopen(OutputFile);
formatSpec  = repmat('%f',1,10);
FASTResults = textscan(fid,formatSpec,'HeaderLines',8);
Time        = FASTResults{:,1};
Wind1VelX   = FASTResults{:,2};
RotSpeed    = FASTResults{:,4};
TTDspFA     = FASTResults{:,5};
fclose(fid);

%% Compare Results
fprintf('Time Ratio SLOW (Sim/CPU): %f\n',TimeRatio)

figure

% plot wind
subplot(311)
hold on;box on;grid on;
plot(Time,Wind1VelX)
plot(t,v_0)
ylabel('wind speed [m/s]')

% plot rotor speed
subplot(312)
hold on;box on;grid on;
plot(Time,RotSpeed)
plot(t,Omega/2/pi*60) % rad/s to rpm
ylabel('rotor speed [rpm]')
legend({'FAST','SLOW'},'location','best')

% plot tower top position
subplot(313)
hold on;box on;grid on;
plot(Time,TTDspFA)
plot(t,x_T)
xlabel('time [s]')
ylabel('tower top position [m]')