% Usual cleaning
clc
close all
clear all

% Initialize parameters
Nr = 5; % Total number of robots
Ns = 5; % Total number of seeds
Wsf0 = Nr; % Number of robots in search-free state
Wsl0 = 0; % Number of robots in search-loaded state
Wr0 = 0; % Number of robots in releasing state
Wg0 = 0; % Number of robots in gripping state
Wof0 = 0; % Number of robots in obstacle avoidance-free state
Wol0 = 0; % Number of robots in obstacle avoidance-floaded state
k_sim = 1000; % Total simulation time
vr = 0.08; %average robot speed, m/s
rs = 0.1; %detection radius of the distance sensor
ds = 0.06; %detection distance of the sensor
da = 1; % edge of the arena
Aa = da^2; % Area of the arena, m^2
Ts = 1; % Sample time, s
Trg = 2; % Releasing and gripping delay, s
Toa = 3; % Obstacle avoidance delay, s
alpha_inc = 60; % Construction angle
alpha_dec = 60; % Destruction angle

% Allocate variables
Wsf = zeros(k_sim,1);
Wsl = zeros(k_sim,1);
Wr = zeros(k_sim,1);
Wg = zeros(k_sim,1);
Wof = zeros(k_sim,1);
Wol = zeros(k_sim,1);
Ncn = zeros(k_sim,Ns+1,1); % Number of clusters of size n (max Ns)
Nc = zeros(k_sim,1); % Number of clusters NC
Nac = zeros(k_sim,1); % Average cluster size ACS
NC = zeros(k_sim,1);
ACS = zeros(k_sim,1);
SBC = zeros(k_sim,1);

Td_max = max(Trg,Toa)+1;

% Initialize variables
Wsf(Td_max) = Wsf0;
Wsl(Td_max) = Wsl0;
Wr(Td_max) = Wr0;
Wg(Td_max) = Wg0;
Wol(Td_max) = Wol0;
Wof(Td_max) = Wof0;
Ncn(1:Td_max,1) = Ns;
Nac(1:Td_max) = 1;
NC(Td_max) = 0;
ACS(Td_max) = 0;
SBC(Td_max) = 0;

% Perform the simulation
for k = Td_max:k_sim
%TODO
                    
end

%%
% Plot number of robots in each state
figure
plot(Wsf,'LineWidth',2)
hold on
grid on
plot(Wsl,'LineWidth',2)
hold on
plot(Wg,'LineWidth',2)
hold on
plot(Wr,'LineWidth',2)
hold on
plot(Wof,'LineWidth',2)
hold on
plot(Wol,'LineWidth',2)
hold on
plot(Wsf + Wsl + Wg + Wr + Wof + Wol, 'LineWidth',2);
legend('Wsf','Wsl','Wg','Wr','Wof','Wol','Wtot');
xlabel('Discrete time (k)');
ylabel('Number');

% Plot metrics
figure
plot(NC,'LineWidth',2)
hold on
grid on
plot(ACS,'LineWidth',2)
hold on
plot(SBC,'LineWidth',2)
legend('NC','ACS','SBC');
xlabel('Discrete time (k)');
ylabel('Number');



