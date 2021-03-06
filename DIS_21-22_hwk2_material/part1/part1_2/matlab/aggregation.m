%% Usual cleaning
clc
close all
clear all

% Initialize parameters
Nr = 5; % Total number of robots
Ns = 10; % Total number of seeds
Wsf0 = Nr; % Number of robots in search-free state
Wsl0 = 0; % Number of robots in search-loaded state
Wr0 = 0; % Number of robots in releasing state
Wg0 = 0; % Number of robots in gripping state
Wof0 = 0; % Number of robots in obstacle avoidance-free state
Wol0 = 0; % Number of robots in obstacle avoidance-floaded state
k_sim = 10080; % Total simulation time
vr = 0.08; %average robot speed, m/s
rs = 0.1; %detection radius of the distance sensor
ds = 2*rs; %detection distance of the sensor
da = 1; % edge of the arena
Aa = da^2; % Area of the arena, m^2
Ts = 1; % Sample time, s
Trg = 2; % Releasing and gripping delay, s
Toa = 3; % Obstacle avoidance delay, s
alpha_inc =@(n) (n>=2)*60 + (n==1)*180; % Construction angle
alpha_dec =@(n) (n>=2)*60 + (n==1)*180; % Destruction angle
Td_max = max(Trg,Toa)+1; % Simulation start
p_thr_sbc = 0.1; % Probability threshold for size of the biggest cluster
dr=0.07;
seed_diameter=0.037;
% Allocate variables
Wsf = zeros(k_sim,1);
Wsl = zeros(k_sim,1);
Wr = zeros(k_sim,1);
Wg = zeros(k_sim,1);
Wof = zeros(k_sim,1);
Wol = zeros(k_sim,1);
Ncn = zeros(k_sim,Ns+1,1); % Number of clusters of size n (max Ns)
NC = zeros(k_sim,1); % Number of clusters 
ACS = zeros(k_sim,1); % Average size of the clusters
SBC = zeros(k_sim,1); % Size of the biggest cluster
SC = zeros(k_sim,1); % Sanity check
p_dec =@(k,n,Ncn) Ts*(alpha_dec(n)/180).*(Ncn(k,n)*vr*ds/Aa) ;
p_inc =@(k,n,Ncn) Ts*(alpha_inc(n)/180).*(Ncn(k,n)*vr*ds/Aa) ;
p_1 =@(k,Ncn) dot(p_dec(k,(1:Ns),Ncn),Ncn(k,1:Ns,1));
p_2 =@(k,Ncn) dot(p_inc(k,(1:Ns),Ncn),Ncn(k,1:Ns,1));
p_3 = (vr*Ts/(da-2*rs)) + (Nr-1)*(2*rs+dr)*vr/Aa + (2*Ns*(2*rs+seed_diameter)*vr)/(3*Aa);
p_4 = p_3; 

% Initialize variables
Wsf(Td_max) = Wsf0;
Wsl(Td_max) = Wsl0;
Wr(Td_max) = Wr0;
Wg(Td_max) = Wg0;
Wol(Td_max) = Wol0;
Wof(Td_max) = Wof0;
Ncn(1:Td_max,1) = Ns;
NC(Td_max) = 0;
ACS(Td_max) = 0;
SBC(Td_max) = 0;

% Perform the simulation
for k = Td_max:k_sim
    
    Ncn(k+1,1) = Ncn(k,1) + Wsf(k-Trg)*(p_dec(k-Trg,1+1,Ncn)*Ncn(k-Trg,1+1) - p_dec(k-Trg,1,Ncn)*Ncn(k-Trg,1)) + ...
               Wsl(k-Trg)*(0.0 - p_inc(k-Trg,1,Ncn)*Ncn(k-Trg,1)); 
    for n=2:Ns-1
           Ncn(k+1,n) = Ncn(k,n) + Wsf(k-Trg)*(p_dec(k-Trg,n+1,Ncn)*Ncn(k-Trg,n+1) - p_dec(k-Trg,n,Ncn)*Ncn(k-Trg,n)) + ...
               Wsl(k-Trg)*(p_inc(k-Trg,n-1,Ncn)*Ncn(k-Trg,n-1) - p_inc(k-Trg,n,Ncn)*Ncn(k-Trg,n));                 
    end
    n=Ns;
    Ncn(k+1,n) = Ncn(k,n) + Wsf(k-Trg)*(0.0 - p_dec(k-Trg,n,Ncn)*Ncn(k-Trg,n)) + ...
               Wsl(k-Trg)*(p_inc(k-Trg,n-1,Ncn)*Ncn(k-Trg,n-1) - 0.0); 
    %--
    Wof(k+1) = Wof(k) + p_3*Wsf(k)    - p_3*Wsf(k-Toa)*(k-Toa>0);
    Wol(k+1) = Wol(k) + p_4*Wsl(k)    - p_4*Wsl(k-Toa)*(k-Toa>0);
    Wr(k+1)  = Wr(k)  + p_2(k,Ncn)*Wsl(k) - p_2(k-Trg,Ncn)*Wsl(k-Trg);
    Wg(k+1)  = Wg(k)  + p_1(k,Ncn)*Wsf(k) - p_1(k-Trg,Ncn)*Wsf(k-Trg);
    Wsl(k+1) = Wsl(k)*(1-p_2(k,Ncn)-p_4) + p_1(k-Trg,Ncn)*Wsf(k-Trg)+ p_4*Wsl(k-Toa);
    Wsf(k+1) = Wsf(k)*(1-p_1(k,Ncn)-p_3) + p_2(k-Trg,Ncn)*Wsl(k-Trg)+ p_3*Wsf(k-Toa);
    
    %disp('-------'); disp(k);
    %[Wof(k), Wol(k), Wr(k), Wg(k),Wsl(k),Wsf(k)]
    %[Wof(k+1), Wol(k+1), Wr(k+1), Wg(k+1),Wsl(k+1),Wsf(k+1)]
    
    %--
    NC(k)  = sum(Ncn(k,:));
    ACS(k) = dot(Ncn(k,:),(1:Ns+1))/NC(k);
    [a,b] = max(Ncn(k,:).*(Ncn(k,:)>p_thr_sbc));
    SBC(k) = b;
    
                    
end

%% Plot number of robots in each state
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
