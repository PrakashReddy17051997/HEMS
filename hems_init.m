%% ========================================================================
%  hems_init.m
%  Initialization script for the HEMS Simulink model
%  Run this BEFORE opening / simulating hems_model.slx
%
%  It loads every parameter into the base workspace and builds the
%  forecast and load profiles for one 24-hour day at 15-minute resolution.
%
%  Project: Forecast-based optimization for a Home Energy Management System
%  ========================================================================

clear; clc; close all;

%% ------------------------------------------------------------------------
%  1. TIME GRID
%  ------------------------------------------------------------------------
dt       = 900;                 % step size [s]  (15 min = 900 s)
dt_h     = dt/3600;             % step size [h]  (0.25 h)
N        = 96;                  % steps in a 24 h day  (24/0.25)
t_s      = (0:dt:(N-1)*dt)';    % time vector [s]   column, length N
t_h      = t_s/3600;            % time vector [h]
k        = (1:N)';              % integer step index 1..96

% Helper: convert "clock hour" to a step index (e.g. 7:00 -> step 29)
hourIdx  = @(hr) round(hr/dt_h) + 1;

%% ------------------------------------------------------------------------
%  2. BUILDING ENVELOPE  (1R1C thermal model)   -- German single-family EFH
%  ------------------------------------------------------------------------
H_T       = 128;                % total heat-loss coefficient [W/K]
C_bldg    = 8000 * 3600;        % thermal mass [J/K]  (8 kWh/K x 3600)
A_win     = 15;                 % south window area [m^2]
g_val     = 0.55;               % glazing solar transmittance g-value [-]
Q_int     = 600;                % internal gains (people + devices) [W]
T_in0     = 20;                 % initial indoor temperature [degC]
T_set     = 21;                 % comfort set point [degC]
T_min     = 19.0;               % lower comfort bound [degC]
T_max     = 24.0;               % upper comfort bound [degC]

% Building time constant: tau = C/H_T
tau_bldg_h = C_bldg / H_T / 3600;     % [hours]  expect ~62 h

%% ------------------------------------------------------------------------
%  3. DHW TANK  (second thermal state)
%  ------------------------------------------------------------------------
V_tank      = 0.300;            % tank volume [m^3]  (300 litres)
rho_w       = 1000;             % water density [kg/m^3]
c_w         = 4186;             % water specific heat [J/(kg.K)]
C_tank      = V_tank*rho_w*c_w; % tank thermal mass [J/K]  (= 1,255,800)
UA_tank     = 2.5;              % standby loss coefficient [W/K]
T_tank0     = 55;               % initial tank temperature [degC]
T_tank_min  = 40;               % minimum usable temperature [degC]
T_tank_max  = 60;               % maximum temperature [degC]
T_legionella= 55;               % periodic anti-Legionella target [degC]

%% ------------------------------------------------------------------------
%  4. HEAT PUMP
%  ------------------------------------------------------------------------
P_hp_max  = 4000;               % max electrical input power [W]
T_sup     = 40;                 % supply water temperature (UFH) [degC]
eta_hp    = 0.45;               % Carnot efficiency factor [-]
% NB: thermal output Q_hp = P_el * COP(T_out, T_sup, eta_hp)

%% ------------------------------------------------------------------------
%  5. PV ARRAY
%  ------------------------------------------------------------------------
pv_peak   = 8000;               % PV system peak power [W]  (8 kWp)
% PV output is built from the irradiance profile below (Section 10).

%% ------------------------------------------------------------------------
%  6. BATTERY STORAGE SYSTEM
%  ------------------------------------------------------------------------
bat_cap     = 10000;            % usable capacity [Wh]   (10 kWh)
bat_soc0    = 5000;             % initial state of charge [Wh] (50 %)
bat_pmax    = 5000;             % max charge / discharge power [W]
bat_eff     = 0.95;            % round-trip efficiency [-]
bat_eff_1way= sqrt(bat_eff);    % one-way efficiency [-]
bat_soc_min = 0.10 * bat_cap;   % lower SoC limit [Wh]
bat_soc_max = 0.95 * bat_cap;   % upper SoC limit [Wh]

%% ------------------------------------------------------------------------
%  7. ESSENTIAL LOADS  (Type 1: fixed power profile, cannot be shifted)
%  ------------------------------------------------------------------------
%  Built as a single combined profile P_ess [W] over the 96 steps.
P_ess = zeros(N,1);

% 7a. Continuous base load: fridge, router, standby electronics
P_ess = P_ess + 200;                                   % 200 W all day

% 7b. Lighting (morning + evening)
P_ess(t_h>=6  & t_h<8)   = P_ess(t_h>=6  & t_h<8)   + 150;
P_ess(t_h>=18 & t_h<23)  = P_ess(t_h>=18 & t_h<23)  + 200;

% 7c. Cooking stove (short peaks at meal times)
P_ess(t_h>=7   & t_h<7.5)  = P_ess(t_h>=7   & t_h<7.5)  + 2000;  % breakfast
P_ess(t_h>=12  & t_h<12.5) = P_ess(t_h>=12  & t_h<12.5) + 1500;  % lunch
P_ess(t_h>=18.5& t_h<19.5) = P_ess(t_h>=18.5& t_h<19.5)+ 2500;   % dinner

% 7d. TV / monitors (evening)
P_ess(t_h>=18 & t_h<23)  = P_ess(t_h>=18 & t_h<23)  + 150;

% 7e. Laptop + phone charging (evening)
P_ess(t_h>=19 & t_h<23)  = P_ess(t_h>=19 & t_h<23)  + 100;

%% ------------------------------------------------------------------------
%  8. SHIFTABLE LOADS  (Type 2: schedulable on/off / energy target)
%  ------------------------------------------------------------------------
% 8a. Washing machine -- fixed cycle, run before a deadline
wm_power    = 1500;             % power while running [W]
wm_duration = 4;                % cycle length [steps] (4 x 15min = 1 h)
wm_deadline = hourIdx(22);      % must finish by 22:00 [step index]
wm_release  = hourIdx(8);       % may start from 08:00 [step index]

% 8b. Clothes dryer -- fixed cycle, run before a deadline
dr_power    = 2500;             % power while running [W]
dr_duration = 4;                % cycle length [steps] (1 h)
dr_deadline = hourIdx(23);      % must finish by 23:00
dr_release  = hourIdx(9);       % may start from 09:00

% 8c. EV charging -- energy target, reach SoC before departure
ev_pmax     = 11000;            % max charge power [W]   (11 kW wallbox)
ev_energy   = 30000;            % energy needed before departure [Wh]
ev_arrive   = hourIdx(18);      % plugged in at 18:00
ev_depart   = hourIdx(7) + N;   % leaves 07:00 next day (wraps midnight)
% (For a single-day sim, treat departure as end-of-horizon at 07:00.)
%% ------------------------------------------------------------------------
%  8b. DUMB-BASELINE COMMAND PROFILES (temporary, until optimizer built)
%      Appliances run on arrival in the evening peak — worst case.
%  ------------------------------------------------------------------------
% Washing machine: runs 18:00-19:00 (gate = 1 during the cycle)
gate_wm_vec = zeros(N,1);
gate_wm_vec(t_h>=18 & t_h<19) = 1;

% Dryer: runs 19:00-20:00 (after the washer)
gate_dr_vec = zeros(N,1);
gate_dr_vec(t_h>=19 & t_h<20) = 1;

% EV: charges at full power from 18:00 until the 30 kWh target is met.
% 30000 Wh / 11000 W = 2.73 h, so ~18:00-20:45 at full power.
P_ev_vec = zeros(N,1);
ev_hours_needed = ev_energy / ev_pmax;                 % 2.73 h
P_ev_vec(t_h>=18 & t_h<(18+ev_hours_needed)) = ev_pmax;

%% ------------------------------------------------------------------------
%  9. DHW DRAW PROFILE  (hot water taken by the household) [W]
%  ------------------------------------------------------------------------
Q_draw_vec = zeros(N,1);
Q_draw_vec(t_h>=6  & t_h<8)  = 2500;   % morning showers
Q_draw_vec(t_h>=12 & t_h<13) = 800;    % midday
Q_draw_vec(t_h>=18 & t_h<21) = 2000;   % evening

%% ------------------------------------------------------------------------
%  10. WEATHER FORECAST
%  ------------------------------------------------------------------------
% 10a. Outdoor temperature: daily sinusoid (Bavarian late-winter day)
%      Minimum before dawn (~ -2 degC), peak mid-afternoon (~ 8 degC)
T_out_vec = 3 + 5*sin(2*pi*(t_h - 9)/24);     % [degC]

% 10b. Solar irradiance: Gaussian bell centred at solar noon [W/m^2]
G_peak    = 600;                              % clear-ish winter peak
G_vec     = G_peak * exp(-0.5*((t_h - 12.5)/2.8).^2);
G_vec(t_h < 7.5) = 0;                         % no sun before sunrise
G_vec(t_h > 17 ) = 0;                         % none after sunset

% 10c. PV generation forecast [W]  (scaled so clear noon ~ pv_peak)
P_pv_vec  = pv_peak * (G_vec / G_peak);

%% ------------------------------------------------------------------------
%  11. GRID PRICE FORECAST  (EPEX / dynamic-tariff style) [ct/kWh]
%  ------------------------------------------------------------------------
price_vec = 22*ones(N,1);                              % base price
price_vec(t_h>=7  & t_h<9 ) = price_vec(t_h>=7  & t_h<9 ) + 12;  % morning peak
price_vec(t_h>=17 & t_h<21) = price_vec(t_h>=17 & t_h<21) + 10;  % evening peak
price_vec(t_h>=11 & t_h<14) = price_vec(t_h>=11 & t_h<14) - 8;   % midday dip
price_vec(t_h>=0  & t_h<5 ) = price_vec(t_h>=0  & t_h<5 ) - 5;   % cheap night
feed_in_tariff = 8;                                    % PV export [ct/kWh]

% 11b. Peak-period flag (1 = high-price slot the HEMS should avoid)
peak_flag = double(price_vec >= 28);

%% ------------------------------------------------------------------------
%  12. PACKAGE SIGNALS FOR "From Workspace" BLOCKS  ([time, value] format)
%  ------------------------------------------------------------------------
T_out_data    = [t_s, T_out_vec];
G_data        = [t_s, G_vec];
P_pv_data     = [t_s, P_pv_vec];
Q_draw_data   = [t_s, Q_draw_vec];
price_data    = [t_s, price_vec];
peak_data     = [t_s, peak_flag];
P_ess_data    = [t_s, P_ess];
gate_wm_data = [t_s, gate_wm_vec];
gate_dr_data = [t_s, gate_dr_vec];
P_ev_data    = [t_s, P_ev_vec];

%% ------------------------------------------------------------------------
%  13. SANITY CHECKS + SUMMARY PLOTS
%  ------------------------------------------------------------------------
fprintf('\n==== hems_init.m complete ====\n');
fprintf(' Time grid           : %d steps x %g min = 24 h\n', N, dt/60);
fprintf(' Building time const  : %.1f h (expect ~62 h)\n', tau_bldg_h);
fprintf(' Tank thermal mass    : %.0f J/K\n', C_tank);
fprintf(' Daily essential energy: %.2f kWh\n', sum(P_ess)*dt_h/1000);
fprintf(' Daily DHW draw energy : %.2f kWh\n', sum(Q_draw_vec)*dt_h/1000);
fprintf(' Daily PV potential    : %.2f kWh\n', sum(P_pv_vec)*dt_h/1000);
fprintf(' Price range           : %.0f - %.0f ct/kWh\n', min(price_vec), max(price_vec));
fprintf('===============================\n\n');

figure('Name','HEMS inputs','Color','w');

subplot(3,2,1);
plot(t_h, T_out_vec,'LineWidth',1.3); grid on;
xlabel('Hour'); ylabel('T_{out} (\circC)'); title('Outdoor temperature'); xlim([0 24]);

subplot(3,2,2);
plot(t_h, P_pv_vec/1000,'LineWidth',1.3); grid on;
xlabel('Hour'); ylabel('PV (kW)'); title('PV generation forecast'); xlim([0 24]);

subplot(3,2,3);
stairs(t_h, price_vec,'LineWidth',1.3); grid on;
xlabel('Hour'); ylabel('Price (ct/kWh)'); title('Electricity price'); xlim([0 24]);

subplot(3,2,4);
area(t_h, P_ess/1000); grid on;
xlabel('Hour'); ylabel('P (kW)'); title('Essential load profile'); xlim([0 24]);

subplot(3,2,5);
stairs(t_h, Q_draw_vec/1000,'LineWidth',1.3); grid on;
xlabel('Hour'); ylabel('DHW draw (kW)'); title('Hot water draw'); xlim([0 24]);

subplot(3,2,6);
stairs(t_h, peak_flag,'LineWidth',1.3); grid on;
xlabel('Hour'); ylabel('Peak flag'); title('Peak-price periods'); xlim([0 24]); ylim([-0.1 1.1]);

disp('Workspace ready. Open hems_model.slx and start building subsystems.');