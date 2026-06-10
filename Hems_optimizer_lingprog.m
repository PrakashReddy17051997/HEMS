%% ========================================================================
%  hems_optimizer_linprog.m
%  Single-shot forecast-based HEMS optimizer  (Path 1)  -- MATLAB linprog
%
%  Uses MATLAB's Optimization Toolbox (optimproblem + linprog). No CasADi,
%  no external solvers, no dylib dependencies. Solves the same linear
%  program as the CasADi version: minimise the day's electricity cost by
%  scheduling the heat pump, EV, battery and shiftable appliances.
%
%  USAGE:  run('hems_init.m')   then   run('hems_optimizer_linprog.m')
%  ========================================================================

if ~exist('C_bldg','var')
    error('Workspace not initialised. Run hems_init.m first.');
end

%% ---- EV availability: home overnight & evening, away 07:00-17:00 -------
ev_available = ones(N,1);
ev_available(t_h>=7 & t_h<17) = 0;

%% ---- precompute COP and solar gain per slot ---------------------------
COP   = eta_hp * (T_sup+273.15) ./ ((T_sup+273.15) - (T_out_vec+273.15));
COP   = min(max(COP,1.5),6.0);
Q_sol = A_win * g_val * G_vec;

%% ========================================================================
%  BUILD THE LP WITH optimproblem  (readable; MATLAB assembles matrices)
%  ========================================================================
prob = optimproblem('ObjectiveSense','minimize');

% ---- decision variables (continuous) -----------------------------------
P_hp_heat = optimvar('P_hp_heat',N,'LowerBound',0,'UpperBound',P_hp_max);
P_hp_dhw  = optimvar('P_hp_dhw', N,'LowerBound',0,'UpperBound',P_hp_max);
P_ev      = optimvar('P_ev',     N,'LowerBound',0,'UpperBound',ev_pmax);
P_ch      = optimvar('P_ch',     N,'LowerBound',0,'UpperBound',bat_pmax);
P_dis     = optimvar('P_dis',    N,'LowerBound',0,'UpperBound',bat_pmax);
g_wm      = optimvar('g_wm',     N,'LowerBound',0,'UpperBound',1);
g_dr      = optimvar('g_dr',     N,'LowerBound',0,'UpperBound',1);
P_grid    = optimvar('P_grid',   N,'LowerBound',0);
P_exp     = optimvar('P_exp',    N,'LowerBound',0);

Tin   = optimvar('Tin',  N+1,'LowerBound',T_min,     'UpperBound',T_max);
Ttank = optimvar('Ttank',N+1,'LowerBound',T_tank_min,'UpperBound',T_tank_max);
SoC   = optimvar('SoC',  N+1,'LowerBound',bat_soc_min,'UpperBound',bat_soc_max);

% ---- objective: daily electricity cost [EUR] ---------------------------
prob.Objective = sum( (price_vec.*P_grid - feed_in_tariff*P_exp) * dt_h/1000/100 );

% ---- EV availability upper bounds (time-varying) -----------------------
prob.Constraints.evavail = P_ev <= ev_pmax*ev_available;

% ---- energy balance (per slot) -----------------------------------------
Pload_fixed = P_ess + wm_power*g_wm + dr_power*g_dr;
prob.Constraints.balance = ...
    P_grid + P_pv_vec + P_dis == Pload_fixed + P_hp_heat + P_hp_dhw + P_ev + P_ch + P_exp;

% ---- building & tank 1R1C recurrences ----------------------------------
prob.Constraints.bldg = ...
    Tin(2:N+1) == Tin(1:N) + (dt/C_bldg)*( P_hp_heat.*COP + Q_sol + Q_int - H_T*(Tin(1:N)-T_out_vec) );
prob.Constraints.tank = ...
    Ttank(2:N+1) == Ttank(1:N) + (dt/C_tank)*( P_hp_dhw.*COP - Q_draw_vec - UA_tank*(Ttank(1:N)-Tin(1:N)) );

% ---- battery SoC --------------------------------------------------------
prob.Constraints.soc = ...
    SoC(2:N+1) == SoC(1:N) + (bat_eff_1way*P_ch - P_dis/bat_eff_1way)*dt_h;

% ---- heat-pump combined power cap --------------------------------------
prob.Constraints.hpcap = P_hp_heat + P_hp_dhw <= P_hp_max;

% ---- initial states -----------------------------------------------------
prob.Constraints.Tin0   = Tin(1)   == T_in0;
prob.Constraints.Ttank0 = Ttank(1) == T_tank0;
prob.Constraints.SoC0   = SoC(1)   == bat_soc0;

% ---- global requirements ------------------------------------------------
prob.Constraints.ev_energy = sum(P_ev)*dt_h >= ev_energy;
prob.Constraints.wm_cycle  = sum(g_wm) == wm_duration;
prob.Constraints.dr_cycle  = sum(g_dr) == dr_duration;

% ---- appliance allowed windows (force 0 outside window) ----------------
wm_block = true(N,1); wm_block(wm_release:wm_deadline-1) = false;  % true = forced off
dr_block = true(N,1); dr_block(dr_release:dr_deadline-1) = false;
if any(wm_block), prob.Constraints.wm_win = g_wm(wm_block) == 0; end
if any(dr_block), prob.Constraints.dr_win = g_dr(dr_block) == 0; end

%% ---- solve with linprog (HiGHS dual-simplex, built in) ----------------
opts = optimoptions('linprog','Display','off');
[sol,fval,exitflag] = solve(prob,'Options',opts);

if exitflag <= 0
    error('LP did not solve (exitflag %d). Check constraints for infeasibility.', exitflag);
end

%% ========================================================================
%  EXTRACT THE OPTIMAL SCHEDULE
%  ========================================================================
P_hp_heat_opt = sol.P_hp_heat;
P_hp_dhw_opt  = sol.P_hp_dhw;
P_ev_opt      = sol.P_ev;
P_ch_opt      = sol.P_ch;
P_dis_opt     = sol.P_dis;
g_wm_opt      = sol.g_wm;
g_dr_opt      = sol.g_dr;
P_grid_opt    = sol.P_grid;
P_exp_opt     = sol.P_exp;
Tin_opt       = sol.Tin;
Ttank_opt     = sol.Ttank;
SoC_opt       = sol.SoC;

P_hp_opt = P_hp_heat_opt + P_hp_dhw_opt;
mode_opt = zeros(N,1);
mode_opt(P_hp_heat_opt > P_hp_dhw_opt & P_hp_heat_opt > 1) = 1;
mode_opt(P_hp_dhw_opt  > P_hp_heat_opt & P_hp_dhw_opt  > 1) = 2;

P_bat_opt   = P_ch_opt - P_dis_opt;
gate_wm_opt = double(g_wm_opt > 0.5);
gate_dr_opt = double(g_dr_opt > 0.5);

cost_opt = sum( (price_vec.*P_grid_opt - feed_in_tariff*P_exp_opt) * dt_h )/1000/100;

fprintf('\n==== HEMS optimizer solved (linprog) ====\n');
fprintf(' Optimized daily cost : EUR %.2f\n', cost_opt);
fprintf(' EV energy delivered  : %.1f kWh (target %.0f)\n', sum(P_ev_opt)*dt_h/1000, ev_energy/1000);
fprintf(' T_in  range          : %.1f .. %.1f degC\n', min(Tin_opt), max(Tin_opt));
fprintf(' T_tank range         : %.1f .. %.1f degC\n', min(Ttank_opt), max(Ttank_opt));
fprintf(' SoC   range          : %.0f .. %.0f Wh\n', min(SoC_opt), max(SoC_opt));
fprintf('=========================================\n\n');

%% ---- package schedules for From Workspace ------------------------------
P_ev_opt_data    = [t_s, P_ev_opt];
gate_wm_opt_data = [t_s, gate_wm_opt];
gate_dr_opt_data = [t_s, gate_dr_opt];
mode_opt_data    = [t_s, mode_opt];
P_hp_opt_data    = [t_s, P_hp_opt];
P_bat_opt_data   = [t_s, P_bat_opt];

%% ---- plot the optimal schedule against price --------------------------
figure('Name','HEMS optimized schedule (linprog)','Color','w');

subplot(4,1,1);
stairs(t_h, price_vec,'LineWidth',1.3); grid on;
ylabel('Price (ct/kWh)'); title('Electricity price'); xlim([0 24]);

subplot(4,1,2);
stairs(t_h, P_ev_opt/1000,'LineWidth',1.3); hold on;
stairs(t_h, P_pv_vec/1000,'--'); grid on;
ylabel('kW'); legend('EV charge','PV'); title('EV charging vs PV (cheap/solar hours)'); xlim([0 24]);

subplot(4,1,3);
stairs(t_h, P_bat_opt/1000,'LineWidth',1.3); hold on;
plot(t_h, SoC_opt(1:N)/1000,'LineWidth',1.0); grid on;
ylabel('kW / kWh'); legend('P_{bat}','SoC'); title('Battery'); xlim([0 24]);

subplot(4,1,4);
plot(t_h, Tin_opt(1:N),'LineWidth',1.3); hold on;
yline(T_min,':'); yline(T_max,':'); grid on;
ylabel('T_{in} (\circC)'); title('Indoor temperature (within comfort band)'); xlim([0 24]);
xlabel('Hour');

disp('Optimal schedules ready: P_ev_opt_data, gate_wm_opt_data, gate_dr_opt_data,');
disp('mode_opt_data, P_hp_opt_data, P_bat_opt_data  -> feed into Simulink From Workspace.');