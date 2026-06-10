%% ========================================================================
%  sim_validation.m
%  Three-way HEMS comparison on ONE figure:
%     (1) BASELINE              -- Simulink plant, rule-based controllers
%     (2) OPTIMIZED single-shot -- Simulink plant, linprog schedule
%     (3) MPC                   -- lightweight plant, receding-horizon, under
%                                  a deliberate forecast error
%
%  This script is SELF-CONTAINED: it runs hems_init.m and the optimizer
%  itself (below), runs the baseline + optimized Simulink simulations, then
%  CALLS hems_mpc.m for the MPC result (single source of truth -- the MPC is
%  no longer duplicated here). It overlays the signals common to all three
%  (T_in, cost, SoC, grid power) and prints a limitations table.
%
%  USAGE:  just run this file.   run('sim_validation.m')
%  Note: calling hems_mpc.m also opens its own MPC-vs-single-shot figure and
%        prints its own report -- that is expected, not an error.
%  ========================================================================
clc;clear,close all;
run("hems_init.m")
run("Hems_optimizer_lingprog.m")
open("hems_model.slx")
assert(exist('P_ev_opt_data','var')==1, ...
    'Run hems_optimizer_linprog.m first (need *_opt_data schedules).');

%% ========================================================================
%  RUN 1 : BASELINE  (Simulink, live rule-based controllers)
%  ========================================================================
P_ev_active     = P_ev_data;
gate_wm_active  = gate_wm_data;
gate_dr_active  = gate_dr_data;
mode_opt_active = mode_opt_data;     % present but unused (switch = live)
P_hp_opt_active = P_hp_opt_data;
P_bat_opt_active= P_bat_opt_data;
set_param('hems_model/Mode_Switch','sw','0');
set_param('hems_model/Phl_Switch' ,'sw','0');
set_param('hems_model/Pbat_Switch','sw','0');
out_baseline = sim('hems_model');

%% ========================================================================
%  RUN 2 : OPTIMIZED single-shot  (Simulink, follows linprog schedule)
%  ========================================================================
P_ev_active     = P_ev_opt_data;
gate_wm_active  = gate_wm_opt_data;
gate_dr_active  = gate_dr_opt_data;
set_param('hems_model/Mode_Switch','sw','1');
set_param('hems_model/Phl_Switch' ,'sw','1');
set_param('hems_model/Pbat_Switch','sw','1');
out_optimized = sim('hems_model');

%% ---- pull common Simulink signals (from logsout) ----------------------
[tB,  Tin_B]  = getlog(out_baseline ,'T_in');
[~,   cost_B] = getlog(out_baseline ,'cost_eur');
[~,   SoC_B]  = getlog(out_baseline ,'SoC');
[~,   Pl_B]   = getlog(out_baseline ,'P_load');

[tO,  Tin_O]  = getlog(out_optimized,'T_in');
[~,   cost_O] = getlog(out_optimized,'cost_eur');
[~,   SoC_O]  = getlog(out_optimized,'SoC');
[~,   Pl_O]   = getlog(out_optimized,'P_load');

%% ========================================================================
%  RUN 3 : MPC  (lightweight plant, receding horizon, forecast error)
%  ------------------------------------------------------------------------
%  The MPC is NOT re-implemented here. We CALL the standalone hems_mpc.m so
%  there is a single source of truth for the MPC logic; any fix there flows
%  through to this comparison automatically. hems_mpc.m produces:
%     Tin_mpc, SoC_mpc, Pgrid_mpc (per-slot)  and  cost_mpc (scalar total).
%  We re-map those to the M-suffixed names the plotting code below expects,
%  and rebuild the cumulative cost curve from the per-slot grid power.
%  ========================================================================
run('hems_mpc.m');                       % defines Tin_mpc, SoC_mpc, Pgrid_mpc, cost_mpc, viol_mpc

Tin_M  = Tin_mpc(:);
SoC_M  = SoC_mpc(:);
Pg_M   = Pgrid_mpc(:);
cost_M = cumsum( price_vec(:).*max(Pg_M,0)*dt_h/1000/100 );   % cumulative EUR curve
tM     = t_h(:);

%% ---- final headline numbers -------------------------------------------
cB = cost_B(end);  cO = cost_O(end);  cM = cost_M(end);

%% ========================================================================
%  COMBINED FIGURE  (4 stacked panels, all three overlaid)
%  ========================================================================
figure('Name','HEMS: baseline vs optimized vs MPC','Color','w','Position',[80 80 1000 820]);

% --- 1. indoor temperature ---
subplot(4,1,1);
plot(tB,Tin_B,'b-','LineWidth',1.3); hold on;
plot(tO,Tin_O,'g-','LineWidth',1.3);
plot(tM,Tin_M,'r--','LineWidth',1.3);
yline(T_min,':'); yline(T_max,':'); grid on; xlim([0 24]);
ylabel('T_{in} (\circC)');
legend('Baseline','Optimized (single-shot)','MPC (forecast err)','Location','best');
title('Indoor temperature - comfort band');

% --- 2. cumulative cost ---
subplot(4,1,2);
plot(tB,cost_B,'b-','LineWidth',1.3); hold on;
plot(tO,cost_O,'g-','LineWidth',1.3);
plot(tM,cost_M,'r--','LineWidth',1.3); grid on; xlim([0 24]);
ylabel('Cost (EUR)'); title('Cumulative electricity cost');
legend(sprintf('Baseline  %.2f',cB), sprintf('Optimized %.2f',cO), ...
       sprintf('MPC       %.2f',cM),'Location','northwest');

% --- 3. battery SoC ---
subplot(4,1,3);
plot(tB,SoC_B/1000,'b-','LineWidth',1.3); hold on;
plot(tO,SoC_O/1000,'g-','LineWidth',1.3);
plot(tM,SoC_M/1000,'r--','LineWidth',1.3); grid on; xlim([0 24]);
ylabel('SoC (kWh)'); title('Battery state of charge');
legend('Baseline','Optimized','MPC','Location','best');

% --- 4. grid power ---
subplot(4,1,4);
plot(tB,Pl_B/1000,'b-','LineWidth',1.3); hold on;
plot(tO,Pl_O/1000,'g-','LineWidth',1.3);
plot(tM,Pg_M/1000,'r--','LineWidth',1.3); grid on; xlim([0 24]);
ylabel('Power (kW)'); xlabel('Hour'); title('Household load / grid power');
legend('Baseline P_{load}','Optimized P_{load}','MPC P_{grid}','Location','best');

sgtitle('HEMS three-way comparison: baseline vs single-shot optimized vs receding-horizon MPC');

% --- caption box describing what each line is + its limitation ----------
annotation('textbox',[0.07 0.0 0.86 0.045],'EdgeColor',[.8 .8 .8], ...
  'BackgroundColor',[.97 .97 .97],'FontSize',8,'Interpreter','none', ...
  'String',['Baseline: Simulink plant, reactive rules.  ', ...
            'Optimized: Simulink plant, day-ahead linprog (perfect forecast).  ', ...
            'MPC: lightweight plant, re-planned each step under -3C forecast error.  ', ...
            'Plants/forecasts differ -> compare TRENDS, not absolute levels.']);

%% ========================================================================
%  LIMITATIONS TABLE (console)
%  ========================================================================
fprintf('\n==================== THREE-WAY HEMS COMPARISON ====================\n');
fprintf(' Method        | Final cost | Plant            | Forecast | Key limitation\n');
fprintf(' --------------+------------+------------------+----------+--------------------------------\n');
fprintf(' Baseline      | EUR %5.2f  | Simulink (full)  | n/a      | Reactive; no foresight; charges in peak\n', cB);
fprintf(' Optimized SS  | EUR %5.2f  | Simulink (full)  | perfect  | Open-loop; assumes forecast exact; no feedback\n', cO);
fprintf(' MPC           | EUR %5.2f  | 1R1C (light)     | -3C err  | Continuous loads only (no discrete appliances);\n', cM);
fprintf('               |            |                  |          | simplified plant model vs Simulink\n');
fprintf(' ==================================================================\n');
fprintf(' Notes:\n');
fprintf('  * Baseline vs Optimized share the SAME Simulink plant & forecast\n');
fprintf('    -> their cost gap is a clean measure of optimization value.\n');
fprintf('  * MPC runs on a lighter plant WITH forecast error to show robustness,\n');
fprintf('    so its absolute cost is not directly comparable to the Simulink runs.\n');
fprintf('  * Option 3: appliances (washer/dryer) are scheduled day-ahead, not in\n');
fprintf('    the MPC -- discrete loads would make it a MILP; LP-MPC stays fast.\n');
fprintf(' ==================================================================\n\n');

%% ========================================================================
%  LOCAL FUNCTIONS
%  ========================================================================
function [t,d] = getlog(out, name)
% Read a logged signal from logsout by name; return time(hours) and data.
    el = out.logsout.getElement(name);
    v  = el.Values;
    t  = v.Time/3600;
    d  = squeeze(v.Data);
    d  = d(:); t = t(:);
end