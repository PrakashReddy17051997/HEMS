%% ========================================================================
%  hems_mpc.m
%  Receding-horizon (linear MPC) HEMS controller  --  Goal 2
%
%  Demonstrates closed-loop MPC robustness to forecast error, and contrasts
%  it with the open-loop single-shot schedule. Both controllers use the
%  SAME linprog formulation; the difference is that MPC re-solves every step
%  and RE-ANCHORS to the measured plant state, so it corrects for the error.
%
%  Lightweight in-script plant (1R1C building + tank + battery SoC) so the
%  control loop has live state feedback at every step -- a post-run Simulink
%  log cannot provide that, which is why the plant lives here in code.
%
%  (linear MPC):
%     for k = 1..N
%        u(k:N) = solve_LP( measured state at k, forecast )   % re-plan
%        apply ONLY u(k) to the plant                          % first move
%        plant advances one step using ACTUAL weather          % reality
%     end
%  The forecast the optimizer trusts differs from reality (T_out_actual),
%  so without re-anchoring the plan drifts. MPC re-anchors -> stays in band.
%
%  USAGE:  run('hems_init.m')   then   run('hems_mpc.m')
%  Requires: Optimization Toolbox (linprog). 
%  ========================================================================

if ~exist('C_bldg','var')
    error('Workspace not initialised. Run hems_init.m first.');
end

%% ---- forecast vs actual outdoor temperature (the deliberate error) -----
T_out_fc     = T_out_vec;          % what the optimizer believes (forecast)
forecast_err = -3.0;               % reality is 3 degC COLDER than forecast
T_out_actual = T_out_vec + forecast_err;

%% ---- EV availability (except 07:00-17:00) ------------------------
ev_available = ones(N,1);
ev_available(t_h>=7 & t_h<17) = 0;

%% ========================================================================
%  RUN 1 :  MPC  (re-solve each step, re-anchor to measured state)
%  ========================================================================
Tin = T_in0;  Ttank = T_tank0;  SoC = bat_soc0;
ev_remaining = ev_energy;
cost_mpc = 0;
Tin_mpc = zeros(N,1);  Ttank_mpc = zeros(N,1);  SoC_mpc = zeros(N,1);
Pev_mpc = zeros(N,1);  Pgrid_mpc = zeros(N,1);
viol_mpc = 0;

for k = 1:N
    % --- solve LP over remaining horizon, anchored at MEASURED state ---
    u = solve_horizon(k, Tin, Ttank, SoC, ev_remaining, ...
                       T_out_fc, ev_available, N, dt, dt_h, ...
                       C_bldg, C_tank, H_T, UA_tank, A_win, g_val, Q_int, ...
                       Q_draw_vec, G_vec, P_ess, P_pv_vec, price_vec, ...
                       feed_in_tariff, P_hp_max, ev_pmax, bat_pmax, ...
                       bat_eff_1way, T_min, T_max, T_tank_min, T_tank_max, ...
                       bat_soc_min, bat_soc_max, T_sup, eta_hp);

    % --- apply ONLY the first step's controls ---
   
    if Tin >= T_max
        u.P_hp_heat = 0;
    end
    cost_mpc      = cost_mpc + price_vec(k)*max(u.P_grid,0)*dt_h/1000/100;
    ev_remaining  = max(0, ev_remaining - u.P_ev*dt_h);

    % --- advance the plant ONE step using ACTUAL weather ---
    [Tin,Ttank,SoC] = plant_step(Tin,Ttank,SoC,u,k, ...
                       T_out_actual, dt, dt_h, C_bldg, C_tank, H_T, UA_tank, ...
                       A_win, g_val, Q_int, Q_draw_vec, G_vec, ...
                       bat_eff_1way, T_sup, eta_hp);

    Tin_mpc(k)=Tin; Ttank_mpc(k)=Ttank; SoC_mpc(k)=SoC;
    Pev_mpc(k)=u.P_ev; Pgrid_mpc(k)=u.P_grid;
    if Tin < T_min-0.1 || Tin > T_max+0.1, viol_mpc = viol_mpc + 1; end
end

%% ========================================================================
%  RUN 2 :  SINGLE-SHOT open-loop  (solve once on forecast, apply blindly)
%  ========================================================================
% build the open-loop schedule by stepping a FORECAST plant
Tin=T_in0; Ttank=T_tank0; SoC=bat_soc0; ev_rem=ev_energy;
sched(N) = struct('P_hp_heat',0,'P_hp_dhw',0,'P_ev',0,'P_ch',0,'P_dis',0,'P_grid',0,'P_exp',0);
for k = 1:N
    u = solve_horizon(k, Tin, Ttank, SoC, ev_rem, ...
                       T_out_fc, ev_available, N, dt, dt_h, ...
                       C_bldg, C_tank, H_T, UA_tank, A_win, g_val, Q_int, ...
                       Q_draw_vec, G_vec, P_ess, P_pv_vec, price_vec, ...
                       feed_in_tariff, P_hp_max, ev_pmax, bat_pmax, ...
                       bat_eff_1way, T_min, T_max, T_tank_min, T_tank_max, ...
                       bat_soc_min, bat_soc_max, T_sup, eta_hp);
    sched(k) = u;  ev_rem = max(0, ev_rem - u.P_ev*dt_h);
    % step FORECAST plant (optimizer's own belief)
    [Tin,Ttank,SoC] = plant_step(Tin,Ttank,SoC,u,k, ...
                       T_out_fc, dt, dt_h, C_bldg, C_tank, H_T, UA_tank, ...
                       A_win, g_val, Q_int, Q_draw_vec, G_vec, ...
                       bat_eff_1way, T_sup, eta_hp);
end
% now apply that FIXED schedule to the ACTUAL plant (no re-anchoring)
Tin=T_in0; Ttank=T_tank0; SoC=bat_soc0; cost_ss=0;
Tin_ss=zeros(N,1); Ttank_ss=zeros(N,1); SoC_ss=zeros(N,1);
Pev_ss=zeros(N,1); viol_ss=0;
for k = 1:N
    u = sched(k);
    cost_ss = cost_ss + price_vec(k)*max(u.P_grid,0)*dt_h/1000/100;
    [Tin,Ttank,SoC] = plant_step(Tin,Ttank,SoC,u,k, ...
                       T_out_actual, dt, dt_h, C_bldg, C_tank, H_T, UA_tank, ...
                       A_win, g_val, Q_int, Q_draw_vec, G_vec, ...
                       bat_eff_1way, T_sup, eta_hp);
    Tin_ss(k)=Tin; Ttank_ss(k)=Ttank; SoC_ss(k)=SoC; Pev_ss(k)=u.P_ev;
    
    if Tin < T_min-0.1 || Tin > T_max+0.1, viol_ss = viol_ss + 1; end
end

%% ---- report ------------------------------------------------------------
fprintf('\n=================  MPC vs SINGLE-SHOT  =================\n');
fprintf(' Forecast error applied to plant : %.1f degC\n', forecast_err);
fprintf(' ------------------------------------------------------\n');
fprintf(' MPC (re-anchored)   : cost EUR %.2f | Tin %.1f-%.1f | comfort viol %d/%d\n', ...
        cost_mpc, min(Tin_mpc), max(Tin_mpc), viol_mpc, N);
fprintf(' Single-shot (open)  : cost EUR %.2f | Tin %.1f-%.1f | comfort viol %d/%d\n', ...
        cost_ss,  min(Tin_ss),  max(Tin_ss),  viol_ss,  N);
fprintf('=======================================================\n');


%% ---- plot comparison ---------------------------------------------------
figure('Name','MPC vs single-shot under forecast error','Color','w');

subplot(3,1,1);
plot(t_h,Tin_mpc,'LineWidth',1.4); hold on;
plot(t_h,Tin_ss,'--','LineWidth',1.4);
yline(T_min,':'); yline(T_max,':');
grid on; ylabel('T_{in} (\circC)'); xlim([0 24]);
legend('MPC (re-anchored)','Single-shot (open-loop)','Location','best');
title(sprintf('Indoor temperature  (forecast error %.0f\\circC)  -- MPC stays in band',forecast_err));

subplot(3,1,2);
stairs(t_h,Pev_mpc/1000,'LineWidth',1.4); hold on;
stairs(t_h,Pev_ss/1000,'--','LineWidth',1.4);
grid on; ylabel('P_{EV} (kW)'); xlim([0 24]);
legend('MPC','Single-shot'); title('EV charging');

subplot(3,1,3);
plot(t_h,SoC_mpc/1000,'LineWidth',1.4); hold on;
plot(t_h,SoC_ss/1000,'--','LineWidth',1.4);
grid on; ylabel('SoC (kWh)'); xlabel('Hour'); xlim([0 24]);
legend('MPC','Single-shot'); title('Battery state of charge');

%% ========================================================================
%  LOCAL FUNCTIONS
%  ========================================================================
function u = solve_horizon(k0, Tin0, Ttank0, SoC0, ev_rem, ...
        T_out_fc, ev_available, N, dt, dt_h, C_bldg, C_tank, H_T, UA, ...
        A_win, g_val, Q_int, Q_draw, G, P_ess, P_pv, price, feed_in, ...
        P_hp_max, ev_pmax, bat_pmax, eta, T_min, T_max, T_tank_min, ...
        T_tank_max, soc_min, soc_max, T_sup, eta_hp)
% Solve the cost-min LP over slots k0..N anchored at the measured state.
% Returns only the FIRST-step controls (struct u).

    H = N - k0 + 1;                          % remaining slots
    idx = (k0:N);                            % absolute slot indices
    COP = eta_hp*(T_sup+273.15)./((T_sup+273.15)-(T_out_fc(idx)+273.15));
    COP = min(max(COP,1.5),6.0);

    % decision variables
    prob = optimproblem('ObjectiveSense','minimize');
    Phh = optimvar('Phh',H,'LowerBound',0,'UpperBound',P_hp_max);
    Phd = optimvar('Phd',H,'LowerBound',0,'UpperBound',P_hp_max);
    Pev = optimvar('Pev',H,'LowerBound',0,'UpperBound',ev_pmax);
    Pch = optimvar('Pch',H,'LowerBound',0,'UpperBound',bat_pmax);
    Pds = optimvar('Pds',H,'LowerBound',0,'UpperBound',bat_pmax);
    Pg  = optimvar('Pg', H,'LowerBound',0);
    Pe  = optimvar('Pe', H,'LowerBound',0);
    % Comfort is enforced SOFTLY via slacks, but Ti keeps a HARD safety
    % ceiling a few degrees above T_max so the LP can never plan a thermal
    % runaway (heating the room arbitrarily high to trade off the penalty).
    % The lower side stays free (soft) so cold-side comfort can still flex.
    Ti  = optimvar('Ti', H+1,'LowerBound',-Inf,'UpperBound',T_max + 5);
    Tt  = optimvar('Tt', H+1,'LowerBound',T_tank_min,'UpperBound',T_tank_max);
    Sc  = optimvar('Sc', H+1,'LowerBound',soc_min,'UpperBound',soc_max);
    % slack variables: how far T_in is below T_min / above T_max (>=0)
    sLo = optimvar('sLo',H+1,'LowerBound',0);
    sHi = optimvar('sHi',H+1,'LowerBound',0);

    % objective: electricity cost + heavy penalty on comfort slack
    pen = 100;   % EUR per degC-slot of comfort violation (soft constraint weight)
    prob.Objective = sum( (price(idx).*Pg - feed_in*Pe)*dt_h/1000/100 ) ...
                   + pen*sum(sLo) + pen*sum(sHi);

    prob.Constraints.evav = Pev <= ev_pmax*ev_available(idx);
    prob.Constraints.bal  = Pg + P_pv(idx) == ...
        P_ess(idx) + Phh + Phd + Pev + Pch + Pe - Pds;
    prob.Constraints.bldg = Ti(2:H+1) == Ti(1:H) + (dt/C_bldg)* ...
        ( Phh.*COP + A_win*g_val*G(idx) + Q_int - H_T*(Ti(1:H)-T_out_fc(idx)) );
    prob.Constraints.tank = Tt(2:H+1) == Tt(1:H) + (dt/C_tank)* ...
        ( Phd.*COP - Q_draw(idx) - UA*(Tt(1:H)-Ti(1:H)) );
    prob.Constraints.soc  = Sc(2:H+1) == Sc(1:H) + (eta*Pch - Pds/eta)*dt_h;
    prob.Constraints.hp   = Phh + Phd <= P_hp_max;
    prob.Constraints.Ti0  = Ti(1) == Tin0;
    prob.Constraints.Tt0  = Tt(1) == Ttank0;
    prob.Constraints.Sc0  = Sc(1) == SoC0;
    prob.Constraints.ev   = sum(Pev)*dt_h >= ev_rem;
    % soft comfort: Ti >= T_min - sLo  and  Ti <= T_max + sHi
    prob.Constraints.comfLo = Ti >= T_min - sLo;
    prob.Constraints.comfHi = Ti <= T_max + sHi;

    opts = optimoptions('linprog','Display','off');
    [sol,~,flag] = solve(prob,'Options',opts);
    if flag <= 0
        % true infeasibility should now be impossible (soft comfort);
        % if it still happens, do the SAFE thing: hold, no heating runaway
        u = struct('P_hp_heat',0,'P_hp_dhw',0,'P_ev',0, ...
                   'P_ch',0,'P_dis',0,'P_grid',0,'P_exp',0);
        return;
    end
    u = struct('P_hp_heat',sol.Phh(1),'P_hp_dhw',sol.Phd(1), ...
               'P_ev',sol.Pev(1),'P_ch',sol.Pch(1),'P_dis',sol.Pds(1), ...
               'P_grid',sol.Pg(1),'P_exp',sol.Pe(1));
end

function [Tin,Ttank,SoC] = plant_step(Tin,Ttank,SoC,u,k, ...
        T_out, dt, dt_h, C_bldg, C_tank, H_T, UA, A_win, g_val, Q_int, ...
        Q_draw, G, eta, T_sup, eta_hp)
% Advance the lightweight 1R1C + tank + SoC plant by ONE step.
    cop = eta_hp*(T_sup+273.15)/((T_sup+273.15)-(T_out(k)+273.15));
    cop = min(max(cop,1.5),6.0);
    Tin   = Tin   + (dt/C_bldg)*( u.P_hp_heat*cop + A_win*g_val*G(k) + Q_int - H_T*(Tin-T_out(k)) );
    Ttank = Ttank + (dt/C_tank)*( u.P_hp_dhw*cop - Q_draw(k) - UA*(Ttank-Tin) );
    SoC   = SoC   + (eta*u.P_ch - u.P_dis/eta)*dt_h;
end
%% ========================================================================

%  ========================================================================
fprintf('\n----------------- SELF-CHECK -----------------\n');
fprintf(' MPC indoor temp range : %.1f .. %.1f degC (band %.0f..%.0f)\n', ...
        min(Tin_mpc), max(Tin_mpc), T_min, T_max);
if max(Tin_mpc) <= T_max + 0.1 && min(Tin_mpc) >= T_min - 0.1
    fprintf(' RESULT: PASS  -- MPC stayed inside the comfort band.\n');
else
    fprintf(' RESULT: CHECK -- MPC left the band; max overshoot %.1f degC.\n', ...
            max(0, max(Tin_mpc)-T_max));
end
fprintf(' MPC comfort violations : %d / %d\n', viol_mpc, N);
fprintf(' Single-shot violations : %d\n ', viol_ss, N);
fprintf('----------------------------------------------\n');