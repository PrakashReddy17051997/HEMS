# Home Energy Management System (HEMS) — Prognosebasierte Optimierung & MPC


---
---

# DEUTSCH

## 1. Überblick

Das Projekt modelliert ein Einfamilienhaus mit Wärmepumpe, Warmwasserspeicher, PV-Anlage, Batteriespeicher, E-Auto und schaltbaren Haushaltsgeräten. Es vergleicht drei Regelungsstrategien anhand desselben Tagesszenarios (24 h, 96 Zeitschritte zu je 15 min):

1. **Baseline** — regelbasierte Thermostat-/Batterieregelung ohne Vorausschau.
2. **Optimiert (Single-Shot)** — prognosebasierte 24-h-Optimierung als lineares Programm (LP).
3. **MPC** — Receding-Horizon-Regelung (Model Predictive Control) mit Rückführung des gemessenen Zustands.

## 2. Ergebnisse

| Strategie | Tageskosten | Strecke | Prognose |
|---|---|---|---|
| Baseline | ~€23,04 | Simulink (vollständig) | — |
| Optimiert (Single-Shot) | ~€8,28 | Simulink (vollständig) | perfekt |
| MPC | ~€4,98 | 1R1C (vereinfacht) | −3 °C Fehler |

Die Optimierung senkt die Tageskosten auf dem identischen Simulink-Modell um ca. **64 %** (€23,04 → €8,28), bei eingehaltenem Komfortband. Unter Prognosefehler (−3 °C) hält die MPC das Komfortband ein, während die Steuerung ohne Rückführung (Single-Shot, offene Schleife) aus dem Band driftet.

> **Wichtig:** Baseline und Optimized laufen auf demselben Simulink-Modell mit identischer Prognose — ihr Kostenunterschied misst sauber den Nutzen der Optimierung. Die MPC läuft auf einem vereinfachten Modell **mit** Prognosefehler, um Robustheit zu zeigen; ihre Absolutkosten sind daher **nicht direkt** mit den Simulink-Läufen vergleichbar.

## 3. Technischer Ansatz

### 3.1 Anlagenmodell

Thermisches 1R1C-Gebäudemodell (Wärmekapazität `C_bldg`, Verlustkoeffizient `H_T`), separater Warmwasserspeicher (`C_tank`, `UA_tank`) und Batterie-Ladezustandsmodell (SoC). Alle Parameter sind auf Normen rückführbar (EN ISO 13790, ISO 13789/6946, DIN V 18599, EN 410, GEG).

### 3.2 Optimierung

Formuliert als lineares Programm. Entscheidungsvariablen pro Zeitschritt: Wärmepumpenleistung (aufgeteilt in Heizen `P_hp_heat` und Warmwasser `P_hp_dhw` — die Aufteilung hält das Problem **linear**), E-Auto-Ladung, Batterie laden/entladen, schaltbare Geräte, Netzbezug/-einspeisung. Zielfunktion: Minimierung der Tagesstromkosten. Gelöst mit MATLAB `linprog` (Optimization Toolbox) über das `optimproblem`-Framework.



### 3.3 Model Predictive Control (MPC)

Receding-Horizon-Schleife: In jedem Schritt wird das LP über den verbleibenden Horizont gelöst, **am gemessenen Zustand verankert** (Re-Anchoring), nur der erste Stellschritt angewandt, dann ein Zeitschritt der realen Strecke ausgeführt. Komfortgrenzen sind als **weiche Nebenbedingungen** mit Schlupfvariablen und Strafterm formuliert — das garantiert Lösbarkeit auch unter Störungen (harte Zustandsgrenzen sind eine klassische Ursache für Infeasibility in MPC).

### 3.4 Architektur (Option 3)

Die MPC regelt die kontinuierlich modulierbaren Lasten (Wärmepumpe, E-Auto, Batterie). Die diskreten Gerätezyklen (Waschmaschine/Trockner) werden in der Day-Ahead-Ebene geplant — sie als Binärvariablen in die MPC aufzunehmen, würde ein gemischt-ganzzahliges Problem (MILP-MPC) erfordern. Diese Zwei-Ebenen-Trennung (Day-Ahead-MILP + Echtzeit-LP-MPC) entspricht dem Aufbau realer HEMS-Produkte.

## 4. Dateien

| Datei | Beschreibung |
|---|---|
| `hems_init.m` | Parameter, Profile (Preis, PV, T_out, Lasten), Baseline-Daten |
| `hems_model.slx` | Simulink-Anlagenmodell + Regler + Umschalter |
| `hems_optimizer_linprog.m` | Single-Shot-LP-Optimierer (`linprog`) |
| `hems_mpc.m` | Receding-Horizon-MPC mit Leichtmodell |
| `sim_validation.m` | Drei-Wege-Vergleich + Abbildung + Limitationstabelle |
| `hems_showcase.m` | Kombinierte Showcase-Abbildung (2×2) |
| `hems_optimizer.m` | CasADi-Variante (historisch, ersetzt durch linprog) |

## 5. Ausführung

```matlab
run('hems_init.m')                  % Parameter + Profile
run('hems_optimizer_linprog.m')     % erzeugt *_opt_data
run('sim_validation.m')             % 3-Wege-Vergleich + Abbildung
```

**Voraussetzungen:** MATLAB R2024b, Simulink, Optimization Toolbox. Für die MPC: `T_in0 = 21`, `T_min = 20` in `hems_init.m` (Komfortband realistisch und Single-Shot lösbar).

## 6. Wichtigste Erkenntnisse

- Lastverschiebung (v. a. E-Auto) aus der Hochpreiszeit ist der größte Hebel.
- Das Gebäude wird als thermischer Speicher genutzt (Vorheizen in günstigen/solaren Stunden).
- Re-Anchoring am Messzustand unterscheidet echte MPC vom bloßen Abspielen eines Plans.
- Weiche Komfort-Nebenbedingungen sichern Lösbarkeit unter Prognosefehler.
- Modellabweichung (LP-Modell vs. Simulink-Strecke) motiviert die MPC.


---
---

#  ENGLISH

## 1. Overview

The project models a single-family house with a heat pump, hot-water tank, PV system, battery, EV, and shiftable appliances. It compares three control strategies on the same 24-hour scenario (96 fifteen-minute slots):

1. **Baseline** — rule-based thermostat/battery control with no foresight.
2. **Optimized (single-shot)** — forecast-based 24-hour optimization as a linear program (LP).
3. **MPC** — receding-horizon control with measured-state feedback.

## 2. Results

| Strategy | Daily cost | Plant | Forecast |
|---|---|---|---|
| Baseline | ~€23.04 | Simulink (full) | — |
| Optimized (single-shot) | ~€8.28 | Simulink (full) | perfect |
| MPC | ~€4.98 | 1R1C (lightweight) | −3 °C error |

On the identical Simulink plant, optimization reduces daily cost by about **64 %** (€23.04 → €8.28) while maintaining the comfort band. Under a −3 °C forecast error, MPC keeps the comfort band, whereas the open-loop single-shot schedule drifts out of band.

> **Note:** Baseline and Optimized run on the same Simulink plant with the same forecast — their cost gap cleanly measures optimization value. MPC runs on a simplified plant **with** forecast error to demonstrate robustness; its absolute cost is therefore **not directly** comparable to the Simulink runs.

## 3. Technical Approach

### 3.1 Plant model

1R1C thermal building model (capacitance `C_bldg`, loss coefficient `H_T`), a separate hot-water tank (`C_tank`, `UA_tank`), and a battery state-of-charge model. All parameters trace to standards (EN ISO 13790, ISO 13789/6946, DIN V 18599, EN 410, GEG).

### 3.2 Optimization

Formulated as a linear program. Decision variables per slot: heat-pump power (split into heating `P_hp_heat` and DHW `P_hp_dhw` — the split keeps the problem **linear**), EV charging, battery charge/discharge, shiftable appliances, grid import/export. Objective: minimize daily electricity cost. Solved with MATLAB `linprog` (Optimization Toolbox) via the `optimproblem` framework.



### 3.3 Model Predictive Control (MPC)

Receding-horizon loop: at each step the LP is solved over the remaining horizon, **anchored at the measured state** (re-anchoring), only the first control move is applied, then the real plant advances one step. Comfort limits are modeled as **soft constraints** with slack variables and a penalty term — guaranteeing feasibility even under disturbances (hard state constraints are a classic cause of MPC infeasibility).

### 3.4 Architecture (Option 3)

The MPC controls continuously-modulated loads (heat pump, EV, battery). Discrete appliance cycles (washer/dryer) are scheduled in the day-ahead layer — including them as binaries in the MPC would require a mixed-integer problem (MILP-MPC). This two-layer split (day-ahead MILP + real-time LP-MPC) mirrors how production HEMS are structured.

## 4. Files

| File | Description |
|---|---|
| `hems_init.m` | parameters, profiles (price, PV, T_out, loads), baseline data |
| `hems_model.slx` | Simulink plant + controllers + switches |
| `hems_optimizer_linprog.m` | single-shot LP optimizer (`linprog`) |
| `hems_mpc.m` | receding-horizon MPC with lightweight plant |
| `sim_validation.m` | three-way comparison + figure + limitations table |
| `hems_showcase.m` | combined showcase figure (2×2) |
| `hems_optimizer.m` | CasADi version (historical, superseded by linprog) |

## 5. How to Run

```matlab
run('hems_init.m')                  % parameters + profiles
run('hems_optimizer_linprog.m')     % produces *_opt_data schedules
run('sim_validation.m')             % 3-way comparison + figure
```

**Requirements:** MATLAB R2024b, Simulink, Optimization Toolbox. For the MPC: set `T_in0 = 21`, `T_min = 20` in `hems_init.m` (realistic comfort band and feasible single-shot).

## 6. Key Takeaways

- Load shifting (especially the EV) out of peak hours is the largest lever.
- The building is used as thermal storage (pre-heating in cheap/solar hours).
- Re-anchoring to the measured state is what distinguishes true MPC from replaying a plan.
- Soft comfort constraints ensure feasibility under forecast error.
- Model mismatch (LP model vs. Simulink plant) is the motivation for MPC.


