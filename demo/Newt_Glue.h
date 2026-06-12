/*
**        Shared state between the newt_* Tcl commands and the generic
**        Scenario program: the scenario path flows in before prg_setup,
**        the parsed YAML snapshots flow out for the run record.
*/

# pragma once

# include <string>

struct Tcl_Interp;

// set by `newt_scenario <path>`; read by Scenario::setup
extern std::string Newt_ScenarioPath;

// stashed by Scenario::setup for newt_open_run; empty for legacy demos
extern std::string Newt_ScenarioYaml;
extern std::string Newt_CreatureYaml;

// configured record_dt from the scenario file (newt_open_run's dt
// argument overrides when positive, falls back to this when 0)
extern double Newt_RecordDt;

extern "C" int Newt_Init(Tcl_Interp *interp);
