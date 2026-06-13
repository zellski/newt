/*
**        Tcl commands for run recording:
**
**          newt_scenario <abs-path>            before prg_setup
**          newt_open_run <db> <name> <dt>      after prg_setup
**          newt_record <k> <obj> <inf> <grdL>  after each accepted iterate
**          newt_finish <status>                when the solve ends
**
**        newt_record arms the Sweeper and asks HQP to re-evaluate at the
**        current (accepted) iterate via prg_update_fbd; the sweep result
**        is then written to the database. Recording problems warn rather
**        than kill a long optimization.
*/

# include <tcl.h>
# include <algorithm>
# include <cmath>
# include <cstdio>
# include <sstream>
# include <utility>
# include <vector>

# include "Newt_Glue.h"
# include "World.h"
# include "DOF.h"
# include "newt/Sweep.h"
# include "newt/Residuals.h"
# include "newt/Recorder.h"

std::string Newt_ScenarioPath;
std::string Newt_ScenarioYaml;
std::string Newt_CreatureYaml;
std::string Newt_CensusText;
double Newt_RecordDt = 0;

static newt::Recorder *Rec = 0;

static int error(Tcl_Interp *interp, const std::string &msg) {
   Tcl_SetObjResult(interp, Tcl_NewStringObj(msg.c_str(), -1));
   return TCL_ERROR;
}

static int CmdScenario(ClientData, Tcl_Interp *interp,
                       int objc, Tcl_Obj *const objv[]) {
   if (objc != 2) {
      return error(interp, "usage: newt_scenario <path>");
   }
   Newt_ScenarioPath = Tcl_GetString(objv[1]);
   return TCL_OK;
}

static int CmdOpenRun(ClientData, Tcl_Interp *interp,
                      int objc, Tcl_Obj *const objv[]) {
   if (objc != 4) {
      return error(interp, "usage: newt_open_run <dbPath> <name> <dt>");
   }
   if (!World::Active) {
      return error(interp, "newt_open_run: no World yet (run prg_setup first)");
   }
   if (Rec) {
      return error(interp, "newt_open_run: a run is already open");
   }
   double dt;
   if (Tcl_GetDoubleFromObj(interp, objv[3], &dt) != TCL_OK) {
      return TCL_ERROR;
   }
   if (dt <= 0) {
      dt = Newt_RecordDt > 0 ? Newt_RecordDt : 0.01;
   }

   World *W = World::Active;
   newt::RunInfo info;
   info.scenarioName = Tcl_GetString(objv[2]);
   info.scenarioPath = Newt_ScenarioPath;
   info.scenarioYaml = Newt_ScenarioYaml;
   info.creatureYaml = Newt_CreatureYaml;
   for (uint i = 0; i < W->DOFs.size(); i ++) {
      info.dofNames.push_back(W->DOFs[i]->Name);
   }
   info.nVars = W->xIx;
   info.frameDt = dt;

   std::string err;
   Rec = newt::Recorder::Open(Tcl_GetString(objv[1]), info, err);
   if (!Rec) {
      return error(interp, "newt_open_run: " + err);
   }
   newt::Sweeper::dt = dt;

   Tcl_SetObjResult(interp, Tcl_NewWideIntObj(Rec->runId()));
   return TCL_OK;
}

static int CmdRecord(ClientData, Tcl_Interp *interp,
                     int objc, Tcl_Obj *const objv[]) {
   if (objc != 4 && objc != 5) {
      return error(interp, "usage: newt_record <k> <objective> <inf> ?<grdL>?");
   }
   if (!Rec) {
      std::fprintf(stderr, "newt_record: no run open, skipping\n");
      return TCL_OK;
   }
   int k;
   double obj, inf;
   double grdL = NAN;   // omitted => stored as NULL until newt_grdl backfills
   if (Tcl_GetIntFromObj(interp, objv[1], &k) != TCL_OK ||
       Tcl_GetDoubleFromObj(interp, objv[2], &obj) != TCL_OK ||
       Tcl_GetDoubleFromObj(interp, objv[3], &inf) != TCL_OK) {
      return TCL_ERROR;
   }
   if (objc == 5 &&
       Tcl_GetDoubleFromObj(interp, objv[4], &grdL) != TCL_OK) {
      return TCL_ERROR;
   }

   // re-evaluate at the accepted iterate; the sweep runs in World::Update
   newt::Sweeper::requested = true;
   if (Tcl_Eval(interp, "prg_update_fbd") != TCL_OK) {
      newt::Sweeper::requested = false;
      std::fprintf(stderr, "newt_record: prg_update_fbd failed: %s\n",
                   Tcl_GetStringResult(interp));
      return TCL_OK;
   }

   newt::SweepResult &R = newt::Sweeper::result;
   if (!R.valid) {
      std::fprintf(stderr, "newt_record: sweep did not run, skipping k=%d\n", k);
      return TCL_OK;
   }
   Rec->RecordIteration(k, obj, inf, grdL, R.x, R.times, R.frames, R.stageEnds);
   R.valid = false;
   return TCL_OK;
}

static int CmdGrdL(ClientData, Tcl_Interp *interp,
                   int objc, Tcl_Obj *const objv[]) {
   if (objc != 3) {
      return error(interp, "usage: newt_grdl <k> <norm_grd_L>");
   }
   if (!Rec) {
      return TCL_OK;
   }
   int k;
   double grdL;
   if (Tcl_GetIntFromObj(interp, objv[1], &k) != TCL_OK ||
       Tcl_GetDoubleFromObj(interp, objv[2], &grdL) != TCL_OK) {
      return TCL_ERROR;
   }
   Rec->UpdateNormGrdL(k, grdL);
   return TCL_OK;
}

// newt_residuals ?N? -- re-evaluate at the current iterate and report
// the N largest constraint violations against their labeled rows
static int CmdResiduals(ClientData, Tcl_Interp *interp,
                        int objc, Tcl_Obj *const objv[]) {
   int top = 10;
   if (objc > 2 ||
       (objc == 2 && Tcl_GetIntFromObj(interp, objv[1], &top) != TCL_OK)) {
      return error(interp, "usage: newt_residuals ?N?");
   }
   World *W = World::Active;
   if (!W) {
      return error(interp, "newt_residuals: no World yet (run prg_setup first)");
   }

   newt::Residuals::requested = true;
   if (Tcl_Eval(interp, "prg_update_fbd") != TCL_OK) {
      newt::Residuals::requested = false;
      return error(interp, std::string("newt_residuals: prg_update_fbd failed: ")
                   + Tcl_GetStringResult(interp));
   }
   if (!newt::Residuals::valid) {
      return error(interp, "newt_residuals: capture did not run");
   }
   newt::Residuals::valid = false;

   const std::vector<double> &c = newt::Residuals::c;
   std::vector<std::pair<double, int> > viol;
   for (size_t i = 0; i < c.size(); i ++) {
      // meschach's max() macro shadows std::max here
      double below = W->cMins[i] - c[i], above = c[i] - W->cMaxs[i];
      double v = below > above ? below : above;
      if (v > 0) {
         viol.push_back(std::make_pair(v, (int) i));
      }
   }
   std::sort(viol.begin(), viol.end());
   std::reverse(viol.begin(), viol.end());

   std::ostringstream o;
   if (viol.empty()) {
      o << "no constraint violations at the current iterate";
   } else {
      o << viol.size() << " of " << c.size()
        << " constraint rows violated; the largest:\n";
      for (size_t i = 0; i < viol.size() && i < (size_t) top; i ++) {
         char num[32];
         std::snprintf(num, sizeof num, "  %9.3g  ", viol[i].first);
         o << num << W->cLabels[viol[i].second] << "\n";
      }
   }
   Tcl_SetObjResult(interp, Tcl_NewStringObj(o.str().c_str(), -1));
   return TCL_OK;
}

static int CmdCensus(ClientData, Tcl_Interp *interp,
                     int objc, Tcl_Obj *const objv[]) {
   if (objc != 1) {
      return error(interp, "usage: newt_census");
   }
   if (Newt_CensusText.empty()) {
      return error(interp, "newt_census: no scenario set up");
   }
   Tcl_SetObjResult(interp,
                    Tcl_NewStringObj(Newt_CensusText.c_str(), -1));
   return TCL_OK;
}

static int CmdFinish(ClientData, Tcl_Interp *interp,
                     int objc, Tcl_Obj *const objv[]) {
   if (objc != 2) {
      return error(interp, "usage: newt_finish <status>");
   }
   if (!Rec) {
      std::fprintf(stderr, "newt_finish: no run open, skipping\n");
      return TCL_OK;
   }
   Rec->Finish(Tcl_GetString(objv[1]));
   delete Rec;
   Rec = 0;
   return TCL_OK;
}

extern "C" int Newt_Init(Tcl_Interp *interp) {
   Tcl_CreateObjCommand(interp, "newt_scenario", CmdScenario, 0, 0);
   Tcl_CreateObjCommand(interp, "newt_open_run", CmdOpenRun, 0, 0);
   Tcl_CreateObjCommand(interp, "newt_record", CmdRecord, 0, 0);
   Tcl_CreateObjCommand(interp, "newt_grdl", CmdGrdL, 0, 0);
   Tcl_CreateObjCommand(interp, "newt_census", CmdCensus, 0, 0);
   Tcl_CreateObjCommand(interp, "newt_residuals", CmdResiduals, 0, 0);
   Tcl_CreateObjCommand(interp, "newt_finish", CmdFinish, 0, 0);
   return TCL_OK;
}
