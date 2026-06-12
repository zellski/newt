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
# include <cstdio>

# include "Newt_Glue.h"
# include "World.h"
# include "DOF.h"
# include "newt/Sweep.h"
# include "newt/Recorder.h"

std::string Newt_ScenarioPath;
std::string Newt_ScenarioYaml;
std::string Newt_CreatureYaml;
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
   if (objc != 5) {
      return error(interp, "usage: newt_record <k> <objective> <inf> <grdL>");
   }
   if (!Rec) {
      std::fprintf(stderr, "newt_record: no run open, skipping\n");
      return TCL_OK;
   }
   int k;
   double obj, inf, grdL;
   if (Tcl_GetIntFromObj(interp, objv[1], &k) != TCL_OK ||
       Tcl_GetDoubleFromObj(interp, objv[2], &obj) != TCL_OK ||
       Tcl_GetDoubleFromObj(interp, objv[3], &inf) != TCL_OK ||
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
   Tcl_CreateObjCommand(interp, "newt_finish", CmdFinish, 0, 0);
   return TCL_OK;
}
