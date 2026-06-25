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
# include <string>
# include <utility>
# include <vector>

# include <Hqp_SqpProgram.h>
# include <Hqp_Program.h>

# include "Newt_Glue.h"
# include "World.h"
# include "DOF.h"
# include "newt/Sweep.h"
# include "newt/Residuals.h"
# include "newt/Recorder.h"

// the active nonlinear program (set by `prg_name`); the finite-difference
// derivative check reads the linear-quadratic approximation it assembles
extern Hqp_SqpProgram *theSqpProgram;

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

/*
**        Finite-difference derivative check (newt_dcheck).
**
**        Grades whatever derivative path assembled the linear-quadratic
**        approximation -- it reads the already-built Jacobian/objective
**        gradient out of the program's QP and compares them against
**        central differences of the constraint residuals and objective,
**        perturbing one optimizer variable at a time. Provider-agnostic:
**        it never looks at how the QP was filled, only at the result, so
**        it passes on the ADOL-C path today and would keep passing if the
**        derivative provider changed.
*/

namespace {

// running max absolute / relative deviation over one QP block, with the
// worst-offending location remembered for the report
struct BlockErr {
   double maxAbs;
   double maxRel;
   int rows;            // block height (0 => "no rows", nothing to check)
   int worstRow, worstCol;
   double worstAnalytic, worstFd;   // the pair behind maxRel, for the report
   bool active;
   BlockErr() : maxAbs(0), maxRel(0), rows(0), worstRow(-1), worstCol(-1),
                worstAnalytic(0), worstFd(0), active(false) {}

   void note(double analytic, double fd, int row, int col) {
      active = true;
      double a = analytic < 0 ? -analytic : analytic;
      double b = fd < 0 ? -fd : fd;
      double dev = analytic - fd;
      if (dev < 0) dev = -dev;
      if (dev > maxAbs) maxAbs = dev;
      // a relative error is only meaningful once the entry clears the
      // finite-difference noise floor; below it (e.g. a structural zero
      // against ~1e-10 of roundoff) the absolute deviation is the
      // honest measure and a relative figure would just read 100%
      double scale = a > b ? a : b;
      if (scale > 1e-6) {
         double rel = dev / scale;
         if (rel > maxRel) {
            maxRel = rel; worstRow = row; worstCol = col;
            worstAnalytic = analytic; worstFd = fd;
         }
      }
   }
};

struct DCheckResult {
   bool ok;
   std::string error;
   int n, me, mi;
   BlockErr grad;      // qp->c, the objective gradient
   BlockErr jacEq;     // qp->A, the equality-constraint Jacobian
   BlockErr jacIneq;   // qp->C, the inequality-constraint Jacobian
   BlockErr grdL;      // gradient of the Lagrangian (c - A'y - C'z)
   bool grdLDone;
   DCheckResult() : ok(false), n(0), me(0), mi(0), grdLDone(false) {}

   // the single scalar a per-iterate diagnostic records: the worst
   // relative deviation across every checked first-derivative block
   double maxRel() const {
      double r = grad.maxRel;
      if (jacEq.maxRel > r) r = jacEq.maxRel;
      if (jacIneq.maxRel > r) r = jacIneq.maxRel;
      if (grdLDone && grdL.maxRel > r) r = grdL.maxRel;
      return r;
   }
   double maxAbs() const {
      double a = grad.maxAbs;
      if (jacEq.maxAbs > a) a = jacEq.maxAbs;
      if (jacIneq.maxAbs > a) a = jacIneq.maxAbs;
      if (grdLDone && grdL.maxAbs > a) a = grdL.maxAbs;
      return a;
   }
};

// read an If_RealVec interface variable (e.g. sqp_y) as plain doubles;
// false if the eval fails or the length is not what we expect, which is
// the normal "solver not initialized yet" case
bool readTclVec(Tcl_Interp *interp, const char *var, int want,
                std::vector<double> &out) {
   if (Tcl_Eval(interp, var) != TCL_OK) {
      return false;
   }
   int n = 0;
   Tcl_Obj **el = 0;
   if (Tcl_ListObjGetElements(interp, Tcl_GetObjResult(interp), &n, &el)
       != TCL_OK || n != want) {
      return false;
   }
   out.resize(n);
   for (int i = 0; i < n; i ++) {
      if (Tcl_GetDoubleFromObj(interp, el[i], &out[i]) != TCL_OK) {
         return false;
      }
   }
   return true;
}

// the worst-offending variable's label, when the World's labels line up
// with the optimizer's variable order (NewT packs every coefficient into
// a single Omuses stage, so they do)
std::string varLabel(int col) {
   World *W = World::Active;
   if (W && col >= 0 && col < (int) W->xLabels.size()) {
      return W->xLabels[col];
   }
   return "?";
}

DCheckResult RunDcheck(Tcl_Interp *interp) {
   DCheckResult R;
   Hqp_SqpProgram *prg = theSqpProgram;
   if (!prg) {
      R.error = "no program set up (run prg_setup first)";
      return R;
   }
   Hqp_Program *qp = prg->qp();
   if (!qp || !(const VEC *) qp->c
       || !(const VEC *) qp->b || !(const VEC *) qp->d) {
      R.error = "the QP is not allocated yet";
      return R;
   }
   int n = qp->c->dim, me = qp->b->dim, mi = qp->d->dim;
   R.n = n; R.me = me; R.mi = mi;
   R.jacEq.rows = me; R.jacIneq.rows = mi;

   // the iterate to check, kept so we can put the solver back exactly
   VEC *x0 = v_get(n);
   v_copy(prg->x(), x0);

   // the solver's current multipliers, when it is running; with them the
   // QP assembly matches an ordinary solver step exactly, and they let us
   // also check the gradient of the Lagrangian. Absent (e.g. before
   // sqp_init), zero multipliers still give the right first derivatives,
   // because NewT's path leaves qp->Q to the BFGS approximation regardless.
   std::vector<double> y, z;
   bool haveMult = readTclVec(interp, "sqp_y", me, y)
                && readTclVec(interp, "sqp_z", mi, z);

   VEC *yv = v_get(me), *zv = v_get(mi);
   for (int j = 0; j < me; j ++) yv->ve[j] = haveMult ? y[j] : 0.0;
   for (int j = 0; j < mi; j ++) zv->ve[j] = haveMult ? z[j] : 0.0;

   // assemble the analytic linear approximation at x0
   prg->update(yv, zv);

   // snapshot the objective gradient and, if we have multipliers, the
   // gradient of the Lagrangian -- the same contraction the solver uses,
   // grd_L = c - A'y - C'z -- before the FD sweep overwrites b and d
   VEC *gradf = v_get(n);
   v_copy(qp->c, gradf);
   std::vector<double> grdLa(n, 0.0);
   if (haveMult) {
      VEC *gL = v_zero(v_get(n));
      sp_vm_mltadd(gL, yv, qp->A, 1.0, gL);
      sp_vm_mltadd(gL, zv, qp->C, 1.0, gL);
      for (int i = 0; i < n; i ++) grdLa[i] = gradf->ve[i] - gL->ve[i];
      v_free(gL);
      R.grdLDone = true;
   }

   VEC *xw = v_get(n);
   std::vector<double> bP(me), bM(me), dP(mi), dM(mi);

   for (int i = 0; i < n; i ++) {
      double xi = x0->ve[i];
      double h = 1e-6 * (1.0 + (xi < 0 ? -xi : xi));

      v_copy(x0, xw);
      xw->ve[i] = xi + h;
      prg->set_x(xw);
      prg->update_fbd();
      double fP = prg->f();
      for (int j = 0; j < me; j ++) bP[j] = qp->b->ve[j];
      for (int j = 0; j < mi; j ++) dP[j] = qp->d->ve[j];

      xw->ve[i] = xi - h;
      prg->set_x(xw);
      prg->update_fbd();
      double fM = prg->f();
      for (int j = 0; j < me; j ++) bM[j] = qp->b->ve[j];
      for (int j = 0; j < mi; j ++) dM[j] = qp->d->ve[j];

      double inv = 1.0 / (2.0 * h);
      R.grad.note(gradf->ve[i], (fP - fM) * inv, 0, i);
      for (int j = 0; j < me; j ++) {
         R.jacEq.note(sp_get_val(qp->A, j, i), (bP[j] - bM[j]) * inv, j, i);
      }
      for (int j = 0; j < mi; j ++) {
         R.jacIneq.note(sp_get_val(qp->C, j, i), (dP[j] - dM[j]) * inv, j, i);
      }
      if (haveMult) {
         double Lp = fP, Lm = fM;
         for (int j = 0; j < me; j ++) { Lp -= y[j] * bP[j]; Lm -= y[j] * bM[j]; }
         for (int j = 0; j < mi; j ++) { Lp -= z[j] * dP[j]; Lm -= z[j] * dM[j]; }
         R.grdL.note(grdLa[i], (Lp - Lm) * inv, 0, i);
      }
   }

   // restore the iterate and leave b/d/f consistent with it
   prg->set_x(x0);
   prg->update_fbd();

   v_free(x0); v_free(xw); v_free(yv); v_free(zv); v_free(gradf);
   R.ok = true;
   return R;
}

void formatBlock(std::ostringstream &o, const char *label,
                 const BlockErr &b, bool perVar) {
   char line[160];
   if (b.rows == 0 && !perVar) {
      std::snprintf(line, sizeof line, "  %-22s : no rows\n", label);
      o << line;
      return;
   }
   if (!b.active) {
      std::snprintf(line, sizeof line, "  %-22s : skipped\n", label);
      o << line;
      return;
   }
   std::snprintf(line, sizeof line,
                 "  %-22s : max|abs| %9.3g   max|rel| %9.3g",
                 label, b.maxAbs, b.maxRel);
   o << line;
   if (b.worstCol >= 0) {
      if (perVar) {
         o << "   worst var '" << varLabel(b.worstCol) << "'";
      } else {
         o << "   worst row " << b.worstRow
           << ", var '" << varLabel(b.worstCol) << "'";
      }
      char nums[80];
      std::snprintf(nums, sizeof nums, " (analytic %.4g vs fd %.4g)",
                    b.worstAnalytic, b.worstFd);
      o << nums;
   }
   o << "\n";
}

std::string formatDcheck(const DCheckResult &R, double tol) {
   std::ostringstream o;
   o << "newt_dcheck: central finite-difference derivative check\n";
   char hdr[160];
   std::snprintf(hdr, sizeof hdr,
                 "  %d vars, %d equality rows, %d inequality rows;"
                 " step h = 1e-6*(1+|x|)\n", R.n, R.me, R.mi);
   o << hdr;
   formatBlock(o, "objective gradient c", R.grad, true);
   formatBlock(o, "equality Jacobian A", R.jacEq, false);
   formatBlock(o, "inequality Jacobian C", R.jacIneq, false);
   if (R.grdLDone) {
      formatBlock(o, "Lagrangian gradient", R.grdL, true);
   } else {
      o << "  Lagrangian gradient    : skipped"
           " (no multipliers; solver not initialized)\n";
   }
   char tail[96];
   std::snprintf(tail, sizeof tail, "  worst relative deviation %9.3g -- %s\n",
                 R.maxRel(), R.maxRel() <= tol ? "within tolerance"
                                               : "EXCEEDS tolerance");
   o << tail;
   return o.str();
}

}  // namespace

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

// newt_dcheck ?tol? -- finite-difference check of the assembled QP
// derivatives at the current iterate; reports per-block max abs/rel error
static int CmdDcheck(ClientData, Tcl_Interp *interp,
                     int objc, Tcl_Obj *const objv[]) {
   // loose enough that central-difference truncation on a stiff objective
   // reads "within tolerance", tight enough to flag a genuinely wrong
   // (e.g. missing-term) derivative, which shows a relative error near 1
   double tol = 1e-3;
   if (objc > 2 ||
       (objc == 2 && Tcl_GetDoubleFromObj(interp, objv[1], &tol) != TCL_OK)) {
      return error(interp, "usage: newt_dcheck ?tol?");
   }
   DCheckResult R = RunDcheck(interp);
   if (!R.ok) {
      return error(interp, "newt_dcheck: " + R.error);
   }
   Tcl_SetObjResult(interp,
                    Tcl_NewStringObj(formatDcheck(R, tol).c_str(), -1));
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
   Tcl_CreateObjCommand(interp, "newt_dcheck", CmdDcheck, 0, 0);
   Tcl_CreateObjCommand(interp, "newt_residuals", CmdResiduals, 0, 0);
   Tcl_CreateObjCommand(interp, "newt_finish", CmdFinish, 0, 0);
   return TCL_OK;
}
