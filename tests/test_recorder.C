/*
**        Unit tests for the SQLite run recorder: schema creation, blob
**        round-tripping, run finalization, and two concurrently open
**        writers on one database (the multi-process WAL scenario, in
**        miniature).
*/

# include <cstdio>
# include <cstring>
# include <cstdlib>
# include <cmath>
# include <string>
# include <vector>

# include <sqlite3.h>
# include <unistd.h>

# include "newt/Recorder.h"

using namespace newt;
using std::string;
using std::vector;

static int failures = 0;

static void check(const char *what, bool ok) {
   if (!ok) {
      std::fprintf(stderr, "FAIL %s\n", what);
      failures ++;
   }
}

static RunInfo info(const char *name) {
   RunInfo I;
   I.scenarioName = name;
   I.scenarioPath = "/tmp/somewhere.yaml";
   I.scenarioYaml = "name: " + string(name) + "\n";
   I.creatureYaml = "name: beast\n";
   I.dofNames.push_back("X");
   I.dofNames.push_back("Y");
   I.dofNames.push_back("Alpha");
   I.nVars = 4;
   I.frameDt = 0.01;
   return I;
}

static vector<double> ramp(int n, double base) {
   vector<double> v;
   for (int i = 0; i < n; i ++) {
      v.push_back(base + i);
   }
   return v;
}

int main() {
   char dir[] = "/tmp/newt-test-XXXXXX";
   if (!mkdtemp(dir)) {
      std::fprintf(stderr, "FAIL mkdtemp\n");
      return 1;
   }
   string dbPath = string(dir) + "/test.db";
   string err;

   // --- single writer round-trip ---
   Recorder *R = Recorder::Open(dbPath, info("alpha"), err);
   check("open", R != 0);
   if (!R) {
      std::fprintf(stderr, "  %s\n", err.c_str());
      return 1;
   }
   check("run id", R->runId() == 1);

   vector<double> x = ramp(4, 100), times = ramp(3, 0);
   vector<double> frames = ramp(3*3*2, 1000), bounds = ramp(2, 0.5);
   R->RecordIteration(0, 42.5, 1e-3, 2e-2, x, times, frames, bounds);
   R->RecordIteration(1, 41.0, 1e-4, 1e-2, x, times, frames, bounds);

   // --- second writer on the same database, interleaved ---
   Recorder *R2 = Recorder::Open(dbPath, info("beta"), err);
   check("second open", R2 != 0);
   if (R2) {
      check("second run id", R2->runId() == 2);
      R2->RecordIteration(0, 7.0, 1e-1, 1e-1, x, times, frames, bounds);
      R->RecordIteration(2, 40.5, 1e-5, 1e-3, x, times, frames, bounds);
      R2->Finish("iters");
      delete R2;
   }
   R->Finish("optimal");
   delete R;

   // --- verify with a raw connection ---
   sqlite3 *db = 0;
   check("reopen", sqlite3_open(dbPath.c_str(), &db) == SQLITE_OK);
   sqlite3_stmt *st = 0;

   sqlite3_prepare_v2(db, "PRAGMA journal_mode;", -1, &st, 0);
   sqlite3_step(st);
   check("wal mode", string((const char *) sqlite3_column_text(st, 0)) == "wal");
   sqlite3_finalize(st);

   sqlite3_prepare_v2(db,
      "SELECT scenario_name, status, final_iter, n_dofs, n_vars, dof_names"
      " FROM runs ORDER BY id;", -1, &st, 0);
   check("run 1 row", sqlite3_step(st) == SQLITE_ROW);
   check("run 1 name", string((const char *) sqlite3_column_text(st, 0)) == "alpha");
   check("run 1 status", string((const char *) sqlite3_column_text(st, 1)) == "optimal");
   check("run 1 final_iter", sqlite3_column_int(st, 2) == 2);
   check("run 1 n_dofs", sqlite3_column_int(st, 3) == 3);
   check("run 1 n_vars", sqlite3_column_int(st, 4) == 4);
   check("run 1 dof json",
         string((const char *) sqlite3_column_text(st, 5)) == "[\"X\",\"Y\",\"Alpha\"]");
   check("run 2 row", sqlite3_step(st) == SQLITE_ROW);
   check("run 2 status", string((const char *) sqlite3_column_text(st, 1)) == "iters");
   check("run 2 final_iter", sqlite3_column_int(st, 2) == 0);
   sqlite3_finalize(st);

   sqlite3_prepare_v2(db,
      "SELECT objective, n_frames, x, frames, stage_bounds FROM iterations"
      " WHERE run_id=1 AND k=2;", -1, &st, 0);
   check("iteration row", sqlite3_step(st) == SQLITE_ROW);
   check("objective", sqlite3_column_double(st, 0) == 40.5);
   check("n_frames", sqlite3_column_int(st, 1) == 3);
   check("x blob size", sqlite3_column_bytes(st, 2) == 4*8);
   check("x blob bytes",
         memcmp(sqlite3_column_blob(st, 2), &x[0], 4*8) == 0);
   check("frames blob size", sqlite3_column_bytes(st, 3) == (int) (frames.size()*8));
   check("frames blob bytes",
         memcmp(sqlite3_column_blob(st, 3), &frames[0], frames.size()*8) == 0);
   check("stage bounds json",
         string((const char *) sqlite3_column_text(st, 4)) == "[0.5,1.5]");
   sqlite3_finalize(st);

   sqlite3_prepare_v2(db,
      "SELECT COUNT(*) FROM iterations;", -1, &st, 0);
   sqlite3_step(st);
   check("total iterations", sqlite3_column_int(st, 0) == 4);
   sqlite3_finalize(st);
   sqlite3_close(db);

   // --- schema version guard: a too-new file must be refused ---
   {
      string vPath = string(dir) + "/future.db";
      sqlite3 *vdb = 0;
      sqlite3_open(vPath.c_str(), &vdb);
      sqlite3_exec(vdb, "PRAGMA user_version=99;", 0, 0, 0);
      sqlite3_close(vdb);
      string verr;
      Recorder *VR = Recorder::Open(vPath, info("gamma"), verr);
      check("version guard", VR == 0 && verr.find("schema version") != string::npos);
      delete VR;
   }

   if (failures) {
      std::fprintf(stderr, "test_recorder: %d failure(s)\n", failures);
      return 1;
   }
   std::printf("test_recorder: all tests passed\n");
   return 0;
}
