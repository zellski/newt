# include "newt/Recorder.h"

# include <sqlite3.h>
# include <unistd.h>
# include <cstdio>
# include <sstream>

using namespace newt;
using std::string;
using std::vector;

# define SCHEMA_VERSION 1

static const char *DDL =
   "CREATE TABLE IF NOT EXISTS runs ("
   "  id            INTEGER PRIMARY KEY,"
   "  started_at    TEXT NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ','now')),"
   "  finished_at   TEXT,"
   "  scenario_name TEXT NOT NULL,"
   "  scenario_path TEXT,"
   "  scenario_yaml TEXT,"
   "  creature_yaml TEXT,"
   "  dof_names     TEXT NOT NULL,"          // JSON array; frame blob dof order
   "  n_dofs        INTEGER NOT NULL,"
   "  n_vars        INTEGER NOT NULL,"
   "  frame_dt      REAL NOT NULL,"
   "  status        TEXT NOT NULL DEFAULT 'running',"
   "  final_iter    INTEGER,"
   "  hostname      TEXT,"
   "  pid           INTEGER"
   ");"
   "CREATE INDEX IF NOT EXISTS runs_by_scenario"
   "  ON runs(scenario_name, started_at);"
   "CREATE TABLE IF NOT EXISTS iterations ("
   "  run_id        INTEGER NOT NULL REFERENCES runs(id) ON DELETE CASCADE,"
   "  k             INTEGER NOT NULL,"
   "  recorded_at   TEXT NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ','now')),"
   "  objective     REAL NOT NULL,"
   "  infeasibility REAL NOT NULL,"
   "  norm_grd_l    REAL,"
   "  n_frames      INTEGER NOT NULL,"
   "  x             BLOB NOT NULL,"          // n_vars float64, native LE
   "  times         BLOB NOT NULL,"          // n_frames float64
   "  frames        BLOB NOT NULL,"          // n_frames*n_dofs*2 float64
   "  stage_bounds  TEXT NOT NULL,"          // JSON cumulative stage end times
   "  PRIMARY KEY (run_id, k)"
   ") WITHOUT ROWID;";

// the strings we store are identifiers and ISO numbers; escape minimally
static string jsonString(const string &s) {
   string out = "\"";
   for (size_t i = 0; i < s.size(); i ++) {
      if (s[i] == '"' || s[i] == '\\') {
         out += '\\';
      }
      out += s[i];
   }
   return out + "\"";
}

static string jsonArray(const vector<string> &v) {
   string out = "[";
   for (size_t i = 0; i < v.size(); i ++) {
      out += (i ? "," : "") + jsonString(v[i]);
   }
   return out + "]";
}

static string jsonArray(const vector<double> &v) {
   std::ostringstream o;
   o.precision(17);
   o << "[";
   for (size_t i = 0; i < v.size(); i ++) {
      o << (i ? "," : "") << v[i];
   }
   o << "]";
   return o.str();
}

static bool exec(sqlite3 *db, const char *sql, string &err) {
   char *msg = 0;
   if (sqlite3_exec(db, sql, 0, 0, &msg) != SQLITE_OK) {
      err = msg ? msg : "unknown sqlite error";
      sqlite3_free(msg);
      return false;
   }
   return true;
}

static void bindBlob(sqlite3_stmt *st, int ix, const vector<double> &v) {
   sqlite3_bind_blob64(st, ix, v.empty() ? (const void *) "" : (const void *) &v[0],
                       v.size() * sizeof(double), SQLITE_TRANSIENT);
}

Recorder *Recorder::Open(const string &dbPath, const RunInfo &info, string &err) {
   sqlite3 *db = 0;
   if (sqlite3_open(dbPath.c_str(), &db) != SQLITE_OK) {
      err = db ? sqlite3_errmsg(db) : "cannot open database";
      sqlite3_close(db);
      return 0;
   }
   sqlite3_busy_timeout(db, 10000);
   if (!exec(db, "PRAGMA journal_mode=WAL;"
                 "PRAGMA synchronous=NORMAL;"
                 "PRAGMA foreign_keys=ON;", err)) {
      sqlite3_close(db);
      return 0;
   }

   // schema versioning via user_version: 0 = fresh file, else must match
   {
      sqlite3_stmt *st = 0;
      sqlite3_prepare_v2(db, "PRAGMA user_version;", -1, &st, 0);
      int version = sqlite3_step(st) == SQLITE_ROW ? sqlite3_column_int(st, 0) : -1;
      sqlite3_finalize(st);
      if (version != 0 && version != SCHEMA_VERSION) {
         std::ostringstream o;
         o << dbPath << ": schema version " << version
           << " unsupported (want " << SCHEMA_VERSION << ")";
         err = o.str();
         sqlite3_close(db);
         return 0;
      }
   }

   if (!exec(db, DDL, err) ||
       !exec(db, "PRAGMA user_version=1;", err)) {   // == SCHEMA_VERSION
      sqlite3_close(db);
      return 0;
   }

   char host[256] = "";
   gethostname(host, sizeof host);

   sqlite3_stmt *st = 0;
   const char *sql =
      "INSERT INTO runs (scenario_name, scenario_path, scenario_yaml,"
      " creature_yaml, dof_names, n_dofs, n_vars, frame_dt, hostname, pid)"
      " VALUES (?,?,?,?,?,?,?,?,?,?);";
   if (sqlite3_prepare_v2(db, sql, -1, &st, 0) != SQLITE_OK) {
      err = sqlite3_errmsg(db);
      sqlite3_close(db);
      return 0;
   }
   string dofJson = jsonArray(info.dofNames);
   sqlite3_bind_text(st, 1, info.scenarioName.c_str(), -1, SQLITE_TRANSIENT);
   sqlite3_bind_text(st, 2, info.scenarioPath.c_str(), -1, SQLITE_TRANSIENT);
   sqlite3_bind_text(st, 3, info.scenarioYaml.c_str(), -1, SQLITE_TRANSIENT);
   sqlite3_bind_text(st, 4, info.creatureYaml.c_str(), -1, SQLITE_TRANSIENT);
   sqlite3_bind_text(st, 5, dofJson.c_str(), -1, SQLITE_TRANSIENT);
   sqlite3_bind_int(st, 6, (int) info.dofNames.size());
   sqlite3_bind_int(st, 7, info.nVars);
   sqlite3_bind_double(st, 8, info.frameDt);
   sqlite3_bind_text(st, 9, host, -1, SQLITE_TRANSIENT);
   sqlite3_bind_int(st, 10, (int) getpid());

   bool ok = sqlite3_step(st) == SQLITE_DONE;
   sqlite3_finalize(st);
   if (!ok) {
      err = sqlite3_errmsg(db);
      sqlite3_close(db);
      return 0;
   }
   return new Recorder(db, sqlite3_last_insert_rowid(db));
}

void Recorder::RecordIteration(int k, double objective, double infeasibility,
                               double normGrdL,
                               const vector<double> &x,
                               const vector<double> &times,
                               const vector<double> &frames,
                               const vector<double> &stageEnds) {
   sqlite3_stmt *st = 0;
   const char *sql =
      "INSERT OR REPLACE INTO iterations (run_id, k, objective, infeasibility,"
      " norm_grd_l, n_frames, x, times, frames, stage_bounds)"
      " VALUES (?,?,?,?,?,?,?,?,?,?);";
   if (sqlite3_prepare_v2(db, sql, -1, &st, 0) != SQLITE_OK) {
      std::fprintf(stderr, "Recorder: %s\n", sqlite3_errmsg(db));
      return;
   }
   sqlite3_bind_int64(st, 1, run);
   sqlite3_bind_int(st, 2, k);
   sqlite3_bind_double(st, 3, objective);
   sqlite3_bind_double(st, 4, infeasibility);
   sqlite3_bind_double(st, 5, normGrdL);
   sqlite3_bind_int(st, 6, (int) times.size());
   bindBlob(st, 7, x);
   bindBlob(st, 8, times);
   bindBlob(st, 9, frames);
   string bounds = jsonArray(stageEnds);
   sqlite3_bind_text(st, 10, bounds.c_str(), -1, SQLITE_TRANSIENT);

   if (sqlite3_step(st) != SQLITE_DONE) {
      std::fprintf(stderr, "Recorder: %s\n", sqlite3_errmsg(db));
   }
   sqlite3_finalize(st);
}

void Recorder::UpdateNormGrdL(int k, double normGrdL) {
   sqlite3_stmt *st = 0;
   const char *sql =
      "UPDATE iterations SET norm_grd_l = ? WHERE run_id = ? AND k = ?;";
   if (sqlite3_prepare_v2(db, sql, -1, &st, 0) != SQLITE_OK) {
      std::fprintf(stderr, "Recorder: %s\n", sqlite3_errmsg(db));
      return;
   }
   sqlite3_bind_double(st, 1, normGrdL);
   sqlite3_bind_int64(st, 2, run);
   sqlite3_bind_int(st, 3, k);
   if (sqlite3_step(st) != SQLITE_DONE) {
      std::fprintf(stderr, "Recorder: %s\n", sqlite3_errmsg(db));
   }
   sqlite3_finalize(st);
}

void Recorder::Finish(const string &status) {
   sqlite3_stmt *st = 0;
   const char *sql =
      "UPDATE runs SET status = ?,"
      " finished_at = strftime('%Y-%m-%dT%H:%M:%fZ','now'),"
      " final_iter = (SELECT MAX(k) FROM iterations WHERE run_id = ?)"
      " WHERE id = ?;";
   if (sqlite3_prepare_v2(db, sql, -1, &st, 0) != SQLITE_OK) {
      std::fprintf(stderr, "Recorder: %s\n", sqlite3_errmsg(db));
      return;
   }
   sqlite3_bind_text(st, 1, status.c_str(), -1, SQLITE_TRANSIENT);
   sqlite3_bind_int64(st, 2, run);
   sqlite3_bind_int64(st, 3, run);
   if (sqlite3_step(st) != SQLITE_DONE) {
      std::fprintf(stderr, "Recorder: %s\n", sqlite3_errmsg(db));
   }
   sqlite3_finalize(st);
}

Recorder::~Recorder() {
   sqlite3_close(db);
}
