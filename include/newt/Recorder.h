/*
**        Structured run/iteration storage in a per-project SQLite file.
**        One Recorder instance per solver run; multiple concurrent runs
**        may append to the same database (WAL mode + busy timeout).
**
**        Deliberately free of solver and YAML types -- unit-testable
**        standalone. Recording failures warn on stderr rather than
**        aborting the optimization.
*/

# pragma once

# include <string>
# include <vector>

struct sqlite3;

namespace newt {

struct RunInfo {
   std::string scenarioName;
   std::string scenarioPath;     // absolute path as launched; may be empty
   std::string scenarioYaml;     // full text snapshot; empty for legacy demos
   std::string creatureYaml;     // ditto
   std::vector<std::string> dofNames;   // order == frame blob dof order
   int nVars = 0;                // length of the x blobs
   double frameDt = 0.01;
};

class Recorder {
   sqlite3 *db;
   long long run;

   Recorder(sqlite3 *d, long long r) : db(d), run(r) {}

public:
   // opens/creates the database, checks the schema version, inserts the
   // runs row; returns null (with err set) on failure
   static Recorder *Open(const std::string &dbPath, const RunInfo &info,
                         std::string &err);

   // one accepted SQP iterate; frames holds nFrames*nDofs*2 doubles
   // (q, qdot pairs, frame-major); times holds nFrames global times.
   // pass NaN for normGrdL to store NULL (value not yet computed)
   void RecordIteration(int k, double objective, double infeasibility,
                        double normGrdL,
                        const std::vector<double> &x,
                        const std::vector<double> &times,
                        const std::vector<double> &frames,
                        const std::vector<double> &stageEnds);

   // the Lagrangian gradient norm for an accepted iterate only becomes
   // available at the next iteration's QP update; this backfills it
   void UpdateNormGrdL(int k, double normGrdL);

   // stamps status/finished_at/final_iter on the runs row
   void Finish(const std::string &status);

   long long runId() const { return run; }

   ~Recorder();
};

}
