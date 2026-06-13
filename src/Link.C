# include "Stage.h"
# include "Link.h"
# include "Creature.h"
# include "Impulse.h"
# include "World.h"
# include "DOF.h"
# include "Fun.h"

Link::Link(Stage *const a, Stage *const b) : A(a), B(b) {
   int cnt = 0;
   for (int i = A->W->DOFs.size()-1; i >= 0; i --) {
      DOF *D = A->W->DOFs[i];
      if (!D->Rep(A->sIx)->isConstant() ||
          !D->Rep(B->sIx)->isConstant()) {
         cerr << "Enforcing continuity for " << D->Name << " between "
              << A->sIx << " and " << B->sIx << "\n";
         cnt +=2;
      } else {
         cerr << "Doing nothing for " << D->Name << " between "
              << A->sIx << " and " << B->sIx << "\n";
      }
   }
   cerr << "Claiming a total of " << cnt << " constraints.\n";
   // every DOF constant (and agreeing) on both stages -> nothing to
   // enforce; claim no rows and stay unregistered so Evaluate is never
   // called. chain-by-default makes such links the common case.
   if (cnt == 0) {
      cIx = -1;
      return;
   }
   cIx = A->claimCons(cnt);
   int ix = cIx;
   for (int i = A->W->DOFs.size()-1; i >= 0; i --) {
      DOF *D = A->W->DOFs[i];
      if (!D->Rep(A->sIx)->isConstant() ||
          !D->Rep(B->sIx)->isConstant()) {
         string prefix = string("link ") + A->Name + "->" + B->Name +
            ": " + D->Name;
         A->W->LabelCon(ix ++, prefix + " continuity");
         A->W->LabelCon(ix ++, prefix + " momentum");
      }
   }
   A->Register((Constraint * const) this);
}

void Link::Evaluate(const adoublev &x, adoublev &c) {
   A->SnapShot(x, A->N-1, 1);
   for (vector<Impulse *>::const_iterator p = A->Impulses.begin();
        p != A->Impulses.end(); p++) {
      (*p)->SnapShot(x);
   }
   for (vector<Creature *>::const_iterator p = A->W->Creatures.begin();
        p != A->W->Creatures.end(); p++) {
      (*p)->DistImpulse(x);
   }
   int ix = cIx;
   for (int i = A->W->DOFs.size()-1; i >= 0; i --) {
      DOF *D = A->W->DOFs[i];
      if (!D->Rep(A->sIx)->isConstant() ||
          !D->Rep(B->sIx)->isConstant()) {
         c[ix ++] = D->qVal;
         c[ix ++] = D->qMomentum + D->JVal;
//         cerr << "Hi! Here, D->JVal is " << D->JVal << "!\n";
      }
   }
   B->SnapShot(x, 0, 0);
   ix = cIx;
   for (int i = A->W->DOFs.size()-1; i >= 0; i --) {
      DOF *D = A->W->DOFs[i];
      if (!D->Rep(A->sIx)->isConstant() ||
          !D->Rep(B->sIx)->isConstant()) {
         c[ix ++] -= D->qVal;
         c[ix ++] -= D->qMomentum;
      }
   }
}
