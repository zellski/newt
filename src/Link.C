# include "Link.h"
# include "Creature.h"
# include "Impulse.h"
# include "World.h"
# include "Stage.h"
# include "DOF.h"
# include "Fun.h"

Link::Link(Stage *const a, Stage *const b) : A(a), B(b) {
   int cnt = 0;
   for (int i = A->W->DOFs.size()-1; i >= 0; i --) {
      DOF *D = A->W->DOFs[i];
      if (!D->DOFReps[A->sIx]->isConstant() || 
	  !D->DOFReps[B->sIx]->isConstant()) {
	 cerr << "Enforcing continuity for " << D->Name << " between "
	      << A->sIx << " and " << B->sIx << "\n";
	 cnt +=2;
      } else {
	 cerr << "Doing nothing for " << D->Name << " between "
	      << A->sIx << " and " << B->sIx << "\n";
      }
   }
   cerr << "Claiming a total of " << cnt << " constraints.\n";
   cIx = A->claimCons(cnt);
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
      if (!D->DOFReps[A->sIx]->isConstant() ||
	  !D->DOFReps[B->sIx]->isConstant()) {
	 c[ix ++] = D->qVal;
	 c[ix ++] = D->qMomentum + D->JVal;
//	 cerr << "Hi! Here, D->JVal is " << D->JVal << "!\n";
      }
   }
   B->SnapShot(x, 0, 0);
   ix = cIx;
   for (int i = A->W->DOFs.size()-1; i >= 0; i --) {
      DOF *D = A->W->DOFs[i];
      if (!D->DOFReps[A->sIx]->isConstant() ||
	  !D->DOFReps[B->sIx]->isConstant()) {
	 c[ix ++] -= D->qVal;
	 c[ix ++] -= D->qMomentum;
      }
   }
}
