# include "Stage.h"
# include "AnchorPoint.h"
# include "BodyPoint.h"
# include "Constraints.h"

AnchorPoint::AnchorPoint(const char *const S) :
   Name(S),
   AttachedPoints(),
   TotF(2), TotJ(2),
   Val(2), Dot(2), Bis(2)
{};

void AnchorPoint::Attach(BodyPoint *const P) {
   AttachedPoints.push_back(P);
}
