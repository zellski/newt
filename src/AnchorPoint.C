# include "Stage.h"
# include "AnchorPoint.h"
# include "BodyPoint.h"
# include "Constraints.h"

AnchorPoint::AnchorPoint(const char *const S) :
   Name(S),
   AttachedPoints()
{};

void AnchorPoint::Attach(BodyPoint *const P) {
   AttachedPoints.push_back(P);
}
