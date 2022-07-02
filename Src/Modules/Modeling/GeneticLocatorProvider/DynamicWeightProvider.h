#include "Tools/Module/Module.h"
#include "Representations/Modeling/DynamicWeight.h"
#include "Representations/Modeling/RobotPose.h"

MODULE(DynamicWeightProvider,
{,
  REQUIRES(RobotPose),
  PROVIDES(DynamicWeight),
});

class DynamicWeightProvider : public DynamicWeightProviderBase
{
public:
  DynamicWeightProvider();

private:
	void update(DynamicWeight& dynamicWeight);
};