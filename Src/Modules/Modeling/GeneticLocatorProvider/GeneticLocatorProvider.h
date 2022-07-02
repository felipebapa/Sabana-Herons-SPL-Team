#include "Tools/Module/Module.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/GeneticLocator.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/DynamicWeight.h"

MODULE(GeneticLocatorProvider,
{,
  REQUIRES(TeamData),
  REQUIRES(DynamicWeight),
  REQUIRES(TeamBallModel),
  REQUIRES(RobotPose),
  REQUIRES(BallModel),
  REQUIRES(GeneticLocator),
  PROVIDES(GeneticLocator),
});

class GeneticLocatorProvider : public GeneticLocatorProviderBase
{
public:
  GeneticLocator* geneticLocatorr;
	GeneticLocatorProvider();
	float result(int a, int b, float ballWeight, bool activates);
  std::vector<std::vector<float>> ordenador(int a, int b);

private:
	void update(GeneticLocator& geneticLocator);
};