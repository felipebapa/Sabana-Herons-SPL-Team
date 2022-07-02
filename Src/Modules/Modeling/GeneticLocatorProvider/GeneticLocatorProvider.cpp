#include "GeneticLocatorProvider.h"

using namespace std;

#define BALLWEIGHT  0.9f
#define AREAWEIGHT  1.65f
#define CORNERWEIGHT 0.f
#define ARCOWEIGHT 0.25f

MAKE_MODULE(GeneticLocatorProvider, modeling);

GeneticLocatorProvider::GeneticLocatorProvider(){}

void GeneticLocatorProvider::update(GeneticLocator& geneticLocator){
	this -> geneticLocatorr = &geneticLocator;

	geneticLocator.activation = [this](bool activates, int ctime, float ballWeight) -> Pose2f
	{
		if(activates && ctime - theGeneticLocator.lastime > 1500){
			vector<vector<float>> poblacion;
			vector<vector<int>> posbls;

			for (int i = -40; i <= 0; i++){
				for (int j = -45; j <= 45; j++){
					posbls.push_back({i*90,j*60});
				}
			}

			for (int w = 0; w < 3731; w++)
			{
				poblacion.push_back({static_cast<float>(posbls[w][0]), static_cast<float>(posbls[w][1]), this -> result(posbls[w][0],posbls[w][1],ballWeight,activates)});
			}

			for(int ym = 0; ym < 3731; ym++){
				if(poblacion[ym][2] < poblacion[0][2]){
					vector<float> aux = poblacion[0];
					poblacion[0] = poblacion[ym];
					poblacion[ym] = aux;
				}
			}
			this -> geneticLocatorr -> optimalX = poblacion[0][0];
			this -> geneticLocatorr -> optimalY = poblacion[0][1];

			this -> geneticLocatorr -> Hmtimes = theGeneticLocator.Hmtimes + 1;
			this -> geneticLocatorr -> lastime = ctime;
		}

		Vector2f lul = {this -> geneticLocatorr -> optimalX,this -> geneticLocatorr -> optimalY};
		// return Pose2f((theRobotPose.inversePose * ((Vector2f){theTeamBallModel.position.x(),theTeamBallModel.position.y()})).angle(),theRobotPose.inversePose * lul);
		// WalkToTarget(Pose2f(1.f, 1.f, 1.f), theGeneticLocator.activation(true, (int)theFrameInfo.time));
		return Pose2f((theRobotPose.inversePose * ((Vector2f){theTeamBallModel.position.x(),theTeamBallModel.position.y()})).angle(),lul);
	};

	geneticLocator.optimalX = theGeneticLocator.optimalX;
	geneticLocator.optimalY = theGeneticLocator.optimalY;
	geneticLocator.lastime = theGeneticLocator.lastime;
	geneticLocator.Hmtimes = theGeneticLocator.Hmtimes;
}

float GeneticLocatorProvider::result(int a, int b, float ballWeight, bool activates) {
	float area = 0;
	float ball;
	float corner = (float)(pow((a - 0), 2) + pow((b - 0), 2));
	float arco = (float)(pow((a - (-4500)), 2) + pow((b - 0), 2));

	if(activates){
		ball = (float)(pow((a - theTeamBallModel.position.x()), 2) + pow((b -  theTeamBallModel.position.y()), 2));
	}
	else{
		ball = (float)(pow((a - theBallModel.estimate.position.x()), 2) + pow((b -  theBallModel.estimate.position.y()), 2));

		return (ball * 0.75f) + (arco * 0.25f);
	}

	if(theTeamData.teammates.size()>0){
		std::vector<std::vector<float>> vov = ordenador(a,b);
		int tam = (int)vov.size()-1;

		for (int i = 0; i < tam; i++)
			area = area + ((vov[i][0] * vov[i+1][1]) - (vov[i][1] * vov[i+1][0]));
		area *= -0.5;
	}
	else if(theTeamData.teammates.size() == 1){
			area = (float)(sqrt(pow(abs(theTeamData.teammates[0].theRobotPose.translation.y()-b),2) + pow(abs(theTeamData.teammates[0].theRobotPose.translation.x()-a),2)));
			//area = (float)(sqrt(pow(abs(3000-(-3000)),2) + pow(abs(0-0),2)));
	}

	return (ball * ballWeight) - (area * theDynamicWeight.areaWeight) + (corner * theDynamicWeight.cornerWeight) + (arco * theDynamicWeight.arcoWeight);
}

vector<vector<float>> GeneticLocatorProvider::ordenador(int a, int b){
	vector<vector<float>> vov = {{static_cast<float>(b),static_cast<float>(a)},{theTeamBallModel.position.y(),theTeamBallModel.position.x()}};

	for (int i = 0; i < int(theTeamData.teammates.size()); i++)
		if(theTeamData.teammates[i].status == Teammate::PLAYING && !theTeamData.teammates[i].isGoalkeeper)
			vov.push_back({theTeamData.teammates[i].theRobotPose.translation.y(),theTeamData.teammates[i].theRobotPose.translation.x()});

	int num_compas = (int)vov.size();

	for(int ym = 1; ym < num_compas; ym++){
    if(vov[ym][1] < vov[0][1]){
      std::vector<float> aux = vov[0];
      vov[0] = vov[ym];
      vov[ym] = aux;
    }
    else if(vov[ym][1] == vov[0][1])
      if(vov[ym][0] < vov[0][0]){
        std::vector<float> aux = vov[0];
        vov[0] = vov[ym];
        vov[ym] = aux;
      }
  }

	vector<float> point = vov[0];

	for(int ct = 1; ct < num_compas; ct++){
    float cost = 0;
    float y = vov[ct][1] - point[1];
    float x = vov[ct][0] - point[0];

		cost = (x == 0)? y + 150 : cost = y / x;

    if (cost < 0)
      cost = 22500 / abs(cost) + 9001;

    vov[ct].push_back(cost);
  }

	for (int i = 1; i < int(vov.size()); i++)
    for (int j = 1; j < int(vov.size()); j++)
      if (vov[i][2]>vov[j][2]){
        vector<float> aux = vov[i];
        vov[i] = vov[j];
        vov[j] = aux;
      }

	vov.push_back(point);

	return vov;
}