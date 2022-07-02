
#include "DynamicWeightProvider.h"

using namespace std;

MAKE_MODULE(DynamicWeightProvider, modeling);

DynamicWeightProvider::DynamicWeightProvider(){}

void DynamicWeightProvider::update(DynamicWeight& dynamicWeight){
  dynamicWeight.ballWeight = 0.9f;//Cerca al balón = 1.8
  dynamicWeight.areaWeight = 1.65f;//Cerca al balón = 3.25
  dynamicWeight.cornerWeight = 0.f;
  dynamicWeight.arcoWeight = 0.25f;
}