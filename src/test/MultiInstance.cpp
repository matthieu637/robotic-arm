#include <ODEFactory.hpp>
#include "RoboticArmWorld.hpp"
#include "bib/Utils.hpp"


int main(int, char **) {

    ODEFactory::getInstance();
    
    RoboticArmWorld* simu1 = new RoboticArmWorld;   
    
    RoboticArmWorld* simu2 = new RoboticArmWorld;   
    
    simu1->step(bib::Utils::rand01(), bib::Utils::rand01(), bib::Utils::rand01());
    simu2->step(bib::Utils::rand01(), bib::Utils::rand01(), bib::Utils::rand01());
    simu2->step(bib::Utils::rand01(), bib::Utils::rand01(), bib::Utils::rand01());
    simu2->step(bib::Utils::rand01(), bib::Utils::rand01(), bib::Utils::rand01());
 
    simu1->step(bib::Utils::rand01(), bib::Utils::rand01(), bib::Utils::rand01());
    simu1->step(bib::Utils::rand01(), bib::Utils::rand01(), bib::Utils::rand01());
    
    simu2->step(bib::Utils::rand01(), bib::Utils::rand01(), bib::Utils::rand01());
    
    delete simu2;
    delete simu1;
    
    ODEFactory::endInstance();

    return 0;
}
