#define NO_PARALLEL
#include "ODEFactory.hpp"

#include "RoboticArmWorld.hpp"
#include <bib/Utils.hpp>
#include <drawstuff.h>
#include "Draw.hpp"
#include <RoboticArmWorldView.hpp>

// void way1() {
//     RoboticArmWorld* simu;
//     simu = new RoboticArmWorld();
// 
//     dsFunctions fn;
//     fn.version = DS_VERSION;
//     fn.start = 0;
//     fn.step = &Draw::drawLoop;
//     fn.command = 0;
//     fn.stop = 0;
//     fn.path_to_textures = "data/textures";
// 
//     HACKinitDs(1024, 768, &fn);
// 
//     static float xyz[3] = {0.9,0.10,1.6};
//     static float hpr[3] = {-180,-50,0};
//     dsSetViewpoint (xyz,hpr);
// 
//     while(true) {
//         HACKdraw(&fn);
//         simu->step(bib::Utils::rand01(), bib::Utils::rand01(), bib::Utils::rand01());
//     }
// }

void way2() {
    RoboticArmWorldView simu("data/textures");

    for(int n=0;n<6; n++){
      simu.resetPositions((goalType)(n% NUMBER_GOAL), n/NUMBER_GOAL);
    for(int i=0;i<100;i++) {
        simu.step(bib::Utils::rand01(), bib::Utils::rand01(), 0);
    }
    }

}


int main(int, char **) {

//     way1();
    way2();
    ODEFactory::endInstance();

    return 0;
}
