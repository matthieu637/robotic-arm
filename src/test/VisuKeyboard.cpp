#define NO_PARALLEL

#include <ode/ode.h>
#include <drawstuff.h>
#include <utility>
#include <ODEFactory.hpp>
#include "bib/Logger.hpp"
#include <RoboticArmWorld.hpp>
#include "bib/Prober.hpp"
#include "Draw.hpp"

RoboticArmWorld* simu;
bib::Prober probme;
float motorx, motory, motorz;

void start()
{
    dAllocateODEDataForThread(dAllocateMaskAll);

    static float xyz[3] = {0.9,0.10,1.6};
    static float hpr[3] = {-180,-50,0};
    dsSetViewpoint (xyz,hpr);

    float xyz2[3] = {1.,-1.7,1.34};
    float hpr2[3] = {124,-28.5,0};
    dsSetViewpoint (xyz2,hpr2);
}

void command (int cmd)
{
    float xyz[3] = {0.9,0.10,1.6};
    float hpr[3] = {-180,-50,0};
    float xyz2[3] = {1.,-1.7,1.34};
    float hpr2[3] = {124,-28.5,0};

    switch (cmd) {
    case 'a':
        motorz=1;
        break;
    case 'z':
        motorz=0;
        break;
    case 'q':
        motory=1;
        break;
    case 's':
        motory=0;
        break;
    case 'w':
        motorx=1;
        break;
    case 'x':
        motorx=0;
        break;
    case 'i':
        dsSetViewpoint (xyz,hpr);
        break;
    case 'j':
        dsSetViewpoint (xyz2,hpr2);
        break;
    case 'r':
        simu->resetPositions(MOVE_LEFT, dRandInt(3));
    }

//     LOG_DEBUG(cmd);
}
int nn = 0;
void simLoop (int)
{

    simu->step(motorx, motory, motorz);

    motorx=0.5;
    motory=0.5;
    motorz=0.5;

    Draw::drawLoop(0);
    nn++;
//     probme.probe(simu->sensorTY2());

//      if(nn % 5 == 0)
//        LOG_DEBUG(simu->sensorMX() << " " << simu->sensorMY() << " " << simu->sensorMZ() << " " <<
// 		 simu->sensorD1X() << " " << simu->sensorD2X() << " " << simu->sensorD1Y() << " " <<
// 		 simu->sensorD1Y() << " " << simu->sensorDZ() << " " << simu->sensorCX() << " " <<
// 		 simu->sensorCY() <<  " " << simu->sensorTX1() << " " << simu->sensorTY1() << " "<<
// 		 simu->sensorTY2()
//
//       );
//        LOG_DEBUG(simu->sensorTY2());

//     const dReal* pos = dGeomGetPosition(simu->hand->getGeom());
//     LOG_DEBUG(pos[0] << " " << pos[1] << " " << pos[2]);
}

int main(int argc, char ** argv) {

    srand(time(nullptr));

    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.command = &command;
    fn.stop = 0;
    fn.path_to_textures = "data/textures";

    simu = new RoboticArmWorld();
//     simu->resetPositions(MOVE_RIGHT);

    ODEObject* foot1 = ODEFactory::getInstance()->createBox(simu->odeworld, simu->table->getX() + ((ODEBox*)simu->table)->getLX()/2. - 0.1/2.f,
                       simu->table->getY() + ((ODEBox*)simu->table)->getLY()/2. - 0.1/2.f,
                       (simu->table->getZ()- ((ODEBox*)simu->table)->getLZ()/2)/2, 0.1, 0.1,simu->table->getZ() - ((ODEBox*)simu->table)->getLZ()/2,1,1, false);
    dGeomSetPosition(foot1->getGeom(), foot1->getX(), foot1->getY(), foot1->getZ());


    ODEObject* foot2 = ODEFactory::getInstance()->createBox(simu->odeworld, simu->table->getX() - ((ODEBox*)simu->table)->getLX()/2. + 0.1/2.f,
                       simu->table->getY() + ((ODEBox*)simu->table)->getLY()/2. - 0.1/2.f,
                       (simu->table->getZ()- ((ODEBox*)simu->table)->getLZ()/2)/2, 0.1, 0.1,simu->table->getZ() - ((ODEBox*)simu->table)->getLZ()/2,1,1, false);
    dGeomSetPosition(foot2->getGeom(), foot2->getX(), foot2->getY(), foot2->getZ());

    ODEObject* foot3 = ODEFactory::getInstance()->createBox(simu->odeworld, simu->table->getX() + ((ODEBox*)simu->table)->getLX()/2. - 0.1/2.f,
                       simu->table->getY() - ((ODEBox*)simu->table)->getLY()/2. + 0.1/2.f,
                       (simu->table->getZ()- ((ODEBox*)simu->table)->getLZ()/2)/2, 0.1, 0.1,simu->table->getZ() - ((ODEBox*)simu->table)->getLZ()/2,1,1, false);
    dGeomSetPosition(foot3->getGeom(), foot3->getX(), foot3->getY(), foot3->getZ());

//
    ODEObject* foot4 = ODEFactory::getInstance()->createBox(simu->odeworld, simu->table->getX() - ((ODEBox*)simu->table)->getLX()/2. + 0.1/2.f,
                       simu->table->getY() - ((ODEBox*)simu->table)->getLY()/2. + 0.1/2.f,
                       (simu->table->getZ()- ((ODEBox*)simu->table)->getLZ()/2)/2, 0.1, 0.1,simu->table->getZ() - ((ODEBox*)simu->table)->getLZ()/2,1,1, false);
    dGeomSetPosition(foot4->getGeom(), foot4->getX(), foot4->getY(), foot4->getZ());


    ODEObject* head = ODEFactory::getInstance()->createSphere(simu->odeworld, simu->human->getX(), simu->human->getY(), simu->human->getZ()+((ODEBox*)simu->human)->getLZ()/2+0.135, 0.135, 1, 1, false);
    dGeomSetPosition(head->getGeom(), head->getX(), head->getY(), head->getZ());

    std::list<dGeomID> geoms;

    geoms.push_back(foot1->getGeom());
    geoms.push_back(foot2->getGeom());
    geoms.push_back(foot3->getGeom());
    geoms.push_back(foot4->getGeom());

    geoms.push_back(simu->table->getGeom());
    geoms.push_back(simu->human->getGeom());
    geoms.push_back(simu->zLevel->getGeom());
    geoms.push_back(simu->yLevel->getGeom());
    geoms.push_back(simu->xLevel->getGeom());
    geoms.push_back(simu->hand->getGeom());
    geoms.push_back(head->getGeom());
    geoms.push_back(simu->ogoal->getGeom());

    Draw::geoms = &geoms;

    //TODO:free memory not handled

    dsSimulationLoop (argc,argv,1024,768,&fn);

    ODEFactory::endInstance();

    return 0;
}




