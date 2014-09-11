
#include "RoboticArmWorldView.hpp"
#include <boost/filesystem.hpp>
#include "Draw.hpp"
#include "bib/Logger.hpp"

static RoboticArmWorldView* inst;

void parseCommand(int cmd) {
    float xyz[3] = {0.9,0.10,1.6};
    float hpr[3] = {-180,-50,0};
    float xyz2[3] = {1.,-1.7,1.34};
    float hpr2[3] = {124,-28.5,0};

    switch (cmd) {
    case 'f':
        inst->speedUp = true;
        break;
    case 'd':
        inst->speedUp = false;
        break;
    case 'a':
        inst->motorz=1;
        break;
    case 'z':
        inst->motorz=0;
        break;
    case 'q':
        inst->motory=1;
        break;
    case 's':
        inst->motory=0;
        break;
    case 'w':
        inst->motorx=1;
        break;
    case 'x':
        inst->motorx=0;
        break;
    case 'i':
        dsSetViewpoint (xyz,hpr);
        break;
    case 'j':
        dsSetViewpoint (xyz2,hpr2);
        break;
    case 'r':
        inst->resetPositions(MOVE_LEFT, dRandInt(3));
        LOG_DEBUG("resetPositions should not be used");
        break;
    }
}

void threadloop(const std::string& goodpath) {
    inst->fn.version = DS_VERSION;
    inst->fn.start = 0;
    inst->fn.step = &Draw::drawLoop;
    inst->fn.command = &parseCommand;
    inst->fn.stop = 0;
    inst->fn.path_to_textures = goodpath.c_str();

    Draw::geoms = &inst->geoms;

    HACKinitDs(1280, 720, &inst->fn);

    static float xyz[3] = {0.9,0.10,1.6};
    static float hpr[3] = {-180,-50,0};
    dsSetViewpoint (xyz,hpr);

    while(!inst->requestEnd)
    {
        HACKdraw(&inst->fn);
        usleep(10 *1000);
    }
}

RoboticArmWorldView::RoboticArmWorldView(const std::string& path) : RoboticArmWorld(),speedUp(false), requestEnd(false), motorx(-1), motory(-1), motorz(-1)
{
    std::string goodpath = path;

    int n;
    for(n=0; n<5; n++)
        if(!boost::filesystem::exists(goodpath)) {
            LOG_DEBUG(goodpath << " doesnt exists");
            goodpath = std::string("../") + goodpath;
        }
        else break;

    if(n >= 5) {
        LOG_ERROR("cannot found " << path);
        exit(1);
    }
    inst = this;

    ODEObject* foot1 = ODEFactory::getInstance()->createBox(odeworld, table->getX() + ((ODEBox*)table)->getLX()/2. - 0.1/2.f,
                       table->getY() + ((ODEBox*)table)->getLY()/2. - 0.1/2.f,
                       (table->getZ()- ((ODEBox*)table)->getLZ()/2)/2, 0.1, 0.1,table->getZ() - ((ODEBox*)table)->getLZ()/2,1,1, false);
    dGeomSetPosition(foot1->getGeom(), foot1->getX(), foot1->getY(), foot1->getZ());
    geoms.push_back(foot1->getGeom());

    ODEObject* foot2 = ODEFactory::getInstance()->createBox(odeworld, table->getX() - ((ODEBox*)table)->getLX()/2. + 0.1/2.f,
                       table->getY() + ((ODEBox*)table)->getLY()/2. - 0.1/2.f,
                       (table->getZ()- ((ODEBox*)table)->getLZ()/2)/2, 0.1, 0.1,table->getZ() - ((ODEBox*)table)->getLZ()/2,1,1, false);
    dGeomSetPosition(foot2->getGeom(), foot2->getX(), foot2->getY(), foot2->getZ());
    geoms.push_back(foot2->getGeom());

    ODEObject* foot3 = ODEFactory::getInstance()->createBox(odeworld, table->getX() + ((ODEBox*)table)->getLX()/2. - 0.1/2.f,
                       table->getY() - ((ODEBox*)table)->getLY()/2. + 0.1/2.f,
                       (table->getZ()- ((ODEBox*)table)->getLZ()/2)/2, 0.1, 0.1,table->getZ() - ((ODEBox*)table)->getLZ()/2,1,1, false);
    dGeomSetPosition(foot3->getGeom(), foot3->getX(), foot3->getY(), foot3->getZ());
    geoms.push_back(foot3->getGeom());
//
    ODEObject* foot4 = ODEFactory::getInstance()->createBox(odeworld, table->getX() - ((ODEBox*)table)->getLX()/2. + 0.1/2.f,
                       table->getY() - ((ODEBox*)table)->getLY()/2. + 0.1/2.f,
                       (table->getZ()- ((ODEBox*)table)->getLZ()/2)/2, 0.1, 0.1,table->getZ() - ((ODEBox*)table)->getLZ()/2,1,1, false);
    dGeomSetPosition(foot4->getGeom(), foot4->getX(), foot4->getY(), foot4->getZ());
    geoms.push_back(foot4->getGeom());

    ODEObject* head = ODEFactory::getInstance()->createSphere(odeworld, human->getX(), human->getY(), human->getZ()+((ODEBox*)human)->getLZ()/2+0.135, 0.135, 1, 1, false);
    dGeomSetPosition(head->getGeom(), head->getX(), head->getY(), head->getZ());


    geoms.push_back(table->getGeom());
    geoms.push_back(human->getGeom());
    geoms.push_back(head->getGeom());
    geoms.push_back(zLevel->getGeom());
    geoms.push_back(yLevel->getGeom());
    geoms.push_back(xLevel->getGeom());
    geoms.push_back(hand->getGeom());
    geoms.push_back(ogoal->getGeom());

    delete_me_later.push_back(foot1);
    delete_me_later.push_back(foot2);
    delete_me_later.push_back(foot3);
    delete_me_later.push_back(foot4);
    delete_me_later.push_back(head);

    eventThread = new tbb::tbb_thread(threadloop,goodpath);
}

RoboticArmWorldView::~RoboticArmWorldView()
{
    for(auto it=delete_me_later.begin(); it != delete_me_later.end(); ++it) {
        dGeomDestroy((*it)->getGeom());
        delete *it;
    }

    requestEnd=true;
    eventThread->join();
    delete eventThread;
    HACKclose();
}


void RoboticArmWorldView::step(float mx, float my, float mz)
{
    if(!speedUp) {
        if(motorx + motory + motorz != -3) {
            mx=0.5;
            my=0.5;
            mz=0.5;
        }

        if(motorx != -1)
            mx =motorx;
        if(motory != -1)
            my = motory;
        if(motorz != -1)
            mz = motorz;
        motorx = -1;
        motory = -1;
        motorz = -1;
    }

    RoboticArmWorld::step(mx, my, mz);

    if(!speedUp)
        usleep(250*1000);

    usleep(20*1000);//needed to don't be faster than the view
}

