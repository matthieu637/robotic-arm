#include "RoboticArmWorld.hpp"
#include "ODEFactory.hpp"
#include <bib/Utils.hpp>
#include <ode/ode.h>
#include <functional>


RoboticArmWorld::RoboticArmWorld():zkeep(-40), odeworld(ODEFactory::getInstance()->createWorld())
{
    dWorldSetGravity(odeworld.world_id, 0, 0.0, -10.0);
//     dWorldSetLinearDamping(odeworld.world_id, 0.1);

    createWorld();
}

RoboticArmWorld::~RoboticArmWorld()
{

    dGeomDestroy(ogoal->getGeom());
    dBodyDestroy(ogoal->getID());
    delete ogoal;

    dGeomDestroy(hand->getGeom());
    delete hand;

    dJointDestroy(xAndZ);

    dGeomDestroy(xLevel->getGeom());
    dBodyDestroy(xLevel->getID());
    delete xLevel;

    dJointDestroy(yAndZ);

    dGeomDestroy(yLevel->getGeom());
    dBodyDestroy(yLevel->getID());
    delete yLevel;

    dJointDestroy(zAndHuman);

    dGeomDestroy(zLevel->getGeom());
    dBodyDestroy(zLevel->getID());
    delete zLevel;

    dGeomDestroy(human->getGeom());
    delete human;

    dGeomDestroy(table->getGeom());
    delete table;

    dGeomDestroy(ground);

    ODEFactory::getInstance()->destroyWorld(odeworld);
}

void RoboticArmWorld::createWorld()
{
    float table_lx = 0.7;
    float table_ly = 1.1;

    ground = ODEFactory::getInstance()->createGround(odeworld);
    //table
//     table = ODEFactory::getInstance()->createBox(odeworld, 0.2,0,0.4 - 0.06, table_lx, table_ly,0.7,1,1, false);
    table = ODEFactory::getInstance()->createBox(odeworld, 0.2,0,0.4 - 0.06 + 0.6/2., table_lx, table_ly,0.1,1,1, false);
    dGeomSetPosition(table->getGeom(), table->getX(), table->getY(), table->getZ());

    //human geom
    human = ODEFactory::getInstance()->createBox(odeworld, 0.67, 0.38,0.9/2. ,0.2,0.4,0.9,1, 1, false);
    dGeomSetPosition(human->getGeom(), human->getX(), human->getY(), human->getZ());

    zLevel = ODEFactory::getInstance()->createBox(odeworld, 0.62,-0.1, 0.75, 0.05, table_ly*0.5,0.05,1,1);

    zAndHuman = dJointCreateSlider(odeworld.world_id, nullptr);
    dJointAttach(zAndHuman, 0, zLevel->getID());
    dJointSetSliderAxis(zAndHuman, 0., 0.,1.f);
    dJointSetSliderParam(zAndHuman, dParamLoStop, - 0.02);
    dJointSetSliderParam(zAndHuman, dParamHiStop, 0.08);
    dJointSetSliderParam(zAndHuman, dParamStopERP, 0.);
    dBodySetDamping(zLevel->getID(), 1, 1);

    yLevel = ODEFactory::getInstance()->createBox(odeworld, 0.62,0, 0.778, 0.05,0.05,0.001,1,1);

    yAndZ = dJointCreateSlider(odeworld.world_id, nullptr);
    dJointAttach(yAndZ, zLevel->getID(), yLevel->getID());
    dJointSetSliderAxis(yAndZ, 0.f, 1.f, 0);
    dJointSetSliderParam(yAndZ, dParamLoStop, -0.11);
    dJointSetSliderParam(yAndZ, dParamHiStop, 0.34);
    dJointSetSliderParam(yAndZ, dParamStopERP, 0.);
    dBodySetDamping(yLevel->getID(), 1, 1);


    xLevel = ODEFactory::getInstance()->createBox(odeworld, 0.4,0, 0.802, table_lx*0.75,0.04,0.04,1,1);


    hand = ODEFactory::getInstance()->createBox(odeworld, 0.,0, 0.802, 0.08, 0.1,0.04,1,1, false);


    //composition
    dGeomSetBody(hand->getGeom(), xLevel->getID());
    hand->setID(xLevel->getID());
    dGeomSetOffsetPosition(hand->getGeom(), -0.3,0,0);

    dMassAdd(&xLevel->getMass(), &hand->getMass());
    dBodySetMass(xLevel->getID(), &xLevel->getMass());
    dBodySetDamping(xLevel->getID(), 1, 1);


    xAndZ = dJointCreateSlider(odeworld.world_id, nullptr);
    dJointAttach(xAndZ, yLevel->getID(), xLevel->getID());
    dJointSetSliderAxis(xAndZ, 1.f, 0., 0);
    dJointSetSliderParam(xAndZ, dParamLoStop, -0.5);
    dJointSetSliderParam(xAndZ, dParamHiStop, 0.);
    dJointSetSliderParam(xAndZ, dParamStopERP, 0.);

    ogoal = ODEFactory::getInstance()->createBox(odeworld, 0.2, -0.4, 1.2, 0.08,0.08,0.04,0.01,0.01);
    dBodySetDamping(ogoal->getID(), 0.00001, 0.00001);
}

void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
    int i,n;
    nearCallbackData* d = (nearCallbackData*) data;
    RoboticArmWorld* inst = d->inst;

    // exit without doing anything if the two bodies are connected by a joint
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;

    const int N = 10;
    dContact contact[N];
    n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
    if (n > 0) {
        if( (o1 == inst->ogoal->getGeom() || o2 == inst->ogoal->getGeom()) &&
                (o1 == inst->hand->getGeom() || o2 == inst->hand->getGeom())) {
            i=0;


            const dReal* pos_o = dGeomGetPosition(inst->ogoal->getGeom());
            const dReal* pos_h = dGeomGetPosition(inst->hand->getGeom());

            const float difficulty = 0.02;

            if(contact[i].geom.normal[0] >= 0.98 && bib::Utils::euclidien_dist2D(pos_o[1], pos_h[1], pos_o[2], pos_h[2]) < difficulty  ){
//                 LOG_DEBUG("GOAL : touched down");
                inst->goalReached(MOVE_FORWARD);
	    }
            else if (contact[i].geom.normal[1] >= 0.98 && bib::Utils::euclidien_dist2D(pos_o[0], pos_h[0], pos_o[2], pos_h[2]) < difficulty){
//                 LOG_DEBUG("GOAL : touched left");
                inst->goalReached(MOVE_LEFT);
	    }
            else if (contact[i].geom.normal[1] <= -0.98 && bib::Utils::euclidien_dist2D(pos_o[0], pos_h[0], pos_o[2], pos_h[2]) < difficulty){
//                 LOG_DEBUG("GOAL : touched right");
                inst->goalReached(MOVE_RIGHT);
	    }
            else if (contact[i].geom.normal[2] >= 0.98 && bib::Utils::euclidien_dist2D(pos_o[0], pos_h[0], pos_o[1], pos_h[1]) < difficulty) {
//                 LOG_DEBUG("GOAL : touched up");
                inst->goalReached(GRAP);
            } 
            else {
// 	      LOG_DEBUG("GOAL FAILED (but touched)");
	      inst->goalReached(NUMBER_GOAL);//failed
	    }
	    
	    if (contact[i].geom.normal[1] >= 0.98){
// 		LOG_DEBUG("SUB GOAL : touched left");
		inst->touchedLeft = true;
	    }

        } else if ( (o1 == inst->ogoal->getGeom() || o2 == inst->ogoal->getGeom() ) &&
                    (o1 == inst->ground || o2 == inst->ground )) {
            inst->goalReached(TO_FLOOR);
// 		LOG_DEBUG("GOAL : to floor");
        }

//         if((o1 == inst->ground || o2 == inst->ground ) &&
//                 ( (o1 == inst->human->getGeom() || o2 == inst->human->getGeom() ) || (o1 == inst->table->getGeom() || o2 == inst->table->getGeom() ) ))
//         {
//             return;
//         }

        for (i=0; i<n; i++) {
            contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
                                      dContactSoftERP | dContactSoftCFM | dContactApprox1 ;
            contact[i].surface.mu = dInfinity;
            contact[i].surface.slip1 = 0.5;
            contact[i].surface.slip2 = 0.5;
            contact[i].surface.soft_erp = 0.95;
            contact[i].surface.soft_cfm = 0.5;

            dJointID c = dJointCreateContact (inst->odeworld.world_id,
                                              inst->odeworld.contactgroup,
                                              &contact[i]);
            dJointAttach (c,
                          dGeomGetBody(contact[i].geom.g1),
                          dGeomGetBody(contact[i].geom.g2));
        }
    }
}



void RoboticArmWorld::step(float mx, float my, float mz)
{
    assert(mx >=0 && mx <= 1. && my >=0 && my <= 1. && mz >=0 && mz <= 1.);
    nearCallbackData d = {this};

    dSpaceCollide (odeworld.space_id, &d, &nearCallback);

    float power = 500;


    dJointAddSliderForce(xAndZ, power*mx*2 - power);
    dJointAddSliderForce(yAndZ, power*my*2 - power);
    dJointAddSliderForce(zAndHuman, zkeep+ power*mz*2 -power);
//     zkeep = -40;

    Mutex::scoped_lock lock(ODEFactory::getInstance()->wannaStep());
    dWorldStep (odeworld.world_id, 0.01);
    lock.release();

    dJointGroupEmpty (odeworld.contactgroup);
}

void RoboticArmWorld::resetPositions(goalTypeMove t, int block_length)
{
//     LOG_DEBUG("currentMoveGoal set to " << t);
    currentMoveGoal = t;
    resetPositions(MOVE, block_length);
}


void RoboticArmWorld::resetPositions(goalType t, int block_length)
{
    assert(block_length >= 0 && block_length < 3 && t >= 0 && t < NUMBER_GOAL);

//     LOG_DEBUG("currentGoal set to " << t);
    currentGoal = t;
    goalBeenReached = false;
    goalFailed = false;
    touchedLeft = false;
//     zkeep = 0;

    dMatrix3 R;
    dRFromEulerAngles(R, 0, 0, 0);

    bool colision;
    do {
        colision = false;

        int width = block_length + 1;
        float lx = 0.03 + width*0.02;
        float ly =  0.03 + width*0.02;
        float lz = 0.04 + width*0.01;
        dGeomBoxSetLengths(ogoal->getGeom(), lx, ly, lz);

        dMass m;
        dMassSetBox(&m, ogoal->getDensity(), lx, ly, lz);
        dMassAdjust(&m, ogoal->getMassValue());
        dBodySetMass(ogoal->getID(), &m);


        dGeomSetRotation(ogoal->getGeom(), R);

        float nx, ny, nz;
        nx =bib::Utils::randin(0.16, 0.53);
        ny = bib::Utils::randin(-0.26, 0.03);
//         nz = ((ODEBox*)table)->getLZ() + lz/2. - 0.01;
	nz = ((ODEBox*)table)->getLZ()/2. + lz/2. + table->getZ();
        dGeomSetPosition(ogoal->getGeom(),
                         nx,
                         ny,
                         nz);
        dBodySetAngularVel(ogoal->getID(), 0, 0, 0);
        dBodySetLinearVel(ogoal->getID(), 0, 0, 0);
        dBodySetForce(ogoal->getID(), 0, 0, 0);
        ogoal->setX(nx);
        ogoal->setY(ny);
        ogoal->setZ(nz);


	
	nx = bib::Utils::randin(0.08, 0.60);
	ny = bib::Utils::randin(-0.34, 0.11);
	nz = bib::Utils::randin(0.72, 0.82);
        dGeomSetPosition(hand->getGeom(), nx, ny, nz);
        dBodySetAngularVel(hand->getID(), 0, 0, 0);
        dBodySetLinearVel(hand->getID(), 0, 0, 0);
        dBodySetForce(hand->getID(), 0, 0, 0);
	hand->setX(nx);
	hand->setY(ny);
	hand->setZ(nz);

        dContactGeom contactpoints[1];
        colision = dCollide(hand->getGeom(), ogoal->getGeom(), 1, contactpoints, sizeof(dContactGeom)) > 0 || colision;
        colision = dCollide(xLevel->getGeom(), ogoal->getGeom(), 1, contactpoints, sizeof(dContactGeom)) > 0 || colision;

//         if(colision)
//             LOG_DEBUG("collision in settting up");
    }
    while(!compatibleGoalType(t, currentMoveGoal) || colision);

    step(0.5,0.5,0.5);
    step(0.5,0.5,0.5);
}

bool RoboticArmWorld::compatibleGoalType(goalType t, goalTypeMove m)
{
    if(t != MOVE)
        return true;

    float offset = 0.1;

    const dReal *position = dGeomGetPosition(ogoal->getGeom());
//     const dReal x = position[0];
    const dReal y = position[1];
//     const dReal z = position[2];

    dVector3 lenghts;
    dGeomBoxGetLengths(hand->getGeom(), lenghts);

    switch(m) {
    case MOVE_LEFT:
        return y < ( 0.15 - lenghts[1] - offset);
    case MOVE_RIGHT:
        return y > ( -0.34 + lenghts[1] + offset);
    default:
        return true;
    }

    return true;
}

void RoboticArmWorld::goalReached(goalType goal)
{
    if(goal == currentGoal) {
        goalBeenReached = true;
//         LOG_DEBUG("goal reached");
    } else if(!goalBeenReached && currentGoal != TO_FLOOR)
        goalFailed = true;
}

void RoboticArmWorld::goalReached(goalTypeMove goal)
{
//     LOG_DEBUG(goal << " " << currentMoveGoal << " " << currentGoal << " " << MOVE);
    if(goal == currentMoveGoal && currentGoal == MOVE) {
        goalBeenReached = true;
//         LOG_DEBUG("goal reached");
    }
    else if(!goalBeenReached && currentGoal != TO_FLOOR)
        goalFailed = true;
}

bool RoboticArmWorld::end()
{
    return goalBeenReached;
}

bool RoboticArmWorld::prematureEnd()
{
    return goalFailed;
}


float RoboticArmWorld::sensorMX() const
{
    const dReal* pos = dGeomGetPosition(hand->getGeom());
    return bib::Utils::transform(pos[0], 0.076f, 0.62f, 0.f, 1.f);
//     return pos[0];
}

float RoboticArmWorld::sensorMY() const
{
    const dReal* pos = dGeomGetPosition(hand->getGeom());
    return bib::Utils::transform(pos[1], -0.35f, 0.11f, 0.f, 1.f);
//     return pos[1];
}

float RoboticArmWorld::sensorMZ() const
{
    const dReal* pos = dGeomGetPosition(hand->getGeom());
    return bib::Utils::transform(pos[2], 0.7145f, 0.827f, 0.f, 1.f);
//     return pos[2];
}

float RoboticArmWorld::sensorD1X() const
{
    const dReal* posh = dGeomGetPosition(hand->getGeom());
    const dReal* poso = dGeomGetPosition(ogoal->getGeom());

    dVector3 LH, LO;
    dGeomBoxGetLengths(hand->getGeom(), LH);
    dGeomBoxGetLengths(ogoal->getGeom(), LO);

    double x = fabs((posh[0] - LH[0]/2.f ) - (poso[0] + LO[0]/2.f ));
    return bib::Utils::transform(x, 0.f, 0.48f, 0.f, 1.f);
}

float RoboticArmWorld::sensorD2X() const
{
    const dReal* posh = dGeomGetPosition(hand->getGeom());
    const dReal* poso = dGeomGetPosition(ogoal->getGeom());

    dVector3 LH, LO;
    dGeomBoxGetLengths(hand->getGeom(), LH);
    dGeomBoxGetLengths(ogoal->getGeom(), LO);

    double x = fabs((posh[0] + LH[0]/2.f ) - (poso[0] - LO[0]/2.f ));
//     return x;
    return bib::Utils::transform(x, 0.f, 0.44f, 0.f, 1.f);
}


float RoboticArmWorld::sensorD1Y() const
{
    const dReal* posh = dGeomGetPosition(hand->getGeom());
    const dReal* poso = dGeomGetPosition(ogoal->getGeom());

    dVector3 LH, LO;
    dGeomBoxGetLengths(hand->getGeom(), LH);
    dGeomBoxGetLengths(ogoal->getGeom(), LO);

    double y = fabs((posh[1] - LH[1]/2.f ) - (poso[1] + LO[1]/2.f ));
//     return y;
    return bib::Utils::transform(y, 0.f, 0.48f, 0.f, 1.f);
}

float RoboticArmWorld::sensorD2Y() const
{
    const dReal* posh = dGeomGetPosition(hand->getGeom());
    const dReal* poso = dGeomGetPosition(ogoal->getGeom());

    dVector3 LH, LO;
    dGeomBoxGetLengths(hand->getGeom(), LH);
    dGeomBoxGetLengths(ogoal->getGeom(), LO);

    double y = fabs((posh[1] + LH[1]/2.f ) - (poso[1] - LO[1]/2.f ));
//     return y;
    return bib::Utils::transform(y, 0.f, 0.55f, 0.f, 1.f);
}

float RoboticArmWorld::sensorDZ() const
{
    const dReal* posh = dGeomGetPosition(hand->getGeom());
    const dReal* poso = dGeomGetPosition(ogoal->getGeom());

    dVector3 LH, LO;
    dGeomBoxGetLengths(hand->getGeom(), LH);
    dGeomBoxGetLengths(ogoal->getGeom(), LO);

    double z = fabs((posh[2] - LH[2]/2.f ) - (poso[2] + LO[2]/2.f ));
//     return z;
    return bib::Utils::transform(z, 0.f, 0.07f, 0.f, 1.f);
}

// float RoboticArmWorld::sensorF() const
// {
//     const dReal* posh = dGeomGetPosition(hand->getGeom());
//     double dfloor = bib::Utils::euclidien_dist3D(posh[0], 0, posh[1], 0, posh[2], 0);
//
//     return dfloor;
// }


float RoboticArmWorld::sensorCX() const
{
    const dReal* posh = dGeomGetPosition(hand->getGeom());
    const dReal* poso = dGeomGetPosition(ogoal->getGeom());

    double x = posh[0] - poso[0];
    return bib::Utils::transform(x, -0.46f, 0.68f, 0.f, 1.f);
//     return x;
}

float RoboticArmWorld::sensorCY() const
{
    const dReal* posh = dGeomGetPosition(hand->getGeom());
    const dReal* poso = dGeomGetPosition(ogoal->getGeom());

    double y = posh[1] - poso[1];
    return bib::Utils::transform(y, -0.54f, 0.56f, 0.f, 1.f);
//     return y;
}

float RoboticArmWorld::sensorTX1() const
{
    const dReal* posh = dGeomGetPosition(table->getGeom());
    const dReal* poso = dGeomGetPosition(ogoal->getGeom());

    dVector3 LH, LO;
    dGeomBoxGetLengths(table->getGeom(), LH);
    dGeomBoxGetLengths(ogoal->getGeom(), LO);

    double x = (poso[0] - LO[0]/2.f ) - (posh[0] - LH[0]/2.f );
//     return x;
    return bib::Utils::transform(x, 0.f, 0.67f, 0.f, 1.f);
}

float RoboticArmWorld::sensorTY1() const
{
    const dReal* posh = dGeomGetPosition(table->getGeom());
    const dReal* poso = dGeomGetPosition(ogoal->getGeom());

    dVector3 LH, LO;
    dGeomBoxGetLengths(table->getGeom(), LH);
    dGeomBoxGetLengths(ogoal->getGeom(), LO);

    double y = (poso[1] - LO[1]/2.f ) - (posh[1] - LH[1]/2.f );

    return bib::Utils::transform(y, 0.f, 0.71f, 0.f, 1.f);
//    return y;
}


float RoboticArmWorld::sensorTY2() const
{
    const dReal* posh = dGeomGetPosition(table->getGeom());
    const dReal* poso = dGeomGetPosition(ogoal->getGeom());

    dVector3 LH, LO;
    dGeomBoxGetLengths(table->getGeom(), LH);
    dGeomBoxGetLengths(ogoal->getGeom(), LO);

    double y = (posh[1] + LH[1]/2.f ) - (poso[1] + LO[1]/2.f ) ;

//     return y;
    return bib::Utils::transform(y, 0.f, 0.93f, 0.f, 1.f);
}



