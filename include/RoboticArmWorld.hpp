#ifndef ROBOTICARMWORLD_H
#define ROBOTICARMWORLD_H
#include <ode/ode.h>
#include "ODEObject.hpp"
#include "ODEFactory.hpp"

enum goalTypeMove{
    MOVE_LEFT=0, 
    MOVE_RIGHT, 
    MOVE_FORWARD,
    NUMBER_GOAL_MOVE
};

enum goalType {
    MOVE=0,
    TO_FLOOR, 
    GRAP,
    NUMBER_GOAL
};

#define NUMBER_SIZE 3


class RoboticArmWorld
{
public:
    RoboticArmWorld();
    virtual ~RoboticArmWorld();

    void resetPositions(goalType t, int block_length);
    void resetPositions(goalTypeMove t, int block_length);
    
    virtual void step(float mx, float my, float mz);
    bool end();
    bool prematureEnd();
    
    float sensorMX() const;
    float sensorMY() const;
    float sensorMZ() const;
    float sensorD1X() const;
    float sensorD2X() const;
    float sensorD1Y() const;
    float sensorD2Y() const;
    float sensorDZ() const;
    
    //optional
    float sensorF() const;
    
    float sensorTY1() const;
    float sensorTY2() const;
    float sensorTX1() const;
    
    float sensorCX() const;
    float sensorCY() const;

    void goalReached(goalType goal);
    void goalReached(goalTypeMove goal);
protected:
    void createWorld();
    bool compatibleGoalType(goalType t, goalTypeMove m);


public:
// protected:
    ODEObject* human;
    ODEObject* table;
    ODEObject* hand;
    ODEObject* xLevel;
    ODEObject* yLevel;
    ODEObject* zLevel;
    ODEObject* ogoal;
    

    dGeomID ground;
    dJointID zAndHuman;
    dJointID yAndZ;
    dJointID xAndZ;
    float zkeep =0;

private:
    goalType currentGoal;
    goalTypeMove currentMoveGoal;
    bool goalBeenReached;
    bool goalFailed;
public:
    ODEWorld odeworld;

    bool touchedLeft;
};

struct nearCallbackData {
    RoboticArmWorld* inst;
} ;

#endif // ROBOTICARMWORLD_H
