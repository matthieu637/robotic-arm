#ifndef ROBOTICARMWORLDVIEW_H
#define ROBOTICARMWORLDVIEW_H

#include <RoboticArmWorld.hpp>
#include <string>
#include <tbb/tbb.h>
#include "drawstuff.h"

class RoboticArmWorldView : public RoboticArmWorld
{
public:
    RoboticArmWorldView(const std::string&);
    ~RoboticArmWorldView();
    void step(float mx, float my, float mz);

public:
    std::list<dGeomID> geoms;
    std::list<ODEObject*> delete_me_later;
    
    tbb::tbb_thread* eventThread;
    dsFunctions fn;
    bool speedUp;
    bool requestEnd;
    float motorx, motory, motorz;
};


#endif // ROBOTICARMWORLDVIEW_H
