#include "ODEObject.hpp"
#include <bib/Utils.hpp>


ODEObject::ODEObject(dBodyID bid, dMass m, dGeomID geom, float x, float y, float z, float density, float massv):
    bid(bid), mass(m), geom(geom), x(x), y(y), z(z), density(density), massv(massv)
{

}

dBodyID ODEObject::getID()
{
    return bid;
}

dGeomID& ODEObject::getGeom()
{
    return geom;
}

dMass& ODEObject::getMass()
{
  return mass;
}

float ODEObject::getDensity()
{
  return density;
}

float ODEObject::getMassValue()
{
  return massv;
}


void ODEObject::setID(const dBodyID _id)
{
    bid = _id;
}

float ODEObject::getX()
{
  return x;
}

float ODEObject::getY()
{
  return y;
}

float ODEObject::getZ()
{
  return z;
}

void ODEObject::setX(float nx)
{
  x=nx;
}

void ODEObject::setY(float ny)
{
  y=ny;
}

void ODEObject::setZ(double nz)
{
  z=nz;
}


float ODEObject::distSince()
{
  const dReal* pos = dGeomGetPosition(geom); 
  return bib::Utils::euclidien_dist3D(x, pos[0], y, pos[1], z, pos[2]);
}

float ODEObject::distSinceX()
{
  const dReal* pos = dGeomGetPosition(geom); 
  return bib::Utils::euclidien_dist1D(x, pos[0]);
}

float ODEObject::distSinceY()
{
  const dReal* pos = dGeomGetPosition(geom); 
  return bib::Utils::euclidien_dist1D(y, pos[1]);
}

float ODEObject::distSinceZ()
{
  const dReal* pos = dGeomGetPosition(geom); 
  return bib::Utils::euclidien_dist1D(z, pos[2]);
}



ODEBox::ODEBox(dBodyID bid, dMass m, dGeomID geom, float x, float y, float z, float density, float mass, float lx, float ly, float lz):
    ODEObject(bid, m, geom, x, y, z, density, mass), lx(lx), ly(ly), lz(lz)
{

}

float ODEBox::getLX()
{
    return lx;
}

float ODEBox::getLY()
{
    return ly;
}

float ODEBox::getLZ()
{
    return lz;
}

ODESphere::ODESphere(dBodyID bid, dMass m, dGeomID geom, float x, float y, float z, float density, float massv, float radius): ODEObject(bid, m, geom, x, y, z, density, massv), radius(radius)
{

}

float ODESphere::getRadius()
{
    return radius;
}
