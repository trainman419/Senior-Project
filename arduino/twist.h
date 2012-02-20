/*
 * twist.h
 *
 * A replacement for geometry_msgs::Twist
 */

#ifndef TWIST_H
#define TWIST_H

struct Vector3 {
   float x, y, z;
};

struct Twist {
   Vector3 linear;
   Vector3 angular;
};

#endif
