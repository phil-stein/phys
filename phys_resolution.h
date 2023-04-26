#ifndef PHYS_PHYS_RESPONSE_H
#define PHYS_PHYS_RESPONSE_H

#include "global/global.h"
#include "phys/phys_types.h" 


// @DOC: resolve collision between two phys_obj_t, by moving them out of one another, an adjusting their velocities
//       obj0: first object to have collided
//       obj1: second object to have collided
//       info: info about the collision, returned by one of phys_collision_check...()
void phys_collision_resolution(phys_obj_t* obj0, phys_obj_t* obj1, collision_info_t info);

// @NOTE: old
// void phys_collision_resolution(phys_obj_t* obj0, phys_obj_t* obj1, collision_info_t info);
// void phys_collision_response_resolve_position(phys_obj_t* obj0, phys_obj_t* obj1, collision_info_t info);
// void phys_collision_response_resolve_velocity(phys_obj_t* obj0, phys_obj_t* obj1, collision_info_t info);

#endif
