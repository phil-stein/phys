#ifndef PHYS_RESPONSE_H
#define PHYS_RESPONSE_H

#include "global/global.h"
#include "phys/phys_types.h" 


void phys_collision_resolution(phys_obj_t* obj0, phys_obj_t* obj1, collision_info_t info);

// void phys_collision_resolution(phys_obj_t* obj0, phys_obj_t* obj1, collision_info_t info);
// void phys_collision_response_resolve_position(phys_obj_t* obj0, phys_obj_t* obj1, collision_info_t info);
// void phys_collision_response_resolve_velocity(phys_obj_t* obj0, phys_obj_t* obj1, collision_info_t info);

#endif
