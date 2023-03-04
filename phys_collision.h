#ifndef PHYS_COLLISION_H
#define PHYS_COLLISION_H

#include "phys/phys_types.h" 

// check collision between two entities
collision_info_t phys_collision_check(phys_obj_t* e1, phys_obj_t* e2);

collision_info_t phys_collision_check_sphere_v_sphere(phys_obj_t* s1, phys_obj_t* s2);

// taken from "https://developer.mozilla.org/en-US/docs/Games/Techniques/3D_collision_detection"
collision_info_t phys_collision_check_aabb_v_aabb(phys_obj_t* b1, phys_obj_t* b2);

collision_info_t phys_collision_check_aabb_v_sphere(phys_obj_t* b, phys_obj_t* s);

#endif
