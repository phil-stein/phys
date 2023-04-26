#ifndef PHYS_PHYS_COLLISION_H
#define PHYS_PHYS_COLLISION_H

#include "phys/phys_types.h" 

// @DOC: check collision between two entities, regardless of collider type
//       e1: first phys obj with  collider
//       e2: second phys obj with collider
collision_info_t phys_collision_check(phys_obj_t* e1, phys_obj_t* e2);

// @DOC: check collision between to phys_obj_t with sphere colliders
//       s1: first phys_obj with sphere collider
//       s2: second phys_obj with sphere collider
collision_info_t phys_collision_check_sphere_v_sphere(phys_obj_t* s1, phys_obj_t* s2);

// taken from "https://developer.mozilla.org/en-US/docs/Games/Techniques/3D_collision_detection"
// @DOC: check collision between to phys_obj_t with box/aabb colliders
//       s1: first phys_obj with box collider
//       s2: second phys_obj with box collider
collision_info_t phys_collision_check_aabb_v_aabb(phys_obj_t* b1, phys_obj_t* b2);

// @DOC: check collision between to phys_obj_t on box/aabb, one sphere collider
//       s1: first phys_obj with box collider
//       s2: second phys_obj with sphere collider
//       switch_obj_places: if true treat s as the active obj, inverses info.direction
collision_info_t phys_collision_check_aabb_v_sphere(phys_obj_t* b, phys_obj_t* s, bool switch_obj_places);

#endif
