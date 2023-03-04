#ifndef PHYS_WORLD_H
#define PHYS_WORLD_H

#include "global/global.h"
#include "phys/phys_types.h"


typedef void (phys_internal_collision_callback)(int id_01, int id_02);
typedef void (phys_internal_trigger_callback)(int id_01, int id_02);


// initialize physics engine, set callbacks, or NULL 
void phys_init(phys_internal_collision_callback* _collision_callback, phys_internal_trigger_callback* _trigger_callback);

// call once a frame to update the state of the physics engine
// "dt" pass delta time, the time passed since last frame
void phys_update(f32 dt);


// add rigidbody to phys_obj_t
void phys_obj_make_rb(f32 mass, f32 friction, phys_obj_t* obj);
// add box collider to phys_obj_t
void phys_obj_make_box(vec3 aabb[2], vec3 offset, bool is_trigger, phys_obj_t* obj);

// add physics object with rigidbody, but no collider
void phys_add_obj_rb(u32 entity_idx, vec3 pos, f32 mass, f32 friction);
// add physics object with box collider but no rigidbody
void phys_add_obj_box(u32 entity_idx, vec3 pos, vec3 scl, vec3 aabb[2], vec3 offset, bool is_trigger);
// add physics object with box collider and rigidbody
void phys_add_obj_rb_box(u32 entity_idx, vec3 pos, vec3 scl, f32 mass, f32 friction, vec3 aabb[2], vec3 offset, bool is_trigger);

// remove object, by the entity its attached to
void phys_remove_obj(u32 entity_idx);

// "roatate" aabb 90° around y
void phys_rotate_box_y(u32 entity_idx);


// remove all objects
void phys_clear_state();

phys_obj_t* phys_get_obj_arr(u32* len);

#endif
