#ifndef PHYS_PHYS_WORLD_H
#define PHYS_PHYS_WORLD_H

#include "global/global.h"
#include "phys/phys_types.h"

#ifdef __cplusplus
extern "C" {
#endif


// @DOC: func type for collision callbacks
typedef void (phys_internal_collision_callback)(int id_01, int id_02);
// @DOC: func type for trigger collision callbacks
typedef void (phys_internal_trigger_callback)(int id_01, int id_02);


typedef struct
{
  int a;
  int b;
}phys_obj_combination_t;

typedef struct
{
  phys_obj_t* obj0;
  phys_obj_t* obj1;
  collision_info_t c;
}phys_collision_t;


// @DOC: gen every combination of objs
//       so only have to check collision once per combination
void phys_generate_combinations();

// @DOC: initialize physics engine, set callbacks, or NULL, call before any oher calls to phys
//       _collision_callback: NULL or gets called on collision
//       _trigger_callback:   NULL or gets called on trigger collision
void phys_init(phys_internal_collision_callback* _collision_callback, phys_internal_trigger_callback* _trigger_callback);

// @DOC: call once a frame to update the state of the physics engine
//       dt: pass delta time, the time passed since last frame
void phys_update(f32 dt);

// @DOC: only checks every possible combination
void phys_update_new(f32 dt);

// @DOC: checks every obj against every other obj
//       meanind does both 
//       obj[1] v obj[2] and obj[2] v obj[1]
void phys_update_old(f32 dt);


// @DOC: add rigidbody to phys_obj_t
void phys_obj_make_rb(f32 mass, f32 friction, phys_obj_t* obj);
// @DOC: add box collider to phys_obj_t
//       aabb: aabb[0] is min aabb[1] is max
void phys_obj_make_box(vec3 aabb[2], vec3 offset, bool is_trigger, phys_obj_t* obj);

// @DOC: add physics object with rigidbody, but no collider
//       entity_id: id of entity to attach to
void phys_add_obj_rb(int entity_idx, vec3 pos, f32 mass, f32 friction);
// @DOC: add physics object with box collider but no rigidbody
//       entity_id: id of entity to attach to
//       aabb: aabb[0] is min, aabb[1] is max
void phys_add_obj_box(int entity_idx, vec3 pos, vec3 scl, vec3 aabb[2], vec3 offset, bool is_trigger);
// @DOC: add physics object with sphere collider but no rigidbody
//       entity_id: id of entity to attach to
void phys_add_obj_sphere(int entity_idx, vec3 pos, vec3 scl, f32 radius, vec3 offset, bool is_trigger);
// @DOC: add physics object with box collider and rigidbody
//       entity_id: id of entity to attach to
//       aabb: aabb[0] is min aabb[1] is max
void phys_add_obj_rb_box(int entity_idx, vec3 pos, vec3 scl, f32 mass, f32 friction, vec3 aabb[2], vec3 offset, bool is_trigger);
// @DOC: add phys obj with rigidbody and sphere collider
//       entity_id: id of entity to attach to
void phys_add_obj_rb_sphere(int entity_idx, vec3 pos, vec3 scl, f32 mass, f32 friction, f32 radius, vec3 offset, bool is_trigger);

  // @DOC: remove object, by the entity its attached to
//       entity_idx: phys obj with phys_obj_t.entity_idx == entity_idx gets removed
void phys_remove_obj(int entity_idx);

// @DOC: 'roatate' aabb 90° around y
//       entity_idx: phys obj with phys_obj_t.entity_idx == entity_idx gets rotated
void phys_rotate_box_y(int entity_idx);


// @DOC: remove all objects
void phys_clear_state();

// @DOC: get arr with all phys_obj_t 
//       len: gets set to arr's length
phys_obj_t* phys_get_obj_arr(u32* len);


#ifdef __cplusplus
} // extern c
#endif

#endif
