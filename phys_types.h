#ifndef PHYS_PHYS_TYPES_H
#define PHYS_PHYS_TYPES_H

#include "global/global.h"
#include "math/math_inc.h"

// @DOC: holds all info about collision needed to resolve it
typedef struct collision_info_t
{
  bool collision; // if any collision happened at all
  vec3 direction; // normal of collision
  f32  depth;     // how deep the penetration is
  int  obj_idx;   // idx into phys_objs array in phys_world.c, of other phys_obj_t
  bool trigger;   // if one or both colliders are trigger
  bool grounded;  // currently colliding in -y direction

}collision_info_t;
// @DOC: default values for collision_info_t
#define COLLISION_INFO_T_INIT() \
{                               \
  .collision = false,           \
  .direction = { 0, 0, 0 },     \
  .depth     = 0,               \
  .obj_idx   = -1,              \
  .trigger   = false,           \
  .grounded  = false,           \
}

#define P_COLLISION_INFO_T(a)   { PF("collision_info_t: %s\n", #a); P_BOOL((a).collision); P_VEC3((a).direction); \
                                  P_F32((a).depth); P_INT((a).obj_idx); P_BOOL((a).trigger); }

// @DOC: box collider, aka. aabb
//       aabb[0] is min
//       aabb[1] is max
typedef struct box_collider_t
{
  vec3 aabb[2];

}box_collider_t;
#define P_BOX_COLLIDER_T(a)     { PF("box_collider_t: %s\n", #a); P_VEC3((a).aabb[0]); P_VEC3((a).aabb[1]); }

// @DOC: sphere collider, only need radius
typedef struct sphere_collider_t
{
  f32 radius;

}sphere_collider_t;
#define P_SPHERE_COLLIDER_T(a)  { PF("sphere_collider_t: %s", #a); P_F32((a).radius); }  

// @DOC: type of collider
typedef enum collider_type_t 
{ 
  PHYS_COLLIDER_SPHERE, 
  PHYS_COLLIDER_BOX 

} collider_type_t;
#define P_COLLIDER_TYPE_T(a)    { PF("collider_type_t: %s: ", #a);                                    \
                                  PF("%s\n", (a) == PHYS_COLLIDER_SPHERE ? "PHYS_COLLIDER_SPHERE" :   \
                                  (a) == PHYS_COLLIDER_BOX ? "PHYS_COLLIDER_BOX" : "UNKNOWN"); }

// @DOC: collider, can be any of collider_type_t's specified types
typedef struct collider_t
{
  collider_type_t type;
  vec3 offset;        // offset from phys_obj_t
  bool is_trigger;    // if true, doesnt affect other colliders, but collisions get registered
  bool is_colliding;  // is currently colliding
  bool is_grounded;   // is currently colliding with something below it
  union               // only need either sphere or box, never both
  {
    sphere_collider_t sphere;
    box_collider_t    box;
  };

  collision_info_t* infos;  // all collision infos, because multiple collisions may occur in one frame
  int infos_len;

}collider_t;


// @NOTE: doesnt print collision infos
#define P_COLLIDER_T(a)       { P_LINE(); PF("collider_t: %s\n", #a); P_COLLIDER_TYPE_T((a).type); P_VEC3((a).offset); P_BOOL((a).is_trigger);  \
                                if ((a).type == PHYS_COLLIDER_SPHERE) { P_SPHERE_COLLIDER_T((a).sphere); }                                      \
                                if ((a).type == PHYS_COLLIDER_BOX)    { P_BOX_COLLIDER_T((a).box); } }                                  

// @DOC: rigidbidy, all data needed to simulate dynamics
typedef struct rigidbody_t
{
  vec3 velocity;    // current velocity
  vec3 force;       // also often called 'acceleration-accumulator'
  f32  mass;        // objs mass
  f32  drag;        // slows object constantly, the lower the more stronger
  f32  friction;    // scales drag when colliding, the lower the more friction, i.e. 0.0f <->  1.0f

  // @NOTE: part of the old resolution
  // f32  restitution;      // default: 1.0f
  // f32  static_friction;  // default: 0.0f
  // f32  dynamic_friction; // default: 0.0f

}rigidbody_t;

// @DOC: default values for rigidbody_t
#define RIGIDBODY_T_INIT()  \
{                           \
  .velocity = { 0, 0, 0 },  \
  .force    = { 0, 0, 0 },  \
  .mass     = 1.0f,         \
  .drag     = 0.3f,         \
  .friction = 0.1f,         \
}

#define P_RIGIDBODY_T(a)      { P_LINE(); PF("rigidbody_t: %s\n", #a); P_VEC3((a).velocity); P_VEC3((a).force);                       \
                                P_F32((a).mass); P_F32((a).restitution); P_F32((a).static_friction); P_F32((a).dynamic_friction); }

// @DOC: flag defining which 'components' a phys_obj_t has
typedef enum phys_obj_flag 
{ 
  PHYS_HAS_RIGIDBODY = FLAG(0), 
  PHYS_HAS_BOX       = FLAG(1), 
  PHYS_HAS_SPHERE    = FLAG(2),

} phys_obj_flag;
#define PHYS_OBJ_HAS_RIGIDBODY(obj) (HAS_FLAG((obj)->flags, PHYS_HAS_RIGIDBODY))
// #define PHYS_OBJ_HAS_COLLIDER(obj)  (HAS_FLAG((obj)->flags, PHYS_HAS_BOX) || HAS_FLAG((obj)->flags, PHYS_HAS_SPHERE))
#define PHYS_OBJ_HAS_COLLIDER(obj)  (HAS_FLAG((obj)->flags, PHYS_HAS_BOX | PHYS_HAS_SPHERE))

// @NOTE: cant use STR_BOOL() and HAS_FLAG() because of how macros are 'unfolded'
#define P_PHYS_OBJ_FLAGS_T(a) { PF("phys_obj_flag: %s\n", #a);                                                       \
                                PF("PHYS_HAS_RIGIDBODY: %s\n",  ((a) & PHYS_HAS_RIGIDBODY) ? "true" : "false");   \
                                PF("PHYS_HAS_BOX: %s\n",        ((a) & PHYS_HAS_BOX)       ? "true" : "false");   \
                                PF("PHYS_HAS_SPHERE: %s\n",     ((a) & PHYS_HAS_SPHERE)    ? "true" : "false"); }

// @DOC: the objs simulated and attached to an entity
typedef struct phys_obj_t
{
  u32  entity_idx;  // id of entity the phys_obj_t simulates
  vec3 pos;         // position
  vec3 scl;         // scale
  // no rotation, not supported

  phys_obj_flag flags;  // 'components' attached to obj
  rigidbody_t rb;
  collider_t  collider;
}phys_obj_t;
#define PHYS_OBJ_T_INIT()     \
{                             \
  .entity_idx = -1,           \
  .pos        = { 0, 0, 0 },  \
  .scl        = { 1, 1, 1 },  \
  .flags      = 0,            \
  .rb = RIGIDBODY_T_INIT(),   \
}

#define P_PHYS_OBJ_T(a)       { P_LINE();                                                                               \
                                PF("phys_obj_t: %s\n", #a); P_U32((a)->entity_idx); P_VEC3((a)->pos); P_VEC3((a)->scl); \
                                P_PHYS_OBJ_FLAGS_T((a)->flags); P_RIGIDBODY_T((a)->rb); P_COLLIDER_T((a)->collider);    \
                                P_LINE(); }

#define P_PHYS_OBJ_T_NAN(a)   { P_VEC3_NAN((a)->pos); P_VEC3_NAN((a)->scl);                 \
                                P_VEC3_NAN((a)->rb.velocity); P_VEC3_NAN((a)->rb.force); }

#define ERR_PHYS_OBJ_T_NAN(a) { P_PHYS_OBJ_T_NAN(a); ERR_CHECK(!VEC3_NAN((a)->pos) && !VEC3_NAN((a)->scl) &&     \
                                !VEC3_NAN((a)->rb.velocity) && !VEC3_NAN((a)->rb.force), "'%s'->idx: %d\n", #a, (a)->entity_idx); }

// @NOTE: mistook mynkowski sum for addition sum, lol
// // INLINE void phys_aabb_add(box_collider_t* b0, box_collider_t* b1, box_collider_t* out)
// INLINE void phys_aabb_add(vec3* b0, vec3* b1, vec3* out)
// {
//   // out->aabb[0][0] = b0->aabb[0][0] + b1->aabb[0][0];
//   // out->aabb[0][1] = b0->aabb[0][1] + b1->aabb[0][2];
//   // out->aabb[0][2] = b0->aabb[0][2] + b1->aabb[0][3];
//   // 
//   // out->aabb[1][0] = b0->aabb[1][0] + b1->aabb[1][0];
//   // out->aabb[1][1] = b0->aabb[1][1] + b1->aabb[1][2];
//   // out->aabb[1][2] = b0->aabb[1][2] + b1->aabb[1][3];
// 
//   out[0][0] = b0[0][0] + b1[0][0];
//   out[0][1] = b0[0][1] + b1[0][2];
//   out[0][2] = b0[0][2] + b1[0][3];
//   
//   out[1][0] = b0[1][0] + b1[1][0];
//   out[1][1] = b0[1][1] + b1[1][2];
//   out[1][2] = b0[1][2] + b1[1][3];
// }
INLINE void phys_get_final_aabb(phys_obj_t* b, vec3* out)
{
  f32* min = (f32*)&out[0];
  f32* max = (f32*)&out[1];
  vec3_copy(b->collider.box.aabb[0], min);
  vec3_copy(b->collider.box.aabb[1], max);
  vec3_mul(min, b->scl, min);
  vec3_mul(max, b->scl, max);
	vec3_add(min, b->pos, min);
	vec3_add(max, b->pos, max);
	vec3_add(min, b->collider.offset, min);
	vec3_add(max, b->collider.offset, max);
}

#endif
