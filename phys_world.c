#include "phys/phys_world.h"
#include "phys/phys_dynamics.h"
#include "phys/phys_resolution.h"
#include "phys/phys_collision.h"
#include "phys/phys_debug_draw.h"
#include "core/debug/debug_draw.h"

#include "stb/stb_ds.h"


// all objects, static and dynamic
phys_obj_t* phys_objs = NULL;
u32 phys_objs_len = 0;

// callbacks, macros to check for null
phys_internal_collision_callback* collision_callback = NULL;
phys_internal_trigger_callback*   trigger_callback   = NULL;
#define COLLISION_CALLBACK(a, b)  if (collision_callback) { collision_callback((a), (b)); }
#define TRIGGER_CALLBACK(a, b)    if (trigger_callback)   { trigger_callback((a), (b)); }


// @TODO: move below update()

void phys_obj_make_rb(f32 mass, f32 friction, phys_obj_t* obj)
{
  obj->flags |= PHYS_HAS_RIGIDBODY;
  obj->rb.mass = mass;
  // obj->rb.drag = drag;       // gets set in RIGIDBODY_T_INIT()
  // obj->rb.friction = 0.2f;   // gets set in RIGIDBODY_T_INIT()
  obj->rb.friction = friction;   
  vec3_copy(VEC3(0), obj->rb.velocity);
  vec3_copy(VEC3(0), obj->rb.force);
}
void phys_obj_make_box(vec3 aabb[2], vec3 offset, bool is_trigger, phys_obj_t* obj)
{
  // ASSERT(!HAS_FLAG(obj->flags, PHYS_HAS_SPHERE));
  ASSERT(!PHYS_OBJ_HAS_COLLIDER(obj));
  obj->flags |= PHYS_HAS_BOX;
  
  obj->collider.type = PHYS_COLLIDER_BOX;
  
  vec3_copy(offset, obj->collider.offset);
  obj->collider.is_trigger   = is_trigger;
  obj->collider.is_colliding = false;
  
  vec3_copy(aabb[0], obj->collider.box.aabb[0]);
  vec3_copy(aabb[1], obj->collider.box.aabb[1]);

  obj->collider.infos = NULL;
  obj->collider.infos_len = 0;
}
void phys_obj_make_sphere(f32 radius, vec3 offset, bool is_trigger, phys_obj_t* obj)
{
  // ASSERT(!HAS_FLAG(obj->flags, PHYS_HAS_SPHERE));
  ASSERT(!PHYS_OBJ_HAS_COLLIDER(obj));
  obj->flags |= PHYS_HAS_SPHERE;
  
  obj->collider.type = PHYS_COLLIDER_SPHERE;
  
  vec3_copy(offset, obj->collider.offset);
  obj->collider.is_trigger   = is_trigger;
  obj->collider.is_colliding = false;
  
  obj->collider.sphere.radius = radius;

  obj->collider.infos = NULL;
  obj->collider.infos_len = 0;
}

void phys_add_obj_rb(u32 entity_idx, vec3 pos, f32 mass, f32 friction)
{
  phys_obj_t obj = PHYS_OBJ_T_INIT();
  obj.entity_idx = entity_idx;
  vec3_copy(pos, obj.pos);
  vec3_copy(VEC3(1), obj.scl);

  phys_obj_make_rb(mass, friction, &obj);

  arrput(phys_objs, obj);
  phys_objs_len++;
}
void phys_add_obj_box(u32 entity_idx, vec3 pos, vec3 scl, vec3 aabb[2], vec3 offset, bool is_trigger)
{
  phys_obj_t obj = PHYS_OBJ_T_INIT();
  obj.entity_idx = entity_idx;
  vec3_copy(pos, obj.pos);
  vec3_copy(scl, obj.scl);

  phys_obj_make_box(aabb, offset, is_trigger, &obj); 

  arrput(phys_objs, obj);
  phys_objs_len++;
}
void phys_add_obj_sphere(u32 entity_idx, vec3 pos, vec3 scl, f32 radius, vec3 offset, bool is_trigger)
{
  phys_obj_t obj = PHYS_OBJ_T_INIT();
  obj.entity_idx = entity_idx;
  vec3_copy(pos, obj.pos);
  vec3_copy(scl, obj.scl);

  phys_obj_make_sphere(radius, offset, is_trigger, &obj); 

  arrput(phys_objs, obj);
  phys_objs_len++;
}
void phys_add_obj_rb_box(u32 entity_idx, vec3 pos, vec3 scl, f32 mass, f32 friction, vec3 aabb[2], vec3 offset, bool is_trigger)
{
  phys_obj_t obj = PHYS_OBJ_T_INIT();
  obj.entity_idx = entity_idx;
  vec3_copy(pos, obj.pos);
  vec3_copy(scl, obj.scl);

  phys_obj_make_rb(mass, friction, &obj);
  phys_obj_make_box(aabb, offset, is_trigger, &obj);

  arrput(phys_objs, obj);
  phys_objs_len++;
}
void phys_add_obj_rb_sphere(u32 entity_idx, vec3 pos, vec3 scl, f32 mass, f32 friction, f32 radius, vec3 offset, bool is_trigger)
{
  phys_obj_t obj = PHYS_OBJ_T_INIT();
  obj.entity_idx = entity_idx;
  vec3_copy(pos, obj.pos);
  vec3_copy(scl, obj.scl);

  phys_obj_make_rb(mass, friction, &obj);
  phys_obj_make_sphere(radius, offset, is_trigger, &obj);

  arrput(phys_objs, obj);
  phys_objs_len++;
}

// @UNSURE: 
void phys_remove_obj(u32 entity_idx)
{
  for (u32 i = 0; i < phys_objs_len; ++i)
  {
    if (phys_objs[i].entity_idx == entity_idx) 
    { 
      arrdel(phys_objs, i); 
      phys_objs_len--;
    }
  }
}

void phys_rotate_box_y(u32 entity_idx)
{
  for (u32 i = 0; i < phys_objs_len; ++i)
  {
    phys_obj_t* obj = &phys_objs[i];
    if (obj->entity_idx == entity_idx && PHYS_OBJ_HAS_COLLIDER(obj) && obj->collider.type == PHYS_COLLIDER_BOX) 
    {
      // switch x / z
      vec3 min, max;
      min[0] = obj->collider.box.aabb[0][2];
      min[1] = obj->collider.box.aabb[0][1];
      min[2] = obj->collider.box.aabb[0][0];

      max[0] = obj->collider.box.aabb[1][2];
      max[1] = obj->collider.box.aabb[1][1];
      max[2] = obj->collider.box.aabb[1][0];

      vec3_copy(min, obj->collider.box.aabb[0]);
      vec3_copy(max, obj->collider.box.aabb[1]);
    }
  }
}

// @TODO:
void phys_clear_state()
{
  ARRFREE(phys_objs);
  phys_objs_len = 0;
}

phys_obj_t* phys_get_obj_arr(u32* len)
{
  *len = phys_objs_len;
  return phys_objs;
}

void phys_init(phys_internal_collision_callback* _collision_callback, phys_internal_trigger_callback* _trigger_callback)
{
  collision_callback = _collision_callback;
  trigger_callback   = _trigger_callback;
}

void phys_update(f32 dt)
{

	// go through all objs
  // skip objects that are static or not colliders
	for (u32 i = 0; i < phys_objs_len; ++i) // array of rigidbodies
	{
    phys_obj_t* obj0 = &phys_objs[i];

    // ---- dynamics ----
		if (!PHYS_OBJ_HAS_RIGIDBODY(obj0)) { continue; }
    phys_dynamics_simulate(obj0, dt);

		// ---- collision ----
		if (!PHYS_OBJ_HAS_COLLIDER(obj0)) { continue; }
    
    obj0->collider.is_colliding = false; 
	  obj0->collider.is_grounded  = false; 	

    // test with all other colliders
		for (int j = 0; j < phys_objs_len; ++j) // array of colliders
		{
			if (j == i) { continue; }
		  if (!PHYS_OBJ_HAS_COLLIDER(obj0)) { continue; }
      phys_obj_t* obj1 = &phys_objs[j];

			collision_info_t c = phys_collision_check(obj0, obj1);
      obj0->collider.is_colliding = obj0->collider.is_colliding || c.collision;
      obj0->collider.is_grounded  = obj0->collider.is_grounded  || c.grounded;
      
      // ---- collision response ----
		  if (c.collision)
		  {
		  	// notify objects of collision
		  	c.trigger = obj0->collider.is_trigger || obj1->collider.is_trigger;

		  	c.obj_idx = obj1->entity_idx;
		  	arrput(obj0->collider.infos, c);
		  	obj0->collider.infos_len++;

		  	c.obj_idx = obj0->entity_idx;
		  	arrput(obj1->collider.infos, c);
		  	obj1->collider.infos_len++;

		    if (!c.trigger && PHYS_OBJ_HAS_RIGIDBODY(obj0)) // no response on trigger collisions
		  	{
          // P_INT(obj1->entity_idx);
		  		phys_collision_resolution(obj0, obj1, c);
          COLLISION_CALLBACK(obj0->entity_idx, obj1->entity_idx);
		  	}
        else if (PHYS_OBJ_HAS_RIGIDBODY(obj0))
        {
          TRIGGER_CALLBACK(obj0->entity_idx, obj1->entity_idx);
        }
		  }

		}
	}
}

