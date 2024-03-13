#include "phys/phys_world.h"
#include "global/global_types.h"
#include "phys/phys_dynamics.h"
#include "phys/phys_resolution.h"
#include "phys/phys_collision.h"
#include "phys/phys_debug_draw.h"
#include "core/debug/debug_draw.h"
#include "phys/phys_types.h"

#ifdef TERRAIN_ADDON
#include "core/core_data.h"
#endif

#include "stb/stb_ds.h"


// all objects, static and dynamic
phys_obj_t* phys_objs = NULL;
u32 phys_objs_len = 0;

// callbacks, macros to check for null
phys_internal_collision_callback* phys_collision_callback = NULL;
phys_internal_trigger_callback*   phys_trigger_callback   = NULL;
#define COLLISION_CALLBACK(a, b)  if (phys_collision_callback) { phys_collision_callback((a), (b)); }
#define TRIGGER_CALLBACK(a, b)    if (phys_trigger_callback)   { phys_trigger_callback((a), (b)); }


phys_obj_combination_t* combination_arr = NULL;
u32                     combination_arr_len = 0;

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

void phys_add_obj_rb(int entity_idx, vec3 pos, f32 mass, f32 friction)
{
  phys_obj_t obj = PHYS_OBJ_T_INIT();
  obj.entity_idx = entity_idx;
  vec3_copy(pos, obj.pos);
  vec3_copy(pos, obj.last_pos);
  vec3_copy(VEC3(1), obj.scl);

  phys_obj_make_rb(mass, friction, &obj);

  arrput(phys_objs, obj);
  phys_objs_len++;
  phys_generate_combinations(); // re-generate combinations after add
}
void phys_add_obj_box(int entity_idx, vec3 pos, vec3 scl, vec3 aabb[2], vec3 offset, bool is_trigger)
{
  phys_obj_t obj = PHYS_OBJ_T_INIT();
  obj.entity_idx = entity_idx;
  vec3_copy(pos, obj.pos);
  vec3_copy(pos, obj.last_pos);
  vec3_copy(scl, obj.scl);

  phys_obj_make_box(aabb, offset, is_trigger, &obj); 

  arrput(phys_objs, obj);
  phys_objs_len++;
  phys_generate_combinations(); // re-generate combinations after add
}
void phys_add_obj_sphere(int entity_idx, vec3 pos, vec3 scl, f32 radius, vec3 offset, bool is_trigger)
{
  phys_obj_t obj = PHYS_OBJ_T_INIT();
  obj.entity_idx = entity_idx;
  vec3_copy(pos, obj.pos);
  vec3_copy(pos, obj.last_pos);
  vec3_copy(scl, obj.scl);

  phys_obj_make_sphere(radius, offset, is_trigger, &obj); 

  arrput(phys_objs, obj);
  phys_objs_len++;
  phys_generate_combinations(); // re-generate combinations after add
}
void phys_add_obj_rb_box(int entity_idx, vec3 pos, vec3 scl, f32 mass, f32 friction, vec3 aabb[2], vec3 offset, bool is_trigger)
{
  phys_obj_t obj = PHYS_OBJ_T_INIT();
  obj.entity_idx = entity_idx;
  vec3_copy(pos, obj.pos);
  vec3_copy(pos, obj.last_pos);
  vec3_copy(scl, obj.scl);

  phys_obj_make_rb(mass, friction, &obj);
  phys_obj_make_box(aabb, offset, is_trigger, &obj);

  arrput(phys_objs, obj);
  phys_objs_len++;
  phys_generate_combinations(); // re-generate combinations after add
}
void phys_add_obj_rb_sphere(int entity_idx, vec3 pos, vec3 scl, f32 mass, f32 friction, f32 radius, vec3 offset, bool is_trigger)
{
  phys_obj_t obj = PHYS_OBJ_T_INIT();
  obj.entity_idx = entity_idx;
  vec3_copy(pos, obj.pos);
  vec3_copy(pos, obj.last_pos);
  vec3_copy(scl, obj.scl);

  phys_obj_make_rb(mass, friction, &obj);
  phys_obj_make_sphere(radius, offset, is_trigger, &obj);

  arrput(phys_objs, obj);
  phys_objs_len++;
  phys_generate_combinations(); // re-generate combinations after add
}

// @UNSURE: 
void phys_remove_obj(int entity_idx)
{
  for (int i = 0; i < (int)phys_objs_len; ++i)
  {
    if (phys_objs[i].entity_idx == entity_idx) 
    { 
      arrdel(phys_objs, (u32)i); 
      phys_objs_len--;
    }
  }
  phys_generate_combinations(); // re-generate combinations after remove 
}

void phys_rotate_box_y(int entity_idx)
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

// gen every combination of objs
// so only have to check collision once per combination
void phys_generate_combinations()
{
  // ARRFREE(combination_arr);
  // combination_arr_len = 0;
  // 
  // // P_LINE_STR("combinations ");
  // // P_INT(phys_objs_len);

	// for (u32 a = 0; a < phys_objs_len; ++a)
  // {
  //   // // skip static objects
  //   phys_obj_t* obj0 = &phys_objs[a];
	// 	if (!PHYS_OBJ_HAS_COLLIDER(obj0)) { continue; }
	//   
  //   for (u32 b = a+1; b < phys_objs_len; ++b)
  //   {
  //     phys_obj_t* obj1 = &phys_objs[b];
	// 	  if (!PHYS_OBJ_HAS_COLLIDER(obj1)) { continue; }
  //     
  //     // _PF("c[%d] a: %d, b: %d\n", combination_arr_len, a, b);
  //     
  //     phys_obj_combination_t c = { .a = a, .b = b };
  //     arrput(combination_arr, c);
  //     combination_arr_len++;
  //   }
  // }
}

void phys_init(phys_internal_collision_callback* _collision_callback, phys_internal_trigger_callback* _trigger_callback)
{
  phys_collision_callback = _collision_callback;
  phys_trigger_callback   = _trigger_callback;
}

void phys_update(f32 dt)
{

  // @NOTE: tried only doing every combination
  //        but, everything vibrating
  //        bc. objects ontop of one another 
  //        push into ground even though already 
  //        collidding with ground
  //
  // phys_update_new(dt);

  
  // @NOTE: old implementation:
  //        checks every obj against every other obj
  //        meanind does both 
  //        obj[1] v obj[2] and obj[2] v obj[1]
  phys_update_old(dt);
}

void phys_update_new(f32 dt)
{
  // ---- dynamics ----
	for (u32 i = 0; i < phys_objs_len; ++i) 
  {
    phys_obj_t* obj0 = &phys_objs[i];
		if (!PHYS_OBJ_HAS_RIGIDBODY(obj0)) { continue; }
    phys_dynamics_simulate(obj0, dt);
  }

  // --- collisions ---
  phys_collision_t* collision_arr = NULL;
  u32               collision_arr_len = 0;
  // go through all combinations 
  // skip objects that are static or not colliders
	for (u32 i = 0; i < combination_arr_len; ++i) 
	{
    phys_obj_combination_t* combination = &combination_arr[i];
    phys_obj_t* a = &phys_objs[combination->a];
    phys_obj_t* b = &phys_objs[combination->b];
    phys_obj_t* obj0 = PHYS_OBJ_HAS_RIGIDBODY(a) ? a : b;
    phys_obj_t* obj1 = PHYS_OBJ_HAS_RIGIDBODY(a) ? b : a;
    // phys_obj_t* obj0 = &phys_objs[combination->a];
    // phys_obj_t* obj1 = &phys_objs[combination->b];
    
    // if obj0 doesnt have rigidbody, neither one has one
    // so both static
    if ( !PHYS_OBJ_HAS_RIGIDBODY(obj0) ) { continue; }
    // _PF("obj0: %d, obj1: %d\n", obj0->entity_idx, obj1->entity_idx);
    
    // debug_draw_sphere_register(obj0->pos, 0.1f, RGB_F(1, 0, 0));
    // debug_draw_line_register(obj0->pos, obj1->pos, RGB_F(0, 1, 0));
		
    if (!PHYS_OBJ_HAS_COLLIDER(obj0)) { continue; }
    
    obj0->collider.is_colliding = false; 
	  obj0->collider.is_grounded  = false; 	

    // test with other collider
		if (!PHYS_OBJ_HAS_COLLIDER(obj1)) { continue; }

		collision_info_t c = phys_collision_check(obj0, obj1);
    obj0->collider.is_colliding = obj0->collider.is_colliding || c.collision;
    obj0->collider.is_grounded  = obj0->collider.is_grounded  || c.grounded;
    
    // ---- collision response ----
		if (c.collision)
		{
			// notify objects of collision
			c.trigger = obj0->collider.is_trigger || obj1->collider.is_trigger;

			// c.obj_idx = obj1->entity_idx;
			// arrput(obj0->collider.infos, c);
			// obj0->collider.infos_len++;

			// c.obj_idx = obj0->entity_idx;
			// arrput(obj1->collider.infos, c);
			// obj1->collider.infos_len++;

		  if (!c.trigger) // && PHYS_OBJ_HAS_RIGIDBODY(obj0)) // no response on trigger collisions
			{
        // // P_INT(obj1->entity_idx);
				// phys_collision_resolution(obj0, obj1, c);
        // COLLISION_CALLBACK(obj0->entity_idx, obj1->entity_idx);
        phys_collision_t collision = { .obj0 = obj0, .obj1 = obj1, .c = c };
        arrput(collision_arr, collision);
        collision_arr_len++;
			}
      else //  if (PHYS_OBJ_HAS_RIGIDBODY(obj0))
      {
        TRIGGER_CALLBACK(obj0->entity_idx, obj1->entity_idx);
      }
		}
	}
   
  // --- resolution ---
  // for (int i = collision_arr_len -1; i >= 0; --i)
  for (int i = 0; i < (int)collision_arr_len; ++i)
  {
    phys_collision_t* collision = &collision_arr[i];
		phys_collision_resolution(collision->obj0, collision->obj1, collision->c);
    COLLISION_CALLBACK(collision->obj0->entity_idx, collision->obj1->entity_idx);
  }
  ARRFREE(collision_arr);
  collision_arr_len = 0;

  return;
}

void phys_update_old(f32 dt)
{
	// go through all objs
  // skip objects that are static or not colliders
	for (int i = 0; i < (int)phys_objs_len; ++i) 
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
		for (int j = 0; j < (int)phys_objs_len; ++j) // array of colliders
		{
			if (j == i) { continue; }
			// if (j == i) { break; } // this would ensure all combinations only get checked once
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
      // vec3 p0;
      // vec3 p1;
      // vec3 p2;
      // if (phys_collision_check_aabb_v_triangle(obj0->pos, obj0->collider.box.aabb[1], p0, p1, p2) )
      // {
      //   obj0->rb.force[1] += 200.0f;
      // }

      goto AFTER_TERRAIN_COLLISION; // skip for now
      #ifdef TERRAIN_ADDON
      f32* pos  = NULL;
      f32* pos1 = NULL;
      f32* pos2 = NULL;

      // @TMP:
      if (obj0->collider.type != PHYS_COLLIDER_BOX) { continue; }
      
      // terrain collision
      for (int chunk_idx = 0; chunk_idx < (int)core_data->terrain_chunks_len; ++chunk_idx) 
      {

        if (!core_data->terrain_chunks[chunk_idx].loaded || !core_data->terrain_chunks[chunk_idx].visible) { continue; }
        terrain_chunk_t* chunk = &core_data->terrain_chunks[chunk_idx];

        // get collider_position using cam_pos
        // map cam-pos to 0.0 <-> 1.0 range inside the chunk
        f32 x = ( obj0->pos[0] + ( core_data->terrain_scl * 0.5f));
        f32 z = ( obj0->pos[2] + ( core_data->terrain_scl * 0.5f));
        f32 col_x_len = (f32)core_data->terrain_collider_positions_x_len;
        f32 col_z_len = (f32)core_data->terrain_collider_positions_z_len;
        f32 x_perc = ( x / core_data->terrain_scl );
        f32 z_perc = ( z / core_data->terrain_scl );
        if (x_perc > 1.0f || x_perc < 0.0f || z_perc > 1.0f || z_perc < 0.0f) { continue; }
        // PF("%.2f, %.2f\n", x_perc, z_perc);
        
        // 1. map 0.0<->1,0 to the positions arrays length's in both dimesnions
        // 2. take those two numbers and turn them into idx for the one dimensional array
        // floor(x +0.5): 1.49 -> 1.0, 1.51 -> 2.0
        int x_idx = (int)floor( (x_perc * col_x_len) + 0.5f );
        int z_idx = (int)floor( (z_perc * col_z_len) + 0.5f );
        // int x_idx = (int)floor( (x_perc * col_x_len) );
        // int z_idx = (int)floor( (z_perc * col_z_len) );
        // int x_idx = (int)ceil( (x_perc * col_x_len) );
        // int z_idx = (int)ceil( (z_perc * col_z_len) );
        int idx = x_idx + (z_idx * (int)core_data->terrain_collider_positions_z_len);
        pos  = &chunk->collider_points[idx*3];
        debug_draw_sphere(pos, 1.0f, RGB_F(1, 0, 0)); 
        // P_V(idx);
        
        // if (idx+1 % core_data->terrain_collider_positions_x_len != 0 && 
        //     idx+1 % core_data->terrain_z_len != 0)
        // {
          pos1 = &chunk->collider_points[((u32)idx + core_data->terrain_collider_positions_x_len-1) *3];
          pos2 = &chunk->collider_points[((u32)idx + core_data->terrain_collider_positions_x_len) *3];
          // if (phys_collision_check_aabb_v_triangle(obj0->pos, obj0->collider.box.aabb[1], pos, pos1, pos2) )
          if (phys_collision_check_aabb_v_triangle_obj(obj0, pos, pos1, pos2) )
          { goto TERRAIN_COLLISION; }
          pos1 = &chunk->collider_points[(idx - 1) *3];
          pos2 = &chunk->collider_points[((u32)idx + core_data->terrain_collider_positions_x_len-1) *3];
          if (phys_collision_check_aabb_v_triangle_obj(obj0, pos, pos1, pos2) )
          { goto TERRAIN_COLLISION; }
          pos1 = &chunk->collider_points[((u32)idx + core_data->terrain_collider_positions_x_len   ) *3];
          pos2 = &chunk->collider_points[((u32)idx + core_data->terrain_collider_positions_x_len +1) *3];
          if (phys_collision_check_aabb_v_triangle_obj(obj0, pos, pos1, pos2) )
          { goto TERRAIN_COLLISION; }
          pos1 = &chunk->collider_points[((u32)idx + core_data->terrain_collider_positions_x_len +1) *3];
          pos2 = &chunk->collider_points[((u32)idx +1) *3];
          if (phys_collision_check_aabb_v_triangle_obj(obj0, pos, pos1, pos2) )
          { goto TERRAIN_COLLISION; }
          pos1 = &chunk->collider_points[((u32)idx +1) *3];
          pos2 = &chunk->collider_points[((u32)idx - core_data->terrain_collider_positions_x_len +1) *3];
          if (phys_collision_check_aabb_v_triangle_obj(obj0, pos, pos1, pos2) )
          { goto TERRAIN_COLLISION; }
          pos1 = &chunk->collider_points[((u32)idx - core_data->terrain_collider_positions_x_len +1) *3];
          pos2 = &chunk->collider_points[((u32)idx - core_data->terrain_collider_positions_x_len) *3];
          if (phys_collision_check_aabb_v_triangle_obj(obj0, pos, pos1, pos2) )
          { goto TERRAIN_COLLISION; }
          pos1 = &chunk->collider_points[((u32)idx - core_data->terrain_collider_positions_x_len) *3];
          pos2 = &chunk->collider_points[((u32)idx - core_data->terrain_collider_positions_x_len -1) *3];
          if (phys_collision_check_aabb_v_triangle_obj(obj0, pos, pos1, pos2) )
          { goto TERRAIN_COLLISION; }
          pos1 = &chunk->collider_points[((u32)idx - core_data->terrain_collider_positions_x_len -1) *3];
          pos2 = &chunk->collider_points[((u32)idx - 1) *3];
          if (phys_collision_check_aabb_v_triangle_obj(obj0, pos, pos1, pos2) )
          { goto TERRAIN_COLLISION; }
        // }
        // else { }

        goto AFTER_TERRAIN_COLLISION;
        TERRAIN_COLLISION:;
        P_INFO("hit terrain: %d\n", obj0->entity_idx);
        phys_debug_draw_collider_col(obj0, RGB_F(1, 0, 0));
        REMOVE_FLAG(obj0->flags, (phys_obj_flag)PHYS_HAS_RIGIDBODY);
        debug_draw_sphere_t(obj0->pos, 0.75f, RGB_F(1, 0, 0), 100.0f);
        AFTER_TERRAIN_COLLISION:;
	    }
      #endif // TERRAIN_ADDON
		}
	}
}
