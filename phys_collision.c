#include "phys_collision.h"
#include "core/debug/debug_draw.h"

// ---- collision checks ----

collision_info_t phys_collision_check(phys_obj_t* obj0, phys_obj_t* obj1)
{
	if (!PHYS_OBJ_HAS_COLLIDER(obj0) || !PHYS_OBJ_HAS_COLLIDER(obj1)) 
  { collision_info_t c; c.collision = false; return c; }
	collision_info_t c;
	c.collision = false;

  // ERR_PHYS_OBJ_T_NAN(obj0);
  // ERR_PHYS_OBJ_T_NAN(obj1);
	
  // same v same
	if (obj0->collider.type == PHYS_COLLIDER_SPHERE && obj1->collider.type == PHYS_COLLIDER_SPHERE)
	{
		c = phys_collision_check_sphere_v_sphere(obj0, obj1);
	}
	else if (obj0->collider.type == PHYS_COLLIDER_BOX && obj1->collider.type == PHYS_COLLIDER_BOX)
	{
		c = phys_collision_check_aabb_v_aabb(obj0, obj1);
	}
	// box v sphere
	else if (obj0->collider.type == PHYS_COLLIDER_BOX && obj1->collider.type == PHYS_COLLIDER_SPHERE)
	{
		c = phys_collision_check_aabb_v_sphere(obj0, obj1, false);  // false: no switching, b is first obj
	}
	else if (obj0->collider.type == PHYS_COLLIDER_SPHERE && obj1->collider.type == PHYS_COLLIDER_BOX)
	{
		c = phys_collision_check_aabb_v_sphere(obj1, obj0, true);   // true: switch, s is first obj  
	}

  // ERR_PHYS_OBJ_T_NAN(obj0);
  // ERR_PHYS_OBJ_T_NAN(obj1);
	return c;
}


collision_info_t phys_collision_check_sphere_v_sphere(phys_obj_t* s0, phys_obj_t* s1)
{
	collision_info_t info;
  info.collision = false;
	
  vec3 pos0 = VEC3_INIT(0);
	vec3 pos1 = VEC3_INIT(0);	
  vec3_add(s0->pos, s0->collider.offset, pos0);
	vec3_add(s1->pos, s1->collider.offset, pos1);
	

  f32 radius0 = s0->collider.sphere.radius * ((s0->scl[0] + s0->scl[1] + s0->scl[2]) * 0.33f);
  f32 radius1 = s1->collider.sphere.radius * ((s1->scl[0] + s1->scl[1] + s1->scl[2]) * 0.33f);

  info.depth = vec3_distance(pos0, pos1);
	info.depth = info.depth - (radius0 + radius1);

	info.collision = info.depth < 0;
  // info.depth *= -1;
	if (!info.collision) { return info; }

	vec3_sub(pos1, pos0, info.direction);
	vec3_normalize(info.direction, info.direction);
	
  return info;
}

collision_info_t phys_collision_check_aabb_v_aabb(phys_obj_t* b0, phys_obj_t* b1)
{
	// add position & offset to min & max of both colliders
	vec3 a_min; vec3 a_max;
	vec3_copy(b0->collider.box.aabb[0], a_min);
	vec3_copy(b0->collider.box.aabb[1], a_max);
  vec3_mul(a_min, b0->scl, a_min);
  vec3_mul(a_max, b0->scl, a_max);
	vec3_add(a_min, b0->pos, a_min);
	vec3_add(a_max, b0->pos, a_max);
	vec3_add(a_min, b0->collider.offset, a_min);
	vec3_add(a_max, b0->collider.offset, a_max);

	vec3 b_min; vec3 b_max;
	vec3_copy(b1->collider.box.aabb[0], b_min);
	vec3_copy(b1->collider.box.aabb[1], b_max);
  vec3_mul(b_min, b1->scl, b_min);
  vec3_mul(b_max, b1->scl, b_max);
	vec3_add(b_min, b1->pos, b_min);
	vec3_add(b_max, b1->pos, b_max);
	vec3_add(b_min, b1->collider.offset, b_min);
	vec3_add(b_max, b1->collider.offset, b_max);

	bool collision = 
		(a_min[0] <= b_max[0] && a_max[0] >= b_min[0]) &&
		(a_min[1] <= b_max[1] && a_max[1] >= b_min[1]) &&
		(a_min[2] <= b_max[2] && a_max[2] >= b_min[2]);

	collision_info_t info = COLLISION_INFO_T_INIT();
	info.collision = collision;
	if (!collision) { return info; }

  // @NOTE: this is where the old code was

  // @NOTE: new dist method --------

  f32 x1 = a_min[0] - b_max[0];
  f32 y1 = a_min[1] - b_max[1];
  f32 z1 = a_min[2] - b_max[2];
  f32 x2 = a_max[0] - b_min[0];
  f32 y2 = a_max[1] - b_min[1];
  f32 z2 = a_max[2] - b_min[2]; 

  f32 ax1 = fabsf(x1);
  f32 ay1 = fabsf(y1);
  f32 az1 = fabsf(z1);
  f32 ax2 = fabsf(x2);
  f32 ay2 = fabsf(y2);
  f32 az2 = fabsf(z2);

  info.depth = ax1 < ay1 && ax1 < az1 && ax1 < ax2 && ax1 < ay2 && ax1 < az2 ? x1 :
               ay1 < az1 && ay1 < ax2 && ay1 < ay2 && ay1 < az2              ? y1 : 
               az1 < ax2 && az1 < ay2 && az1 < az2                           ? z1 :
               ax2 < ay2 && ax2 < az2                                        ? x2 :
               ay2 < az2                                                     ? y2 :
                                                                               z2;

  // @NOTE: mult by this and it sinks into the other aabb, mult by one zero less and it starts bouncing
  //        something fishy is going on here, check values here, but defenitely also in resolution
  // info.depth *= 1.00000001f;  // makes bouncy ._.


  // @NOTE: the direction on everything but y1 hasn't been checked, and might be wrong
  info.grounded = false;
  if      (info.depth == x1) { vec3_copy(VEC3_X(-1), info.direction); }
  else if (info.depth == y1) { vec3_copy(VEC3_Y(-1), info.direction); info.grounded = true; }
  else if (info.depth == z1) { vec3_copy(VEC3_Z(-1), info.direction); }
  else if (info.depth == x2) { vec3_copy(VEC3_X(-1),  info.direction); }
  else if (info.depth == y2) { vec3_copy(VEC3_Y(-1),  info.direction); }
  else                       { vec3_copy(VEC3_Z(-1),  info.direction); }

  // if (info.collision) { PF("| had collision: %.2f\n", info.depth); }
  
  // -------------------------------
	

  return info;
}

// taken from: https://developer.mozilla.org/en-US/docs/Games/Techniques/3D_collision_detection:w
//
collision_info_t phys_collision_check_aabb_v_sphere(phys_obj_t* b, phys_obj_t* s, bool switch_obj_places)
{
  collision_info_t info;
  info.collision = false;

  vec3 b_pos, s_pos;
  vec3_add(b->pos, b->collider.offset, b_pos);
	vec3_add(s->pos, s->collider.offset, s_pos);

  f32 radius = s->collider.sphere.radius * ((s->scl[0] + s->scl[1] + s->scl[2]) * 0.33f);

  // add position & offset to min & max of box collider
	vec3 min; vec3 max;
	vec3_copy(b->collider.box.aabb[0], min);
	vec3_copy(b->collider.box.aabb[1], max);
  vec3_mul(min, b->scl, min);
  vec3_mul(max, b->scl, max);
	vec3_add(min, b->pos, min);
	vec3_add(max, b->pos, max);
	vec3_add(min, b->collider.offset, min);
	vec3_add(max, b->collider.offset, max);
  
  // get box closest point to sphere center by clamping
  f32 x = MAX(min[0], MIN(s_pos[0], max[0]));
  f32 y = MAX(min[1], MIN(s_pos[1], max[1]));
  f32 z = MAX(min[2], MIN(s_pos[2], max[2]));

  // we are using multiplications because is faster than calling powf 
  f32 distance = F32_SQRT(
      (x - s_pos[0]) * (x - s_pos[0]) +
      (y - s_pos[1]) * (y - s_pos[1]) +
      (z - s_pos[2]) * (z - s_pos[2])
      );

  info.depth     = distance - radius;
  info.collision = distance < radius;
  // if (info.collision) { PF("| had collision with sphere[%d]: %.2f\n", s->entity_idx, info.depth); }
 
  vec3 p = { x, y, z };  // closest point on aabb to sphere
  if (switch_obj_places)
  { vec3_sub(p, s_pos, info.direction); }
  else
  { vec3_sub(s_pos, p, info.direction); }
  vec3_normalize(info.direction, info.direction);

  // @TODO:
  info.grounded = false; 

  return info;
}


