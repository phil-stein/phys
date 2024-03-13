#include "phys_collision.h"
#include "phys_types.h"
#include "phys_debug_draw.h"

// ---- collision checks ----

collision_info_t phys_collision_check(phys_obj_t* obj0, phys_obj_t* obj1)
{
	collision_info_t c;
	c.collision = false;
	if (!PHYS_OBJ_HAS_COLLIDER(obj0) || !PHYS_OBJ_HAS_COLLIDER(obj1)) { return c; }

  // ERR_PHYS_OBJ_T_NAN(obj0);
  // ERR_PHYS_OBJ_T_NAN(obj1);
	
  // same v same
	if (obj0->collider.type == PHYS_COLLIDER_SPHERE && obj1->collider.type == PHYS_COLLIDER_SPHERE)
	{
		// @TODO: @UNSURE: why do i have to check both for aabb but not sphere ?
    // c = phys_collision_check_sphere_v_sphere(obj0, obj1);
		c = phys_collision_check_sphere_v_sphere_swept(obj0, obj1);
	}
	else if (obj0->collider.type == PHYS_COLLIDER_BOX && obj1->collider.type == PHYS_COLLIDER_BOX)
	{
    // check normal first and then using swept for tunneling
		c = phys_collision_check_aabb_v_aabb(obj0, obj1);
    
    // if obj hasnt moved enough dont do test bc. normal test gets done before
    // this is just for tunneling
    collision_info_t swept;
    swept.collision = false;

    // get smallest length of aabb's, as min dist travelled by obj for swept check
    f32 obj0_min = phys_aabb_smallest_side(obj0->collider.box.aabb);
    f32 obj1_min = phys_aabb_smallest_side(obj1->collider.box.aabb);
    f32 min_dist = obj0_min <= obj1_min ? obj0_min : obj1_min;
    min_dist *= 0.25f; // 0.5f;  // @NOTE: 0.5f should be enough but this is more precise prob.
  
    if ( !c.collision && vec3_distance(obj0->pos, obj0->last_pos) > min_dist) // 0.1f ) // hasnt moved since last frame
    { 
      // debug_draw_sphere_register(obj0->pos, 0.1f, RGB_F(1, 0, 0));
      swept = phys_collision_check_aabb_v_aabb_swept(obj0, obj1); 
    }
    if (swept.collision) { c = swept; }
	}
	// box v sphere
	// else if (obj0->collider.type == PHYS_COLLIDER_BOX && obj1->collider.type == PHYS_COLLIDER_SPHERE)
	// {
	// 	c = phys_collision_check_aabb_v_sphere(obj0, obj1, false);  // false: no switching, b is first obj
	// }
	// else if (obj0->collider.type == PHYS_COLLIDER_SPHERE && obj1->collider.type == PHYS_COLLIDER_BOX)
	// {
	// 	c = phys_collision_check_aabb_v_sphere(obj1, obj0, true);   // true: switch, s is first obj  
	// }
  else
  {
	  bool obj0_is_sphere = obj0->collider.type == PHYS_COLLIDER_SPHERE;
    phys_obj_t* box    = obj0_is_sphere ? obj1 : obj0;
    phys_obj_t* sphere = obj0_is_sphere ? obj0 : obj1;

    // check normal first and then using swept for tunneling
	  c = phys_collision_check_aabb_v_sphere(box, sphere, obj0_is_sphere);   // true: switch, s is first obj  
    
    // if obj hasnt moved enough dont do test bc. normal test gets done before
    // this is just for tunneling
    collision_info_t swept;
    swept.collision = false;

    // get smallest length of aabb's or radius, as min dist travelled by obj for swept check
    f32 box_min    = phys_aabb_smallest_side(box->collider.box.aabb) * 0.25f;
    f32 sphere_min = sphere->collider.sphere.radius * ((sphere->scl[0] + sphere->scl[1] + sphere->scl[2]) * 0.33f);
    f32 min_dist   = MIN(box_min, sphere_min);
  
    if ( !c.collision && vec3_distance(sphere->pos, sphere->last_pos) > min_dist) // 0.1f ) // hasnt moved since last frame
    { 
      // debug_draw_sphere_register(box->pos, 0.1f, RGB_F(1, 0, 0));
	    swept = phys_collision_check_aabb_v_sphere_swept(box, sphere, obj0_is_sphere);   // true: switch, s is first obj  
    }
    if (swept.collision) { c = swept; }
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

  info.depth =  vec3_distance(pos0, pos1);
	info.depth -= (radius0 + radius1);

	info.collision = info.depth < 0;
  // info.depth *= -1;
	if (!info.collision) { return info; }

	vec3_sub(pos1, pos0, info.direction);
	vec3_normalize(info.direction, info.direction);
	
  return info;
}


collision_info_t phys_collision_check_sphere_v_sphere_swept(phys_obj_t* s0, phys_obj_t* s1)
{
  // treat s0 as point and raycast against s1 with its radius being the sum of both radii
  // this way only one raycast is required
  // raycast is from s0 last pos toward s0 current pos
	
  collision_info_t info;
  info.collision = false;
	
  vec3 pos0      = VEC3_INIT(0);  // current s0 pos
  vec3 last_pos0 = VEC3_INIT(0);  // s0 pos last frame
	vec3 pos1      = VEC3_INIT(0);	// current s1 pos
  vec3_add(s0->pos,      s0->collider.offset, pos0);
  vec3_add(s0->last_pos, s0->collider.offset, last_pos0);
	vec3_add(s1->pos,      s1->collider.offset, pos1);
	

  // s0's and s1's radii combined
  f32 radius = s0->collider.sphere.radius * ((s0->scl[0] + s0->scl[1] + s0->scl[2]) * 0.33f) +
               s1->collider.sphere.radius * ((s1->scl[0] + s1->scl[1] + s1->scl[2]) * 0.33f);

  // ray starting at sphere0 last pos pointing toward sphere0 cur pos
  ray_t ray = RAY_T_INIT_ZERO();
  vec3_sub(pos0, last_pos0, ray.dir);
  vec3_normalize(ray.dir, ray.dir);
  vec3_copy(last_pos0, ray.pos);
  
  // f32  dist = 0;
  // vec3 hit_point;
  ray_hit_t hit;
  info.collision = phys_collision_check_ray_v_sphere(&ray, pos1, radius, &hit); // &dist, hit_point);
  
  if (!info.collision) { return info; }
 
  // @TODO: use dist for this
  // the ray hit but after where the sphere0 moved
  if (vec3_distance(pos0, hit.hit_point) > vec3_distance(last_pos0, pos0))
  {
    info.collision = false;
    return info;
  }


  // get collission depth and dir
  info.depth = vec3_distance(pos0, hit.hit_point);
	vec3_sub(pos0, pos1, info.direction);
	vec3_normalize(info.direction, info.direction);
  
  // @TODO: info.grounded

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
  if      (F32_EQ(info.depth, x1)) { vec3_copy(VEC3_X(-1), info.direction); }
  else if (F32_EQ(info.depth, y1)) { vec3_copy(VEC3_Y(-1), info.direction); info.grounded = true; }
  else if (F32_EQ(info.depth, z1)) { vec3_copy(VEC3_Z(-1), info.direction); }
  else if (F32_EQ(info.depth, x2)) { vec3_copy(VEC3_X(-1),  info.direction); }
  else if (F32_EQ(info.depth, y2)) { vec3_copy(VEC3_Y(-1),  info.direction); }
  else                             { vec3_copy(VEC3_Z(-1),  info.direction); }

  // if (info.collision) { PF("| had collision: %.2f\n", info.depth); }
  
  // -------------------------------
	

  return info;
}


collision_info_t phys_collision_check_aabb_v_aabb_swept(phys_obj_t* b0, phys_obj_t* b1)
{
  // treat b0 as point and raycast against b1 with its aabb being the sum of both aabb's 
  // this way only one raycast is required
  // raycast is from b0 last pos toward b0 current pos
	
  collision_info_t info;
  info.collision = false;
	
  vec3 pos0      = VEC3_INIT(0);  // current b0 pos
  vec3 last_pos0 = VEC3_INIT(0);  // b0 pos last frame
	vec3 pos1      = VEC3_INIT(0);	// current b1 pos
  vec3_add(b0->pos,      b0->collider.offset, pos0);
  vec3_add(b0->last_pos, b0->collider.offset, last_pos0);
	vec3_add(b1->pos,      b1->collider.offset, pos1);
	

  // b0's and b1's aabb's combined , at b1 pos
	vec3 min; vec3 max;
	vec3_copy(b0->collider.box.aabb[0], min);
	vec3_copy(b0->collider.box.aabb[1], max);
	vec3_add(min, b1->collider.box.aabb[0], min);
	vec3_add(max, b1->collider.box.aabb[1], max);
  // add position & offset to min & max 
  vec3_mul(min, b1->scl, min);
  vec3_mul(max, b1->scl, max);
	vec3_add(min, b1->pos, min);
	vec3_add(max, b1->pos, max);
	vec3_add(min, b1->collider.offset, min);
	vec3_add(max, b1->collider.offset, max);

  // ray starting at sphere0 last pos pointing toward sphere0 cur pos
  ray_t ray = RAY_T_INIT_ZERO();
  vec3_sub(pos0, last_pos0, ray.dir);
  vec3_normalize(ray.dir, ray.dir);
  vec3_copy(last_pos0, ray.pos);

  // f32  dist = 0;
  // vec3 hit_point;
  ray_hit_t hit;
  info.collision = phys_collision_check_ray_v_aabb(&ray, min, max, &hit); // &dist, hit_point);
  
  if (!info.collision) { return info; }

  // P_F32(dist);
  // P_F32(vec3_distance(pos0, hit_point));
  // P_F32(vec3_distance(last_pos0, pos0));
  // @TODO: use dist for this
  // the ray hit but after where the sphere0 moved
  if (vec3_distance(pos0, hit.hit_point) > vec3_distance(last_pos0, pos0))
  {
    info.collision = false;
    return info;
  }

  // get direction, bc. aabb can only be in one axis
  vec3_sub(pos0, pos1, info.direction);
	vec3_normalize(info.direction, info.direction);
  vec3 dir_abs;
  vec3_abs(info.direction, dir_abs);
  info.direction[0] = dir_abs[0] >= dir_abs[1] && dir_abs[0] >= dir_abs[2] ? (info.direction[0] < 0 ? -1 : 1) : 0;
  info.direction[1] = dir_abs[1] >  dir_abs[0] && dir_abs[1] >  dir_abs[2] ? (info.direction[1] < 0 ? -1 : 1) : 0;
  info.direction[2] = dir_abs[2] >  dir_abs[0] && dir_abs[2] >  dir_abs[1] ? (info.direction[2] < 0 ? -1 : 1) : 0;
  
  info.depth = !F32_EQ(info.direction[0], 0.0f) ? fabsf( pos0[0] - hit.hit_point[0] ) :
               !F32_EQ(info.direction[1], 0.0f) ? fabsf( pos0[1] - hit.hit_point[1] ) :
               !F32_EQ(info.direction[2], 0.0f) ? fabsf( pos0[2] - hit.hit_point[2] ) : 0;

  // @TODO: info.grounded

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

collision_info_t phys_collision_check_aabb_v_sphere_swept(phys_obj_t* b, phys_obj_t* s, bool switch_obj_places)
{
  (void)switch_obj_places;
  // treat s as point and raycast against b with its aabb scaled by s's radius 
  // this way only one raycast is required
  // raycast is from s last pos toward s current pos
	
  collision_info_t info;
  info.collision = false;
	
  vec3 pos0      = VEC3_INIT(0);  // current s pos
  vec3 last_pos0 = VEC3_INIT(0);  // s pos last frame
	vec3 pos1      = VEC3_INIT(0);	// current b pos
  vec3_add(s->pos,      s->collider.offset, pos0);
  vec3_add(s->last_pos, s->collider.offset, last_pos0);
	vec3_add(b->pos,      b->collider.offset, pos1);
  
  f32 radius = s->collider.sphere.radius * ((s->scl[0] + s->scl[1] + s->scl[2]) * 0.33f);
	

  // b's aabb scaled by s's radius s's pos
	vec3 min; vec3 max;
	vec3_copy(b->collider.box.aabb[0], min);
	vec3_copy(b->collider.box.aabb[1], max);
  // add radius
  vec3_sub_f(min, radius, min);
  vec3_add_f(max, radius, max);
  // add position & offset to min & max 
  vec3_mul(min, b->scl, min);
  vec3_mul(max, b->scl, max);
	vec3_add(min, b->pos, min);
	vec3_add(max, b->pos, max);
	vec3_add(min, b->collider.offset, min);
	vec3_add(max, b->collider.offset, max);


  // // @TMP: 
  // if (b->entity_idx != 1)  { info.collision = false; return info; }
  // if (s->entity_idx != 23) { info.collision = false; return info; }
  // if (s->entity_idx <= 23) { info.collision = false; return info; }
  // f32 box_min    = phys_aabb_smallest_side(b->collider.box.aabb) * 0.25;
  // f32 sphere_min = radius;
  // f32 min_dist   = MIN(box_min, sphere_min);
  // P_F32(min_dist);

  // ray starting at sphere0 last pos pointing toward sphere0 cur pos
  ray_t ray = RAY_T_INIT_ZERO();
  vec3_sub(pos0, last_pos0, ray.dir);
  vec3_normalize(ray.dir, ray.dir);
  vec3_copy(last_pos0, ray.pos);

  // // @TMP:
  // vec3 ray_end;
  // vec3_mul_f(ray.dir, 25.0f, ray_end);
  // vec3_add(ray_end, ray.pos, ray_end);
  // debug_draw_line_register(ray.pos, ray_end, RGB_F(0, 1, 0));
  // debug_draw_sphere_register(ray.pos, 0.1f, RGB_F(0, 1, 0));
  //  
  // // debug_draw_sphere_register(pos1, 0.1f, RGB_F(1, 0, 0));
  // phys_debug_draw_aabb(min, max, RGB_F(1, 0, 0));
 

  // f32  dist = 0;
  // vec3 hit_point;
  ray_hit_t hit;
  info.collision = phys_collision_check_ray_v_aabb(&ray, min, max, &hit); // &dist, hit_point);
  
  if (!info.collision) { return info; }
 
  // @TODO: use dist for this
  // the ray hit but after where the sphere0 moved
  if (vec3_distance(pos0, hit.hit_point) > vec3_distance(last_pos0, pos0))
  {
    // debug_draw_sphere_register(hit_point, 0.05f, RGB_F(1, 1, 1));
    info.collision = false;
    return info;
  }

  // // @TMP:
  // debug_draw_sphere_register(hit_point, 0.2f, RGB_F(0, 0, 1));
  // // info.collision = false;
  // // return info;

  // info.depth = vec3_distance(pos0, hit_point);
	
  // get direction, bc. aabb can only be in one axis
  vec3_sub(pos0, pos1, info.direction);
  // if (switch_obj_places)
  // { vec3_sub(pos0, pos1, info.direction); }
  // else
  // { vec3_sub(pos1, pos0, info.direction); }
	vec3_normalize(info.direction, info.direction);
  vec3 dir_abs;
  vec3_abs(info.direction, dir_abs);
  info.direction[0] = dir_abs[0] >= dir_abs[1] && dir_abs[0] >= dir_abs[2] ? (info.direction[0] < 0 ? -1 : 1) : 0;
  info.direction[1] = dir_abs[1] >  dir_abs[0] && dir_abs[1] >  dir_abs[2] ? (info.direction[1] < 0 ? -1 : 1) : 0;
  info.direction[2] = dir_abs[2] >  dir_abs[0] && dir_abs[2] >  dir_abs[1] ? (info.direction[2] < 0 ? -1 : 1) : 0;
  
  info.depth = !F32_EQ(info.direction[0], 0.0f) ? fabsf( pos0[0] - hit.hit_point[0] ) :
               !F32_EQ(info.direction[1], 0.0f) ? fabsf( pos0[1] - hit.hit_point[1] ) :
               !F32_EQ(info.direction[2], 0.0f) ? fabsf( pos0[2] - hit.hit_point[2] ) : 0;

  // // switch dir if sphere was obj0 not obj1
  // if (switch_obj_places) { vec3_mul_f(info.direction, -1.0f, info.direction); }

  // // @TMP:
	// vec3 end;
	// vec3_mul_f(info.direction, info.depth * 100.0f, end);
  // vec3_add(end, pos0, end);
  // debug_draw_line_register(pos0, end, RGB_F(1, 0, 1));
  // debug_draw_sphere_register(end, 0.1f, RGB_F(1, 0, 1));
	
  // @TMP:
  // info.collision = false;

  // @TODO: info.grounded

  return info;
}

