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
		c = phys_collision_check_aabb_v_sphere(obj0, obj1);
	}
	else if (obj0->collider.type == PHYS_COLLIDER_SPHERE && obj1->collider.type == PHYS_COLLIDER_BOX)
	{
		c = phys_collision_check_aabb_v_sphere(obj1, obj0);  // @NOTE: flipped for some reason, penetration ? 
	}

  // ERR_PHYS_OBJ_T_NAN(obj0);
  // ERR_PHYS_OBJ_T_NAN(obj1);
	return c;
}


collision_info_t phys_collision_check_sphere_v_sphere(phys_obj_t* s1, phys_obj_t* s2)
{
	vec3 pos01 = VEC3_INIT(0);
	vec3 pos02 = VEC3_INIT(0);
	// vec3 rot, scale;
	vec3_add(s1->pos, s1->collider.offset, pos01);
	vec3_add(s1->pos, s2->collider.offset, pos02);
	f32 dist = vec3_distance(pos01, pos02);
	dist = dist - (s1->collider.sphere.radius + s2->collider.sphere.radius);

	collision_info_t info;
	info.collision = dist < 0;
	if (!info.collision) { return info; }

	vec3 dir;
	vec3_sub(s2->pos, s1->pos, dir);
	f32 len = vec3_magnitude(dir);
	vec3_normalize(dir, dir);
	vec3_mul_f(dir, len, dir);
	//vec3 d = { fabsf(dist), fabsf(dist), fabsf(dist) };
	//glm_vec3_mul(dir, d, dir);
	vec3_copy(dir, info.direction);
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

  // -------------------------------
	

  return info;
}


// @NOTE: old aabb v. aabb code
	/*
	vec3 center1, center2;
	f32 dist_x = a_max[0] - b_min[0];
	f32 dist_y = a_max[1] - b_min[1];
	f32 dist_z = a_max[2] - b_min[2];
	f32 dist = dist_x > dist_y && dist_x > dist_z ? dist_x :
			   dist_y > dist_x && dist_y > dist_z ? dist_y :
			   dist_z > dist_x && dist_z > dist_y ? dist_z : 0;
	// assert(dist);

	f32 depth = dist * 0.1f;
	depth	  = depth < 0.01f ? 0.0f : depth; // cut off if too small
	info.penetration_depth = depth;

	vec3 normal;
	const vec3 x   = { 1, 0, 0 };
	const vec3 y   = { 0, 1, 0 };
	const vec3 z   = { 0, 0, 1 };
	const vec3 all = { 1, 1, 1 };
	if (b0->pos[0] >= b1->pos[0])
	{
		if (a_max[0] > b_max[0])
		{
			glm_vec3_copy(x, normal);
			glm_vec3_negate(normal);
		}
	}
	else if (b1->pos[0] >= b0->pos[0])
	{
		if (b_max[0] > a_max[0])
		{
			glm_vec3_copy(x, normal);
		}
	}
	if (b0->pos[1] >= b1->pos[1])
	{
		if (a_max[1] > b_max[1])
		{
			glm_vec3_copy(y, normal);
			glm_vec3_negate(normal);
		}
	}
	else if (b1->pos[1] >= b0->pos[1])
	{
		if (b_max[1] > a_max[1])
		{
			glm_vec3_copy(y, normal);
		}
	}
	if (b0->pos[2] >= b1->pos[2])
	{
		if (a_max[2] > b_max[2])
		{
			glm_vec3_copy(z, normal);
			glm_vec3_negate(normal);
		}
	}
	else if (b1->pos[2] >= b0->pos[2])
	{
		if (b_max[2] > a_max[2])
		{
			glm_vec3_copy(z, normal);
		}
	}
	// else if (a_max[1] > b_max[1])
	// {
	// 	glm_vec3_copy(y, normal);
	// 	if (b0->pos[1] >= b1->pos[1]) { glm_vec3_negate(normal); }
	// }
	// else if (a_max[2] > b_max[2])
	// {
	// 	glm_vec3_copy(z, normal);
	// 	if (b0->pos[2] >= b1->pos[2]) { glm_vec3_negate(normal); }
	// }

	if (b0->has_rb && b1->has_rb)
	{
		// use velocity to figure out what the normal is
		// reflect velocity
	}
	else if (b0->has_rb)
	{

	}
	else if (b1->has_rb)
	{

	}

	if (dist == 0) { glm_vec3_copy(all, normal); }
	*/

  // @NOTE: second method
	// // calc the distance both ways
	// vec3 dist1, dist2;
	// vec3_sub(b_min, a_max, dist1);
	// vec3_sub(a_min, b_max, dist2);

	// // get the larges distances for each axis
	// vec3 dist_v = { dist1[0] <= dist2[0] ? dist1[0] : dist2[0],
	// 			          dist1[1] <= dist2[1] ? dist1[1] : dist2[1],
	// 			          dist1[2] <= dist2[2] ? dist1[2] : dist2[2] };


  // // @NOTE: fucks this supposed to mean ?
  // // @TODO: collision could now also be max(dist) > 0 

	// // vec3 dir;
	// // vec3_sub(a_max, b_min, dir);

	// // get axis by prior velocity
	// vec3 axis = VEC3_INIT(0);
  // f32 dist = 0.0f;
	// if (HAS_FLAG(b0->flags, PHYS_HAS_RIGIDBODY))
	// {
	// 	// get biggest axis
	// 	// int a = fabsf(b0->rb.velocity[0]) >= fabsf(b0->rb.velocity[1]) && fabsf(b0->rb.velocity[0]) >= fabsf(b0->rb.velocity[2]) ? 1 :
	// 	// 		fabsf(b0->rb.velocity[1]) >= fabsf(b0->rb.velocity[0]) && fabsf(b0->rb.velocity[1]) >= fabsf(b0->rb.velocity[2]) ? 2 :
	// 	// 		fabsf(b0->rb.velocity[2]) >= fabsf(b0->rb.velocity[0]) && fabsf(b0->rb.velocity[2]) >= fabsf(b0->rb.velocity[1]) ? 3 : 0;

	// 	f32 x = fabsf(b0->rb.velocity[0]);
	// 	f32 y = fabsf(b0->rb.velocity[1]);
	// 	f32 z = fabsf(b0->rb.velocity[2]);

	// 	int a = x >= y && x >= z ? 1 :
	// 		      y >= x && y >= z ? 2 :
	// 		      z >= x && z >= y ? 3 : 0;


	// 	if (a == 1) // x
	// 	{
	// 		f32 dist_x = b_max[0] - a_min[0];
	// 		vec3 v;
	// 		vec3_copy(b0->rb.velocity, v);
	// 		vec3_normalize(v, v);
	// 		v[0] *= -1; // invert x axis
	// 		vec3_mul_f(v, dist_x, v);
	// 		vec3_copy(v, axis);
	// 		
  //     // vec3_copy(VEC3_X(1.0f), axis);
  //     
  //     dist = dist_x;
	// 	}
	// 	else if (a == 2) // y
	// 	{
	// 		f32 dist_y = b_max[1] - a_min[1];
	// 		vec3 v;
	// 		vec3_copy(b0->rb.velocity, v);
	// 		vec3_normalize(v, v);
	// 		v[1] *= -1; // invert y axis
	// 		vec3_mul_f(v, dist_y, v);
	// 		vec3_copy(v, axis);
	// 		// printf(" -> dist_y: %.2f, b.y: %.2f\n", dist_y, b[1]);
  //     
  //     // vec3_copy(VEC3_Y(1.0f), axis);
  //     
  //     dist = dist_y;
	// 	}
	// 	else if (a == 3) // z
	// 	{
	// 		f32 dist_z = b_max[2] - a_min[2];
	// 		vec3 v;
	// 		vec3_copy(b0->rb.velocity, v);
	// 		vec3_normalize(v, v);
	// 		v[2] *= -1; // invert z axis
	// 		vec3_mul_f(v, dist_z, v);
	// 		vec3_copy(v, axis);

  //     // vec3_copy(VEC3_Z(1.0f), axis);
  //     
  //     dist = dist_z;
	// 	}

	// 	// printf(" -> velocity check, x: %.2f, y: %.2f, z: %.2f, a; %d, v.x: %.2f, v.y: %.2f, v.z: %.2f\n", axis[0], axis[1], axis[2], a, b0->rb.velocity[0], b0->rb.velocity[1], b0->rb.velocity[2]);
	// 
  //   // if (HAS_FLAG(b1->flags, PHYS_HAS_RIGIDBODY))
  //   // {
  //   //   // get biggest axis
  //   //   vec3_copy(fabsf(b1->rb.velocity[0]) >= fabsf(b1->rb.velocity[1]) && fabsf(b1->rb.velocity[0]) >= fabsf(b1->rb.velocity[2]) ? VEC3_X(x) :
  //   //             fabsf(b1->rb.velocity[1]) >= fabsf(b1->rb.velocity[0]) && fabsf(b1->rb.velocity[1]) >= fabsf(b1->rb.velocity[2]) ? VEC3_Y(y) :
  //   //             fabsf(b1->rb.velocity[2]) >= fabsf(b1->rb.velocity[0]) && fabsf(b1->rb.velocity[2]) >= fabsf(b1->rb.velocity[1]) ? VEC3_Z(z) : VEC3(0), axis);

  //   //   // vec3_negate(axis, axis);
  //   //   vec3_mul_f(axis, 0.1f, axis);
  //   // }

	// }
	// // else { printf(" -> velocity wasnt checked\n"); }


  // // @TODO: dont dir * depth
  // // info.depth = vec3_magnitude(axis); //  * 0.5f;  // @UNSURE: * 0.5f, also ->, @NOTE: not sure id actually depth
  // info.depth = dist;  // same as magnitude(axis) 
  // // if (info.depth > 0.3f) { P_F32(info.depth); P_VEC3(dist_v); P_F32(dist); } 
  // vec3_normalize(axis, axis);
  // // if (axis[1] > 0.5f) { P_VEC3(axis); } 
  // vec3_copy(axis, info.direction); // dist
  // 

  // //
  // //  @NOTE: velocity way to big in oposite direction 
  // //
  


collision_info_t phys_collision_check_aabb_v_sphere(phys_obj_t* b, phys_obj_t* s)
{
  P_ERR("collision check aabb v. sphere not implemented");
	// @TODO:
  collision_info_t info;
  info.collision = false;
  return info;
}
