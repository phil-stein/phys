#include "phys_resolution.h"
#include "phys_debug_draw.h"
#include "math/math_inc.h"


// ---- collision response ----


void phys_collision_resolution(phys_obj_t* obj0, phys_obj_t* obj1, collision_info_t info)
{
  
  bool obj0_has_rb = PHYS_OBJ_HAS_RIGIDBODY(obj0);
  bool obj1_has_rb = PHYS_OBJ_HAS_RIGIDBODY(obj1);

	if (!obj0_has_rb) { return; }
	
  // position correction
	vec3 dist;
	vec3_mul_f(info.direction, info.depth * 1.0f, dist);  // 0.5f
	vec3_add(obj0->pos, dist, obj0->pos);

	// @TODO: maybe use ellastic collision equation
	// 	  or at least use mass to determine a ratio
	//
	// @NOTE: yeah, just use ellastic collisions
	// 	  maybe mult by like 0.9 or use drag

	// impact force
	const f32 force_mult = 500.0f; // 500.0f; // 1000.0f;
	if (obj1_has_rb)
	{
		f32 ratio0 = obj1->rb.mass / obj0->rb.mass;
		f32 ratio1 = obj0->rb.mass / obj1->rb.mass;
    
   vec3 f0, f1;
		vec3_mul_f(dist,  force_mult * ratio0, f0);
		vec3_mul_f(dist, -force_mult * ratio1, f1);

    vec3_add(obj0->rb.force, f0, obj0->rb.force);
    vec3_add(obj1->rb.force, f1, obj1->rb.force); 
	}
	else
	{
    // if (info.direction[1] != 0)
    // { 
    //   obj0->rb.velocity[1] = 0;
    //   obj0->rb.force[1]    = 0;
    // }
		vec3 f0;
		// 1.9 * force_mult, because not elastic
		vec3_mul_f(dist, force_mult * 2.0f, f0);
    vec3_add(obj0->rb.force, f0, obj0->rb.force);
	}

}


// @NOTE: old resolution system, based on winter-dev's system
/*
	
// @TODO: get working lol
// @TODO: put together
// friction
// if (info.grounded) // (o1->is_dynamic && o0->is_grounded)
// {
// 	vec3 f;
// 	vec3_mul_f(obj1->rb.velocity, 1, f);
// 	// phys_obj_add_force(o0->idx, f);
//   vec3_add(obj0->rb.velocity, f, obj0->rb.velocity);
// }
// if (o1->is_grounded) // (o1->is_dynamic && o0->is_grounded)
// {
// 	vec2 f;
// 	vec2_mul_f(o0->velocity, 1, f);
// 	// phys_obj_add_force(o1->idx, f);
//   vec3_add(obj0->rb.velocity, f0);
// }

void phys_collision_response(phys_obj_t* obj0, phys_obj_t* obj1, collision_info_t info)
{
	if (!PHYS_OBJ_HAS_RIGIDBODY(obj0)) { return; }
 
	// move the objects out of one another
	phys_collision_response_resolve_position(obj0, obj1, info);

	// adjust velocity accordingly
	phys_collision_response_resolve_velocity(obj0, obj1, info);

}

// taken from winterdev's 'IwEngine' https://github.com/IainWinter/IwEngine/blob/master/IwEngine
void phys_collision_response_resolve_position(phys_obj_t* obj0, phys_obj_t* obj1, collision_info_t info)
{
  // ERR_CHECK(obj->rb.mass > 1.0f, "mass: %f, id: %d\n", obj->rb.mass, obj->entity_idx);
  // PF("id: %d, mass: %f\n", obj->entity_idx, obj->rb.mass);
  // PF("id: %d, ", obj->entity_idx); P_VEC3(obj->rb.velocity);
  // P_COLLIDER_TYPE_T(obj->collider.type);
  // P_COLLIDER_T(obj->collider);
  // P_RIGIDBODY_T(obj->rb);
  // P_PHYS_OBJ_FLAGS_T(obj->flags);
  // P_PHYS_OBJ_T(obj0);
  // P_PHYS_OBJ_T(obj1);
  // P_PHYS_OBJ_T_NAN(obj0);
  // P_PHYS_OBJ_T_NAN(obj1);

  const f32 percent = 0.8f;
  const f32 slop    = 0.01f;
  
  bool obj0_has_rb = PHYS_OBJ_HAS_RIGIDBODY(obj0);
  bool obj1_has_rb = PHYS_OBJ_HAS_RIGIDBODY(obj1);
  
  // @TODO: replace 1 / mass by a inv_mass member in rigidbody_t
  // inverse mass for dynamic and 0 for static
  f32 inv_mass0 = obj0_has_rb ? 1.0f / obj0->rb.mass : 0.0f;
  f32 inv_mass1 = obj1_has_rb ? 1.0f / obj1->rb.mass : 0.0f;
  
  // correction = direction * percent * max(depth - slop, 0) / (inv_mass0 + inv_mass1)
  vec3 correction;
  vec3_mul_f(info.direction, percent, correction);
  vec3_mul_f(correction, MAX(info.depth - slop, 0.0f), correction);
  vec3_div_f(correction, inv_mass0 + inv_mass1, correction);
  
  vec3 delta0 = { 0, 0, 0 };  
  vec3 delta1 = { 0, 0, 0 }; 
  
  if (obj0_has_rb) 
  {
    vec3_mul_f(correction, inv_mass0, delta0);
    vec3_add(obj0->pos, delta0, obj0->pos);     
  }
  if (obj1_has_rb) 
  { 
    vec3_mul_f(correction, inv_mass1, delta1); 
    vec3_sub(obj1->pos, delta1, obj1->pos);
  }
}
// taken from winterdev's 'IwEngine' https://github.com/IainWinter/IwEngine/blob/master/IwEngine
void phys_collision_response_resolve_velocity(phys_obj_t* obj0, phys_obj_t* obj1, collision_info_t info)
{
  // @NOTE: unsure if already normalized
  // vec3_normalize(info.direction, info.direction);

  // P_VEC3(info.direction);

  bool obj0_has_rb = PHYS_OBJ_HAS_RIGIDBODY(obj0);
  bool obj1_has_rb = PHYS_OBJ_HAS_RIGIDBODY(obj1);

  vec3 velocity0, velocity1;
  vec3_copy(obj0_has_rb ? obj0->rb.velocity : VEC3(0), velocity0); 
  vec3_copy(obj1_has_rb ? obj1->rb.velocity : VEC3(0), velocity1);
  
  // P_VEC3(velocity0);
  // P_VEC3(velocity1);

  vec3 r_velocity;
  vec3_sub(velocity1, velocity0, r_velocity);

  // P_VEC3(r_velocity);

  // vec3 dir_norm; 
  // vec3_copy(info.direction, dir_norm);
  // vec3_normalize(dir_norm, dir_norm);
  // P_VEC3(info.direction);
  // P_VEC3(dir_norm);
  f32 n_spd = vec3_dot(r_velocity, info.direction);
  // f32 n_spd = vec3_dot(r_velocity, dir_norm);
  // P_F32(n_spd);
  // negative impulse would drive objs further into each other
  // if (n_spd >= 0) { return; } 
  // if (n_spd < 0) { return; } 
  // P(" -> passed n_spd"); 
 
  // inverse mass for dynamic and 1 for static
  f32 inv_mass0 = obj0_has_rb ? 1.0f / obj0->rb.mass : 1.0f;
  f32 inv_mass1 = obj1_has_rb ? 1.0f / obj1->rb.mass : 1.0f;

  // -- impulse --

  f32 e = (obj0_has_rb ? obj0->rb.restitution : 1.0f) * 
          (obj1_has_rb ? obj1->rb.restitution : 1.0f);

  f32 j = -(1.0f + e) * n_spd / (inv_mass0 + inv_mass1);

  vec3 impulse;
  vec3_mul_f(info.direction, j, impulse);

  if (obj0_has_rb)
  {
    vec3 imp_inv_mass;
    vec3_mul_f(impulse, inv_mass0, imp_inv_mass);
    vec3_sub(velocity0, imp_inv_mass, velocity0);
  }
  if (obj1_has_rb)
  {
    vec3 imp_inv_mass;
    vec3_mul_f(impulse, inv_mass1, imp_inv_mass);
    vec3_sub(velocity1, imp_inv_mass, velocity1);
  }

  // -- friction --
 
  vec3_sub(velocity1, velocity0, r_velocity);
  n_spd = vec3_dot(r_velocity, info.direction); 

  vec3 tangent;
  vec3_mul_f(info.direction, n_spd, tangent);
  vec3_sub(tangent, r_velocity, tangent);

  if (vec3_magnitude(tangent) > 0.0001f)
  { vec3_normalize(tangent, tangent); }

  f32 f_velocity = vec3_dot(r_velocity, tangent);

  f32 static_f0  = obj0_has_rb ? obj0->rb.static_friction  : 0.0f;
  f32 static_f1  = obj1_has_rb ? obj1->rb.static_friction  : 0.0f;
  f32 dynamic_f0 = obj0_has_rb ? obj0->rb.dynamic_friction : 0.0f;
  f32 dynamic_f1 = obj1_has_rb ? obj1->rb.dynamic_friction : 0.0f;

  f32 mu = vec2_magnitude(VEC2_XY(static_f0, static_f1));

  f32 f = -f_velocity / (inv_mass0 + inv_mass1);

  vec3 friction;

  if (fabsf(f) < j * mu)  // @UNSURE: if fabsf or abs
  {
    vec3_mul_f(tangent, f, friction);
  }
  else 
  {
    mu = vec2_magnitude(VEC2_XY(dynamic_f0, dynamic_f1));
    vec3_mul_f(tangent, mu, friction);
    vec3_mul_f(friction, -j, friction);
  }

  // -- apply --
  
  if (obj0_has_rb)
  {
    // if (info.grounded) { velocity0[1] = MAX(velocity0[1], 0.0f); P_F32(velocity0[1]); }
    vec3_mul_f(friction, inv_mass0, obj0->rb.velocity);
    vec3_sub(obj0->rb.velocity, velocity0, obj0->rb.velocity);
  }
  if (obj1_has_rb)
  {
    vec3_mul_f(friction, inv_mass1, obj1->rb.velocity);
    vec3_add(obj1->rb.velocity, velocity1, obj1->rb.velocity);
  }

  // if (info.grounded) { obj0->rb.velocity[1] = MAX(obj0->rb.velocity[1], 0.0f); P_F32(obj0->rb.velocity[1]); }

}

*/

// @NOTE: even older resolution system, based on me poking around
#if 0
void phys_collision_response_resolve_position(phys_obj_t* obj0, phys_obj_t* obj1, collision_info_t info)
{
	// magnitude of vec3, didnt find a glm function
	// f32 dist = sqrtf((info.direction[0] * info.direction[0]) + 
	// 				 (info.direction[1] * info.direction[1]) + 
	// 				 (info.direction[2] * info.direction[2]));

	// const f32 scalar = 0.005f; // @TODO: should not have to do this
	if (PHYS_OBJ_HAS_RIGIDBODY(obj0) && PHYS_OBJ_HAS_RIGIDBODY(obj1))
	{
		// move back according to mass
		f32 s1_ratio = 0;
		f32 s2_ratio = 0;

		if (obj0->rb.mass > obj1->rb.mass)
		{
			f32 ratio = obj1->rb.mass / obj0->rb.mass;
			s1_ratio = ratio;
			s2_ratio = 1 - ratio;
		}
		else if (obj0->rb.mass < obj1->rb.mass)
		{
			// f32 ratio = obj1->rb.mass / obj1->rb.mass;
			// s1_ratio = 1 - ratio;
			// s2_ratio = ratio;
      // @NOTE: above is always 0 & 1
      s1_ratio = 0;
      s2_ratio = 1;
		}
		else if (obj0->rb.mass == obj1->rb.mass)
		{
			s1_ratio = 0.5f;
			s2_ratio = 0.5f;
		}
		// f32 s1_dist = dist * s1_ratio;
		// vec3 offset;
		// vec3 s1_dist_vec = { s1_dist, s1_dist, s1_dist };
		// vec3_mul(info.normal, s1_dist_vec, offset);

    // tmp
	  vec3 obj0_pre_pos;
    vec3_copy(obj0->pos, obj0_pre_pos);

    vec3_add(info.direction, obj0->pos, obj0->pos);

		// vec3 norm_inv;
		// vec3_copy(info.direction, norm_inv);
		// vec3_negate(norm_inv);
		// vec3_mul_f(norm_inv, scalar);
		// vec3_add(norm_inv, e1->pos, e1->pos);

		// f32 s2_dist = dist * s2_ratio;
		// vec3 s2_dist_vec = { s2_dist, s2_dist, s2_dist };
		// vec3_mul(info.normal, s2_dist_vec, offset);

    // tmp
    vec3 obj1_pre_pos;
    vec3_copy(obj1->pos, obj1_pre_pos);

		vec3 norm_inv;
		vec3_copy(info.direction, norm_inv);
		vec3_negate(norm_inv, norm_inv);
		vec3_add(norm_inv, obj1->pos, obj1->pos);

		// vec3_add(info.direction, e2->pos, e2->pos);


	}
	else if (!PHYS_OBJ_HAS_RIGIDBODY(obj0) && PHYS_OBJ_HAS_RIGIDBODY(obj1))
	{
		// only move back the rb
		// vec3 offset;
		// vec3 norm_inv;
		// vec3_copy(info.normal, norm_inv);
		// vec3_negate(norm_inv);
		// vec3 dist_vec = { dist, dist, dist };
		// vec3_mul(info.normal, dist_vec, offset);
		// vec3_add(offset, e2->pos, e2->pos);

		vec3_add(info.direction, obj1->pos, obj1->pos);

		vec3 norm_inv;
		vec3_copy(info.direction, norm_inv);
		vec3_negate(norm_inv, norm_inv);
		vec3_add(norm_inv, obj1->pos, obj1->pos);
	}
	else if (PHYS_OBJ_HAS_RIGIDBODY(obj0) && !PHYS_OBJ_HAS_RIGIDBODY(obj1))
	{
		// only move back the rb
		// vec3 offset;
		// vec3 dist_vec = { dist, dist, dist };
		// vec3_mul(info.normal, dist_vec, offset);
		// vec3_add(offset, e1->pos, e1->pos);

		// vec3 norm_inv;
		// vec3_copy(info.direction, norm_inv);
		// vec3_negate(norm_inv);
		// vec3_mul_f(norm_inv, scalar);
		// vec3_add(norm_inv, e1->pos, e1->pos);

		vec3_add(info.direction, obj0->pos, obj0->pos);
	}
}

void phys_collision_response_resolve_velocity(phys_obj_t* obj0, phys_obj_t* obj1, collision_info_t info)
{
	// calc the force each collider enacted on the other

	// fill the vars according to rb or static
	f32  mass1 = 0;
	f32  mass2 = 0;
	vec3 velocity1;
	vec3 velocity2;
	if (PHYS_OBJ_HAS_RIGIDBODY(obj0))
	{
		mass1 = obj0->rb.mass;
		vec3_copy(obj0->rb.velocity, velocity1);
	}
	else // static
	{
		mass1 = obj0->rb.mass;
		vec3_copy(VEC3(0), velocity1);
	}
	if (PHYS_OBJ_HAS_RIGIDBODY(obj1))
	{
		mass2 = obj1->rb.mass;
		vec3_copy(obj1->rb.velocity, velocity2);
	}
	else // static
	{
		mass2 = mass1;
		vec3_copy(VEC3(0), velocity2);
	}
	// printf("mass1: %.2f, mass2: %.2f\n", mass1, mass2);
	// printf("v1: x: %.2f, y: %.2f, z: %.2f\n", velocity1[0], velocity1[1], velocity1[2]);
	// printf("v2: x: %.2f, y: %.2f, z: %.2f\n", velocity2[0], velocity2[1], velocity2[2]);


	// see: elastic collisions, one-dimensional newtonian

	// velocity1 formula
	// v1,2 = current velocity, m1,2 = mass
	// ((m1 - m2) / (m1 + m2)) * v1
	// +
	// ((2 * m2) / (m1 + m2)) * v2
	f32 m1_m_m2 = mass1 - mass2;
	f32 m1_p_m2 = mass1 + mass2;
	f32 a1_f = (m1_m_m2 / m1_p_m2);
	vec3 a1 = { a1_f, a1_f, a1_f };
	vec3_mul(velocity1, a1, a1);
	f32 b1_f = (2 * mass2) / m1_p_m2;
	vec3 b1 = { b1_f, b1_f, b1_f };
	vec3_mul(velocity2, b1, b1);

	vec3 v1;
	vec3_add(a1, b1, v1);

	// velocity2 formula
	// ((2 * m1) / (m1 + m2)) * v1
	// +
	// ((m2 - m1) / (m1 + m2)) * v2
	f32 a2_f = (2 * mass1) / m1_p_m2;
	vec3 a2 = { a2_f, a2_f, a2_f };
  vec3_mul(velocity1, a2, a2);
	f32 b2_f = (mass2 - mass1) / m1_p_m2;
	vec3 b2 = { b2_f, b2_f, b2_f };
	vec3_mul(velocity2, b2, b2);

	vec3 v2;
	vec3_add(a2, b2, v2);

	// set velocity
	if (PHYS_OBJ_HAS_RIGIDBODY(obj0) && !PHYS_OBJ_HAS_RIGIDBODY(obj1))
	{
		// vec3_copy(v1, e1->rb.velocity);

	  vec3_copy(info.direction, v1);
		vec3_mul_f(v1, 10, v1);
		vec3_copy(v1, obj0->rb.velocity);
		// printf("s1 v: x: %.2f, y: %.2f, z: %.2f\n", e1->rb.velocity[0], e1->rb.velocity[1], e1->rb.velocity[2]);
	}
  else if (!PHYS_OBJ_HAS_RIGIDBODY(obj0) && PHYS_OBJ_HAS_RIGIDBODY(obj1))
	{
		// vec3_copy(v2, e2->rb.velocity);

		vec3_copy(info.direction, v2);
		vec3_mul_f(v2, 10, v2);
		vec3_copy(v2, obj1->rb.velocity);
		// printf("s2 v: x: %.2f, y: %.2f, z: %.2f\n", e2->rb.velocity[0], e2->rb.velocity[1], e2->rb.velocity[2]);
	}
  else if (PHYS_OBJ_HAS_RIGIDBODY(obj0) && PHYS_OBJ_HAS_RIGIDBODY(obj1))
	{
		// vec3_add(v1, e1->rb.velocity, e1->rb.velocity);
		// vec3_add(v2, e2->rb.velocity, e2->rb.velocity);


		vec3_copy(info.direction, v1);

		vec3 dir;
		vec3_copy(info.direction, dir);
		vec3_normalize(dir, dir);

		vec3_mul(v1, dir, v1);
		// vec3_mul_f(v1, 10);
		vec3_copy(v1, obj0->rb.velocity);


		vec3_copy(info.direction, v2);

		vec3_negate(dir, dir);
		vec3_mul(v2, dir, v2);
		// vec3_mul_f(v2, 10);
		// vec3_negate(v2); // ???
		vec3_copy(v2, obj1->rb.velocity);

		// printf("rb1: \"%s\", v: x: %.2f, y: %.2f, z: %.2f\n", e1->name, e1->rb.velocity[0], e1->rb.velocity[1], e1->rb.velocity[2]);
		// printf("rb2: \"%s\", v: x: %.2f, y: %.2f, z: %.2f\n", e2->name, e2->rb.velocity[0], e2->rb.velocity[1], e2->rb.velocity[2]);
	}
}


#endif
