#ifndef PHYS_PHYS_COLLISION_H
#define PHYS_PHYS_COLLISION_H

#include "phys/phys_types.h" 

#ifdef __cplusplus
extern "C" {
#endif

// @DOC: check collision between two entities, regardless of collider type
//       e1: first phys obj with  collider
//       e2: second phys obj with collider
collision_info_t phys_collision_check(phys_obj_t* e1, phys_obj_t* e2);

// @DOC: check collision between to phys_obj_t with sphere colliders
//       s1: first phys_obj with sphere collider
//       s2: second phys_obj with sphere collider
collision_info_t phys_collision_check_sphere_v_sphere(phys_obj_t* s1, phys_obj_t* s2);
collision_info_t phys_collision_check_sphere_v_sphere_swept(phys_obj_t* s0, phys_obj_t* s1);

// taken from "https://developer.mozilla.org/en-US/docs/Games/Techniques/3D_collision_detection"
// @DOC: check collision between to phys_obj_t with box/aabb colliders
//       s1: first phys_obj with box collider
//       s2: second phys_obj with box collider
collision_info_t phys_collision_check_aabb_v_aabb(phys_obj_t* b1, phys_obj_t* b2);
collision_info_t phys_collision_check_aabb_v_aabb_swept(phys_obj_t* b0, phys_obj_t* b1);

// @DOC: check collision between to phys_obj_t on box/aabb, one sphere collider
//       s1: first phys_obj with box collider
//       s2: second phys_obj with sphere collider
//       switch_obj_places: if true treat s as the active obj, inverses info.direction
collision_info_t phys_collision_check_aabb_v_sphere(phys_obj_t* b, phys_obj_t* s, bool switch_obj_places);


// --- inline funcs ---


// taken from: https://gamedev.stackexchange.com/questions/96459/fast-ray-sphere-collision-code
// also used:  '3D Math Primer for Graphics and Game Development' by Dunn Parberry, page 727
// Intersects ray r = p + td, |d| = 1, with sphere s and, if intersecting, 
// returns t value of intersection and intersection point q 
// int IntersectRaySphere(Point p, Vector d, Sphere s, float &t, Point &q)
// @DOC: get hit between ray and sphere
//       returns true if hit
//       puts hit point in hit_out
//       puts dist between ray->pos and hit_out in dist
// INLINE bool phys_collision_check_ray_v_sphere(ray_t* ray, phys_obj_t* sphere, f32* dist, vec3 hit_out) 
// {
//   vec3 sphere_pos = VEC3_INIT(0);
//   vec3_add(sphere->pos, sphere->collider.offset, sphere_pos);
//   f32 radius = sphere->collider.sphere.radius * ( (sphere->scl[0] + sphere->scl[1] + sphere->scl[2]) * 0.33f);
//   
//   vec3 m;
//   vec3_sub(ray->pos, sphere_pos, m); 
//   f32  b = vec3_dot(m, ray->dir); 
//   f32  c = vec3_dot(m, m) - ( radius * radius ); 
//   
//   // Exit if r’s origin outside s (c > 0) and r pointing away from s (b > 0) 
//   if (c > 0.0f && b > 0.0f) { return false; }
//   f32 discriminant = (b * b) - c; 
//   
//   // A negative discriminant corresponds to ray missing sphere 
//   if (discriminant < 0.0f) { return false; }
//   
//   // Ray now found to intersect sphere, compute smallest dist value of intersection
//   *dist = (-b) - (f32)(sqrt((f64)discriminant)); 
//   
//   // If t is negative, ray started inside sphere so clamp dist to zero 
//   if ((*dist) < 0.0f) {  *dist = 0.0f; }
// 
//   // hit_out = p + (*t) * d; 
//   vec3_mul_f(ray->dir, (*dist), hit_out);
//   vec3_add(hit_out, ray->pos, hit_out);
//   
//   return true;
// }
INLINE bool phys_collision_check_ray_v_sphere(ray_t* ray, vec3 sphere_pos, f32 radius, f32* dist, vec3 hit_out) 
{ 
  vec3 m;
  vec3_sub(ray->pos, sphere_pos, m); 
  f32  b = vec3_dot(m, ray->dir); 
  f32  c = vec3_dot(m, m) - ( radius * radius ); 
  
  // Exit if r’s origin outside s (c > 0) and r pointing away from s (b > 0) 
  if (c > 0.0f && b > 0.0f) { return false; }
  f32 discriminant = (b * b) - c; 
  
  // A negative discriminant corresponds to ray missing sphere 
  if (discriminant < 0.0f) { return false; }
  
  // Ray now found to intersect sphere, compute smallest dist value of intersection
  *dist = (-b) - (f32)(sqrt((f64)discriminant)); 
  
  // If t is negative, ray started inside sphere so clamp dist to zero 
  if ((*dist) < 0.0f) {  *dist = 0.0f; }

  // hit_out = p + (*t) * d; 
  vec3_mul_f(ray->dir, (*dist), hit_out);
  vec3_add(hit_out, ray->pos, hit_out);
  
  return true;
}
INLINE bool phys_collision_check_ray_v_sphere_obj(ray_t* ray, phys_obj_t* sphere, f32* dist, vec3 hit_out)
{
  if (!PHYS_OBJ_HAS_COLLIDER(sphere) || sphere->collider.type != PHYS_COLLIDER_SPHERE) { return false; }
  vec3 sphere_pos = VEC3_INIT(0);
  vec3_add(sphere->pos, sphere->collider.offset, sphere_pos);
  f32 radius = sphere->collider.sphere.radius * ( (sphere->scl[0] + sphere->scl[1] + sphere->scl[2]) * 0.33f);
  return phys_collision_check_ray_v_sphere(ray, sphere_pos, radius, dist, hit_out); 
}
 


// taken from: https://gdbooks.gitbooks.io/3dcollisions/content/Chapter3/raycast_aabb.html
// @DOC: get hit between ray and aabb
//       returns true if hit
//       puts hit point in hit_out
//       puts dist between ray->pos and hit_out in dist
INLINE bool phys_collision_check_ray_v_aabb(ray_t* ray, vec3 min, vec3 max, f32* dist, vec3 hit_out) 
{
  f32 t1 = (min[0] - ray->pos[0]) / ray->dir[0];
  f32 t2 = (max[0] - ray->pos[0]) / ray->dir[0];
  f32 t3 = (min[1] - ray->pos[1]) / ray->dir[1];
  f32 t4 = (max[1] - ray->pos[1]) / ray->dir[1];
  f32 t5 = (min[2] - ray->pos[2]) / ray->dir[2];
  f32 t6 = (max[2] - ray->pos[2]) / ray->dir[2];

  // ray intersects aabb twice, these are the distances to these points
  f32 tmin = MAX(MAX(MIN(t1, t2), MIN(t3, t4)), MIN(t5, t6));
  f32 tmax = MIN(MIN(MAX(t1, t2), MAX(t3, t4)), MAX(t5, t6));

  // if tmax < 0, ray (line) is intersecting AABB, but whole AABB is behing us
  if (tmax < 0.0f) { return false; }

  // if tmin > tmax, ray doesn't intersect AABB
  if (tmin > tmax) { return false; }

  // if tmin <  0 dist is tmax
  if (tmin < 0.0f) { *dist = tmax; }
  else           { *dist = tmin; }
  
  // set hit_out
  vec3_mul_f(ray->dir, *dist, hit_out);
  vec3_add(hit_out, ray->pos, hit_out);

  return true;
}
INLINE bool phys_collision_check_ray_v_aabb_obj(ray_t* ray, phys_obj_t* box, f32* dist, vec3 hit_out) 
{
  if (!PHYS_OBJ_HAS_COLLIDER(box) || box->collider.type != PHYS_COLLIDER_BOX) { return false; }
	
    // add position & offset to min & max of both colliders
	vec3 min, max;
	vec3_copy(box->collider.box.aabb[0], min);
	vec3_copy(box->collider.box.aabb[1], max);
  vec3_mul(min, box->scl, min);
  vec3_mul(max, box->scl, max);
	vec3_add(min, box->pos, min);
	vec3_add(max, box->pos, max);
	vec3_add(min, box->collider.offset, min);
	vec3_add(max, box->collider.offset, max);
  
  return phys_collision_check_ray_v_aabb(ray, min, max, dist, hit_out);
}


#ifdef __cplusplus
} // extern c
#endif

#endif
