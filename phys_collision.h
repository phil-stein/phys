#ifndef PHYS_PHYS_COLLISION_H
#define PHYS_PHYS_COLLISION_H

#include "core/debug/debug_draw.h"
#include "global/global.h"
#include "math/math_inc.h"
#include "math/math_vec3.h"
#include "phys/phys_types.h" 
#include "phys/phys_debug_draw.h" 
#include <float.h>

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
collision_info_t phys_collision_check_aabb_v_sphere_swept(phys_obj_t* b, phys_obj_t* s, bool switch_obj_places);


// --- inline funcs ---

// @TODO: make phys_util.h and put there
INLINE void phys_util_obj_get_aabb(phys_obj_t* box, vec3 min, vec3 max)
{
  // add position & offset to min & max of both colliders
	vec3_copy(box->collider.box.aabb[0], min);
	vec3_copy(box->collider.box.aabb[1], max);
  vec3_mul(min, box->scl, min);
  vec3_mul(max, box->scl, max);
	vec3_add(min, box->pos, min);
	vec3_add(max, box->pos, max);
	vec3_add(min, box->collider.offset, min);
	vec3_add(max, box->collider.offset, max);
}
INLINE void phys_util_closest_point_aabb(vec3 min, vec3 max, vec3 p, vec3 out)
{
  out[0] = p[0] > max[0] ? max[0] : (p[0] < min[0] ? min[0] : p[0]);
  out[1] = p[1] > max[1] ? max[1] : (p[1] < min[1] ? min[1] : p[1]);
  out[2] = p[2] > max[2] ? max[2] : (p[2] < min[2] ? min[2] : p[2]);
}
INLINE void phys_util_closest_point_aabb_obj(phys_obj_t* obj, vec3 p, vec3 out)
{
  vec3 min, max;
  phys_util_obj_get_aabb(obj, min, max);
  phys_util_closest_point_aabb(min, max, p, out);
}

// taken from: https://gamedev.stackexchange.com/questions/96459/fast-ray-sphere-collision-code
// also used:  '3D Math Primer for Graphics and Game Development' by Dunn Parberry, page 727
// Intersects ray r = p + td, |d| = 1, with sphere s and, if intersecting, 
// returns t value of intersection and intersection point q 
// int IntersectRaySphere(Point p, Vector d, Sphere s, float &t, Point &q)
// @DOC: get hit between ray and sphere
//       returns true if hit
//       puts hit point in hit_out
//       puts dist between ray->pos and hit_out in dist
// INLINE bool phys_collision_check_ray_v_sphere(ray_t* ray, vec3 sphere_pos, f32 radius, f32* dist, vec3 hit_out) 
INLINE bool phys_collision_check_ray_v_sphere(ray_t* ray, vec3 sphere_pos, f32 radius, ray_hit_t* hit) 
{
  hit->hit = false; 
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
  hit->dist = (-b) - (f32)(sqrt((f64)discriminant)); 
  
  // If t is negative, ray started inside sphere so clamp dist to zero 
  if ((hit->dist) < 0.0f) {  hit->dist = 0.0f; }

  // hit_point = p + (*t) * d; 
  vec3_mul_f(ray->dir, (hit->dist), hit->hit_point);
  vec3_add(hit->hit_point, ray->pos, hit->hit_point);
  
  vec3_sub(hit->hit_point, sphere_pos, hit->normal);
  vec3_normalize(hit->normal, hit->normal);
  
  hit->hit = true; 
  return true;
}
// INLINE bool phys_collision_check_ray_v_sphere_obj(ray_t* ray, phys_obj_t* sphere, f32* dist, vec3 hit_out)
INLINE bool phys_collision_check_ray_v_sphere_obj(ray_t* ray, phys_obj_t* sphere, ray_hit_t* hit)
{
  if (!PHYS_OBJ_HAS_COLLIDER(sphere) || sphere->collider.type != PHYS_COLLIDER_SPHERE) { return false; }
  vec3 sphere_pos = VEC3_INIT(0);
  vec3_add(sphere->pos, sphere->collider.offset, sphere_pos);
  f32 radius = sphere->collider.sphere.radius * ( (sphere->scl[0] + sphere->scl[1] + sphere->scl[2]) * 0.33f);
  return phys_collision_check_ray_v_sphere(ray, sphere_pos, radius, hit); 
}
 


// taken from: https://gdbooks.gitbooks.io/3dcollisions/content/Chapter3/raycast_aabb.html
// @DOC: get hit between ray and aabb
//       returns true if hit
//       puts hit point in hit_out
//       puts dist between ray->pos and hit_out in dist
// INLINE bool phys_collision_check_ray_v_aabb(ray_t* ray, vec3 min, vec3 max, f32* dist, vec3 hit_out) 
INLINE bool phys_collision_check_ray_v_aabb(ray_t* ray, vec3 min, vec3 max, ray_hit_t* hit) 
{
  hit->hit = false; 
  
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
  if (tmin < 0.0f) { hit->dist = tmax; }
  else             { hit->dist = tmin; }
  
  // set ray_hit_t 
  vec3_mul_f(ray->dir, hit->dist, hit->hit_point);
  vec3_add(hit->hit_point, ray->pos, hit->hit_point);

  // @TODO:
  // vec3 pos = { min[0] + max[0], min[1] + max[1], min[2] + max[2] };
  // vec3_sub(hit->hit_point, pos, hit->normal);
  // vec3_normalize(hit->normal, hit->normal);

  hit->hit = true; 
  return true;
}
// INLINE bool phys_collision_check_ray_v_aabb_obj(ray_t* ray, phys_obj_t* box, f32* dist, vec3 hit_out) 
INLINE bool phys_collision_check_ray_v_aabb_obj(ray_t* ray, phys_obj_t* box, ray_hit_t* hit) 
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
 
  bool rtn = phys_collision_check_ray_v_aabb(ray, min, max, hit);
  vec3_sub(hit->hit_point, box->pos, hit->normal);
  vec3_normalize(hit->normal, hit->normal);
  return rtn;
}

// taken from: https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
INLINE bool phys_collision_ray_v_triangle(ray_t* ray, vec3 p0, vec3 p1, vec3 p2, ray_hit_t* hit)
{
  hit->hit = false;
  // constexpr float epsilon std::numeric_limits<float>::epsilon();
  // vec3 edge1 = triangle.b - triangle.a;
  // vec3 edge2 = triangle.c - triangle.a;
  // vec3 ray_cross_e2 = cross(ray_vector, edge2);
  // float det = dot(edge1, ray_cross_e2);
  vec3 edge01 = VEC3_INIT_SUB(p1, p0);
  vec3 edge02 = VEC3_INIT_SUB(p2, p0);
  vec3 ray_cross_e2 = VEC3_INIT_CROSS(ray->dir, edge02);
  f32 det = vec3_dot(edge01, ray_cross_e2);

  // if (det > -epsilon && det < epsilon)
  //     return false;    // This ray is parallel to this triangle.
  if (det > -FLT_EPSILON&& det < FLT_EPSILON) { return false; }

  float inv_det = 1.0f / det;
  // vec3 s = ray_origin - triangle.a;
  // float u = inv_det * dot(s, ray_cross_e2);
  vec3 s = VEC3_INIT_SUB(ray->pos, p0);
  f32 u  = inv_det * vec3_dot(s, ray_cross_e2);

  if (u < 0 || u > 1) { return false; }

  // vec3 s_cross_e1 = cross(s, edge1);
  // float v = inv_det * dot(ray_vector, s_cross_e1);
  vec3 s_cross_e1 = VEC3_INIT_CROSS(s, edge01);
  f32 v = inv_det * vec3_dot(ray->dir, s_cross_e1);

  if (v < 0 || u + v > 1) { return false; }

  // At this stage we can compute t to find out where the intersection point is on the line.
  // float t = inv_det * dot(edge2, s_cross_e1);
  f32 t = inv_det * vec3_dot(edge02, s_cross_e1);

  // if (t > epsilon) // ray intersection
  // {
  //     out_intersection_point = ray_origin + ray_vector * t;
  //     return true;
  // }
  // else // This means that there is a line intersection but not a ray intersection.
  //     return false;
  if (t > FLT_EPSILON) // ray intersection
  {
    // out_intersection_point = ray_origin + ray_vector * t;
    hit->hit = true;
    hit->dist = t;
    vec3_mul_f(ray->dir, t, hit->hit_point);
    vec3_add(ray->pos, hit->hit_point, hit->hit_point);
    return true;
  }
  // This means that there is a line intersection but not a ray intersection.
  return false;
}

// SAT test used in phys_collision_check_aabb_v_triangle()
INLINE bool phys_collision_check_aabb_v_triangle_sat_test(vec3 axis, vec3 e, vec3 v0, vec3 v1, vec3 v2, vec3 u0, vec3 u1, vec3 u2)
{
  // Project all 3 vertices of the triangle onto the Seperating axis
  // float p0 = Vector3.Dot(v0, axis_u0_f0);
  // float p1 = Vector3.Dot(v1, axis_u0_f0);
  // float p2 = Vector3.Dot(v2, axis_u0_f0);
  float p0 = vec3_dot(v0, axis);
  float p1 = vec3_dot(v1, axis);
  float p2 = vec3_dot(v2, axis);
  // Project the AABB onto the seperating axis
  // We don't care about the end points of the prjection
  // just the length of the half-size of the AABB
  // That is, we're only casting the extents onto the 
  // seperating axis, not the AABB center. We don't
  // need to cast the center, because we know that the
  // aabb is at origin compared to the triangle!
  // float r = e.X * Math.Abs(Vector3.Dot(u0, axis_u0_f0)) +
  //             e.Y * Math.Abs(Vector3.Dot(u1, axis_u0_f0)) +
  //             e.Z * Math.Abs(Vector3.Dot(u2, axis_u0_f0));
  float r = e[0] * fabsf(vec3_dot(u0, axis)) +
            e[1] * fabsf(vec3_dot(u1, axis)) +
            e[2] * fabsf(vec3_dot(u2, axis));
  // Now do the actual test, basically see if either of
  // the most extreme of the triangle points intersects r
  // You might need to write Min & Max functions that take 3 arguments
  // if (Max(-Max(p0, p1, p2), Min(p0, p1, p2)) > r) {
  return (MAX(-MAX3(p0, p1, p2), MIN3(p0, p1, p2)) > r);
  // if this is false// if this is false::
  // This means BOTH of the points of the projected triangle
  // are outside the projected half-length of the AABB
  // Therefore the axis is seperating and we can exit

}
// taken from: https://gdbooks.gitbooks.io/3dcollisions/content/Chapter4/aabb-triangle.html
// c: position of aabb, aka. center
// e: extends of aabb, aka. max
// v0, v1, v2: points of triangle
INLINE bool phys_collision_check_aabb_v_triangle(vec3 c, vec3 max, vec3 p0, vec3 p1, vec3 p2)
{
  // // Get the triangle points as vectors
  // Vector3 v0 = triangle.p0.ToVector();
  // Vector3 v1 = triangle.p1.ToVector();
  // Vector3 v2 = triangle.p2.ToVector();

  // Convert AABB to center-extents form
  // Vector3 c = aabb.Center.ToVector();
  // Vector3 e = aabb.Extents;
  // vec3 c, e;
  // vec3_copy(pos, c);
  // vec3_copy(extends, e);

  // Translate the triangle as conceptually moving the AABB to origin
  // This is the same as we did with the point in triangle test
  // v0 -= c;
  // v1 -= c;
  // v2 -= c;
  vec3 v0, v1, v2;
  vec3_sub(p0, c, v0);
  vec3_sub(p1, c, v1);
  vec3_sub(p2, c, v2);

  // Compute the edge vectors of the triangle  (ABC)
  // That is, get the lines between the points as vectors
  // Vector3 f0 = v1 - v0; // B - A
  // Vector3 f1 = v2 - v1; // C - B
  // Vector3 f2 = v0 - v2; // A - C
  vec3 f0, f1, f2;
  vec3_sub(v1, v0, f0);
  vec3_sub(v2, v1, f1);
  vec3_sub(v0, v2, f2);

  // Compute the face normals of the AABB, because the AABB
  // is at center, and of course axis aligned, we know that 
  // it's normals are the X, Y and Z axis.
  // Vector3 u0 = new Vector3(1.0f, 0.0f, 0.0f);
  // Vector3 u1 = new Vector3(0.0f, 1.0f, 0.0f);
  // Vector3 u2 = new Vector3(0.0f, 0.0f, 1.0f);
  vec3 u0 = { 1.0f, 0.0f, 0.0f };
  vec3 u1 = { 0.0f, 1.0f, 0.0f };
  vec3 u2 = { 0.0f, 0.0f, 1.0f };

  // There are a total of 13 axis to test!

  // We first test against 9 axis, these axis are given by
  // cross product combinations of the edges of the triangle
  // and the edges of the AABB. You need to get an axis testing
  // each of the 3 sides of the AABB against each of the 3 sides
  // of the triangle. The result is 9 axis of seperation
  // https://awwapp.com/b/umzoc8tiv/

  // Compute the 9 axis
  // Vector3 axis_u0_f0 = Vector3.Cross(u0, f0);
  // Vector3 axis_u0_f1 = Vector3.Cross(u0, f1);
  // Vector3 axis_u0_f2 = Vector3.Cross(u0, f2);
  // Vector3 axis_u1_f0 = Vector3.Cross(u1, f0);
  // Vector3 axis_u1_f1 = Vector3.Cross(u1, f1);
  // Vector3 axis_u1_f2 = Vector3.Cross(u2, f2);
  // Vector3 axis_u2_f0 = Vector3.Cross(u2, f0);
  // Vector3 axis_u2_f1 = Vector3.Cross(u2, f1);
  // Vector3 axis_u2_f2 = Vector3.Cross(u2, f2);

  vec3 axis_u0_f0 = VEC3_INIT_CROSS(u0, f0);
  vec3 axis_u0_f1 = VEC3_INIT_CROSS(u0, f1);
  vec3 axis_u0_f2 = VEC3_INIT_CROSS(u0, f2);

  vec3 axis_u1_f0 = VEC3_INIT_CROSS(u1, f0);
  vec3 axis_u1_f1 = VEC3_INIT_CROSS(u1, f1);
  vec3 axis_u1_f2 = VEC3_INIT_CROSS(u1, f2);

  vec3 axis_u2_f0 = VEC3_INIT_CROSS(u2, f0);
  vec3 axis_u2_f1 = VEC3_INIT_CROSS(u2, f1);
  vec3 axis_u2_f2 = VEC3_INIT_CROSS(u2, f2);

  if (phys_collision_check_aabb_v_triangle_sat_test(axis_u0_f0, max, v0, v1, v2, u0, u1, u2)) { return false; }
  if (phys_collision_check_aabb_v_triangle_sat_test(axis_u0_f1, max, v0, v1, v2, u0, u1, u2)) { return false; }
  if (phys_collision_check_aabb_v_triangle_sat_test(axis_u0_f2, max, v0, v1, v2, u0, u1, u2)) { return false; }
  if (phys_collision_check_aabb_v_triangle_sat_test(axis_u1_f0, max, v0, v1, v2, u0, u1, u2)) { return false; }
  if (phys_collision_check_aabb_v_triangle_sat_test(axis_u1_f1, max, v0, v1, v2, u0, u1, u2)) { return false; }
  if (phys_collision_check_aabb_v_triangle_sat_test(axis_u1_f2, max, v0, v1, v2, u0, u1, u2)) { return false; }
  if (phys_collision_check_aabb_v_triangle_sat_test(axis_u2_f0, max, v0, v1, v2, u0, u1, u2)) { return false; }
  if (phys_collision_check_aabb_v_triangle_sat_test(axis_u2_f1, max, v0, v1, v2, u0, u1, u2)) { return false; }
  if (phys_collision_check_aabb_v_triangle_sat_test(axis_u2_f2, max, v0, v1, v2, u0, u1, u2)) { return false; }

  // Next, we have 3 face normals from the AABB
  // for these tests we are conceptually checking if the bounding box
  // of the triangle intersects the bounding box of the AABB
  // that is to say, the seperating axis for all tests are axis aligned:
  // axis1: (1, 0, 0), axis2: (0, 1, 0), axis3 (0, 0, 1)
  // Do the SAT given the 3 primary axis of the AABB
  // You already have vectors for this: u0, u1 & u2
  if (phys_collision_check_aabb_v_triangle_sat_test(u0, max, v0, v1, v2, u0, u1, u2)) { return false; }
  if (phys_collision_check_aabb_v_triangle_sat_test(u1, max, v0, v1, v2, u0, u1, u2)) { return false; }
  if (phys_collision_check_aabb_v_triangle_sat_test(u2, max, v0, v1, v2, u0, u1, u2)) { return false; }

  // Finally, we have one last axis to test, the face normal of the triangle
  // We can get the normal of the triangle by crossing the first two line segments
  // Vector3 triangleNormal = Vector3.Cross(f0, f1);
  vec3 triangle_normal = VEC3_INIT_CROSS(f0, f1);
  if (phys_collision_check_aabb_v_triangle_sat_test(triangle_normal, max, v0, v1, v2, u0, u1, u2)) { return false; }

  // Passed testing for all 13 seperating axis that exist!
  // debug_draw_sphere_t(c, 1.0f, RGB_F(0, 1, 0), 100.0f);
  return true;
}
INLINE bool phys_collision_check_aabb_v_triangle_obj(phys_obj_t* box, vec3 v0, vec3 v1, vec3 v2)
{
  // add to pos not min/max bc. aabb_v_triangle() uses center/pos and extends
  vec3 pos; 
	vec3_add(box->pos, box->collider.offset, pos);
  // debug_draw_sphere(pos, 0.5f, RGB_F(0, 1, 0));
  vec3 max;
  vec3_copy(box->collider.box.aabb[1], max);
  return phys_collision_check_aabb_v_triangle(pos, max, v0, v1, v2);
}

INLINE void phys_collision_check_aabb_v_terrain_triangle(ray_t* ray, ray_hit_t* hit, ray_hit_t* hit_final, vec3 min, vec3 max, vec3 p0, vec3 p1, vec3 p2)
{
  hit->hit = false;
  hit->dist = -1.0f;
  // min.x, min.z
  ray->pos[0] = min[0]; 
  ray->pos[2] = min[2]; 
  if ( phys_collision_ray_v_triangle(ray, p0, p1, p2, hit) ) 
  {
    // debug_draw_line(ray->pos, hit->hit_point, RGBF(0, 0, 1));
    // debug_draw_sphere(hit->hit_point, 0.25f,  RGBF(0, 0, 1));
    if (hit->dist < hit_final->dist)
    { *hit_final = *hit; }
  }
  // max.x, min.z
  ray->pos[0] = max[0]; 
  if ( phys_collision_ray_v_triangle(ray, p0, p1, p2, hit) ) 
  {
    // debug_draw_line(ray->pos, hit->hit_point, RGBF(0, 1, 0));
    // debug_draw_sphere(hit->hit_point, 0.25f,  RGBF(0, 1, 0));
    if (hit->dist < hit_final->dist)
    { *hit_final = *hit; }
  }
  // max.x, max.z
  ray->pos[2] = max[2]; 
  if ( phys_collision_ray_v_triangle(ray, p0, p1, p2, hit) ) 
  {
    // debug_draw_line(ray->pos, hit->hit_point, RGBF(1, 0, 0));
    // debug_draw_sphere(hit->hit_point, 0.25f,  RGBF(1, 0, 0));
    if (hit->dist < hit_final->dist)
    { *hit_final = *hit; }
  }
  // min.x, max.z
  ray->pos[0] = min[0]; 
  if ( phys_collision_ray_v_triangle(ray, p0, p1, p2, hit) ) 
  {
    // debug_draw_line(ray->pos, hit->hit_point, RGBF(0, 1, 0));
    // debug_draw_sphere(hit->hit_point, 0.25f,  RGBF(0, 1, 0));
    if (hit->dist < hit_final->dist)
    { *hit_final = *hit; }
  }
}
// @TODO: use this instead 00:56:00 / 02:11:00: https://www.youtube.com/watch?v=Ge3aKEmZcqY&t=9606s
//        wikipedia one is much slower
INLINE bool phys_collision_check_aabb_v_terrain(vec3 min, vec3 max, vec3 p0, vec3 p1, vec3 p2, vec3 p3, vec3 p4, vec3 p5, vec3 p6, vec3 p7, vec3 p8, f32* dist)
{
  const f32 y_offs = 100.0f;
  ray_t ray = 
  {
    .dir = { 0, -1, 0 },
    .pos = { min[0], min[1] + y_offs, min[2] },
    .len = 0.0f,
    .mask_arr = NULL,
    .mask_arr_len = 0,
    .draw_debug = false
  };
  ray_hit_t hit       = { .hit = false, .dist = 0.0f,      .hit_point = { 0, 0, 0 } };
  ray_hit_t hit_final = { .hit = false, .dist = 999999.0f, .hit_point = { 0, 0, 0 } };
  phys_collision_check_aabb_v_terrain_triangle(&ray, &hit, &hit_final, min, max, p0, p3, p4);
  phys_collision_check_aabb_v_terrain_triangle(&ray, &hit, &hit_final, min, max, p0, p4, p1);
  phys_collision_check_aabb_v_terrain_triangle(&ray, &hit, &hit_final, min, max, p1, p4, p5);
  phys_collision_check_aabb_v_terrain_triangle(&ray, &hit, &hit_final, min, max, p1, p5, p2);
  phys_collision_check_aabb_v_terrain_triangle(&ray, &hit, &hit_final, min, max, p3, p6, p7);
  phys_collision_check_aabb_v_terrain_triangle(&ray, &hit, &hit_final, min, max, p3, p7, p4);
  phys_collision_check_aabb_v_terrain_triangle(&ray, &hit, &hit_final, min, max, p4, p7, p8);
  phys_collision_check_aabb_v_terrain_triangle(&ray, &hit, &hit_final, min, max, p4, p8, p5);

  // P_VEC3(hit_final.hit_point);
  *dist = hit_final.hit_point[1] - min[1];
  // if (hit_final.hit && *dist > 0.0f)
  // {
  //   debug_draw_line_width(ray.pos, hit_final.hit_point, RGBF(0, 1, 1), 20);
  //   debug_draw_sphere(hit_final.hit_point, 0.3f,  RGBF(0, 1, 1));
  //   min[1] += *dist;
  //   max[1] += *dist;
  //   phys_debug_draw_aabb(min, max, RGBF(0, 1, 1));
  //   return true;
  // }
  // return false;
  return hit_final.hit && *dist > 0.0f;
}
INLINE bool phys_collision_check_aabb_v_terrain_obj(phys_obj_t* obj, vec3 p0, vec3 p1, vec3 p2, vec3 p3, vec3 p4, vec3 p5, vec3 p6, vec3 p7, vec3 p8, f32* dist)
{
  vec3 min, max;
  phys_util_obj_get_aabb(obj, min, max);
  return phys_collision_check_aabb_v_terrain(min, max, p0, p1, p2, p3, p4, p5, p6, p7, p8, dist);
}

// @TODO:
// INLINE void phys_collision_check_aabb_v_point()

#ifdef __cplusplus
} // extern c
#endif

#endif
