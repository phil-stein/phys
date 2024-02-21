#include "phys_ray.h"
#include "core/core_data.h"
#include "phys/phys_types.h"
#include "phys_world.h"
#include "phys_collision.h"
#include "phys_debug_draw.h"

#include "stb/stb_ds.h"


// @TODO: @OPTIMIZE: optimize this, chunks
bool phys_ray_cast_dbg(ray_t* ray, ray_hit_t* out, const char* _file, const char* _func, const int _line)
{
  u32 len = 0;
  phys_obj_t* arr = phys_get_obj_arr(&len);


  ray_hit_t* hit_arr = NULL;
  u32        hit_arr_len = 0;

  for (int i = 0; i < len; ++i)
  {
    phys_obj_t* obj = &arr[i];
   
    if (ray->mask_arr && ray->mask_arr_len > 0)
    {
      bool skip = false;
      for (int m = 0; m < ray->mask_arr_len; ++m)
      { if (obj->entity_idx == ray->mask_arr[m]) { skip = true; break; } }
      if (skip) { continue; }
    }
    
    if (!PHYS_OBJ_HAS_COLLIDER(obj) || obj->collider.is_trigger) { continue; }

    // f32  dist = 0;
    // vec3 hit_point;
    ray_hit_t hit;
    switch (obj->collider.type)
    {
      case PHYS_COLLIDER_SPHERE:
        if ( phys_collision_check_ray_v_sphere_obj(ray, obj, &hit) ) // &dist, hit_point) )
        {
          hit.entity_idx = obj->entity_idx,
          arrput(hit_arr, hit);
          hit_arr_len++;
        }
        break;
      
      case PHYS_COLLIDER_BOX:
        if ( phys_collision_check_ray_v_aabb_obj(ray, obj, &hit) ) // &dist, hit_point) )
        {
          hit.entity_idx = obj->entity_idx,
          arrput(hit_arr, hit);
          hit_arr_len++;
        }
        break;
    }
  }

  // no hits
  if (hit_arr_len <= 0) { goto no_hit_exit; }


  // get closest hit
  int idx = 0;
  for (int i = 0; i < hit_arr_len; ++i)
  {
    if (hit_arr[i].dist < hit_arr[idx].dist) { idx = i; }
  }

  // set out to hit
  *out = hit_arr[idx];
  // if len <= 0 ignore len
  out->hit = ray->len <= 0.0f || out->dist <= ray->len;
  if (!out->hit) { goto no_hit_exit; }

  // debug lines on hit
  if (ray->draw_debug)
  {
    debug_draw_line_register_t(ray->pos, hit_arr[idx].hit_point, RGB_F(0, 1, 1), 1.0f);
    debug_draw_sphere_register_t(hit_arr[idx].hit_point, 0.1f, RGB_F(0, 1, 0), 1.0f);
    vec3 norm;
    vec3_copy(hit_arr[idx].hit_point, norm);
    vec3_add(norm, hit_arr[idx].normal, norm);
    debug_draw_line_register_t(hit_arr[idx].hit_point, norm, RGB_F(0, 1, 0), 1.0f);
  }
  
  ARRFREE(hit_arr);    

  return out->hit;

no_hit_exit:;
  if (ray->draw_debug)
  {
    vec3 ray_end;
    if (ray->len <= 0.0f)
    { vec3_mul_f(ray->dir, 25, ray_end); }
    else
    { vec3_mul_f(ray->dir, ray->len, ray_end); }
    vec3_add(ray_end, ray->pos, ray_end);
    debug_draw_line_register(ray->pos, ray_end, RGB_F(1, 0, 0));
  }
  return false;
}

// // @TODO: @OPTIMIZE: optimize this, chunks
// bool phys_ray_cast_mask_dbg(ray_t* ray, ray_hit_t* out, u32* mask_arr, int mask_arr_len, const char* _file, const char* _func, const int _line)
// {
//   u32 len = 0;
//   phys_obj_t* arr = phys_get_obj_arr(&len);
// 
// 
//   ray_hit_t* hit_arr = NULL;
//   u32        hit_arr_len = 0;
// 
//   for (int i = 0; i < len; ++i)
//   {
//     phys_obj_t* obj = &arr[i];
// 
//     bool skip = false;
//     for (int m = 0; m < mask_arr_len; ++m)
//     { if (obj->entity_idx == mask_arr[m]) { skip = true; break; } }
//     if (skip) { continue; }
// 
//     if (!PHYS_OBJ_HAS_COLLIDER(obj) || obj->collider.is_trigger) { continue; }
// 
//     // f32  dist = 0;
//     // vec3 hit_point;
//     ray_hit_t hit;
//     switch (obj->collider.type)
//     {
//       case PHYS_COLLIDER_SPHERE:
//         if ( phys_collision_check_ray_v_sphere_obj(ray, obj, &hit) ) // &dist, hit_point) )
//         {
//           hit.entity_idx = obj->entity_idx,
//           arrput(hit_arr, hit);
//           hit_arr_len++;
//         }
//         break;
//       
//       case PHYS_COLLIDER_BOX:
//         if ( phys_collision_check_ray_v_aabb_obj(ray, obj, &hit) ) // &dist, hit_point) )
//         {
//           hit.entity_idx = obj->entity_idx,
//           arrput(hit_arr, hit);
//           hit_arr_len++;
//         }
//         break;
//     }
//   }
// 
//   // no hits
//   if (hit_arr_len <= 0) { goto no_hit_exit; }
// 
//   // get closest hit
//   int idx = 0;
//   for (int i = 0; i < hit_arr_len; ++i)
//   {
//     if (hit_arr[i].dist < hit_arr[idx].dist) { idx = i; }
//   }
// 
//   // set out to hit
//   *out = hit_arr[idx];
//   // if len <= 0 ignore len
//   out->hit = ray->len <= 0.0f || out->dist <= ray->len;
//   if (!out->hit) { goto no_hit_exit; }
//  
//   // debug lines on hit
//   debug_draw_line_register_t(ray->pos, hit_arr[idx].hit_point, RGB_F(0, 1, 1), 1.0f);
//   debug_draw_sphere_register_t(hit_arr[idx].hit_point, 0.1f, RGB_F(0, 1, 0), 1.0f);
//   vec3 norm;
//   vec3_copy(hit_arr[idx].hit_point, norm);
//   vec3_add(norm, hit_arr[idx].normal, norm);
//   debug_draw_line_register_t(hit_arr[idx].hit_point, norm, RGB_F(0, 1, 0), 1.0f);
//   
//   ARRFREE(hit_arr);    
// 
//   return out->hit;
//   
// no_hit_exit:;
//   vec3 ray_end;
//   if (ray->len <= 0.0f)
//   { vec3_mul_f(ray->dir, 25, ray_end); }
//   else
//   { vec3_mul_f(ray->dir, ray->len, ray_end); }
//   vec3_add(ray_end, ray->pos, ray_end);
//   debug_draw_line_register(ray->pos, ray_end, RGB_F(1, 0, 0));
//   return false; 
// }
