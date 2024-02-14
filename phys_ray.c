#include "phys_world.h"
#include "phys_collision.h"
#include "phys_debug_draw.h"

#include "stb/stb_ds.h"


// @TODO: @OPTIMIZE: optimize this, chunks
bool phys_ray_cast(ray_t* ray, ray_hit_t* out)
{
  u32 len = 0;
  phys_obj_t* arr = phys_get_obj_arr(&len);


  ray_hit_t* hit_arr = NULL;
  u32        hit_arr_len = 0;

  for (int i = 0; i < len; ++i)
  {
    phys_obj_t* obj = &arr[i];
    
    if (!PHYS_OBJ_HAS_COLLIDER(obj)) { continue; }

    f32  dist = 0;
    vec3 hit_point;
    switch (obj->collider.type)
    {
      case PHYS_COLLIDER_SPHERE:
        if ( phys_collision_check_ray_v_sphere_obj(ray, obj, &dist, hit_point) )
        {
          ray_hit_t hit = 
          { 
            .hit        = true, 
            .dist       = dist, 
            .hit_point  = { hit_point[0], hit_point[1], hit_point[2] },
            .entity_idx = obj->entity_idx,
          };
          arrput(hit_arr, hit);
          hit_arr_len++;
        }
        break;
      
      case PHYS_COLLIDER_BOX:
        if ( phys_collision_check_ray_v_aabb_obj(ray, obj, &dist, hit_point) )
        {
          ray_hit_t hit = 
          { 
            .hit        = true, 
            .dist       = dist, 
            .hit_point  = { hit_point[0], hit_point[1], hit_point[2] },
            .entity_idx = obj->entity_idx,
          };
          arrput(hit_arr, hit);
          hit_arr_len++;
        }
        break;
    }
  }

  // no hits
  if (hit_arr_len <= 0) 
  { 
    vec3 ray_end;
    vec3_mul_f(ray->dir, 25, ray_end);
    vec3_add(ray_end, ray->pos, ray_end);
    debug_draw_line_register(ray->pos, ray_end, RGB_F(1, 0, 0));

    return false; 
  }

  // get closest hit
  int idx = 0;
  for (int i = 0; i < hit_arr_len; ++i)
  {
    if (hit_arr[i].dist < hit_arr[idx].dist) { idx = i; }
  }

  // set out to hit
  *out = hit_arr[idx];
  
  debug_draw_line_register(ray->pos, hit_arr[idx].hit_point, RGB_F(0, 1, 1));
  debug_draw_sphere_register(hit_arr[idx].hit_point, 0.1f, RGB_F(0, 1, 0));
  
  ARRFREE(hit_arr);
    
  return true;
}
