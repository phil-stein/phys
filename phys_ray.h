#ifndef PHYS_PHYS_RAY_H
#define PHYS_PHYS_RAY_H

#include "global/global.h"
#include "phys_types.h" 

#ifdef __cplusplus
extern "C" {
#endif


// @DOC: check every object against ray
//       returns true if hit 
//       out gets set to hit info
bool phys_ray_cast_dbg(ray_t* ray, ray_hit_t* out, const char* _file, const char* _func, const int _line);
#define phys_ray_cast(ray, out) phys_ray_cast_dbg(ray, out, __FILE__, __func__, __LINE__)

INLINE bool phys_ray_cast_len(ray_t* ray, ray_hit_t* out, f32 max_len)
{
  bool rtn = phys_ray_cast(ray, out);
  return rtn && out->dist <= max_len;
}

bool phys_ray_cast_mask_dbg(ray_t* ray, ray_hit_t* out, u32* mask_arr, int mask_arr_len, const char* _file, const char* _func, const int _line);
#define phys_ray_cast_mask(ray, out, mask_arr, mask_arr_len) phys_ray_cast_mask_dbg(ray, out, mask_arr, mask_arr_len, __FILE__, __func__, __LINE__)

#ifdef __cplusplus
} // extern c
#endif

#endif  // PHYS_PHYS_RAY_H
