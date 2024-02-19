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
//       ! set ray->len to <= 0 for inmfinite length, 
//         or to max length for ray
bool phys_ray_cast_dbg(ray_t* ray, ray_hit_t* out, const char* _file, const char* _func, const int _line);
#define phys_ray_cast(ray, out) phys_ray_cast_dbg(ray, out, __FILE__, __func__, __LINE__)

// @DOC: check every object against ray
//       returns true if hit 
//       out gets set to hit info
//       ignores entity id's given in mask_arr
//       ignore single id: phys_ray_cast_mask(..., &id, 1)
//       ! set ray->len to <= 0 for inmfinite length, 
//         or to max length for ray
bool phys_ray_cast_mask_dbg(ray_t* ray, ray_hit_t* out, u32* mask_arr, int mask_arr_len, const char* _file, const char* _func, const int _line);
#define phys_ray_cast_mask(ray, out, mask_arr, mask_arr_len) phys_ray_cast_mask_dbg(ray, out, mask_arr, mask_arr_len, __FILE__, __func__, __LINE__)

#ifdef __cplusplus
} // extern c
#endif

#endif  // PHYS_PHYS_RAY_H
