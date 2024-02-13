#ifndef PHYS_PHYS_RAY_H
#define PHYS_PHYS_RAY_H

#include "phys/phys_types.h" 

#ifdef __cplusplus
extern "C" {
#endif


// @DOC: check every object against ray
//       returns true if hit 
//       out gets set to hit info
bool phys_ray_cast(ray_t* ray, ray_hit_t* out);

#ifdef __cplusplus
} // extern c
#endif

#endif  // PHYS_PHYS_RAY_H
