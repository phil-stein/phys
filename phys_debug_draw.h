#ifndef PHYS_DEBUG_DRAW_H
#define PHYS_DEBUG_DRAW_H

#include "core/debug/debug_draw.h"

#ifdef PHYS_DEBUG

#include "phys/phys_types.h"

#define PHYS_DEBUG_VELOCITY_COLOR         RGB_F(1, 0, 1)
#define PHYS_DEBUG_COLLIDER_COLOR_DYNAMIC RGB_F(1, 0, 1)
#define PHYS_DEBUG_COLLIDER_COLOR_STATIC  RGB_F(1, 1, 0)

void phys_debug_draw_velocity_func(phys_obj_t* obj);
void phys_debug_draw_collider_func(phys_obj_t* obj, f32* color);
void phys_debug_draw_box_collider_func(phys_obj_t* obj, f32* color);

#define phys_debug_draw_velocity(obj)         phys_debug_draw_velocity_func(obj)
#define phys_debug_draw_collider(obj, c)      phys_debug_draw_collider_func(obj, c)
#define phys_debug_draw_box_collider(obj, c)  phys_debug_draw_box_collider_func(obj, c)

#else

#define phys_debug_draw_velocity(obj)  
#define phys_debug_draw_collider(obj, c)      
#define phys_debug_draw_box_collider(obj, c)  

#endif

#endif
