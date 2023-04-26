#ifndef PHYS_PHYS_DEBUG_DRAW_H
#define PHYS_PHYS_DEBUG_DRAW_H

#ifdef PHYS_DEBUG

#include "core/debug/debug_draw.h"
#include "phys/phys_types.h"

// @DOC: color given to velocity debug display
#define PHYS_DEBUG_VELOCITY_COLOR          RGB_F(1, 0, 1)
// @DOC: color given to dynamic collider debug display
#define PHYS_DEBUG_COLLIDER_COLOR_DYNAMIC  RGB_F(1, 0, 1)
// @DOC: color given to static collider debug display
#define PHYS_DEBUG_COLLIDER_COLOR_STATIC   RGB_F(1, 1, 0)
// @DOC: color given to trigger collider debug display
#define PHYS_DEBUG_COLLIDER_COLOR_TRIGGER  RGB_F(0, 1, 1)

#define PHYS_DEBUG_SPHERE_COLLIDER_COLOR   RGB_F(0, 1, 1)
// #define PHYS_DEBUG_SPHERE_COLLIDER_RADIUS  1.0f

// @DOC: draw a line indicating a phys_objs_t's velocity
//       obj: objects velocity to be drawn
void phys_debug_draw_velocity_func(phys_obj_t* obj);
// @DOC: draw a debug display for collider
//       obj:   object whichs collider will be drawn
//       color: f32[3] rgb, defing color of collider debug display
void phys_debug_draw_collider_func(phys_obj_t* obj, f32* color);
// @DOC: draw a cross of lines as a debug display for a sphere collider, @TODO: should be a sphere
//       obj:   object whichs collider will be drawn
//       color: f32[3] rgb, defing color of collider debug display
void phys_debug_draw_sphere_collider_func(phys_obj_t* obj, f32* color);
// @DOC: draw a box of lines as a debug display for a box collider
//       obj:   object whichs collider will be drawn
//       color: f32[3] rgb, defing color of collider debug display
void phys_debug_draw_box_collider_func(phys_obj_t* obj, f32* color);

// @NOTE: funcs as macros so they can be compiled out when PHYS_DEBUG isnt defined
#define phys_debug_draw_velocity(obj)                phys_debug_draw_velocity_func(obj)
#define phys_debug_draw_collider_col(obj, c)         phys_debug_draw_collider_func(obj, c)
#define phys_debug_draw_collider(obj)                phys_debug_draw_collider_func(obj,                                     \
                                                     (obj)->collider.is_trigger ? PHYS_DEBUG_COLLIDER_COLOR_TRIGGER :       \
                                                     (HAS_FLAG((obj)->flags, PHYS_HAS_RIGIDBODY) ?                          \
                                                      PHYS_DEBUG_COLLIDER_COLOR_DYNAMIC :                                   \
                                                      PHYS_DEBUG_COLLIDER_COLOR_STATIC))

#define phys_debug_draw_box_collider_col(obj, c)     phys_debug_draw_box_collider_func(obj, c)
#define phys_debug_draw_box_collider(obj)            phys_debug_draw_box_collider_func(obj,                                 \
                                                     (obj)->collider.is_trigger ? PHYS_DEBUG_COLLIDER_COLOR_TRIGGER :       \
                                                     (HAS_FLAG((obj)->flags, PHYS_HAS_RIGIDBODY) ?                          \
                                                      PHYS_DEBUG_COLLIDER_COLOR_DYNAMIC :                                   \
                                                      PHYS_DEBUG_COLLIDER_COLOR_STATIC))

#define phys_debug_draw_sphere_collider_col(obj, c)  phys_debug_draw_sphere_collider_func(obj, c)
#define phys_debug_draw_sphere_collider(obj)         phys_debug_draw_sphere_collider_func(obj,                              \
                                                     (obj)->collider.is_trigger ? PHYS_DEBUG_COLLIDER_COLOR_TRIGGER :       \
                                                     (HAS_FLAG((obj)->flags, PHYS_HAS_RIGIDBODY) ?                          \
                                                      PHYS_DEBUG_COLLIDER_COLOR_DYNAMIC :                                   \
                                                      PHYS_DEBUG_COLLIDER_COLOR_STATIC))

#else // PHYS_DEBUG

#define phys_debug_draw_velocity(obj)  
#define phys_debug_draw_collider(obj, c)      
#define phys_debug_draw_box_collider_col(obj, c)
#define phys_debug_draw_box_collider(obj)  
#define phys_debug_draw_sphere_collider_col(obj, c)
#define phys_debug_draw_sphere_collider(obj)       

#endif // PHYS_DEBUG

#endif
