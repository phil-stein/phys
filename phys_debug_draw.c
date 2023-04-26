#ifdef PHYS_DEBUG

#include "phys/phys_debug_draw.h"

void phys_debug_draw_velocity_func(phys_obj_t* obj)
{ 
  if (!PHYS_OBJ_HAS_RIGIDBODY(obj)) { return; }

  vec3 v_scaled, v_pos;
	vec3_copy(obj->rb.velocity, v_scaled);
	vec3_mul_f(v_scaled, 0.2f, v_scaled);
	vec3_add(obj->pos, v_scaled, v_pos);
  debug_draw_line_register(obj->pos, v_pos, PHYS_DEBUG_VELOCITY_COLOR); 
}

void phys_debug_draw_collider_func(phys_obj_t* obj, f32* color)
{
	if (!PHYS_OBJ_HAS_COLLIDER(obj)) { return; }
  switch (obj->collider.type)
  {
    case PHYS_COLLIDER_SPHERE:
      phys_debug_draw_sphere_collider_func(obj, color);
      break;
    case PHYS_COLLIDER_BOX:
      phys_debug_draw_box_collider_func(obj, color);
      break;
  }
}

void phys_debug_draw_sphere_collider_func(phys_obj_t* obj, f32* color)
{
	if (!PHYS_OBJ_HAS_COLLIDER(obj)) { return; }
  
  f32 radius = obj->collider.sphere.radius * ((obj->scl[0] + obj->scl[1] + obj->scl[2]) * 0.33f);
  debug_draw_draw_circle(VEC3_XYZ(1, 1, 0), obj->pos, radius, color);
  debug_draw_draw_circle(VEC3_XYZ(1, 0, 1), obj->pos, radius, color);
  debug_draw_draw_circle(VEC3_XYZ(0, 1, 1), obj->pos, radius, color);
}


void phys_debug_draw_box_collider_func(phys_obj_t* obj, f32* color)
{
	if (!PHYS_OBJ_HAS_COLLIDER(obj)) { return; }
  
  vec3* aabb = obj->collider.box.aabb;
  vec3  min, max;
  vec3_copy(aabb[0], min);
  vec3_copy(aabb[1], max);
  vec3_mul(obj->scl, min, min);
  vec3_mul(obj->scl, max, max);
 	vec3_add(min, obj->collider.offset, min);
	vec3_add(max, obj->collider.offset, max); 
  
  vec3_add(obj->pos, min, min);
  vec3_add(obj->pos, max, max);
  
  vec3 top0 = { max[0], max[1], max[2] };
  vec3 top1 = { max[0], max[1], min[2] }; 
  vec3 top2 = { min[0], max[1], min[2] }; 
  vec3 top3 = { min[0], max[1], max[2] }; 
  
  vec3 bot0 = { max[0], min[1], max[2] };
  vec3 bot1 = { max[0], min[1], min[2] };
  vec3 bot2 = { min[0], min[1], min[2] };
  vec3 bot3 = { min[0], min[1], max[2] };
  
  // vec3_add(obj->pos, top0, top0);
  // vec3_add(obj->pos, top1, top1);
  // vec3_add(obj->pos, top2, top2);
  // vec3_add(obj->pos, top3, top3);
  // vec3_add(obj->pos, bot0, bot0);
  // vec3_add(obj->pos, bot1, bot1);
  // vec3_add(obj->pos, bot2, bot2);
  // vec3_add(obj->pos, bot3, bot3);
  
  // debug_draw_line_register(top0, top1, PHYS_DEBUG_COLLIDER_COLOR); 
  // debug_draw_line_register(top1, top2, PHYS_DEBUG_COLLIDER_COLOR); 
  // debug_draw_line_register(top2, top3, PHYS_DEBUG_COLLIDER_COLOR); 
  // debug_draw_line_register(top3, top0, PHYS_DEBUG_COLLIDER_COLOR); 
  // 
  // debug_draw_line_register(bot0, bot1, PHYS_DEBUG_COLLIDER_COLOR); 
  // debug_draw_line_register(bot1, bot2, PHYS_DEBUG_COLLIDER_COLOR); 
  // debug_draw_line_register(bot2, bot3, PHYS_DEBUG_COLLIDER_COLOR); 
  // debug_draw_line_register(bot3, bot0, PHYS_DEBUG_COLLIDER_COLOR); 
  // 
  // debug_draw_line_register(bot0, top0, PHYS_DEBUG_COLLIDER_COLOR); 
  // debug_draw_line_register(bot1, top1, PHYS_DEBUG_COLLIDER_COLOR); 
  // debug_draw_line_register(bot2, top2, PHYS_DEBUG_COLLIDER_COLOR); 
  // debug_draw_line_register(bot3, top3, PHYS_DEBUG_COLLIDER_COLOR);

#define V(v)  { (v)[0], (v)[1], (v)[2] }
  vec3 points[8] = 
  {
    V(top0), V(top1), V(top2), V(top3),
    V(bot0), V(bot1), V(bot2), V(bot3)
  };
#undef V

  // debug_draw_box_register(points, PHYS_OBJ_HAS_RIGIDBODY(obj) ? PHYS_DEBUG_COLLIDER_COLOR_DYNAMIC : PHYS_DEBUG_COLLIDER_COLOR_STATIC);
  // debug_draw_box_register(points, obj->is_trigger ? trigger_col : PHYS_OBJ_HAS_RIGIDBODY(obj) ? normal_col : static_col);
  debug_draw_box_register(points, color);

  // debug_draw_sphere_register(top0, 0.25f, RGB_F(1, 1, 0));
  // debug_draw_sphere_register(top1, 0.25f, RGB_F(1, 0, 0));
  // debug_draw_sphere_register(top2, 0.25f, RGB_F(0, 1, 0));
  // debug_draw_sphere_register(top3, 0.25f, RGB_F(0, 0, 1));
  // 
  // debug_draw_sphere_register(bot0, 0.25f, RGB_F(1, 1, 0));
  // debug_draw_sphere_register(bot1, 0.25f, RGB_F(1, 0, 0));
  // debug_draw_sphere_register(bot2, 0.25f, RGB_F(0, 1, 0));
  // debug_draw_sphere_register(bot3, 0.25f, RGB_F(0, 0, 1));
}

#endif
