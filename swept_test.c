
// link: https://www.gamedev.net/articles/programming/general-and-gameplay-programming/swept-aabb-collision-detection-and-response-r3084/
collision_info_t phys_collision_check_aabb_v_aabb_swept(phys_obj_t* b0, phys_obj_t* b1)
{
  // find time of collision and time of leaving for each axis (if statement is to prevent divide by zero) 
  float xEntry, yEntry; 
  float xExit, yExit; 

  if (b1.vx == 0.0f) 
  { 
    xEntry = -std::numeric_limits::infinity(); 
    xExit = std::numeric_limits::infinity(); 
  } 
  else 
  { 
    xEntry = xInvEntry / b1.vx; 
    xExit = xInvExit / b1.vx; 
  } 

  if (b1.vy == 0.0f) 
  { 
    yEntry = -std::numeric_limits::infinity(); 
    yExit = std::numeric_limits::infinity(); 
  } 
  else 
  { 
    yEntry = yInvEntry / b1.vy; 
    yExit = yInvExit / b1.vy; 
  }
}

// dylan falconer: https://www.youtube.com/watch?v=3dIiTo7mlnU 
collision_info_t phys_collision_check_aabb_v_aabb_swept(phys_obj_t* b0, phys_obj_t* b1)
{
  collision_info_t info;

  

  return info;
}
