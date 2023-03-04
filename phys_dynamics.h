#ifndef PHYS_DYNAMICS_H
#define PHYS_DYNAMICS_H

#include "phys/phys_world.h" // has phys_types.h, global.h, object_data.h

// taken from "WinterDev: Designing a Physics Engine in 5 minutes", "https://www.youtube.com/watch?v=-_lspRG548E&t=301s"
void phys_dynamics_simulate(phys_obj_t* phys, f32 dt);

#endif
