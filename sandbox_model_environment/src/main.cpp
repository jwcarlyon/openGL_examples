#include <iostream>
#include <Models_Shaders_Physics/Models_Shaders_Physics.h>

int main() {
  open_gl_setup();
  // physics_setup();
  game_engine_loop();
  open_gl_destroy();
  // physics_destroy();
  return 0;
}
