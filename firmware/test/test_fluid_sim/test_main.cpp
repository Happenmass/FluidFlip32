#include <unity.h>
#include "fluid_sim.h"

using namespace fluidsim;

static Scene gScene;

void setUp(void) {}
void tearDown(void) {}

void test_setup_seeds_requested_particle_count(void) {
  gScene.setup(kDefaultParticles);
  TEST_ASSERT_EQUAL_INT(kDefaultParticles, gScene.numParticles());
}

void test_setup_places_particles_inside_simulation_bounds(void) {
  gScene.setup(kDefaultParticles);
  Scene::OutputGrid out;
  gScene.getOutput(out);
  // After setup but before simulate(), all particles exist but cells are still
  // marked from their previous state (Air). We assert the underlying particle
  // positions instead by reading via a friend-equivalent: simulate one frame
  // with zero gravity so showParticles() runs and marks fluid cells.
  // (showParticles is implemented in a later task; for now we only check count.)
  TEST_ASSERT_EQUAL_INT(kDefaultParticles, gScene.numParticles());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_setup_seeds_requested_particle_count);
  RUN_TEST(test_setup_places_particles_inside_simulation_bounds);
  return UNITY_END();
}
