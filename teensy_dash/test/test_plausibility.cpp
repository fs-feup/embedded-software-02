#include <unity.h>

#include <cstdio>

#include "io_settings.hpp"

// Test-specific implementations - no class dependency
uint16_t scale_apps_lower_to_apps_higher(uint16_t apps_lower) {
  return apps_lower + config::apps::LINEAR_OFFSET;
}

bool test_plausibility(int apps_higher, int apps_lower) {
  const bool valid_input = (apps_higher >= apps_lower) &&
                           (apps_higher >= config::apps::LOWER_BOUND_APPS_HIGHER &&
                            apps_higher <= config::apps::UPPER_BOUND_APPS_HIGHER) &&
                           (apps_lower >= config::apps::LOWER_BOUND_APPS_LOWER &&
                            apps_lower <= config::apps::UPPER_BOUND_APPS_LOWER);

  if (!valid_input) {
    return false;
  }
  printf("valid input\n");

  if (apps_higher >= config::apps::DEAD_THRESHOLD_APPS_HIGHER) {
    constexpr int min_expected_apps_lower =
        config::apps::APPS_HIGHER_DEADZONE_IN_APPS_LOWER_SCALE - config::apps::MAX_ERROR_ABS;
    return (apps_lower >= min_expected_apps_lower);
  }
  printf("threshold1\n");
  if (apps_lower <= config::apps::DEAD_THRESHOLD_APPS_LOWER) {
    constexpr int max_expected_apps_higher =
        config::apps::APPS_LOWER_DEADZONE_IN_APPS_HIGHER_SCALE + config::apps::MAX_ERROR_ABS;
    return (apps_higher <= max_expected_apps_higher);
  }
  printf("threshold2\n");
  const int scaled_apps_lower = scale_apps_lower_to_apps_higher(apps_lower);
  const int difference = abs(scaled_apps_lower - apps_higher);
  const int percentage_difference = (difference * 100) / apps_higher;
  printf("apps_higher: %d\n", apps_higher);
  printf("apps_lower: %d\n", apps_lower);
  printf("scaled_apps_lower: %d\n", scaled_apps_lower);
  printf("difference: %d\n", difference);
  printf("percentage_difference: %d\n", percentage_difference);
  printf("max_error_percent: %d\n", config::apps::MAX_ERROR_PERCENT);
  return (percentage_difference < config::apps::MAX_ERROR_PERCENT);
}

int abs(int value) { return value < 0 ? -value : value; }

void setUp(void) {
}

void tearDown(void) {
}

void test_plausibility_valid_input_normal_range(void) {
  int apps_higher = 600;
  int apps_lower = 480;
  bool result = test_plausibility(apps_higher, apps_lower);
  TEST_ASSERT_TRUE(result);
}

void test_plausibility_invalid_input_higher_lower_than_lower(void) {
  int apps_higher = 300;
  int apps_lower = 400;
  bool result = test_plausibility(apps_higher, apps_lower);
  TEST_ASSERT_FALSE(result);
}

void test_plausibility_deadzone_higher(void) {
  int apps_higher = config::apps::DEAD_THRESHOLD_APPS_HIGHER + 10;
  int apps_lower =
      config::apps::APPS_HIGHER_DEADZONE_IN_APPS_LOWER_SCALE - config::apps::MAX_ERROR_ABS - 1;
  bool result = test_plausibility(apps_higher, apps_lower);
  TEST_ASSERT_FALSE(result);
}

int main(void) {
  UNITY_BEGIN();

  RUN_TEST(test_plausibility_valid_input_normal_range);
  RUN_TEST(test_plausibility_invalid_input_higher_lower_than_lower);
  RUN_TEST(test_plausibility_deadzone_higher);

  return UNITY_END();
}