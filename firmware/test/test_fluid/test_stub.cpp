// Placeholder test — real tests land in Task 3.
#include <unity.h>

void setUp() {}
void tearDown() {}

void test_placeholder() {
    TEST_ASSERT_EQUAL(0, 0);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_placeholder);
    return UNITY_END();
}
