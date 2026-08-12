#include "test.h"
#include <t_syslog.h>

Test::Test(Robot& robot)
    : robot(robot) {
}

void Test::run() {
    syslog(LOG_NOTICE, "--- Test Started ---");
}
