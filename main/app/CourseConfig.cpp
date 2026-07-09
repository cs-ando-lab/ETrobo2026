#include "CourseConfig.h"

CourseConfig::Course CourseConfig::current = Course::L;

void CourseConfig::setCourse(Course c) {
    current = c;
}

bool CourseConfig::isLeftCourse() {
    return current == Course::L;
}

float CourseConfig::sign() {
    return isLeftCourse() ? -1.0f : 1.0f;
}
