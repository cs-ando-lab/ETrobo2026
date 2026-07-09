#ifndef COURSECONFIG_H_
#define COURSECONFIG_H_

/**
 * L/Rコースの選択状態を保持するクラス。
 * 走行体全体で共有する状態なので、インスタンス化せずstaticメンバとして扱う。
 */
class CourseConfig {
public:
    enum class Course { L,
                        R };

    static void setCourse(Course c);
    static bool isLeftCourse();

    // 旋回方向の符号を返す: Lコース = -1.0f, Rコース = +1.0f
    // 使い方: robot.turn(CourseConfig::sign() * 90.0f);
    static float sign();

private:
    static Course current;
};

#endif  // !COURSECONFIG_H_
