#ifndef APPROACH_END_H_
#define APPROACH_END_H_

/**
 * ボトルへの低速接近を終える条件の判定だけを取り出したもの。
 * センサーにも時計にも触らず、読んだ値を渡されて判断するだけなのでホストで試験できる。
 *
 * 超音波が反応しない走行があったため、従来の超音波条件に加えて
 * 「低速接近を始めてから一定距離進んだ」を代替条件として持つ。どちらか早い方で終える。
 *
 * ultrasonicMm: 生の超音波距離[mm]。0や負値は「読めていない」の意味で、距離条件には影響しない。
 * movedMm:      低速接近の開始地点からの符号付き前進量[mm]。後退しても絶対値では見ない。
 */
class ApproachEnd {
public:
    enum class Reason { NONE,
                        ULTRASONIC,
                        DISTANCE };

    // 同じ周期に両方成立したら超音波を理由として返す（どちらでも動作は同じで、記録だけ分ける）
    static Reason decide(int ultrasonicMm, int ultrasonicTargetMm, int movedMm, int fallbackMm) {
        if(ultrasonicMm > 0 && ultrasonicMm <= ultrasonicTargetMm) return Reason::ULTRASONIC;
        if(movedMm >= fallbackMm) return Reason::DISTANCE;
        return Reason::NONE;
    }

    static const char* reasonName(Reason reason) {
        switch(reason) {
            case Reason::ULTRASONIC: return "ULTRASONIC";
            case Reason::DISTANCE: return "DISTANCE";
            default: return "NONE";
        }
    }
};

#endif  // !APPROACH_END_H_
