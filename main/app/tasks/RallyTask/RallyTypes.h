#ifndef RALLYTYPES_H_
#define RALLYTYPES_H_

/**
 * ETラリーで使う共通の型
 */

// ────────── ゲート座標 ─────────────────────
// ゲートの色
class RallyTypes {
public:
    // ゲートの色とその通過順
    enum struct GateColor {
        RED,
        BLUE,
        YELLOW
    };

    // ゲート片足の座標
    struct GateLeg {
        int gX;  // 1〜5
        int gY;  // 1〜5
    };

    // ゲートの座標
    struct Gate {
        GateColor color;
        GateLeg leftLeg;
        GateLeg rightLeg;
    };

    // ────────── 移動グリッド ──────────────────
    enum class Direction : int {  // 各Nodeでの走行体の向き
        NORTH = 0,
        EAST = 1,
        SOUTH = 2,
        WEST = 3,
        NONE = 4
    };

    struct Node {
        int x;  // 0〜5
        int y;  // 0〜5
    };

    struct Edge {  // エッジ
        Node endpointA;
        Node endpointB;
    };

    struct Segment {
        Node start;
        Node end;
        Direction direction;
    };
};

#endif  // !RALLYTYPES_H_
