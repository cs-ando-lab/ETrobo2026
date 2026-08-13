#ifndef RALLYTYPES_H_
#define RALLYTYPES_H_

/**
 * ETラリーで使う共通の型
 */

// ────────── ゲート座標 ─────────────────────
// ゲートの色
class RallyTypes {
public:
    enum struct GateColor {
        RED,
        BLUE,
        YELLOW
    };
    // ゲートの足がどのグリッド上にあるかを保持する構造体
    struct GatePoint {
        int row;  // 1〜5
        int col;  // 1〜5
    };
    // ゲートの両足の座標
    struct Gate {
        GateColor color;
        GatePoint leftLeg;
        GatePoint rightLeg;
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

    struct Segment {
        Node start;
        Node end;
        Direction direction;
    };
};

#endif  // !RALLYTYPES_H_
