#ifndef BFS_H_
#define BFS_H_

#include "RallyTypes.h"

#include <vector>

class RallyRoute {
public:
    void blockEdge(RallyTypes::Node nodeA, RallyTypes::Node nodeB);  // Edgeを通行不可に設定する
    std::vector<RallyTypes::Segment> groupStraightSegments(
        const std::vector<RallyTypes::Node>& path);  // 入力は必ず隣接Node

private:
    static constexpr int ROWS = 6;
    static constexpr int COLS = 6;
    static constexpr int NODE_COUNT = ROWS * COLS;

    static constexpr int DIRECTION_COUNT = 4;
    static constexpr int ROW_OFFSET[DIRECTION_COUNT] = { -1, 0, 1, 0 };
    static constexpr int COL_OFFSET[DIRECTION_COUNT] = { 0, 1, 0, -1 };

    bool blocked[NODE_COUNT][NODE_COUNT] = {};  // 隣接行列：trueであればそのEdgeは通行できない

    using NodeId = int;

    // 座標とNodeIDの変換
    constexpr NodeId toNodeId(int row, int col) {
        return row * COLS + col;
    }
    constexpr int getRow(NodeId node) {
        return node / COLS;
    }
    constexpr int getCol(NodeId node) {
        return node % COLS;
    }

    RallyTypes::Direction getDirection(int dx, int dy);
};

#endif  // !BFS_H
