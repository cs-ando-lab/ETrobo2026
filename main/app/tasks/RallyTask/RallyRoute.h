#ifndef RALLY_ROUTE_H_
#define RALLY_ROUTE_H_

#include "RallyTypes.h"
#include "Config.h"

#include <stddef.h>
#include <array>
#include <vector>

class RallyRoute {
private:
    using Node = RallyTypes::Node;
    using Edge = RallyTypes::Edge;
    using Segment = RallyTypes::Segment;
    using Direction = RallyTypes::Direction;
    using GateColor = RallyTypes::GateColor;
    using Gate = RallyTypes::Gate;

    /* ゲート情報 */
    static constexpr size_t GATE_COUNT = 3;                     // ゲートの種数
    static const std::array<Gate, GATE_COUNT> GATES;            // ゲートの配置
    static const std::array<GateColor, GATE_COUNT> GATE_CYCLE;  // ゲートの通過順
    struct GatePassingEdge {
        GateColor color;
        Edge edge;
    };
    std::array<GatePassingEdge, GATE_COUNT> gatePassingEdges_;  // ゲート通過エッジ

    /* 移動グリッド */
    Node initNode_;                                           // 初期ノード
    static constexpr int GRID_SIZE = 6;                       // 移動グリッド全体における一辺のノード個数
    static constexpr int NODE_COUNT = GRID_SIZE * GRID_SIZE;  // 移動グリッドのノード個数

    /* 経路探索 */
    struct SearchState {  // 同じノードでも到着方向が異なれば別の探索状態として扱う
        Node node;
        Direction direction;
    };
    struct RouteCost {  // 曲がる回数を優先し、同数なら移動ステップ数を比較する
        int turnCount;
        int stepCount;
    };
    struct SearchRecord {
        bool reached = false;
        bool hasPrevious = false;
        RouteCost cost = { 0, 0 };
        SearchState previous = { { 0, 0 }, Direction::NONE };
    };
    struct ResultPath {  // 経路探索の結果を格納
        bool found = false;
        std::vector<Node> path;
        Direction lastDirection = Direction::NONE;
    };
    struct QueueEntry {
        SearchState state;
        RouteCost cost;
    };
    struct QueueEntryGreater {
        bool operator()(const QueueEntry& lhs, const QueueEntry& rhs) const;
    };

    static constexpr size_t DIRECTION_COUNT = 4;       // 方角によるx,y座標の差を表す配列の個数
    static const std::array<int, DIRECTION_COUNT> DX;  // 北、東、南、西
    static const std::array<int, DIRECTION_COUNT> DY;  // 北、東、南、西

    ResultPath findPath(Node startNode,
                        const std::vector<Node>& endNodes,
                        Direction initDirection);  // 曲がる回数、移動ステップ数の順に最小となる経路を探す
    bool isBetterCost(const RouteCost& lhs, const RouteCost& rhs) const;
    bool isSameCost(const RouteCost& lhs, const RouteCost& rhs) const;
    size_t toSearchRecordIndex(Node node, size_t direction) const;
    RallyRoute::ResultPath findGateThroughPath(Node front, GateColor color);

    /* 経路探索条件判定 */
    bool isNodeInsideGrid(Node node) const;                                    // nodeがグリッド内に存在するかを判定
    bool isSameNode(Node nodeA, Node nodeB) const;                             // nodeAとnodeBの同一判定
    bool checkReachedEnd(Node node, const std::vector<Node>& endNodes) const;  // 探索ノードがゴールノード（複数可）と等しいか判定
    bool isSameEdge(Edge edgeA, Edge edgeB) const;                             // edgeAとedgeBの同一判定
    bool isEdgePassGate(Node nodeA, Node nodeB) const;                         // nodeAとnodeBを結ぶエッジがゲートを通過するかを判定

    /* その他メソッド */
    Edge findGateEdge(Gate gate);                        // 各ゲートの前後ノードを調べる
    int calcNodeDistance(Node node1, Node node2) const;  // node1とnode2のマンハッタン距離を調べる
    Direction getDirection(int dx, int dy) const;
    bool appendPath(std::vector<Node>& destination, const ResultPath& result);

public:
    explicit RallyRoute(Node initNode);

    std::vector<Segment> groupStraightSegments(         // Node列の直線部分をまとめてSegment列にする。
        const std::vector<Node>& path);                 // 入力するNode列のNodeの前後は隣接している必要がある。
    std::vector<Segment> calculateRoute(int lapCount);  // lapCount(1 ~ 3)周のETラリーを行うためのルートをSegment列で返す。
};

#endif  // RALLY_ROUTE_H_
