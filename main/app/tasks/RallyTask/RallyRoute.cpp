#include "RallyRoute.h"
#include <stdlib.h>
#include <queue>
#include <t_syslog.h>
#include <algorithm>
#include <utility>

const std::array<RallyTypes::Gate, RallyRoute::GATE_COUNT>
    RallyRoute::GATES = { { { RallyTypes::GateColor::RED,
                              { Config::ETRALLY_RED_GATE_LEFT_X,
                                Config::ETRALLY_RED_GATE_LEFT_Y },
                              { Config::ETRALLY_RED_GATE_RIGHT_X,
                                Config::ETRALLY_RED_GATE_RIGHT_Y } },
                            { RallyTypes::GateColor::BLUE,
                              { Config::ETRALLY_BLUE_GATE_LEFT_X,
                                Config::ETRALLY_BLUE_GATE_LEFT_Y },
                              { Config::ETRALLY_BLUE_GATE_RIGHT_X,
                                Config::ETRALLY_BLUE_GATE_RIGHT_Y } },
                            { RallyTypes::GateColor::YELLOW,
                              { Config::ETRALLY_YELLOW_GATE_LEFT_X,
                                Config::ETRALLY_YELLOW_GATE_LEFT_Y },
                              { Config::ETRALLY_YELLOW_GATE_RIGHT_X,
                                Config::ETRALLY_YELLOW_GATE_RIGHT_Y } } } };

const std::array<RallyTypes::GateColor, RallyRoute::GATE_COUNT>
    RallyRoute::GATE_CYCLE = { { RallyTypes::GateColor::RED,
                                 RallyTypes::GateColor::BLUE,
                                 RallyTypes::GateColor::YELLOW } };

const std::array<int, RallyRoute::DIRECTION_COUNT>
    RallyRoute::DX = { 0, 1, 0, -1 };  // 北、東、南、西
const std::array<int, RallyRoute::DIRECTION_COUNT>
    RallyRoute::DY = { -1, 0, 1, 0 };  // 北、東、南、西

RallyRoute::RallyRoute(RallyTypes::Node initNode) {
    initNode_ = initNode;
    for(size_t i = 0; i < GATE_COUNT; i++) {
        gatePassingEdges_[i] = {
            GATES[i].color,
            findGateEdge(GATES[i])
        };
    }
}

std::vector<RallyTypes::Segment> RallyRoute::calculateRoute(int lapCount) {
    if(lapCount < 1 || lapCount > 3) {
        syslog(LOG_ERROR, "ERROR[findPath]: invalid lapCount");
        return {};
    }

    std::vector<Node> path;
    path.reserve(100);
    ResultPath result;
    Node currentNode;
    Direction currentDireciton;

    // 現在ノードを初期ノードに設定
    currentNode = initNode_;
    currentDireciton = Direction::NORTH;

    // 現在ノードから最寄りのゲートノードまでのルートをPathに追加
    // ゲートを通過するノードをPathに追加し、現在ノードを更新
    for(int i = 0; i < lapCount; i++) {  // lapゲート周繰り返す
        for(GateColor color : GATE_CYCLE) {
            // 次ゲートの通過エッジの両端ノードを保持
            std::vector<Node> targetNodes;
            for(GatePassingEdge gpe : gatePassingEdges_) {
                if(gpe.color == color) {
                    targetNodes = { gpe.edge.endpointA, gpe.edge.endpointB };
                }
            }

            // 次のゲートまでのルートを探索し、pathに追加
            result = findPath(currentNode,
                              targetNodes,
                              currentDireciton);
            if(!appendPath(path, result)) {
                syslog(LOG_ERROR, "ERROR[appendPath]: invalid result");
                return {};
            }
            currentNode = path.back();
            currentDireciton = result.lastDirection;

            // ゲートをくぐるルートをpathに追加
            result = findGateThroughPath(currentNode,
                                         color);
            if(!appendPath(path, result)) {
                syslog(LOG_ERROR, "ERROR[appendPath]: invalid result");
                return {};
            }
            currentNode = path.back();
            currentDireciton = result.lastDirection;
        }
    }

    // 初期ノードまでのルートをPathに追加
    result = findPath(currentNode,
                      { initNode_ },
                      currentDireciton);
    if(!appendPath(path, result)) {
        syslog(LOG_ERROR, "ERROR[appendPath]: invalid result");
        return {};
    }

    return groupStraightSegments(path);
}

RallyRoute::ResultPath RallyRoute::findPath(Node startNode, const std::vector<Node>& endNodes, Direction initDirection) {
    // バリデーション
    int initDirectionIndex = static_cast<int>(initDirection);
    bool invalidArgument = endNodes.empty()
                           || !isNodeInsideGrid(startNode)
                           || initDirectionIndex < 0
                           || initDirectionIndex > static_cast<int>(Direction::NONE);
    for(Node node : endNodes) {
        if(!isNodeInsideGrid(node))
            invalidArgument = true;
    }
    if(invalidArgument) {
        syslog(LOG_ERROR, "ERROR[findPath]: invalid argument");
        return {};
    }

    for(Node node : endNodes) {
        if(isSameNode(startNode, node)) {
            ResultPath result;
            result.found = true;
            result.path.push_back(startNode);
            result.lastDirection = initDirection;
            return result;
        }
    }

    // 144状態分をタスクスタックへ置くと4KBのスタックを圧迫するため、ヒープへ確保する。
    std::vector<SearchRecord> records(NODE_COUNT * DIRECTION_COUNT);
    std::vector<QueueEntry> queueStorage;
    queueStorage.reserve(NODE_COUNT * DIRECTION_COUNT);
    std::priority_queue<QueueEntry,
                        std::vector<QueueEntry>,
                        QueueEntryGreater>
        q(
            QueueEntryGreater{},
            std::move(queueStorage));

    // 初期方向が不明な場合は、最初の移動を方向転換として数えない。
    size_t firstDirection = initDirection == Direction::NONE
                                ? 0
                                : static_cast<size_t>(initDirection);
    size_t lastDirection = initDirection == Direction::NONE
                               ? DIRECTION_COUNT
                               : firstDirection + 1;
    for(size_t i = firstDirection; i < lastDirection; i++) {
        Direction direction = static_cast<Direction>(i);
        SearchRecord& startRecord = records[toSearchRecordIndex(startNode, i)];
        startRecord.reached = true;
        startRecord.cost = { 0, 0 };
        q.push({ { startNode, direction }, { 0, 0 } });
    }

    SearchState goalState = { startNode, initDirection };
    bool found = false;

    while(!q.empty()) {
        QueueEntry current = q.top();
        q.pop();

        size_t currentDirection = static_cast<size_t>(current.state.direction);
        const SearchRecord& currentRecord
            = records[toSearchRecordIndex(current.state.node, currentDirection)];

        // 同じ状態がより良いコストで更新された後の古いキュー要素は無視する。
        if(!currentRecord.reached || !isSameCost(current.cost, currentRecord.cost))
            continue;

        // Dijkstra法では、キューから取り出した時点で最小コストが確定する。
        if(checkReachedEnd(current.state.node, endNodes)) {
            goalState = current.state;
            found = true;
            break;
        }

        for(size_t i = 0; i < DIRECTION_COUNT; i++) {
            Node targetNode = {
                current.state.node.x + DX[i],
                current.state.node.y + DY[i]
            };
            if(!isNodeInsideGrid(targetNode)
               || isEdgePassGate(current.state.node, targetNode))
                continue;

            Direction targetDirection = static_cast<Direction>(i);
            RouteCost targetCost = {
                current.cost.turnCount
                    + (current.state.direction == targetDirection ? 0 : 1),
                current.cost.stepCount + 1
            };
            SearchRecord& targetRecord = records[toSearchRecordIndex(targetNode, i)];
            if(targetRecord.reached && !isBetterCost(targetCost, targetRecord.cost))
                continue;

            targetRecord.reached = true;
            targetRecord.hasPrevious = true;
            targetRecord.cost = targetCost;
            targetRecord.previous = current.state;
            q.push({ { targetNode, targetDirection }, targetCost });
        }
    }

    if(!found) {
        syslog(LOG_NOTICE, "Path Finding Failed");
        return {};
    }

    ResultPath result;
    result.found = true;
    result.lastDirection = goalState.direction;

    SearchState currentState = goalState;
    while(true) {
        result.path.push_back(currentState.node);
        size_t direction = static_cast<size_t>(currentState.direction);
        const SearchRecord& record
            = records[toSearchRecordIndex(currentState.node, direction)];
        if(!record.hasPrevious)
            break;
        currentState = record.previous;
    }
    std::reverse(result.path.begin(), result.path.end());

    return result;
}

RallyRoute::ResultPath RallyRoute::findGateThroughPath(Node front, GateColor color) {
    Node back;
    for(GatePassingEdge gpe : gatePassingEdges_) {
        if(gpe.color == color)
            back = isSameNode(front, gpe.edge.endpointA) ? gpe.edge.endpointB : gpe.edge.endpointA;
    }

    ResultPath result = {
        true,
        { front, back },
        getDirection(back.x - front.x,
                     back.y - front.y)
    };

    return result;
}

bool RallyRoute::QueueEntryGreater::operator()(const QueueEntry& lhs, const QueueEntry& rhs) const {
    if(lhs.cost.turnCount != rhs.cost.turnCount)
        return lhs.cost.turnCount > rhs.cost.turnCount;
    return lhs.cost.stepCount > rhs.cost.stepCount;
}

bool RallyRoute::isBetterCost(const RouteCost& lhs, const RouteCost& rhs) const {
    if(lhs.turnCount != rhs.turnCount)
        return lhs.turnCount < rhs.turnCount;
    return lhs.stepCount < rhs.stepCount;
}

bool RallyRoute::isSameCost(const RouteCost& lhs, const RouteCost& rhs) const {
    return lhs.turnCount == rhs.turnCount
           && lhs.stepCount == rhs.stepCount;
}

size_t RallyRoute::toSearchRecordIndex(Node node, size_t direction) const {
    return (node.x * GRID_SIZE + node.y) * DIRECTION_COUNT + direction;
}

bool RallyRoute::isNodeInsideGrid(Node node) const {
    return (0 <= node.x && node.x < GRID_SIZE)
           && (0 <= node.y && node.y < GRID_SIZE);
}

bool RallyRoute::isSameNode(Node nodeA, Node nodeB) const {
    return (nodeA.x == nodeB.x) && (nodeA.y == nodeB.y);
}

bool RallyRoute::checkReachedEnd(Node node, const std::vector<Node>& endNodes) const {
    for(Node endNode : endNodes) {
        if(isSameNode(node, endNode))
            return true;
    }
    return false;
}

bool RallyRoute::isSameEdge(Edge edgeA, Edge edgeB) const {
    bool forward = isSameNode(edgeA.endpointA, edgeB.endpointA)
                   && isSameNode(edgeA.endpointB, edgeB.endpointB);
    bool backward = isSameNode(edgeA.endpointA, edgeB.endpointB)
                    && isSameNode(edgeA.endpointB, edgeB.endpointA);
    return forward || backward;
}

bool RallyRoute::isEdgePassGate(Node nodeA, Node nodeB) const {
    bool passed = false;
    Edge edge = {
        nodeA,
        nodeB
    };
    for(GatePassingEdge passingEdge : gatePassingEdges_) {
        if(!passed)
            passed = isSameEdge(edge, passingEdge.edge);
    }
    return passed;
}

RallyRoute::Edge RallyRoute::findGateEdge(RallyTypes::Gate gate) {
    // ゲート脚座標のうち最小の座標を求める
    int minGX, minGY;
    minGX = gate.leftLeg.gX < gate.rightLeg.gX ? gate.leftLeg.gX : gate.rightLeg.gX;
    minGY = gate.leftLeg.gY < gate.rightLeg.gY ? gate.leftLeg.gY : gate.rightLeg.gY;

    Edge edge;

    // ゲートが赤・黄の場合は、(minGX, minGY - 1), (minGX, minGY)
    // ゲートが青の場合は、　　(minGX - 1, minGY), (minGX, minGY)
    switch(gate.color) {
        case RallyTypes::GateColor::RED:
        case RallyTypes::GateColor::YELLOW:
            edge.endpointA.x = minGX;
            edge.endpointA.y = minGY - 1;
            edge.endpointB.x = minGX;
            edge.endpointB.y = minGY;
            break;
        case RallyTypes::GateColor::BLUE:
            edge.endpointA.x = minGX - 1;
            edge.endpointA.y = minGY;
            edge.endpointB.x = minGX;
            edge.endpointB.y = minGY;
            break;
    }

    return edge;
}

RallyTypes::Direction RallyRoute::getDirection(int dx, int dy) const {
    if(dx == 0 && dy < 0) {
        return RallyTypes::Direction::NORTH;
    } else if(dx > 0 && dy == 0) {
        return RallyTypes::Direction::EAST;
    } else if(dx == 0 && dy > 0) {
        return RallyTypes::Direction::SOUTH;
    } else if(dx < 0 && dy == 0) {
        return RallyTypes::Direction::WEST;
    }

    syslog(LOG_NOTICE, "Invalid Direction");
    return Direction::NONE;
}

bool RallyRoute::appendPath(std::vector<Node>& path, const ResultPath& result) {
    if(!result.found || result.path.empty()) {
        return false;
    }

    for(const Node& node : result.path) {
        if(path.empty()
           || !isSameNode(path.back(), node)) {
            path.push_back(node);
        }
    }

    return true;
}

std::vector<RallyTypes::Segment> RallyRoute::groupStraightSegments(
    const std::vector<RallyTypes::Node>& path) {
    std::vector<RallyTypes::Segment> segments;

    if(path.size() < 2) {
        return segments;
    }

    RallyTypes::Node segmentStart = path[0];

    RallyTypes::Direction currentDirection = getDirection(
        path[1].x - path[0].x,
        path[1].y - path[0].y);

    for(std::size_t i = 1; i + 1 < path.size(); i++) {
        RallyTypes::Direction nextDirection = getDirection(
            path[i + 1].x - path[i].x,
            path[i + 1].y - path[i].y);

        if(currentDirection != nextDirection) {
            // 方向が変わったので、現在地点までを直線として保存
            segments.push_back({ segmentStart, path[i], currentDirection });

            // 現在地点から新しい直線を開始
            segmentStart = path[i];
            currentDirection = nextDirection;
        }
    }
    // 最後の直線を保存
    segments.push_back({ segmentStart, path.back(), currentDirection });

    return segments;
}
