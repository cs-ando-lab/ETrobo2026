#include "RallyRoute.h"

void RallyRoute::blockEdge(RallyTypes::Node nodeA, RallyTypes::Node nodeB) {
    NodeId nodeAId = toNodeId(nodeA.x, nodeA.y);
    NodeId nodeBId = toNodeId(nodeB.x, nodeB.y);
    blocked[nodeAId][nodeBId] = true;
    blocked[nodeBId][nodeAId] = true;
};  // Edgeを通行不可に設定する

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

RallyTypes::Direction RallyRoute::getDirection(int dx, int dy) {
    if(dx == 0 && dy == -1) {
        return RallyTypes::Direction::NORTH;
    } else if(dx == 1 && dy == 0) {
        return RallyTypes::Direction::EAST;
    } else if(dx == 0 && dy == 1) {
        return RallyTypes::Direction::SOUTH;
    } else if(dx == -1 && dy == 0) {
        return RallyTypes::Direction::WEST;
    } else {
        return RallyTypes::Direction::NONE;
    }
}
