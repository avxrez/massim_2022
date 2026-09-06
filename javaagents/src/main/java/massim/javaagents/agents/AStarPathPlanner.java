package massim.javaagents.agents;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

/** Calculates shortest four-directional paths on the agent's known map. */
public class AStarPathPlanner {

    private record Node(InternalMap.Position position, int cost, int estimate) {}

    public List<String> findPath(InternalMap.Position start,
                                 InternalMap.Position goal,
                                 List<InternalMap.Position> blockedPositions) {
        Set<InternalMap.Position> blocked = new HashSet<>(blockedPositions);
        blocked.remove(start);
        blocked.remove(goal);

        PriorityQueue<Node> open = new PriorityQueue<>(Comparator
                .comparingInt(Node::estimate)
                .thenComparingInt(Node::cost));
        Map<InternalMap.Position, Integer> costs = new HashMap<>();
        Map<InternalMap.Position, InternalMap.Position> parents = new HashMap<>();

        costs.put(start, 0);
        open.add(new Node(start, 0, heuristic(start, goal)));

        int margin = blocked.size() + 1;
        int minX = Math.min(start.x(), goal.x()) - margin;
        int maxX = Math.max(start.x(), goal.x()) + margin;
        int minY = Math.min(start.y(), goal.y()) - margin;
        int maxY = Math.max(start.y(), goal.y()) + margin;

        while (!open.isEmpty()) {
            Node current = open.poll();
            if (!current.position().equals(goal)) {
                for (InternalMap.Position neighbor : neighbors(current.position())) {
                    if (!insideBounds(neighbor, minX, maxX, minY, maxY)
                            || blocked.contains(neighbor)) {
                        continue;
                    }

                    int newCost = current.cost() + 1;
                    if (newCost < costs.getOrDefault(neighbor, Integer.MAX_VALUE)) {
                        costs.put(neighbor, newCost);
                        parents.put(neighbor, current.position());
                        open.add(new Node(neighbor, newCost,
                                newCost + heuristic(neighbor, goal)));
                    }
                }
                continue;
            }
            return reconstructPath(parents, start, goal);
        }

        return List.of();
    }

    private List<InternalMap.Position> neighbors(InternalMap.Position position) {
        return List.of(
                new InternalMap.Position(position.x(), position.y() - 1),
                new InternalMap.Position(position.x() + 1, position.y()),
                new InternalMap.Position(position.x(), position.y() + 1),
                new InternalMap.Position(position.x() - 1, position.y()));
    }

    private int heuristic(InternalMap.Position first, InternalMap.Position second) {
        return Math.abs(first.x() - second.x()) + Math.abs(first.y() - second.y());
    }

    private boolean insideBounds(InternalMap.Position position,
                                 int minX, int maxX, int minY, int maxY) {
        return position.x() >= minX && position.x() <= maxX
                && position.y() >= minY && position.y() <= maxY;
    }

    private List<String> reconstructPath(Map<InternalMap.Position, InternalMap.Position> parents,
                                          InternalMap.Position start,
                                          InternalMap.Position goal) {
        List<String> path = new ArrayList<>();
        InternalMap.Position current = goal;
        while (!current.equals(start)) {
            InternalMap.Position parent = parents.get(current);
            if (parent == null) {
                return List.of();
            }
            path.add(directionFrom(parent, current));
            current = parent;
        }
        Collections.reverse(path);
        return path;
    }

    private String directionFrom(InternalMap.Position from, InternalMap.Position to) {
        if (to.x() > from.x()) return "e";
        if (to.x() < from.x()) return "w";
        if (to.y() > from.y()) return "s";
        return "n";
    }
}