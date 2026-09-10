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

/**
 * Calculates shortest four-directional paths on the agent's known map.
 *
 * The planner first tries to find a path that avoids all known obstacles.
 * If no such path exists within the search area, it falls back to a path
 * that ignores obstacles entirely, so the agent still has a direction to
 * move towards (e.g. into a cell it can then try to clear).
 */
public class AStarPathPlanner {

    /** Search-area margin (in cells) added around start/goal for the obstacle-free search. */
    private static final int MAX_SEARCH_MARGIN = 10;

    /** Search-area margin used for the fallback search that ignores obstacles. */
    private static final int FALLBACK_SEARCH_MARGIN = 10;

    private static final Comparator<Node> BY_ESTIMATE_THEN_COST =
            Comparator.comparingInt(Node::estimate).thenComparingInt(Node::cost);

    /** A node in the search frontier: a position plus its cost-so-far and estimated total cost. */
    private record Node(InternalMap.Position position, int cost, int estimate) {}

    /**
     * Finds the shortest path from {@code start} to {@code goal}.
     *
     * @param blockedPositions cells the path should avoid if at all possible
     * @return the path as a list of directions ("n"/"e"/"s"/"w"), or an empty
     *         list if no path could be found even while ignoring obstacles
     */
    public List<String> findPath(InternalMap.Position start,
                                  InternalMap.Position goal,
                                  List<InternalMap.Position> blockedPositions) {
        Set<InternalMap.Position> blocked = new HashSet<>(blockedPositions);
        blocked.remove(start);

        int margin = Math.min(blocked.size() + 1, MAX_SEARCH_MARGIN);
        List<String> path = search(start, goal, blocked, margin);
        if (path != null) {
            return path;
        }

        // Kein hindernisfreier Pfad gefunden: Notfallpfad berechnen, der
        // Hindernisse ignoriert, damit der Agent trotzdem eine Richtung hat.
        List<String> fallbackPath = search(start, goal, Set.of(), FALLBACK_SEARCH_MARGIN);
        return fallbackPath != null ? fallbackPath : List.of();
    }

    /**
     * Runs an A* search from {@code start} to {@code goal} within a bounding
     * box of {@code margin} cells around both points, treating any position
     * in {@code blocked} as impassable.
     *
     * @return the path as a list of directions, or {@code null} if the goal
     *         could not be reached within the search bounds
     */
    private List<String> search(InternalMap.Position start,
                                 InternalMap.Position goal,
                                 Set<InternalMap.Position> blocked,
                                 int margin) {
        PriorityQueue<Node> open = new PriorityQueue<>(BY_ESTIMATE_THEN_COST);
        Map<InternalMap.Position, Integer> costs = new HashMap<>();
        Map<InternalMap.Position, InternalMap.Position> parents = new HashMap<>();

        costs.put(start, 0);
        open.add(new Node(start, 0, heuristic(start, goal)));

        int minX = Math.min(start.x(), goal.x()) - margin;
        int maxX = Math.max(start.x(), goal.x()) + margin;
        int minY = Math.min(start.y(), goal.y()) - margin;
        int maxY = Math.max(start.y(), goal.y()) + margin;

        while (!open.isEmpty()) {
            Node current = open.poll();

            if (current.position().equals(goal)) {
                return reconstructPath(parents, start, goal);
            }

            for (InternalMap.Position neighbor : neighbors(current.position())) {
                if (!insideBounds(neighbor, minX, maxX, minY, maxY) || blocked.contains(neighbor)) {
                    continue;
                }

                int newCost = current.cost() + 1;
                if (newCost < costs.getOrDefault(neighbor, Integer.MAX_VALUE)) {
                    costs.put(neighbor, newCost);
                    parents.put(neighbor, current.position());
                    open.add(new Node(neighbor, newCost, newCost + heuristic(neighbor, goal)));
                }
            }
        }

        return null;
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

    /** Walks the {@code parents} chain from {@code goal} back to {@code start} and turns it into directions. */
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