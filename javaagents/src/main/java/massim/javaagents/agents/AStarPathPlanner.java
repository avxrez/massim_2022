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

    /** Rotation is cheap in the carrying fallback so the agent can re-align the block easily. */
    private static final int CARRYING_ROTATION_COST = 4;
    private static final int CARRYING_FALLBACK_ROTATION_COST = 1;

    /** Straight movement is the cheapest option; changing direction is intentionally more expensive. */
    private static final int CARRYING_STRAIGHT_MOVE_COST = 10;
    private static final int CARRYING_FALLBACK_MOVE_COST = 3;
    private static final int CARRYING_DIRECTION_CHANGE_COST = 30;
    private static final int CARRYING_FALLBACK_DIRECTION_CHANGE_COST = 12;

    private static final Comparator<Node> BY_ESTIMATE_THEN_COST =
            Comparator.comparingInt(Node::estimate).thenComparingInt(Node::cost);

    /** A node in the search frontier: a position plus its cost-so-far and estimated total cost. */
    private record Node(InternalMap.Position position, int cost, int estimate) {}

    private record CarryState(InternalMap.Position position, String blockDirection, String lastMoveDirection) {}

    private record CarryNode(CarryState state, int cost, int estimate) {}

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
        return findPath(start, goal, blockedPositions, Set.of());
    }

    public List<String> findPath(InternalMap.Position start,
                                 InternalMap.Position goal,
                                 List<InternalMap.Position> blockedPositions,
                                 Set<InternalMap.Position> occupiedPositions) {
        Set<InternalMap.Position> blocked = new HashSet<>(blockedPositions);
        blocked.addAll(occupiedPositions);
        blocked.remove(start);

        int margin = Math.min(blocked.size() + 1, MAX_SEARCH_MARGIN);
        List<String> path = search(start, goal, blocked, margin);
        if (path != null) {
            return path;
        }

        // Kein hindernisfreier Pfad gefunden: Notfallpfad berechnen, der
        // Hindernisse ignoriert, damit der Agent trotzdem eine Richtung hat.
        Set<InternalMap.Position> fallbackBlocked = new HashSet<>(occupiedPositions);
        fallbackBlocked.remove(start);
        List<String> fallbackPath = search(start, goal, fallbackBlocked, FALLBACK_SEARCH_MARGIN);
        return fallbackPath != null ? fallbackPath : List.of();
    }

    /** Finds a path for an agent carrying one block, including required rotations. */
    public List<String> findCarryingPath(InternalMap.Position start,
                                          InternalMap.Position goal,
                                          String blockDirection,
                                          List<InternalMap.Position> blockedPositions) {
        return findCarryingPath(start, goal, blockDirection, blockedPositions,
            Set.of(), null, false, true);
    }

    public List<String> findCarryingPath(InternalMap.Position start,
                                         InternalMap.Position goal,
                                         String blockDirection,
                                         List<InternalMap.Position> blockedPositions,
                                         Set<InternalMap.Position> occupiedPositions) {
        return findCarryingPath(start, goal, blockDirection, blockedPositions,
            occupiedPositions, null, false, true);
    }

    /** Finds a carrying path that ends with the agent at the goal and the block at the required side. */
    public List<String> findCarryingPathToAgentPosition(InternalMap.Position start,
                                                         InternalMap.Position goal,
                                                         String blockDirection,
                                                         String requiredBlockDirection,
                                                         List<InternalMap.Position> blockedPositions,
                                                         Set<InternalMap.Position> occupiedPositions) {
        return findCarryingPath(start, goal, blockDirection, blockedPositions,
            occupiedPositions, requiredBlockDirection, true, true);
    }

    private List<String> findCarryingPath(InternalMap.Position start,
                                           InternalMap.Position goal,
                                           String blockDirection,
                                           List<InternalMap.Position> blockedPositions,
                                           Set<InternalMap.Position> occupiedPositions,
                                           String requiredBlockDirection,
                                           boolean goalIsAgentPosition,
                                           boolean fallback) {
        Set<InternalMap.Position> blocked = fallback
                    ? new HashSet<>() : new HashSet<>(blockedPositions);
        blocked.addAll(occupiedPositions);
        blocked.remove(start);
        CarryState startState = new CarryState(start, blockDirection, null);
        PriorityQueue<CarryNode> open = new PriorityQueue<>(
                Comparator.comparingInt(CarryNode::estimate).thenComparingInt(CarryNode::cost));
        Map<CarryState, Integer> costs = new HashMap<>();
        Map<CarryState, CarryState> parents = new HashMap<>();
        Map<CarryState, String> actions = new HashMap<>();
        int margin = fallback ? FALLBACK_SEARCH_MARGIN
            : Math.min(blocked.size() + 1, MAX_SEARCH_MARGIN);

        costs.put(startState, 0);
        open.add(new CarryNode(startState, 0, heuristic(start, goal)));
        int minX = Math.min(start.x(), goal.x()) - margin;
        int maxX = Math.max(start.x(), goal.x()) + margin;
        int minY = Math.min(start.y(), goal.y()) - margin;
        int maxY = Math.max(start.y(), goal.y()) + margin;

        while (!open.isEmpty()) {
            CarryNode current = open.poll();
            CarryState state = current.state();
            InternalMap.Position blockPosition = offsetPosition(state.position(), state.blockDirection());
                boolean reachedGoal = goalIsAgentPosition
                    ? state.position().equals(goal)
                    && state.blockDirection().equals(requiredBlockDirection)
                    : blockPosition.equals(goal);
                if (reachedGoal && isCarryGoalValid(state, blocked, occupiedPositions)) {
                return reconstructCarryingPath(parents, actions, startState, state);
            }

            for (boolean clockwise : List.of(true, false)) {
                String rotatedDirection = rotateDirection(state.blockDirection(), clockwise);
                InternalMap.Position rotatedBlock = offsetPosition(state.position(), rotatedDirection);
                if (blocked.contains(rotatedBlock)) {
                    continue;
                }
                CarryState next = new CarryState(state.position(), rotatedDirection, state.lastMoveDirection());
                addCarryState(open, costs, parents, actions, state, next,
                    "rotate:" + (clockwise ? "cw" : "ccw"),
                    fallback ? CARRYING_FALLBACK_ROTATION_COST : CARRYING_ROTATION_COST, goal);
            }

            for (String direction : List.of("n", "e", "s", "w")) {
                if (!oppositeDirection(direction).equals(state.blockDirection())) {
                    continue;
                }
                int[] offset = directionOffset(direction);
                InternalMap.Position nextPosition = new InternalMap.Position(
                        state.position().x() + offset[0], state.position().y() + offset[1]);
                InternalMap.Position nextBlockPosition = offsetPosition(nextPosition, state.blockDirection());
                if (!insideBounds(nextPosition, minX, maxX, minY, maxY)
                        || blocked.contains(nextPosition) || blocked.contains(nextBlockPosition)) {
                    continue;
                }
                CarryState next = new CarryState(nextPosition, state.blockDirection(), direction);
                addCarryState(open, costs, parents, actions, state, next,
                        direction, moveCost(state.lastMoveDirection(), direction, fallback), goal);
            }
        }
        if (!fallback && !blocked.isEmpty()) {
            return findCarryingPath(start, goal, blockDirection, List.of(),
                occupiedPositions, requiredBlockDirection, goalIsAgentPosition, true);
        }
        return List.of();
    }

    private boolean isCarryGoalValid(CarryState state,
                                    Set<InternalMap.Position> blocked,
                                    Set<InternalMap.Position> occupiedPositions) {
        InternalMap.Position blockPosition = offsetPosition(state.position(), state.blockDirection());
        return !blocked.contains(blockPosition) && !occupiedPositions.contains(blockPosition);
    }

    private void addCarryState(PriorityQueue<CarryNode> open,
                               Map<CarryState, Integer> costs,
                               Map<CarryState, CarryState> parents,
                               Map<CarryState, String> actions,
                               CarryState current,
                               CarryState next,
                               String action,
                               int actionCost,
                               InternalMap.Position goal) {
        int newCost = costs.get(current) + actionCost;
        if (newCost < costs.getOrDefault(next, Integer.MAX_VALUE)) {
            costs.put(next, newCost);
            parents.put(next, current);
            actions.put(next, action);
            open.add(new CarryNode(next, newCost,
                    newCost + heuristic(next.position(), goal) * 10));
        }
    }

    private int moveCost(String previousDirection, String currentDirection, boolean fallback) {
        int baseCost = fallback ? CARRYING_FALLBACK_MOVE_COST : CARRYING_STRAIGHT_MOVE_COST;
        if (previousDirection == null || previousDirection.equals(currentDirection)) {
            return baseCost;
        }
        return baseCost + (fallback ? CARRYING_FALLBACK_DIRECTION_CHANGE_COST
                : CARRYING_DIRECTION_CHANGE_COST);
    }

    private List<String> reconstructCarryingPath(Map<CarryState, CarryState> parents,
                                                  Map<CarryState, String> actions,
                                                  CarryState start,
                                                  CarryState goal) {
        List<String> path = new ArrayList<>();
        CarryState current = goal;
        while (!current.equals(start)) {
            CarryState parent = parents.get(current);
            if (parent == null) {
                return List.of();
            }
            path.add(actions.get(current));
            current = parent;
        }
        Collections.reverse(path);
        return path;
    }

    private InternalMap.Position offsetPosition(InternalMap.Position position, String direction) {
        int[] offset = directionOffset(direction);
        return new InternalMap.Position(position.x() + offset[0], position.y() + offset[1]);
    }

    private String rotateDirection(String direction, boolean clockwise) {
        return switch (direction) {
            case "n" -> clockwise ? "e" : "w";
            case "e" -> clockwise ? "s" : "n";
            case "s" -> clockwise ? "w" : "e";
            case "w" -> clockwise ? "n" : "s";
            default -> throw new IllegalArgumentException("Invalid direction: " + direction);
        };
    }

    private String oppositeDirection(String direction) {
        return switch (direction) {
            case "n" -> "s";
            case "e" -> "w";
            case "s" -> "n";
            case "w" -> "e";
            default -> throw new IllegalArgumentException("Invalid direction: " + direction);
        };
    }

    private int[] directionOffset(String direction) {
        return switch (direction) {
            case "n" -> new int[]{0, -1};
            case "e" -> new int[]{1, 0};
            case "s" -> new int[]{0, 1};
            case "w" -> new int[]{-1, 0};
            default -> throw new IllegalArgumentException("Invalid direction: " + direction);
        };
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