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
 * Finds shortest paths for an agent moving alone and cost-aware paths for an
 * agent carrying an attached block.
 */
public class AStarPathPlanner {

    private static final int MAX_SEARCH_MARGIN = 10;

    private static final int CARRYING_ROTATION_COST = 4;
    private static final int CARRYING_FALLBACK_ROTATION_COST = 1;

    private static final int CARRYING_STRAIGHT_MOVE_COST = 10;
    private static final int CARRYING_FALLBACK_MOVE_COST = 3;
    private static final int CARRYING_DIRECTION_CHANGE_COST = 30;
    private static final int CARRYING_FALLBACK_DIRECTION_CHANGE_COST = 12;

    private static final Comparator<Node> BY_ESTIMATE_THEN_COST =
            Comparator.comparingInt(Node::estimate).thenComparingInt(Node::cost);

    private record Node(InternalMap.Position position, int cost, int estimate) {}

    private record CarryState(InternalMap.Position position, String blockDirection, String lastMoveDirection) {}

    private record CarryNode(CarryState state, int cost, int estimate) {}

    /**
     * Finds a path while avoiding blocked, occupied, and forbidden agent cells.
     * If static obstacles prevent a route, retries without those obstacles while
     * continuing to respect occupied and forbidden cells.
     *
     * @param start agent's starting position
     * @param goal destination position
     * @param blockedPositions known static obstacles
     * @param occupiedPositions currently occupied cells
     * @param forbiddenAgentPositions cells the agent must not enter
     * @return movement directions from start to goal, or an empty list if no path exists
     */
    public List<String> findPath(InternalMap.Position start,
                                 InternalMap.Position goal,
                                 List<InternalMap.Position> blockedPositions,
                                 Set<InternalMap.Position> occupiedPositions,
                                 Set<InternalMap.Position> forbiddenAgentPositions) {
        Set<InternalMap.Position> blocked = new HashSet<>(blockedPositions);
        blocked.addAll(occupiedPositions);
        blocked.addAll(forbiddenAgentPositions);
        blocked.remove(start);

        int margin = Math.min(blocked.size() + 1, MAX_SEARCH_MARGIN);
        List<String> path = search(start, goal, blocked, margin);
        if (path != null) {
            return path;
        }

        Set<InternalMap.Position> fallbackBlocked = new HashSet<>(occupiedPositions);
        fallbackBlocked.addAll(forbiddenAgentPositions);
        fallbackBlocked.remove(start);
        List<String> fallbackPath = search(start, goal, fallbackBlocked, MAX_SEARCH_MARGIN);
        return fallbackPath != null ? fallbackPath : List.of();
    }

    /**
     * Finds a path that brings the carried block to {@code goal}.
     * The agent may rotate the block and move only when the attached block remains
     * behind it; turns and direction changes have configurable search costs.
     *
     * @param start agent's starting position
     * @param goal destination position for the block
     * @param blockDirection direction of the block relative to the agent
     * @param blockedPositions known static obstacles
     * @param occupiedPositions currently occupied cells
     * @param forbiddenAgentPositions cells the agent must not enter
     * @return movement and rotation steps, or an empty list if no path exists
     */
    public List<String> findCarryingPath(InternalMap.Position start,
                                         InternalMap.Position goal,
                                         String blockDirection,
                                         List<InternalMap.Position> blockedPositions,
                                         Set<InternalMap.Position> occupiedPositions,
                                         Set<InternalMap.Position> forbiddenAgentPositions) {
        return findCarryingPath(start, goal, blockDirection, blockedPositions,
                occupiedPositions, forbiddenAgentPositions, null, false, false);
    }

    /**
     * Finds a carrying path to an agent position with the block in a required
     * relative direction.
     *
     * @param start agent's starting position
     * @param goal destination position for the agent
     * @param blockDirection current direction of the attached block
     * @param requiredBlockDirection required block direction at the destination
     * @param blockedPositions known static obstacles
     * @param occupiedPositions currently occupied cells
     * @param forbiddenAgentPositions cells the agent must not enter
     * @return movement and rotation steps, or an empty list if no path exists
     */
    public List<String> findCarryingPathToAgentPosition(InternalMap.Position start,
                                                         InternalMap.Position goal,
                                                         String blockDirection,
                                                         String requiredBlockDirection,
                                                         List<InternalMap.Position> blockedPositions,
                                                         Set<InternalMap.Position> occupiedPositions,
                                                         Set<InternalMap.Position> forbiddenAgentPositions) {
        return findCarryingPath(start, goal, blockDirection, blockedPositions,
            occupiedPositions, forbiddenAgentPositions, requiredBlockDirection, true, false);
    }

    /**
     * Shared carrying search. In fallback mode static obstacles are ignored,
     * while occupied cells and forbidden agent positions remain impassable.
     */
    private List<String> findCarryingPath(InternalMap.Position start,
                                           InternalMap.Position goal,
                                           String blockDirection,
                                           List<InternalMap.Position> blockedPositions,
                                           Set<InternalMap.Position> occupiedPositions,
                                           Set<InternalMap.Position> forbiddenAgentPositions,
                                           String requiredBlockDirection,
                                           boolean goalIsAgentPosition,
                                           boolean fallback) {
        Set<InternalMap.Position> blocked = fallback
                ? new HashSet<>() : new HashSet<>(blockedPositions);
        blocked.addAll(occupiedPositions);
        blocked.remove(start);
        Set<InternalMap.Position> forbiddenForAgent = new HashSet<>(forbiddenAgentPositions);
        forbiddenForAgent.remove(start);
        CarryState startState = new CarryState(start, blockDirection, null);
        PriorityQueue<CarryNode> open = new PriorityQueue<>(
                Comparator.comparingInt(CarryNode::estimate).thenComparingInt(CarryNode::cost));
        Map<CarryState, Integer> costs = new HashMap<>();
        Map<CarryState, CarryState> parents = new HashMap<>();
        Map<CarryState, String> actions = new HashMap<>();
        int margin = fallback ? MAX_SEARCH_MARGIN
            : Math.min(blocked.size() + 1, MAX_SEARCH_MARGIN);

        costs.put(startState, 0);
        open.add(new CarryNode(startState, 0, AgentUtils.manhattanDistance(start, goal)));
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
            if (reachedGoal && !blocked.contains(blockPosition)) {
                return reconstructCarryingPath(parents, actions, startState, state);
            }

            for (boolean clockwise : List.of(true, false)) {
                String rotatedDirection = AgentUtils.rotateDirection(state.blockDirection(), clockwise);
                InternalMap.Position rotatedBlock = offsetPosition(state.position(), rotatedDirection);
                if (blocked.contains(rotatedBlock)) {
                    continue;
                }
                CarryState next = new CarryState(state.position(), rotatedDirection, state.lastMoveDirection());
                addCarryState(open, costs, parents, actions, state, next,
                    "rotate:" + (clockwise ? "cw" : "ccw"),
                    fallback ? CARRYING_FALLBACK_ROTATION_COST : CARRYING_ROTATION_COST, goal);
            }

            for (String direction : AgentUtils.CARDINAL_DIRECTIONS) {
                if (!AgentUtils.oppositeDirection(direction).equals(state.blockDirection())) {
                    continue;
                }
                int[] offset = AgentUtils.directionOffset(direction);
                InternalMap.Position nextPosition = new InternalMap.Position(
                        state.position().x() + offset[0], state.position().y() + offset[1]);
                InternalMap.Position nextBlockPosition = offsetPosition(nextPosition, state.blockDirection());
                if (!insideBounds(nextPosition, minX, maxX, minY, maxY)
                    || blocked.contains(nextPosition) || forbiddenForAgent.contains(nextPosition)
                    || blocked.contains(nextBlockPosition)) {
                    continue;
                }
                CarryState next = new CarryState(nextPosition, state.blockDirection(), direction);
                addCarryState(open, costs, parents, actions, state, next,
                        direction, moveCost(state.lastMoveDirection(), direction, fallback), goal);
            }
        }
        if (!fallback && !blocked.isEmpty()) {
            return findCarryingPath(start, goal, blockDirection, List.of(),
                occupiedPositions, forbiddenAgentPositions,
                requiredBlockDirection, goalIsAgentPosition, true);
        }
        return List.of();
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
                    newCost + AgentUtils.manhattanDistance(next.position(), goal) * 10));
        }
    }

    /** Returns the cost of continuing straight or changing the movement direction. */
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
        int[] offset = AgentUtils.directionOffset(direction);
        return new InternalMap.Position(position.x() + offset[0], position.y() + offset[1]);
    }

    /** Runs bounded A* search for an agent that is not carrying a block. */
    private List<String> search(InternalMap.Position start,
                                 InternalMap.Position goal,
                                 Set<InternalMap.Position> blocked,
                                 int margin) {
        PriorityQueue<Node> open = new PriorityQueue<>(BY_ESTIMATE_THEN_COST);
        Map<InternalMap.Position, Integer> costs = new HashMap<>();
        Map<InternalMap.Position, InternalMap.Position> parents = new HashMap<>();

        costs.put(start, 0);
        open.add(new Node(start, 0, AgentUtils.manhattanDistance(start, goal)));

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
                    open.add(new Node(neighbor, newCost,
                            newCost + AgentUtils.manhattanDistance(neighbor, goal)));
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

    private boolean insideBounds(InternalMap.Position position,
                                  int minX, int maxX, int minY, int maxY) {
        return position.x() >= minX && position.x() <= maxX
                && position.y() >= minY && position.y() <= maxY;
    }

    /** Reconstructs movement directions from the predecessor map. */
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
                    path.add(AgentUtils.directionFrom(parent, current));
            current = parent;
        }

        Collections.reverse(path);
        return path;
    }

}