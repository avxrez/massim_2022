package massim.javaagents.agents;

import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Selects unexplored targets while keeping agents from repeatedly exploring
 * the same part of the map.
 */
public class ExplorationTargetSelector {

    private static final int MIN_TARGET_DISTANCE = 30;

    private static final List<InternalMap.Position> DIRECTIONS = List.of(
            new InternalMap.Position(0, -1),
            new InternalMap.Position(1, 0),
            new InternalMap.Position(0, 1),
            new InternalMap.Position(-1, 0)
    );

        /**
         * Selects the nearest eligible frontier next to a known observation.
         * If no such frontier exists, returns a distant fallback target.
         *
         * @param internalMap current map and agent position
         * @param knownTargets targets currently assigned to other agents
         * @param agentName name used to resolve target ownership priority
         * @return an unexplored target position
         */
        public InternalMap.Position selectTarget(InternalMap internalMap,
            Map<String, InternalMap.Position> knownTargets,
            String agentName) {
        int agentX = internalMap.getAgentX();
        int agentY = internalMap.getAgentY();

        List<InternalMap.Observation> observations = internalMap.getObservations();
        Set<InternalMap.Position> blockedPositions = new HashSet<>(internalMap.getBlockedPositions());

        return observations.stream()
                .flatMap(observation -> DIRECTIONS.stream()
                        .map(direction -> new InternalMap.Position(
                                observation.x() + direction.x(),
                                observation.y() + direction.y())))
                .filter(position -> !internalMap.isKnownPosition(position.x(), position.y()))
                .filter(position -> position.x() != agentX || position.y() != agentY)
                .filter(position -> !blockedPositions.contains(position))
                .filter(position -> isTargetAllowed(position, knownTargets, agentName))
                .min(Comparator.comparingInt(position ->
                        AgentUtils.manhattanDistance(position.x(), position.y(), agentX, agentY)))
                .orElseGet(() -> fallbackTarget(internalMap, agentX, agentY, knownTargets, agentName));
    }

        /**
         * Finds a distant target when all nearby frontiers are known or reserved.
         *
         * @param internalMap current map
         * @param agentX current agent x-coordinate
         * @param agentY current agent y-coordinate
         * @param knownTargets targets assigned to other agents
         * @param agentName name used to resolve target ownership priority
         * @return a target outside the known map and other agents' protected ranges
         */
        private InternalMap.Position fallbackTarget(InternalMap internalMap,
            int agentX,
            int agentY,
            Map<String, InternalMap.Position> knownTargets,
            String agentName) {
        InternalMap.Position target = new InternalMap.Position(agentX + MIN_TARGET_DISTANCE, agentY);

        while (internalMap.isKnownPosition(target.x(), target.y())
                || !isTargetAllowed(target, knownTargets, agentName)) {
            target = new InternalMap.Position(target.x() + MIN_TARGET_DISTANCE, target.y());
        }

        return target;
    }

        /**
         * Checks whether a target is outside the exclusion range of higher-priority
         * known agents.
         *
         * @param target candidate target position
         * @param knownTargets targets assigned to other agents
         * @param agentName name used to determine this agent's priority
         * @return {@code true} if this agent may use the target
         */
        public boolean isTargetAllowed(InternalMap.Position target,
            Map<String, InternalMap.Position> knownTargets,
            String agentName) {
        return knownTargets.entrySet().stream().noneMatch(entry ->
                AgentUtils.manhattanDistance(target, entry.getValue()) < MIN_TARGET_DISTANCE
                        && AgentUtils.nameNumber(agentName) > AgentUtils.nameNumber(entry.getKey()));
    }
}