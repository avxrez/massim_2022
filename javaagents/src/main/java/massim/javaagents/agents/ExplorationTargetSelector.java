package massim.javaagents.agents;

import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class ExplorationTargetSelector {

        // Mindestabstand zwischen den Erkundungszielen der Agents.
        private static final int MIN_TARGET_DISTANCE = 30;

        public InternalMap.Position selectTarget(InternalMap internalMap,
                                                                                         Map<String, InternalMap.Position> knownTargets,
                                                                                         String agentName) {

        int agentX = internalMap.getAgentX();
        int agentY = internalMap.getAgentY();
        List<InternalMap.Observation> observations = internalMap.getObservations();
        Set<InternalMap.Position> knownPositions = new HashSet<>();
        for (InternalMap.Observation observation : observations) {
                knownPositions.add(new InternalMap.Position(observation.x(), observation.y()));
        }
        Set<InternalMap.Position> blockedPositions =
                new HashSet<>(internalMap.getBlockedPositions());

        return observations.stream()

                // Von jedem bekannten Feld aus die 4 Nachbarn anschauen
                .flatMap(observation -> directions().stream()
                        .map(direction -> new InternalMap.Position(
                                observation.x() + direction.x(),
                                observation.y() + direction.y()
                        ))
                )

                // Nur unbekannte Felder
                .filter(position ->
                        !knownPositions.contains(position)
                )

                // The agent's current cell is not an exploration target.
                .filter(position ->
                        position.x() != agentX || position.y() != agentY
                )

                // Nicht direkt auf ein Hindernis gehen
                .filter(position ->
                        !blockedPositions.contains(position)
                )

                // Agents with a higher number must keep at least ten blocks
                // distance from the target of a lower-numbered agent.
                .filter(position ->
                        isTargetAllowed(position, knownTargets, agentName)
                )

                // Nächstes unbekanntes Feld auswählen
                .min(Comparator.comparingInt(position ->
                        Math.abs(position.x() - agentX)
                        + Math.abs(position.y() - agentY)
                ))

                // Falls nichts gefunden wurde
                                .orElseGet(() -> fallbackTarget(
                                                internalMap, agentX, agentY, knownTargets, agentName));
    }

        private InternalMap.Position fallbackTarget(InternalMap internalMap,
                                                                                                int agentX,
                                                int agentY,
                                                Map<String, InternalMap.Position> knownTargets,
                                                String agentName) {
        InternalMap.Position target = new InternalMap.Position(
                agentX + MIN_TARGET_DISTANCE, agentY);
                while (internalMap.isKnownPosition(target.x(), target.y())
                                || !isTargetAllowed(target, knownTargets, agentName)) {
            target = new InternalMap.Position(
                    target.x() + MIN_TARGET_DISTANCE, target.y());
        }
        return target;
    }

        public boolean isTargetAllowed(InternalMap.Position target,
                                                                   Map<String, InternalMap.Position> knownTargets,
                                                                   String agentName) {
                return knownTargets.entrySet().stream().noneMatch(entry ->
                                distance(target, entry.getValue()) < MIN_TARGET_DISTANCE
                                                && nameNumber(agentName) > nameNumber(entry.getKey())
                );
        }

        private int distance(InternalMap.Position first, InternalMap.Position second) {
                return Math.abs(first.x() - second.x()) + Math.abs(first.y() - second.y());
        }

        private int nameNumber(String name) {
                String number = name.replaceAll("[^0-9]", "");
                return number.isEmpty() ? -1 : Integer.parseInt(number);
        }

    private List<InternalMap.Position> directions() {

        return List.of(
                new InternalMap.Position(0, -1), // Norden
                new InternalMap.Position(1, 0),  // Osten
                new InternalMap.Position(0, 1),  // Süden
                new InternalMap.Position(-1, 0)  // Westen
        );
    }
}