package massim.javaagents.agents;

import java.util.Comparator;
import java.util.List;

public class ExplorationTargetSelector {

    public InternalMap.Position selectTarget(InternalMap internalMap) {

        int agentX = internalMap.getAgentX();
        int agentY = internalMap.getAgentY();

        return internalMap.getObservations().stream()

                // Von jedem bekannten Feld aus die 4 Nachbarn anschauen
                .flatMap(observation -> directions().stream()
                        .map(direction -> new InternalMap.Position(
                                observation.x() + direction.x(),
                                observation.y() + direction.y()
                        ))
                )

                // Nur unbekannte Felder
                .filter(position ->
                        !internalMap.isKnownPosition(
                                position.x(),
                                position.y()
                        )
                )

                // The agent's current cell is not an exploration target.
                .filter(position ->
                        position.x() != agentX || position.y() != agentY
                )

                // Nicht direkt auf ein Hindernis gehen
                .filter(position ->
                        !internalMap.getBlockedPositions()
                                .contains(position)
                )

                // Nächstes unbekanntes Feld auswählen
                .min(Comparator.comparingInt(position ->
                        Math.abs(position.x() - agentX)
                        + Math.abs(position.y() - agentY)
                ))

                // Falls nichts gefunden wurde
                .orElse(
                        new InternalMap.Position(
                                agentX + 1,
                                agentY
                        )
                );
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