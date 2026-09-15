package massim.javaagents.agents;

import eis.iilang.Function;
import eis.iilang.Identifier;
import eis.iilang.Numeral;
import eis.iilang.Parameter;
import eis.iilang.ParameterList;
import eis.iilang.Percept;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * The agent's internal world model: its own position and every cell it (or
 * a teammate) has observed so far, expressed in one shared coordinate
 * system.
 */
public class InternalMap {

    /** Position des Agenten auf der internen Karte. */
    private int agentX = 0;
    private int agentY = 0;

    /**
     * Alle aktuell bekannten Beobachtungen.
     *
     * Der Key stellt sicher, dass wir pro Position/Typ/Detail nicht
     * mehrere alte Informationen behalten.
     */
    private final Map<ObservationKey, Observation> observations = new HashMap<>();

    /** Index of observation keys by position, so replacing a cell is O(1) on average. */
    private final Map<Position, Set<ObservationKey>> keysByPosition = new HashMap<>();

    /** Zellen, auf denen aktuell (in diesem Schritt) eine Entity gesehen wurde. */
    private final Set<Position> occupiedEntityPositions = new HashSet<>();

    /** Zellen, auf denen aktuell sichtbare Teammitglieder stehen. */
    private final Set<Position> visibleTeammates = new HashSet<>();

    // -------------------------------------------------------------------------
    // Data classes
    // -------------------------------------------------------------------------

    public record Observation(String type, int x, int y, String details, int lastSeenStep) {}

    public record Position(int x, int y) {}

    private record ObservationKey(String type, int x, int y, String details) {}

    // -------------------------------------------------------------------------
    // Agent position
    // -------------------------------------------------------------------------

    public int getAgentX() {
        return agentX;
    }

    public int getAgentY() {
        return agentY;
    }

    public void setAgentPosition(int agentX, int agentY) {
        this.agentX = agentX;
        this.agentY = agentY;
    }

    /**
     * Aktualisiert die Position des Agenten nach einer erfolgreichen Bewegung.
     */
    public void updateAgentPosition(String direction) {
        switch (direction) {
            case "n" -> agentY--;
            case "s" -> agentY++;
            case "e" -> agentX++;
            case "w" -> agentX--;
            default -> throw new IllegalArgumentException("Invalid direction: " + direction);
        }
    }

    /** Aktualisiert die eigene Kartenposition aus dem Ergebnis des letzten Zuges. */
    public void updateAgentPositionFromPercepts(List<Percept> percepts) {
        String lastAction = null;
        String lastActionResult = null;
        String direction = null;

        for (Percept percept : percepts) {
            if (percept.getName().equals("lastAction") && !percept.getParameters().isEmpty()
                    && percept.getParameters().get(0) instanceof Identifier identifier) {
                lastAction = identifier.getValue();
            } else if (percept.getName().equals("lastActionResult") && !percept.getParameters().isEmpty()
                    && percept.getParameters().get(0) instanceof Identifier identifier) {
                lastActionResult = identifier.getValue();
            } else if (percept.getName().equals("lastActionParams")
                    && !percept.getParameters().isEmpty()
                    && percept.getParameters().get(0) instanceof ParameterList parameters
                    && parameters.size() == 1
                    && parameters.get(0) instanceof Identifier identifier) {
                direction = identifier.getValue();
            }
        }

        if ("move".equals(lastAction) && "success".equals(lastActionResult) && direction != null) {
            updateAgentPosition(direction);
        }
    }

    // -------------------------------------------------------------------------
    // Remember observations
    // -------------------------------------------------------------------------

    /**
     * Speichert eine beobachtete Zelle.
     *
     * Die übergebenen Koordinaten sind relativ zum Agenten. Intern werden
     * sie in absolute Koordinaten umgerechnet.
     */
    public void rememberObservation(String type, int relativeX, int relativeY, String details, int step) {
        int absoluteX = agentX + relativeX;
        int absoluteY = agentY + relativeY;
        String observationDetails = details == null ? "" : details;

        removeObservationsForUpdate(absoluteX, absoluteY, type);

        ObservationKey key = new ObservationKey(type, absoluteX, absoluteY, observationDetails);
        putObservation(key, new Observation(type, absoluteX, absoluteY, observationDetails, step));
    }

    /**
     * Speichert eine bekannte freie Zelle.
     */
    public void rememberFreeCell(int relativeX, int relativeY, int step) {
        rememberObservation("free", relativeX, relativeY, "", step);
    }

    public void clearOccupiedEntityPositions() {
        occupiedEntityPositions.clear();
    }

    public void rememberOccupiedEntity(int relativeX, int relativeY) {
        occupiedEntityPositions.add(new Position(agentX + relativeX, agentY + relativeY));
    }

    /** Aktualisiert die Karte und die aktuell sichtbaren Teammitglieder aus Percepts. */
    public void updateFromPercepts(List<Percept> percepts, String teamName) {
        int step = -1;
        int vision = -1;
        Set<Position> occupiedRelativePositions = new HashSet<>();

        for (Percept percept : percepts) {
            if (percept.getName().equals("step")
                    && !percept.getParameters().isEmpty()
                    && percept.getParameters().get(0) instanceof Numeral numeral) {
                step = numeral.getValue().intValue();
            } else if (percept.getName().equals("role")
                    && percept.getParameters().size() >= 2
                    && percept.getParameters().get(1) instanceof Numeral numeral) {
                vision = numeral.getValue().intValue();
            }
        }

        if (step < 0) {
            return;
        }

        clearOccupiedEntityPositions();
        visibleTeammates.clear();

        for (Percept percept : percepts) {
            if (percept.getName().equals("thing") && percept.getParameters().size() >= 3
                    && percept.getParameters().get(0) instanceof Numeral x
                    && percept.getParameters().get(1) instanceof Numeral y
                    && percept.getParameters().get(2) instanceof Identifier type) {
                int relativeX = x.getValue().intValue();
                int relativeY = y.getValue().intValue();
                occupiedRelativePositions.add(new Position(relativeX, relativeY));
                String details = percept.getParameters().size() > 3
                        && percept.getParameters().get(3) instanceof Identifier identifier
                        ? identifier.getValue() : "";

                if (type.getValue().equals("entity")) {
                    rememberFreeCell(relativeX, relativeY, step);
                    rememberOccupiedEntity(relativeX, relativeY);
                    if (!teamName.isEmpty() && !(relativeX == 0 && relativeY == 0)
                            && teamName.equals(details)) {
                        visibleTeammates.add(new Position(relativeX, relativeY));
                    }
                } else {
                    rememberObservation(type.getValue(), relativeX, relativeY, details, step);
                }
            } else if ((percept.getName().equals("goalZone") || percept.getName().equals("roleZone"))
                    && percept.getParameters().size() >= 2
                    && percept.getParameters().get(0) instanceof Numeral x
                    && percept.getParameters().get(1) instanceof Numeral y) {
                rememberObservation(percept.getName(), x.getValue().intValue(), y.getValue().intValue(), "", step);
            }
        }

        if (vision >= 0) {
            for (int x = -vision; x <= vision; x++) {
                for (int y = -vision; y <= vision; y++) {
                    if (Math.abs(x) + Math.abs(y) <= vision
                            && !occupiedRelativePositions.contains(new Position(x, y))) {
                        rememberFreeCell(x, y, step);
                    }
                }
            }
        }
    }

    public Set<Position> getVisibleTeammates() {
        return Set.copyOf(visibleTeammates);
    }

    /** Returns the positions currently occupied by visible entities. */
    public Set<Position> getOccupiedEntityPositions() {
        return Set.copyOf(occupiedEntityPositions);
    }

    /** Liest absolute Kartenbeobachtungen aus einer Nachrichtenliste ein. */
    public void mergeObservations(Parameter parameter) {
        if (!(parameter instanceof ParameterList map)) {
            return;
        }
        mergeObservations(parseObservations(map));
    }

    public void setObservations(Parameter parameter) {
        if (!(parameter instanceof ParameterList map)) {
            return;
        }
        setObservations(parseObservations(map));
    }

    /** Serialisiert die Karte für Agentennachrichten. */
    public ParameterList toParameterList() {
        ParameterList map = new ParameterList();
        for (Observation observation : observations.values()) {
            map.add(new Function("observation",
                    new Identifier(observation.type()),
                    new Numeral(observation.x()),
                    new Numeral(observation.y()),
                    new Identifier(observation.details()),
                    new Numeral(observation.lastSeenStep())));
        }
        return map;
    }

    /** Erstellt absolute Kartenbeobachtungen aus den aktuellen lokalen Percepts. */
    public ParameterList currentPercepts(List<Percept> percepts, int step) {
        ParameterList map = new ParameterList();
        for (Percept percept : percepts) {
            if (percept.getName().equals("thing") && percept.getParameters().size() >= 3
                    && percept.getParameters().get(0) instanceof Numeral x
                    && percept.getParameters().get(1) instanceof Numeral y
                    && percept.getParameters().get(2) instanceof Identifier type) {
                if (type.getValue().equals("entity")) {
                    continue;
                }
                String details = percept.getParameters().size() > 3
                        && percept.getParameters().get(3) instanceof Identifier identifier
                        ? identifier.getValue() : "";
                map.add(observationParameter(type.getValue(),
                        agentX + x.getValue().intValue(), agentY + y.getValue().intValue(),
                        details, step));
            } else if ((percept.getName().equals("goalZone") || percept.getName().equals("roleZone"))
                    && percept.getParameters().size() >= 2
                    && percept.getParameters().get(0) instanceof Numeral x
                    && percept.getParameters().get(1) instanceof Numeral y) {
                map.add(observationParameter(percept.getName(),
                        agentX + x.getValue().intValue(), agentY + y.getValue().intValue(), "", step));
            }
        }
        return map;
    }

    private static Function observationParameter(String type, int x, int y, String details, int step) {
        return new Function("observation", new Identifier(type), new Numeral(x), new Numeral(y),
                new Identifier(details), new Numeral(step));
    }

    private static List<Observation> parseObservations(ParameterList map) {
        List<Observation> parsed = new ArrayList<>();
        for (Parameter entry : map) {
            if (entry instanceof Function observation
                    && observation.getName().equals("observation")
                    && observation.getParameters().size() >= 5
                    && observation.getParameters().get(0) instanceof Identifier type
                    && observation.getParameters().get(1) instanceof Numeral x
                    && observation.getParameters().get(2) instanceof Numeral y
                    && observation.getParameters().get(3) instanceof Identifier details
                    && observation.getParameters().get(4) instanceof Numeral step) {
                parsed.add(new Observation(type.getValue(), x.getValue().intValue(), y.getValue().intValue(),
                        details.getValue(), step.getValue().intValue()));
            }
        }
        return parsed;
    }

    // -------------------------------------------------------------------------
    // Query map
    // -------------------------------------------------------------------------

    /**
     * Gibt alle aktuell bekannten Beobachtungen zurück.
     */
    public List<Observation> getObservations() {
        return Collections.unmodifiableList(new ArrayList<>(observations.values()));
    }

    /**
     * Fügt Beobachtungen einer bereits bekannten Karte mit absoluten
     * Koordinaten in diese Karte ein.
     */
    public void mergeObservations(List<Observation> observationsToMerge) {
        for (Observation observation : observationsToMerge) {
            removeObservationsForUpdate(observation.x(), observation.y(), observation.type());

            String details = observation.details() == null ? "" : observation.details();
            ObservationKey key = new ObservationKey(observation.type(), observation.x(), observation.y(), details);
            putObservation(key, new Observation(
                    observation.type(), observation.x(), observation.y(), details, observation.lastSeenStep()));
        }
    }

    /** Replaces the complete observation map with absolute coordinates. */
    public void setObservations(List<Observation> newObservations) {
        observations.clear();
        keysByPosition.clear();
        occupiedEntityPositions.clear();

        for (Observation observation : newObservations) {
            String details = observation.details() == null ? "" : observation.details();
            ObservationKey key = new ObservationKey(observation.type(), observation.x(), observation.y(), details);
            putObservation(key, new Observation(
                    observation.type(), observation.x(), observation.y(), details, observation.lastSeenStep()));
        }
    }

    /**
     * Gibt alle Positionen zurück, die aktuell als blockiert gelten.
     *
     * obstacle:    tatsächlich beobachtetes Hindernis
     * failedPath:  Position, die beim letzten Versuch nicht betreten werden konnte
     *
     * (Zusätzlich werden Zellen mit einer aktuell sichtbaren Entity als
     * blockiert gezählt.)
     */
    public List<Position> getBlockedPositions() {
        Set<Position> blockedPositions = new HashSet<>(occupiedEntityPositions);
        observations.values().stream()
                .filter(observation -> observation.type().equals("obstacle")
                        || observation.type().equals("failedPath"))
                .map(observation -> new Position(observation.x(), observation.y()))
                .forEach(blockedPositions::add);
        return List.copyOf(blockedPositions);
    }

    /**
     * Prüft, ob eine absolute Position bereits bekannt ist.
     */
    public boolean isKnownPosition(int x, int y) {
        return observations.values().stream().anyMatch(observation -> observation.x() == x && observation.y() == y);
    }

    // -------------------------------------------------------------------------
    // Failed paths
    // -------------------------------------------------------------------------

    /**
     * Merkt sich eine Position, die nicht betreten werden konnte.
     */
    public void rememberFailedPath(int x, int y, int step) {
        removeBlockedObservationsAt(x, y);
        ObservationKey key = new ObservationKey("failedPath", x, y, "");
        putObservation(key, new Observation("failedPath", x, y, "", step));
    }

    // -------------------------------------------------------------------------
    // Remove / update information
    // -------------------------------------------------------------------------

    /**
     * Entfernt alle bisherigen Hindernis-Informationen über eine Position
     * und markiert die Position anschließend als frei.
     *
     * Wird beispielsweise nach erfolgreichem Clear verwendet.
     */
    public void forgetObservationsAt(int x, int y) {
        removeBlockedObservationsAt(x, y);
        rememberFreeCell(x - agentX, y - agentY, 0);
    }

    /** Entfernt Hindernis-/failedPath-Beobachtungen an einer absoluten Position. */
    private void removeBlockedObservationsAt(int x, int y) {
        Position position = new Position(x, y);
        Set<ObservationKey> keys = keysByPosition.get(position);
        if (keys == null) {
            return;
        }
        for (ObservationKey key : new ArrayList<>(keys)) {
            if (isBlocked(key.type())) {
                removeObservation(key);
            }
        }
    }

    /**
     * Entfernt bestehende Beobachtungen an einer Position, die durch eine
     * neue Beobachtung vom Typ {@code newType} ersetzt werden. Zonen
     * (goalZone/roleZone) werden dabei nie entfernt, da sie unabhängig von
     * anderen Beobachtungen an derselben Position weiter gelten.
     */
    private void removeObservationsForUpdate(int x, int y, String newType) {
        Position position = new Position(x, y);
        Set<ObservationKey> keys = keysByPosition.get(position);
        if (keys == null) {
            return;
        }
        for (ObservationKey key : new ArrayList<>(keys)) {
            if (!mustKeepTogether(key.type(), newType)) {
                removeObservation(key);
            }
        }
    }

    private void putObservation(ObservationKey key, Observation observation) {
        observations.put(key, observation);
        keysByPosition.computeIfAbsent(
                new Position(key.x(), key.y()), ignored -> new HashSet<>()).add(key);
    }

    private void removeObservation(ObservationKey key) {
        observations.remove(key);
        Position position = new Position(key.x(), key.y());
        Set<ObservationKey> keys = keysByPosition.get(position);
        if (keys != null) {
            keys.remove(key);
            if (keys.isEmpty()) {
                keysByPosition.remove(position);
            }
        }
    }

    private boolean mustKeepTogether(String firstType, String secondType) {
        return isZone(firstType) || isZone(secondType);
    }

    private boolean isZone(String type) {
        return type.equals("goalZone") || type.equals("roleZone");
    }

    private boolean isBlocked(String type) {
        return type.equals("obstacle") || type.equals("failedPath");
    }

    /**
     * Verschiebt die gesamte interne Karte um einen Offset.
     *
     * Alle bekannten Koordinaten und auch die eigene Agentenposition
     * werden entsprechend verschoben.
     */
    public void translate(int offsetX, int offsetY) {
        agentX += offsetX;
        agentY += offsetY;

        Map<ObservationKey, Observation> translatedObservations = new HashMap<>();
        for (Observation observation : observations.values()) {
            int newX = observation.x() + offsetX;
            int newY = observation.y() + offsetY;

            Observation translated = new Observation(
                    observation.type(), newX, newY, observation.details(), observation.lastSeenStep());
            ObservationKey key = new ObservationKey(translated.type(), newX, newY, translated.details());

            translatedObservations.put(key, translated);
        }
        observations.clear();
        observations.putAll(translatedObservations);

        keysByPosition.clear();
        for (ObservationKey key : translatedObservations.keySet()) {
            keysByPosition.computeIfAbsent(
                new Position(key.x(), key.y()), ignored -> new HashSet<>()).add(key);
        }

        Set<Position> translatedEntities = new HashSet<>();
        for (Position position : occupiedEntityPositions) {
            translatedEntities.add(new Position(position.x() + offsetX, position.y() + offsetY));
        }
        occupiedEntityPositions.clear();
        occupiedEntityPositions.addAll(translatedEntities);
    }
}