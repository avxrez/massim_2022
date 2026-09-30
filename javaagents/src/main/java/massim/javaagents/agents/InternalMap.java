package massim.javaagents.agents;

import eis.iilang.Function;
import eis.iilang.Identifier;
import eis.iilang.Numeral;
import eis.iilang.Parameter;
import eis.iilang.ParameterList;
import eis.iilang.Percept;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class InternalMap {

    private static final int DEFAULT_VISION = 5;

    private int agentX = 0;
    private int agentY = 0;

    private final Map<ObservationKey, Observation> observations = new HashMap<>();

    private final Map<Position, Set<ObservationKey>> keysByPosition = new HashMap<>();

    private List<Observation> observationsSnapshot;
    private List<Position> blockedPositionsSnapshot;
    private Set<Position> knownPositionsSnapshot;

    private final Set<Position> occupiedEntityPositions = new HashSet<>();

    private final Set<Position> visibleTeammates = new HashSet<>();

    public record Observation(String type, int x, int y, String details, int lastSeenStep) {}

    public record Position(int x, int y) {}

    private record ObservationKey(String type, int x, int y, String details) {}

    public int getAgentX() {
        return agentX;
    }

    public int getAgentY() {
        return agentY;
    }

    public void updateAgentPosition(String direction) {
        switch (direction) {
            case "n" -> agentY--;
            case "s" -> agentY++;
            case "e" -> agentX++;
            case "w" -> agentX--;
            default -> throw new IllegalArgumentException("Invalid direction: " + direction);
        }
    }

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

    public void rememberObservation(String type, int relativeX, int relativeY, String details, int step) {
        int absoluteX = agentX + relativeX;
        int absoluteY = agentY + relativeY;
        String observationDetails = details == null ? "" : details;

        removeObservationsForUpdate(absoluteX, absoluteY, type);

        ObservationKey key = new ObservationKey(type, absoluteX, absoluteY, observationDetails);
        putObservation(key, new Observation(type, absoluteX, absoluteY, observationDetails, step));
    }

    public void rememberFreeCell(int relativeX, int relativeY, int step) {
        rememberObservation("free", relativeX, relativeY, "", step);
    }

    public void clearOccupiedEntityPositions() {
        occupiedEntityPositions.clear();
        blockedPositionsSnapshot = null;
    }

    public void forgetGoalZone(Position position) {
        removeObservationTypeAt(position.x(), position.y(), "goalZone");
    }

    public void rememberOccupiedEntity(int relativeX, int relativeY) {
        occupiedEntityPositions.add(new Position(agentX + relativeX, agentY + relativeY));
        blockedPositionsSnapshot = null;
    }

    public void updateFromPercepts(List<Percept> percepts, String teamName) {
        int step = -1;
        int vision = -1;
        Set<Position> occupiedRelativePositions = new HashSet<>();
        Set<Position> visibleGoalZonePositions = new HashSet<>();

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

        if (vision < 0) {
            vision = DEFAULT_VISION;
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
                int relativeX = x.getValue().intValue();
                int relativeY = y.getValue().intValue();
                if (percept.getName().equals("goalZone")) {
                    visibleGoalZonePositions.add(new Position(relativeX, relativeY));
                }
                rememberObservation(percept.getName(), relativeX, relativeY, "", step);
            }
        }

        for (int x = -vision; x <= vision; x++) {
            for (int y = -vision; y <= vision; y++) {
                if (Math.abs(x) + Math.abs(y) <= vision
                        && !occupiedRelativePositions.contains(new Position(x, y))) {
                    rememberFreeCell(x, y, step);
                }
                if (Math.abs(x) + Math.abs(y) <= vision
                        && !visibleGoalZonePositions.contains(new Position(x, y))) {
                    removeObservationTypeAt(agentX + x, agentY + y, "goalZone");
                }
            }
        }
    }

    public Set<Position> getVisibleTeammates() {
        return Set.copyOf(visibleTeammates);
    }

    public Set<Position> getPhysicalOccupiedEntityPositions() {
        return Set.copyOf(occupiedEntityPositions);
    }

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

    public ParameterList toParameterList() {
        ParameterList map = new ParameterList();
        for (Observation observation : observations.values()) {
            map.add(observationParameter(observation.type(), observation.x(), observation.y(),
                observation.details(), observation.lastSeenStep()));
        }
        return map;
    }

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

    public List<Observation> getObservations() {
        if (observationsSnapshot == null) {
            observationsSnapshot = List.copyOf(observations.values());
        }
        return observationsSnapshot;
    }

    public void mergeObservations(List<Observation> observationsToMerge) {
        for (Observation observation : observationsToMerge) {
            removeObservationsForUpdate(observation.x(), observation.y(), observation.type());

            String details = observation.details() == null ? "" : observation.details();
            ObservationKey key = new ObservationKey(observation.type(), observation.x(), observation.y(), details);
            putObservation(key, new Observation(
                    observation.type(), observation.x(), observation.y(), details, observation.lastSeenStep()));
        }
    }

    public void setObservations(List<Observation> newObservations) {
        observations.clear();
        keysByPosition.clear();
        occupiedEntityPositions.clear();
        observationsSnapshot = null;
        blockedPositionsSnapshot = null;
        knownPositionsSnapshot = null;

        for (Observation observation : newObservations) {
            String details = observation.details() == null ? "" : observation.details();
            ObservationKey key = new ObservationKey(observation.type(), observation.x(), observation.y(), details);
            putObservation(key, new Observation(
                    observation.type(), observation.x(), observation.y(), details, observation.lastSeenStep()));
        }
    }

    public List<Position> getBlockedPositions() {
        if (blockedPositionsSnapshot == null) {
            Set<Position> blockedPositions = new HashSet<>(occupiedEntityPositions);
            observations.values().stream()
                .filter(observation -> isBlocked(observation.type()))
                .map(observation -> new Position(observation.x(), observation.y()))
                .forEach(blockedPositions::add);
            blockedPositionsSnapshot = List.copyOf(blockedPositions);
        }
        return blockedPositionsSnapshot;
    }

    public boolean isKnownPosition(int x, int y) {
        if (knownPositionsSnapshot == null) {
            Set<Position> knownPositions = new HashSet<>();
            observations.values().forEach(observation ->
                    knownPositions.add(new Position(observation.x(), observation.y())));
            knownPositionsSnapshot = Set.copyOf(knownPositions);
        }
        return knownPositionsSnapshot.contains(new Position(x, y));
    }

    public void rememberFailedPath(int x, int y, int step) {
        removeBlockedObservationsAt(x, y);
        ObservationKey key = new ObservationKey("failedPath", x, y, "");
        putObservation(key, new Observation("failedPath", x, y, "", step));
    }

    public void forgetObservationsAt(int x, int y) {
        removeBlockedObservationsAt(x, y);
        rememberFreeCell(x - agentX, y - agentY, 0);
    }

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

    private void removeObservationTypeAt(int x, int y, String type) {
        Position position = new Position(x, y);
        Set<ObservationKey> keys = keysByPosition.get(position);
        if (keys == null) {
            return;
        }
        for (ObservationKey key : new ArrayList<>(keys)) {
            if (key.type().equals(type)) {
                removeObservation(key);
            }
        }
    }

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
        observationsSnapshot = null;
        knownPositionsSnapshot = null;
        if (isBlocked(key.type())) {
            blockedPositionsSnapshot = null;
        }
    }

    private void removeObservation(ObservationKey key) {
        observations.remove(key);
        observationsSnapshot = null;
        knownPositionsSnapshot = null;
        if (isBlocked(key.type())) {
            blockedPositionsSnapshot = null;
        }
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
        return type.equals("obstacle")
                || type.equals("failedPath")
                || type.equals("block");
    }

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
        observationsSnapshot = null;
        blockedPositionsSnapshot = null;
        knownPositionsSnapshot = null;

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