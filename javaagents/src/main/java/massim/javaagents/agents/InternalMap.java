package massim.javaagents.agents;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class InternalMap {

    private int agentX = 0;
    private int agentY = 0;
    private final Map<ObservationKey, Observation> observations = new HashMap<>();

    public record Observation(String type, int x, int y, String details, int lastSeenStep) {}

    private record ObservationKey(String type, int x, int y, String details) {}

    public int getAgentX() {
        return agentX;
    }

    public int getAgentY() {
        return agentY;
    }

    public void rememberObservation(String type, int relativeX, int relativeY,
                                    String details, int step) {
        int absoluteX = agentX + relativeX;
        int absoluteY = agentY + relativeY;
        String observationDetails = details == null ? "" : details;
        observations.remove(new ObservationKey("free", absoluteX, absoluteY, ""));
        ObservationKey key = new ObservationKey(type, absoluteX, absoluteY, observationDetails);
        observations.put(key, new Observation(type, absoluteX, absoluteY, observationDetails, step));
    }

    public void rememberFreeCell(int relativeX, int relativeY, int step) {
        int absoluteX = agentX + relativeX;
        int absoluteY = agentY + relativeY;
        ObservationKey key = new ObservationKey("free", absoluteX, absoluteY, "");
        observations.putIfAbsent(key, new Observation("free", absoluteX, absoluteY, "", step));
        observations.computeIfPresent(key, (ignored, observation) ->
                new Observation("free", absoluteX, absoluteY, "", step));
    }

    public List<Observation> getObservations() {
        return Collections.unmodifiableList(new ArrayList<>(observations.values()));
    }


    public void updateAgentPosition(String direction) {
        switch (direction) {
            case "n":
                agentY -= 1;
                break;
            case "s":
                agentY += 1;
                break;
            case "e":
                agentX += 1;
                break;
            case "w":
                agentX -= 1;
                break;
            default:
                throw new IllegalArgumentException("Invalid direction: " + direction);
        }
    }
}