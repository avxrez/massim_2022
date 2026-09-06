package massim.javaagents.agents;

import eis.iilang.*;
import massim.javaagents.MailService;

import java.util.EnumSet;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * A very basic agent.
 */
public class BasicAgent extends Agent {

    private enum Desire {
        EXPLORE,
        REACH_GOAL_ZONE,
        WAIT
    }

    private record Intention(Desire desire, List<String> plan, int nextAction) {
        private Intention advance() {
            return new Intention(desire, plan, nextAction + 1);
        }

        private boolean finished() {
            return nextAction >= plan.size();
        }
    }

    private int lastID = -1;
    private int currentStep = -1;
    private int energy = -1;
    private boolean deactivated;
    private String currentTask;
    private Intention currentIntention;
    private String pendingDirection;

    private final InternalMap internalMap = new InternalMap();

    /**
     * Constructor.
     * @param name    the agent's name
     * @param mailbox the mail facility
     */
    public BasicAgent(String name, MailService mailbox) {
        super(name, mailbox);
    }

    @Override
    public void handlePercept(Percept percept) {}

    @Override
    public void handleMessage(Percept message, String sender) {}

    public Action move(String direction) {
        if (direction.equals("n") || direction.equals("s") || direction.equals("e") || direction.equals("w")) {
            return new Action("move", new Identifier(direction));

        } else {
            throw new IllegalArgumentException("Invalid direction: " + direction);
        }
    }

    public void updateAgentPosition(List<Percept> percepts) {
        String lastAction = null;
        String lastActionResult = null;
        String direction = null;

        for (Percept percept : percepts) {
            switch (percept.getName()) {
                case "lastAction" -> {
                    Parameter parameter = percept.getParameters().get(0);
                    if (parameter instanceof Identifier identifier) {
                        lastAction = identifier.getValue();
                    }
                }
                case "lastActionResult" -> {
                    Parameter parameter = percept.getParameters().get(0);
                    if (parameter instanceof Identifier identifier) {
                        lastActionResult = identifier.getValue();
                    }
                }
                case "lastActionParams" -> {
                    Parameter parameter = percept.getParameters().get(0);
                    if (parameter instanceof ParameterList parameters
                            && parameters.size() == 1
                            && parameters.get(0) instanceof Identifier identifier) {
                        direction = identifier.getValue();
                    }
                }
                default -> {
                    // This percept is unrelated to the previous movement.
                }
            }
        }

        if ("move".equals(lastAction)
                && "success".equals(lastActionResult)
                && direction != null) {
            internalMap.updateAgentPosition(direction);
        }
    }

    public void updateInternalMap(List<Percept> percepts) {
        int step = -1;
        int vision = -1;
        Set<String> occupiedPositions = new HashSet<>();

        for (Percept percept : percepts) {
            if (percept.getName().equals("step")
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

        for (Percept percept : percepts) {
            String type = percept.getName();
            if (type.equals("thing") && percept.getParameters().size() >= 3) {
                int x = ((Numeral) percept.getParameters().get(0)).getValue().intValue();
                int y = ((Numeral) percept.getParameters().get(1)).getValue().intValue();
                occupiedPositions.add(x + "," + y);
                String thingType = ((Identifier) percept.getParameters().get(2)).getValue();
                String details = percept.getParameters().size() > 3
                        ? ((Identifier) percept.getParameters().get(3)).getValue()
                        : "";
                if (!thingType.equals("entity")) {
                    internalMap.rememberObservation(thingType, x, y, details, step);
                }
            } else if ((type.equals("goalZone") || type.equals("roleZone"))
                    && percept.getParameters().size() >= 2) {
                int x = ((Numeral) percept.getParameters().get(0)).getValue().intValue();
                int y = ((Numeral) percept.getParameters().get(1)).getValue().intValue();
                internalMap.rememberObservation(type, x, y, "", step);
            }
        }

        if (vision >= 0) {
            for (int x = -vision; x <= vision; x++) {
                for (int y = -vision; y <= vision; y++) {
                    if (Math.abs(x) + Math.abs(y) <= vision
                            && !occupiedPositions.contains(x + "," + y)) {
                        internalMap.rememberFreeCell(x, y, step);
                    }
                }
            }
        }
    }

    private void updateBeliefs(List<Percept> percepts) {
        for (Percept percept : percepts) {
            if (percept.getParameters().isEmpty()) {
                continue;
            }

            switch (percept.getName()) {
                case "step" -> currentStep = numberValue(percept, currentStep);
                case "energy" -> energy = numberValue(percept, energy);
                case "deactivated" -> deactivated = identifierValue(percept, "false").equals("true");
                case "task" -> currentTask = identifierValue(percept, currentTask);
                default -> {
                    // This percept does not update a scalar belief.
                }
            }
        }
    }

    private int numberValue(Percept percept, int fallback) {
        Parameter parameter = percept.getParameters().get(0);
        return parameter instanceof Numeral numeral ? numeral.getValue().intValue() : fallback;
    }

    private String identifierValue(Percept percept, String fallback) {
        Parameter parameter = percept.getParameters().get(0);
        return parameter instanceof Identifier identifier ? identifier.getValue() : fallback;
    }

    private boolean isNewActionCycle(List<Percept> percepts) {
        for (Percept percept : percepts) {
            if (percept.getName().equals("actionID") && !percept.getParameters().isEmpty()
                    && percept.getParameters().get(0) instanceof Numeral numeral) {
                int actionID = numeral.getValue().intValue();
                if (actionID > lastID) {
                    lastID = actionID;
                    return true;
                }
            }
        }
        return false;
    }

    private void updateIntentionAfterAction(List<Percept> percepts) {
        if (pendingDirection == null || currentIntention == null) {
            return;
        }

        String lastAction = null;
        String lastActionResult = null;
        for (Percept percept : percepts) {
            if (percept.getName().equals("lastAction") && !percept.getParameters().isEmpty()) {
                lastAction = identifierValue(percept, null);
            } else if (percept.getName().equals("lastActionResult")
                    && !percept.getParameters().isEmpty()) {
                lastActionResult = identifierValue(percept, null);
            }
        }

        if ("move".equals(lastAction) && "success".equals(lastActionResult)) {
            currentIntention = currentIntention.advance();
        }
        pendingDirection = null;
    }

    private Set<Desire> generateDesires() {
        Set<Desire> desires = EnumSet.noneOf(Desire.class);
        if (deactivated) {
            desires.add(Desire.WAIT);
        } else if (hasObservation("goalZone")) {
            desires.add(Desire.REACH_GOAL_ZONE);
        } else {
            desires.add(Desire.EXPLORE);
        }
        return desires;
    }

    private boolean hasObservation(String type) {
        return internalMap.getObservations().stream()
                .anyMatch(observation -> observation.type().equals(type));
    }

    private Intention selectIntention(Set<Desire> desires) {
        if (desires.contains(Desire.WAIT)) {
            return new Intention(Desire.WAIT, List.of(), 0);
        }

        if (desires.contains(Desire.REACH_GOAL_ZONE)) {
            return new Intention(Desire.REACH_GOAL_ZONE, List.of("n"), 0);
        }

        return new Intention(Desire.EXPLORE, List.of("n", "e", "s", "w"), 0);
    }

    private Action executeIntention() {
        if (currentIntention == null || currentIntention.finished()
                || currentIntention.desire() == Desire.WAIT) {
            return null;
        }

        pendingDirection = currentIntention.plan().get(currentIntention.nextAction());
        return move(pendingDirection);
    }

    @Override
    public Action step() {
        List<Percept> percepts = getPercepts();

        if (!isNewActionCycle(percepts)) {
            return null;
        }

        updateAgentPosition(percepts);
        updateInternalMap(percepts);
        updateBeliefs(percepts);
        updateIntentionAfterAction(percepts);

        if (currentIntention == null || currentIntention.finished()) {
            currentIntention = selectIntention(generateDesires());
        }

        return executeIntention();
    }
}
