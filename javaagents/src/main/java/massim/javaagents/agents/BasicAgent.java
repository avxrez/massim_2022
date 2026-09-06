package massim.javaagents.agents;

import eis.iilang.*;
import massim.javaagents.MailService;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * A very basic agent.
 */
public class BasicAgent extends Agent {

    private int lastID = -1;

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
            System.out.println("hi");
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

    @Override
    public Action step() {
        List<Percept> percepts = getPercepts();
        System.out.println(internalMap.getAgentY());
        updateAgentPosition(percepts);
        updateInternalMap(percepts);
        System.out.println(getName() + " at position (" + internalMap.getAgentX() + ", " + internalMap.getAgentY() + ")");
        System.out.println(internalMap.getObservations());
        for (Percept percept : percepts) {
            if (percept.getName().equals("actionID")) {
                Parameter param = percept.getParameters().get(0);
                if (param instanceof Numeral) {
                    int id = ((Numeral) param).getValue().intValue();
                    if (id > lastID) {
                        lastID = id;
                            try {
                                Thread.sleep(2000);
                                } catch (InterruptedException e) {
                                    Thread.currentThread().interrupt();
                                }
                        return move("n"); 
                    }
                }
            }
        }
        return null;
    }
}
