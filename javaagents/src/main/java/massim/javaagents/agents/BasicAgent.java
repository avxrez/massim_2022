package massim.javaagents.agents;

import eis.iilang.*;
import massim.javaagents.MailService;

import java.util.EnumSet;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**

 * Basic BDI agent.
 *
 * Structure:
 * 1. Perception
 * 2. Beliefs
 * 3. Desires
 * 4. Intention
 * 5. Action
 */
public class BasicAgent extends Agent {

	// ============================================================
	// DESIRES
	// ============================================================

	private enum Desire {
		EXPLORE,
		CLEAR_OBSTACLE,
		REACH_GOAL_ZONE,
		WAIT
	}

	// ============================================================
	// INTENTION
	// ============================================================

	private record Intention(Desire desire, List<String> plan, int nextAction) {

		private Intention advance() {
			return new Intention(desire, plan, nextAction + 1);
		}

		private boolean finished() {
			return nextAction >= plan.size();
		}

	}

	// ============================================================
	// BELIEFS
	// ============================================================

	private int lastID = -1;
	private int currentStep = -1;
	private int energy = -1;
	private boolean deactivated;
	private String currentTask;
	private final Set<String> requiredDispenserTypes = new HashSet<>();
	private final InternalMap internalMap = new InternalMap();

	// ============================================================
	// CURRENT INTENTION
	// ============================================================
    private InternalMap.Position explorationTarget;
	private Intention currentIntention;
	private String pendingAction;
	private String pendingDirection;
	private String clearDirection;

	// ============================================================
	// PATHFINDING
	// ============================================================

	private final AStarPathPlanner pathPlanner = new AStarPathPlanner();

    private final ExplorationTargetSelector explorationTargetSelector =
        new ExplorationTargetSelector();

	// ============================================================
	// CONSTRUCTOR
	// ============================================================

	/**

	 * Constructor.
	 *
	 * @param name    agent name
	 * @param mailbox mail facility
	 */
	public BasicAgent(String name, MailService mailbox) {
		super(name, mailbox);
	}

	// ============================================================
	// PERCEPTION
	// ============================================================

	@Override
	public void handlePercept(Percept percept) {
		// Not used yet.
	}

	@Override
	public void handleMessage(Percept message, String sender) {
		// Not used yet.
	}

	/**

	 * Updates the internal agent position using the result
	 * of the previous move action.
	 */
	private void updateAgentPosition(List<Percept> percepts) {

		String lastAction = null;
		String lastActionResult = null;
		String direction = null;

		for (Percept percept : percepts) {

			switch (percept.getName()) {

			case "lastAction" -> {
				if (!percept.getParameters().isEmpty()) {
					Parameter parameter = percept.getParameters().get(0);

					if (parameter instanceof Identifier identifier) {
						lastAction = identifier.getValue();
					}
				}
			}

			case "lastActionResult" -> {
				if (!percept.getParameters().isEmpty()) {
					Parameter parameter = percept.getParameters().get(0);

					if (parameter instanceof Identifier identifier) {
						lastActionResult = identifier.getValue();
					}
				}
			}

			case "lastActionParams" -> {
				if (!percept.getParameters().isEmpty()) {
					Parameter parameter = percept.getParameters().get(0);

					if (parameter instanceof ParameterList parameters
							&& parameters.size() == 1
							&& parameters.get(0) instanceof Identifier identifier) {

						direction = identifier.getValue();
					}
				}
			}

			default -> {
				// Not relevant for position updates.
			}
			}

		}

		if ("move".equals(lastAction)
				&& "success".equals(lastActionResult)
				&& direction != null) {

			internalMap.updateAgentPosition(direction);

		}
	}

	/**

	 * Updates the internal map with currently visible information.
	 */
	private void updateInternalMap(List<Percept> percepts) {

		int step = -1;
		int vision = -1;

		Set<String> occupiedPositions = new HashSet<>();

		// --------------------------------------------------------
		// Read step and vision
		// --------------------------------------------------------

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

		internalMap.clearOccupiedEntityPositions();

		// --------------------------------------------------------
		// Read visible objects
		// --------------------------------------------------------

		for (Percept percept : percepts) {

			String type = percept.getName();

			if (type.equals("thing") && percept.getParameters().size() >= 3) {

				int x = ((Numeral) percept.getParameters().get(0)).getValue().intValue();
				int y = ((Numeral) percept.getParameters().get(1)).getValue().intValue();

				occupiedPositions.add(x + "," + y);

				String thingType = ((Identifier) percept.getParameters().get(2)).getValue();
				String details = "";

				if (percept.getParameters().size() > 3 && percept.getParameters().get(3) instanceof Identifier identifier) {
					details = identifier.getValue();
				}

				// Entities are dynamic and therefore are not stored
				// as permanent map obstacles.
				if (!thingType.equals("entity")) {

					internalMap.rememberObservation(thingType, x, y, details, step);

				} else {

					internalMap.rememberFreeCell(x, y, step);
					internalMap.rememberOccupiedEntity(x, y);
				}

			} else if ((type.equals("goalZone") || type.equals("roleZone")) && percept.getParameters().size() >= 2) {

				int x = ((Numeral) percept.getParameters().get(0)).getValue().intValue();
				int y = ((Numeral) percept.getParameters().get(1)).getValue().intValue();

				internalMap.rememberObservation(type, x, y, "", step);
			}

		}

		// --------------------------------------------------------
		// Mark visible empty cells as free
		// --------------------------------------------------------

		if (vision >= 0) {
			for (int x = -vision; x <= vision; x++) {
				for (int y = -vision; y <= vision; y++) {
					if (Math.abs(x) + Math.abs(y) <= vision && !occupiedPositions.contains(x + "," + y)) {
						internalMap.rememberFreeCell(x, y, step);
					}
				}
			}

		}
	}

	// ============================================================
	// BELIEFS
	// ============================================================

	/**

	 * Updates scalar beliefs from the current percepts.
	 */
	private void updateBeliefs(List<Percept> percepts) {
		requiredDispenserTypes.clear();

		for (Percept percept : percepts) {

			if (percept.getParameters().isEmpty()) {
				continue;
			}

			switch (percept.getName()) {

			case "step" ->
			currentStep = numberValue(percept, currentStep);

			case "energy" ->
			energy = numberValue(percept, energy);

			case "deactivated" ->
			deactivated = identifierValue(percept, "false").equals("true");

			case "task" -> {
				currentTask = identifierValue( percept,currentTask);
				rememberTaskRequirements(percept);
			}

			default -> {
				// Not a scalar belief.
			}
			}

		}
	}

	/**

	 * Extracts the dispenser requirements from the current task.
	 */
	private void rememberTaskRequirements(Percept taskPercept) {

		if (taskPercept.getParameters().size() < 4
				|| !(taskPercept.getParameters().get(3)
						instanceof ParameterList requirements)) {

			return;

		}

		for (Parameter requirement : requirements) {

			if (requirement instanceof Function function
					&& function.getParameters().size() >= 3
					&& function.getParameters().get(2)
					instanceof Identifier type) {

				requiredDispenserTypes.add(type.getValue());
			}
		}
	}

	private int numberValue(Percept percept, int fallback) {

		Parameter parameter = percept.getParameters().get(0);

		return parameter instanceof Numeral numeral ? numeral.getValue().intValue(): fallback;

	}

	private String identifierValue(Percept percept, String fallback) {

		Parameter parameter = percept.getParameters().get(0);

		return parameter instanceof Identifier identifier ? identifier.getValue() : fallback;

	}

	// ============================================================
	// INTENTION FEEDBACK
	// ============================================================

	/**

	 * Checks whether the environment has advanced to a new
	 * action cycle.
	 */
	private boolean isNewActionCycle(List<Percept> percepts) {

		for (Percept percept : percepts) {

			if (percept.getName().equals("actionID")
					&& !percept.getParameters().isEmpty()
					&& percept.getParameters().get(0)
					instanceof Numeral numeral) {

				int actionID = numeral.getValue().intValue();

				if (actionID > lastID) {
					lastID = actionID;
					return true;
				}
			}
		}
		return false;
	}

	/**

	 * Updates the current intention according to the result
	 * of the previously executed action.
	 */
	private void updateIntentionAfterAction(List<Percept> percepts) {

		if (pendingAction == null || currentIntention == null) {
			return;
		}

		String lastAction = null;
		String lastActionResult = null;

		for (Percept percept : percepts) {

			if (percept.getName().equals("lastAction") && !percept.getParameters().isEmpty()) {

				lastAction = identifierValue(percept, null);

			} else if (percept.getName().equals("lastActionResult") && !percept.getParameters().isEmpty()) {

				lastActionResult = identifierValue(percept, null);
			}
		}

		// --------------------------------------------------------
		// Action succeeded
		// --------------------------------------------------------

		if ("success".equals(lastActionResult)) {

			currentIntention = currentIntention.advance();

			if ("clear".equals(lastAction)) {

				int[] offset = directionOffset(clearDirection);

				internalMap.forgetObservationsAt(internalMap.getAgentX() + offset[0], internalMap.getAgentY() + offset[1]);
			}
		}

		// --------------------------------------------------------
		// Action failed
		// --------------------------------------------------------

		else {
			currentIntention = null;
		}
		pendingAction = null;
		pendingDirection = null;
		clearDirection = null;
	}

	// ============================================================
	// DESIRE GENERATION
	// ============================================================

	/**

	 * Generates the current desires from the agent's beliefs.
	 *
	 * Important:
	 * This method only decides WHAT the agent wants to do.
	 * It does not calculate paths or actions.
	 */
	private Set<Desire> generateDesires() {

		Set<Desire> desires =
				EnumSet.noneOf(Desire.class);

		// Highest priority: wait when deactivated.
		if (deactivated) {
			desires.add(Desire.WAIT);
			return desires;

		}

		// Goal becomes relevant once all required
		// dispensers are known.
		if (hasObservation("goalZone")
				&& hasAllRequiredDispensers()) {

			if (isAtGoalZone()) {
				desires.add(Desire.WAIT);
			} else {
				desires.add(Desire.REACH_GOAL_ZONE);
			}
			return desires;

		}

		// Default behaviour for now.
		desires.add(Desire.EXPLORE);

		return desires;
	}

	// ============================================================
	// DESIRE CONDITIONS
	// ============================================================

	private boolean hasObservation(String type) {

		return internalMap.getObservations().stream().anyMatch(observation -> observation.type().equals(type));

	}

	private boolean hasAllRequiredDispensers() {

		Set<String> dispenserTypes =
				new HashSet<>();

		for (InternalMap.Observation observation : internalMap.getObservations()) {

			if (observation.type().equals("dispenser")) {

				dispenserTypes.add(observation.details());
			}
		}

		return !requiredDispenserTypes.isEmpty() && dispenserTypes.containsAll(requiredDispenserTypes);

	}

	private boolean isAtGoalZone() {

		int agentX = internalMap.getAgentX();
		int agentY = internalMap.getAgentY();

		return internalMap.getObservations().stream().anyMatch(	observation -> observation.type().equals("goalZone") 
        && observation.x() == agentX && observation.y() == agentY );

	}

	// ============================================================
	// INTENTION SELECTION
	// ============================================================

	/**

	 * Converts a desire into an intention.
	 *
	 * This method decides HOW the desired behaviour
	 * should currently be achieved.
	 */
	private Intention selectIntention(Set<Desire> desires) {

		if (desires.contains(Desire.WAIT)) {

			return new Intention(Desire.WAIT, List.of(), 0);

		}

		if (desires.contains(Desire.CLEAR_OBSTACLE)) {
			return new Intention(Desire.CLEAR_OBSTACLE, List.of(), 0);
		}

		if (desires.contains(Desire.REACH_GOAL_ZONE)) {
			return createGoalIntention();
		}

		if (desires.contains(Desire.EXPLORE)) {
			return createExploreIntention();
		}
		return null;
	}

	/**

	 * Creates an intention for reaching the goal.
	 */
	private Intention createGoalIntention() {

		InternalMap.Observation goal = findNearestGoalZone();

		InternalMap.Position start = currentPosition();

		InternalMap.Position target = new InternalMap.Position( goal.x(), goal.y());

		List<String> path = pathPlanner.findPath(start, target, internalMap.getBlockedPositions());

		return new Intention(Desire.REACH_GOAL_ZONE, path, 0);
	}

	/**

	 * Creates the exploration intention.
	 *
	 * Exploration target selection will be implemented
	 * separately.
	 */
    private Intention createExploreIntention() {

        InternalMap.Position start = currentPosition();

        InternalMap.Position target = explorationTargetSelector.selectTarget(internalMap);

        explorationTarget = target;
        System.out.println("Exploration target: " + explorationTarget);

        List<String> path = pathPlanner.findPath(start,target,internalMap.getBlockedPositions());

        if(nextMoveIsBlocked(path)) {
            return new Intention(Desire.CLEAR_OBSTACLE, List.of(path.get(0)), 0);
        }


        return new Intention(Desire.EXPLORE,path,0);
    }

	private InternalMap.Position currentPosition() {

		return new InternalMap.Position(internalMap.getAgentX(), internalMap.getAgentY());

	}

    private boolean nextMoveIsBlocked(List<String> plan) {

        if (plan == null || plan.isEmpty()) {
            return false;
        }

        String direction = plan.get(0);

        int[] offset = directionOffset(direction);

        int nextX = internalMap.getAgentX() + offset[0];
        int nextY = internalMap.getAgentY() + offset[1];

        InternalMap.Position nextPosition =
                new InternalMap.Position(nextX, nextY);

        return internalMap.getBlockedPositions()
                .contains(nextPosition);
    }



	private InternalMap.Observation findNearestGoalZone() {

		int agentX = internalMap.getAgentX();
		int agentY = internalMap.getAgentY();

		return internalMap.getObservations().stream().filter(observation -> observation.type()
        .equals("goalZone")).min((first, second) ->Integer.compare(
            distanceTo(first.x(),first.y(),agentX,agentY),distanceTo(second.x(),second.y(),agentX,agentY))).orElseThrow();

	}

	private int distanceTo(int firstX, int firstY, int secondX, int secondY) {

		return Math.abs(firstX - secondX) + Math.abs(firstY - secondY);

	}

	// ============================================================
	// ACTION
	// ============================================================

	/**

	 * Creates a move action.
	 */
	private Action move(String direction) {

		if (direction.equals("n") || direction.equals("s") || direction.equals("e") || direction.equals("w")) {

			return new Action("move", new Identifier(direction));
		}
		throw new IllegalArgumentException("Invalid direction: " + direction);
	}

	/**

	 * Executes the current intention.
	 *
	 * This method does not make decisions.
	 * It only translates the intention into an action.
	 */
	private Action executeIntention() {

        System.out.println("Current Intention: " + currentIntention);

		if (currentIntention == null) {
			return skip();
		}

		if (currentIntention.finished()) {
			return skip();
		}

		if (currentIntention.desire() == Desire.WAIT) {
			return skip();
		}

		if (currentIntention.desire() == Desire.CLEAR_OBSTACLE) {
			return executeClear();
		}

		return executeMove();
	}

	private Action executeMove() {

		if (currentIntention.plan().isEmpty()) {
			return skip();
		}
		pendingAction = "move";

		pendingDirection = currentIntention.plan().get(currentIntention.nextAction());

		return move(pendingDirection);

	}

	private Action executeClear() {

		if (currentIntention.plan().isEmpty()) {

			return skip();
		}

		pendingAction = "clear";

		clearDirection = currentIntention.plan().get(0);
		int[] offset = directionOffset(clearDirection);

		return new Action("clear", new Numeral(offset[0]), new Numeral(offset[1]));

	}

	private Action skip() {


		return new Action("skip", new Numeral(0), new Numeral(-1));


	}

	// ============================================================
	// UTILITY
	// ============================================================

	private int[] directionOffset(String direction) {

		return switch (direction) {
		case "n" -> new int[]{0, -1};
		case "e" -> new int[]{1, 0};
		case "s" -> new int[]{0, 1};
		case "w" -> new int[]{-1, 0};

		default ->
		throw new IllegalArgumentException(
				"Invalid direction: " + direction
				);
		};

	}

	// ============================================================
	// BDI CYCLE
	// ============================================================

	@Override
	public Action step() {

		// --------------------------------------------------------
		// 1. Perception
		// --------------------------------------------------------

		List<Percept> percepts =
				getPercepts();

		if (!isNewActionCycle(percepts)) {
			return null;
		}

        //System.out.println(internalMap.getObservations());
        System.out.println(getName() + " - Step: " + currentStep + ", Energy: " + energy);


		// --------------------------------------------------------
		// 2. Update beliefs / world model
		// --------------------------------------------------------

		updateAgentPosition(percepts);

		updateInternalMap(percepts);

		updateBeliefs(percepts);


		// --------------------------------------------------------
		// 3. Process previous intention
		// --------------------------------------------------------

		updateIntentionAfterAction(percepts);


		// --------------------------------------------------------
		// 4. BDI decision cycle
		// --------------------------------------------------------

		if (currentIntention == null
				|| currentIntention.finished()) {

			Set<Desire> desires =
					generateDesires();

			currentIntention =
					selectIntention(desires);
		}


		// --------------------------------------------------------
		// 5. Execute intention
		// --------------------------------------------------------

		return executeIntention();

	}
}
