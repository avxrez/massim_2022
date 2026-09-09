package massim.javaagents.agents;

import eis.iilang.*;
import massim.javaagents.MailService;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
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
	
	private String leaderName = "";
	private int lastID = -1;
	private int currentStep = -1;
	private int energy = -1;
	private boolean deactivated;
	private String currentTask;
	private String teamName = "";
	private final Set<String> requiredDispenserTypes = new HashSet<>();
	private final Map<String, InternalMap.Position> knownAgents = new HashMap<>();
	private final Map<String, InternalMap.Position> knownTargets = new HashMap<>();
	private final Set<InternalMap.Position> visibleTeammates = new HashSet<>();
	private record VisibleThing(int x, int y, String type, String details) {
	}
	private final List<VisibleThing> currentVisibleThings = new ArrayList<>();
	private static final int VISION_RANGE = 5;
	private final List<PendingTeammateRequest> pendingTeammateRequests = new ArrayList<>();
	private final InternalMap internalMap = new InternalMap();

	private record PendingTeammateRequest(String sender, String senderLeaderName, int x, int y,
			int senderX, int senderY, List<VisibleThing> senderVisibleThings) {
	}

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
		leaderName = name;
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
		if (message.getName().equals("mapMerge")
				&& message.getParameters().size() >= 1) {
			mergeMap(message.getParameters().get(0));
			return;
		}

		if (message.getName().equals("mapUpdate")
				&& message.getParameters().size() >= 6
				&& message.getParameters().get(0) instanceof Numeral senderX
				&& message.getParameters().get(1) instanceof Numeral senderY
				&& message.getParameters().get(2) instanceof Numeral targetX
				&& message.getParameters().get(3) instanceof Numeral targetY
				&& message.getParameters().get(5) instanceof Identifier senderLeaderName
				&& leaderName.equals(senderLeaderName.getValue())) {
				System.out.println("Name: " + getName() + ", Leader: " + leaderName + ", Sender: " + sender + ", Sender Leader: " + senderLeaderName.getValue());
				System.out.println("Received map update from " + sender + " with target (" + targetX.getValue().intValue() + ", " + targetY.getValue().intValue() + ")");

			mergeMap(message.getParameters().get(4));
			knownAgents.put(sender,
					new InternalMap.Position(
							senderX.getValue().intValue(), senderY.getValue().intValue()));
			if (targetX.getValue().intValue() == Integer.MIN_VALUE) {
				knownTargets.remove(sender);
			} else {
				knownTargets.put(sender,
						new InternalMap.Position(
								targetX.getValue().intValue(), targetY.getValue().intValue()));
			}
			return;
		}

		if (message.getName().equals("teammateRequest")
				&& message.getParameters().size() >= 5
				&& message.getParameters().get(0) instanceof Numeral x
				&& message.getParameters().get(1) instanceof Numeral y
				&& message.getParameters().get(2) instanceof Numeral senderX
				&& message.getParameters().get(3) instanceof Numeral senderY
				&& message.getParameters().get(4) instanceof Identifier senderLeaderName) {
			List<VisibleThing> senderVisibleThings = message.getParameters().size() > 5
					? parseVisibleThings(message.getParameters().get(5))
					: List.of();

			synchronized (pendingTeammateRequests) {
			pendingTeammateRequests.add(new PendingTeammateRequest(sender, senderLeaderName.getValue(),
				x.getValue().intValue(), y.getValue().intValue(),
				senderX.getValue().intValue(), senderY.getValue().intValue(),
				senderVisibleThings));
			}
			return;
		}

		if (message.getName().equals("teammateReply")
				&& message.getParameters().size() >= 6
				&& message.getParameters().get(0) instanceof Identifier identifier
				&& message.getParameters().get(1) instanceof Numeral x
				&& message.getParameters().get(2) instanceof Numeral y
				&& message.getParameters().get(3) instanceof Numeral senderX
				&& message.getParameters().get(4) instanceof Numeral senderY
				&& message.getParameters().get(5) instanceof Identifier senderLeaderName
				&& sender.equals(identifier.getValue())
				&& isVisibleTeammateAt(-x.getValue().intValue(), -y.getValue().intValue())
				&& !knownAgents.containsKey(sender)) {
			List<VisibleThing> senderVisibleThings = message.getParameters().size() > 6
					? parseVisibleThings(message.getParameters().get(6))
					: List.of();

			if (!isConsistentWithOwnView(-x.getValue().intValue(), -y.getValue().intValue(),
					senderVisibleThings)) {
				return;
			}

			if (nameNumber(leaderName) > nameNumber(senderLeaderName.getValue())) {
				int offsetX = senderX.getValue().intValue() + x.getValue().intValue()
						- internalMap.getAgentX();
				int offsetY = senderY.getValue().intValue() + y.getValue().intValue()
						- internalMap.getAgentY();
				translateWorld(offsetX, offsetY);
				leaderName = senderLeaderName.getValue();
			}
			knownAgents.put(sender,
					new InternalMap.Position(
							internalMap.getAgentX() - x.getValue().intValue(),
							internalMap.getAgentY() - y.getValue().intValue()));
			sendMapMerge(sender);
		}
	}

	private void processTeammateRequests() {
		List<PendingTeammateRequest> requests;
		synchronized (pendingTeammateRequests) {
			requests = new ArrayList<>(pendingTeammateRequests);
			pendingTeammateRequests.clear();
		}

		List<PendingTeammateRequest> matchingRequests = requests.stream()
				.filter(request -> isVisibleTeammateAt(-request.x(), -request.y()))
				.filter(request -> !knownAgents.containsKey(request.sender()))
				.filter(request -> isConsistentWithOwnView(-request.x(), -request.y(),
						request.senderVisibleThings()))
				.toList();

		if (matchingRequests.size() != 1) {
			return;
		}

		PendingTeammateRequest request = matchingRequests.get(0);
		if (nameNumber(leaderName) > nameNumber(request.senderLeaderName())) {
			int offsetX = request.senderX() + request.x() - internalMap.getAgentX();
			int offsetY = request.senderY() + request.y() - internalMap.getAgentY();
			translateWorld(offsetX, offsetY);
			leaderName = request.senderLeaderName();
		}

		knownAgents.put(request.sender(),
				new InternalMap.Position(
						internalMap.getAgentX() - request.x(),
						internalMap.getAgentY() - request.y()));
		sendMessage(new Percept("teammateReply",
				new Identifier(getName()),
				new Numeral(-request.x()),
				new Numeral(-request.y()),
				new Numeral(internalMap.getAgentX()),
				new Numeral(internalMap.getAgentY()),
				new Identifier(leaderName),
				currentVisibleThingsParameters()), request.sender(), getName());
		sendMapMerge(request.sender());
	}

	private void updateCurrentVisibleThings(List<Percept> percepts) {
		currentVisibleThings.clear();
		for (Percept percept : percepts) {
			if (percept.getName().equals("thing")
					&& percept.getParameters().size() >= 3
					&& percept.getParameters().get(0) instanceof Numeral x
					&& percept.getParameters().get(1) instanceof Numeral y
					&& percept.getParameters().get(2) instanceof Identifier type) {

				String details = "";
				if (percept.getParameters().size() > 3
						&& percept.getParameters().get(3) instanceof Identifier identifier) {
					details = identifier.getValue();
				}

				currentVisibleThings.add(new VisibleThing(
						x.getValue().intValue(), y.getValue().intValue(),
						type.getValue(), details));
			}
		}
	}

	private ParameterList currentVisibleThingsParameters() {
		ParameterList list = new ParameterList();
		for (VisibleThing thing : currentVisibleThings) {
			list.add(new Function("seen",
					new Identifier(thing.type()),
					new Numeral(thing.x()),
					new Numeral(thing.y()),
					new Identifier(thing.details())));
		}
		return list;
	}

	private List<VisibleThing> parseVisibleThings(Parameter parameter) {
		List<VisibleThing> things = new ArrayList<>();
		if (!(parameter instanceof ParameterList list)) {
			return things;
		}
		for (Parameter entry : list) {
			if (entry instanceof Function function
					&& function.getName().equals("seen")
					&& function.getParameters().size() >= 4
					&& function.getParameters().get(0) instanceof Identifier type
					&& function.getParameters().get(1) instanceof Numeral x
					&& function.getParameters().get(2) instanceof Numeral y
					&& function.getParameters().get(3) instanceof Identifier details) {
				things.add(new VisibleThing(
						x.getValue().intValue(), y.getValue().intValue(),
						type.getValue(), details.getValue()));
			}
		}
		return things;
	}

	private boolean isConsistentWithOwnView(int relativeX, int relativeY,
			List<VisibleThing> reportedThings) {
		for (VisibleThing thing : reportedThings) {
			int ownX = relativeX + thing.x();
			int ownY = relativeY + thing.y();

			if (Math.abs(ownX) + Math.abs(ownY) > VISION_RANGE) {
				continue;
			}

			boolean matches = currentVisibleThings.stream().anyMatch(own ->
					own.x() == ownX && own.y() == ownY
							&& own.type().equals(thing.type()));

			if (!matches) {
				return false;
			}
		}
		return true;
	}

	private void sendMapMerge(String recipient) {
		sendMessage(new Percept("mapMerge", mapParameters()), recipient, getName());
	}

	private ParameterList mapParameters() {
		ParameterList map = new ParameterList();
		for (InternalMap.Observation observation : internalMap.getObservations()) {
			map.add(new Function("observation",
					new Identifier(observation.type()),
					new Numeral(observation.x()),
					new Numeral(observation.y()),
					new Identifier(observation.details()),
					new Numeral(observation.lastSeenStep())));
		}
		return map;
	}

	private void mergeMap(Parameter parameter) {
		if (!(parameter instanceof ParameterList map)) {
			return;
		}

		List<InternalMap.Observation> observations = new java.util.ArrayList<>();
		for (Parameter entry : map) {
			if (entry instanceof Function observation
					&& observation.getName().equals("observation")
					&& observation.getParameters().size() >= 5
					&& observation.getParameters().get(0) instanceof Identifier type
					&& observation.getParameters().get(1) instanceof Numeral x
					&& observation.getParameters().get(2) instanceof Numeral y
					&& observation.getParameters().get(3) instanceof Identifier details
					&& observation.getParameters().get(4) instanceof Numeral step) {
				observations.add(new InternalMap.Observation(
						type.getValue(), x.getValue().intValue(), y.getValue().intValue(),
						details.getValue(), step.getValue().intValue()));
			}
		}
		internalMap.mergeObservations(observations);
	}

	private ParameterList currentMapPercepts(List<Percept> percepts) {
		ParameterList map = new ParameterList();
		int step = currentStep;

		for (Percept percept : percepts) {
			if (percept.getName().equals("thing")
					&& percept.getParameters().size() >= 3
					&& percept.getParameters().get(0) instanceof Numeral x
					&& percept.getParameters().get(1) instanceof Numeral y
					&& percept.getParameters().get(2) instanceof Identifier type) {

				if (type.getValue().equals("entity")) {
					continue;
				}

				String details = "";
				if (percept.getParameters().size() > 3
						&& percept.getParameters().get(3) instanceof Identifier identifier) {
					details = identifier.getValue();
				}

				map.add(new Function("observation",
						new Identifier(type.getValue()),
						new Numeral(internalMap.getAgentX() + x.getValue().intValue()),
						new Numeral(internalMap.getAgentY() + y.getValue().intValue()),
						new Identifier(details),
						new Numeral(step)));
			} else if ((percept.getName().equals("goalZone")
					|| percept.getName().equals("roleZone"))
					&& percept.getParameters().size() >= 2
					&& percept.getParameters().get(0) instanceof Numeral x
					&& percept.getParameters().get(1) instanceof Numeral y) {

				map.add(new Function("observation",
						new Identifier(percept.getName()),
						new Numeral(internalMap.getAgentX() + x.getValue().intValue()),
						new Numeral(internalMap.getAgentY() + y.getValue().intValue()),
						new Identifier(""),
						new Numeral(step)));
			}
		}
		return map;
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
		visibleTeammates.clear();

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

					if (!teamName.isEmpty()
							&& !(x == 0 && y == 0)
							&& teamName.equals(details)) {
						visibleTeammates.add(new InternalMap.Position(x, y));
					}
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

			case "team" ->
			teamName = identifierValue(percept, teamName);

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

	private boolean isVisibleTeammateAt(int x, int y) {
		return visibleTeammates.contains(new InternalMap.Position(x, y));
	}

	private int nameNumber(String name) {
		String number = name.replaceAll("[^0-9]", "");
		return number.isEmpty() ? -1 : Integer.parseInt(number);
	}

	private void exchangeTeammateNames() {
	    for (InternalMap.Position teammate : visibleTeammates) {
			if (isKnownTeammateAt(teammate)) {
				continue;
			}

	        broadcast(new Percept(
	                "teammateRequest",
	                new Numeral(teammate.x()),
	                new Numeral(teammate.y()),
	                new Numeral(internalMap.getAgentX()),
	                new Numeral(internalMap.getAgentY()),
	                new Identifier(leaderName),
	                currentVisibleThingsParameters()
	        ), getName());
	    }
	}

	private boolean isKnownTeammateAt(InternalMap.Position relativePosition) {
		InternalMap.Position absolutePosition = new InternalMap.Position(
				internalMap.getAgentX() + relativePosition.x(),
				internalMap.getAgentY() + relativePosition.y());
		return knownAgents.containsValue(absolutePosition);
	}

	private void exchangeMapUpdates(List<Percept> percepts) {
		ParameterList currentPercepts = currentMapPercepts(percepts);
		for (String agent : knownAgents.keySet()) {
			if (agent.equals(getName())) {
				continue;
			}
			InternalMap.Position target = explorationTarget;
			sendMessage(new Percept(
					"mapUpdate",
					new Numeral(internalMap.getAgentX()),
					new Numeral(internalMap.getAgentY()),
					new Numeral(target == null ? Integer.MIN_VALUE : target.x()),
					new Numeral(target == null ? Integer.MIN_VALUE : target.y()),
					currentPercepts,
					new Identifier(leaderName)), agent, getName());
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

		if (nextMoveIsBlocked(path)) {
			return new Intention(Desire.CLEAR_OBSTACLE, List.of(path.get(0)), 0);
		}

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

		InternalMap.Position target = explorationTargetSelector.selectTarget(
			internalMap, knownTargets, getName());

        explorationTarget = target;
        //System.out.println("Exploration target: " + explorationTarget);

        List<String> path = pathPlanner.findPath(start,target,internalMap.getBlockedPositions());

        if(nextMoveIsBlocked(path)) {
            return new Intention(Desire.CLEAR_OBSTACLE, List.of(path.get(0)), 0);
        }


        return new Intention(Desire.EXPLORE,path,0);
    }

	private InternalMap.Position currentPosition() {

		return new InternalMap.Position(internalMap.getAgentX(), internalMap.getAgentY());

	}

	private boolean explorationTargetSeen() {
		return explorationTarget != null
				&& internalMap.isKnownPosition(
						explorationTarget.x(), explorationTarget.y());
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

        //System.out.println("Current Intention: " + currentIntention);

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

	private void translateWorld(int offsetX, int offsetY) {
	    internalMap.translate(offsetX, offsetY);

	    for (Map.Entry<String, InternalMap.Position> entry : knownAgents.entrySet()) {
	        InternalMap.Position position = entry.getValue();

	        entry.setValue(new InternalMap.Position(
	                position.x() + offsetX,
	                position.y() + offsetY
	        ));
	    }

	    for (Map.Entry<String, InternalMap.Position> entry : knownTargets.entrySet()) {
	        InternalMap.Position position = entry.getValue();

	        entry.setValue(new InternalMap.Position(
	                position.x() + offsetX,
	                position.y() + offsetY
	        ));
	    }

	    if (explorationTarget != null) {
	        explorationTarget = new InternalMap.Position(
	                explorationTarget.x() + offsetX,
	                explorationTarget.y() + offsetY
	        );
	    }
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

        //System.out.println(getName() + " - Step: " + currentStep + ", Energy: " + energy + ", Position: (" + internalMap.getAgentX() + ", " + internalMap.getAgentY() + ")");
		//System.out.println("currentIntention: " + currentIntention);
		//System.out.println("Current target: " + explorationTarget);
		//System.out.println(percepts);


		// --------------------------------------------------------
		// 2. Update beliefs / world model
		// --------------------------------------------------------

		updateAgentPosition(percepts);

		updateBeliefs(percepts);

		updateInternalMap(percepts);
		updateCurrentVisibleThings(percepts);
		processTeammateRequests();
		exchangeTeammateNames();


		// --------------------------------------------------------
		// 3. Process previous intention
		// --------------------------------------------------------

		updateIntentionAfterAction(percepts);

		if (currentIntention != null
				&& currentIntention.desire() == Desire.EXPLORE
				&& explorationTargetSeen()) {
			currentIntention = null;
			explorationTarget = null;
		}


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

		if (currentIntention != null
				&& currentIntention.desire() == Desire.EXPLORE
				&& explorationTarget != null
				&& !explorationTargetSelector.isTargetAllowed(
						explorationTarget, knownTargets, getName())) {
			explorationTarget = null;
			currentIntention = selectIntention(EnumSet.of(Desire.EXPLORE));
		}

		exchangeMapUpdates(percepts);


		// --------------------------------------------------------
		// 5. Execute intention
		// --------------------------------------------------------

		return executeIntention();

	}
}
