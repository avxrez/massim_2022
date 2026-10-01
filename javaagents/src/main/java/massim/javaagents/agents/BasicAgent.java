package massim.javaagents.agents;

import static massim.javaagents.agents.AgentUtils.directionOffset;
import static massim.javaagents.agents.AgentUtils.isMovementDirection;
import static massim.javaagents.agents.AgentUtils.nameNumber;
import static massim.javaagents.agents.AgentUtils.oppositeDirection;
import static massim.javaagents.agents.AgentUtils.rotateDirection;

import eis.iilang.*;
import massim.javaagents.MailService;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.Predicate;

/**
 * BDI agent that explores the map, selects tasks, coordinates group work, and
 * executes movement and block-handling plans.
 */
public class BasicAgent extends Agent {

    /** The high-level behavior currently considered by the agent. */
    private enum Desire {
        EXPLORE,
        CLEAR_OBSTACLE,
        REACH_GOAL_ZONE,
        RETRIEVE_BLOCK,
        ATTACH_ASSEMBLY,
        CONNECT_ASSEMBLY,
        DETACH_ASSEMBLY,
        WAIT,
        REACH_ROLE_ZONE,
        ADAPT_ROLE,
        SUBMIT
    }

    /** An ordered plan and the index of its next action. */
    private record Intention(Desire desire, List<String> plan, int nextAction) {

        private Intention advance() {
            return new Intention(desire, plan, nextAction + 1);
        }

        private boolean finished() {
            return nextAction >= plan.size();
        }
    }

    private static final int VISION_RANGE = 5;
    private static final int MIN_SHARED_VERIFICATION_THINGS = 3;
    private static final int ROLE_ZONE_MAX_DISTANCE = 20;
    private static final int GOAL_RESERVATION_MAX_AGE = 2;
    private static final String DEFAULT_ROLE = "default";
    private static final String WORKER_ROLE = "worker";
    private static final String SERVER_AGENT_PREFIX = "agent";

    private String leaderName = "";
    private int lastID = -1;
    private int currentStep = -1;
    private boolean deactivated;
    private String currentRole = "";
    private String currentTask;
    /** Parsed task data used for task selection and assembly planning. */
    private record TaskInfo(String name, int deadline, List<String> blockTypes,
            List<InternalMap.Position> offsets) {

        /** Creates task data from the server's task percept. */
        private static TaskInfo fromPercept(Percept taskPercept) {
            String name = taskPercept.getParameters().get(0) instanceof Identifier identifier
                    ? identifier.getValue() : "";
            int deadline = taskPercept.getParameters().size() > 1
                    && taskPercept.getParameters().get(1) instanceof Numeral numeral
                    ? numeral.getValue().intValue() : -1;
            List<String> blockTypes = new ArrayList<>();
            List<InternalMap.Position> offsets = new ArrayList<>();
            if (taskPercept.getParameters().size() > 3
                    && taskPercept.getParameters().get(3) instanceof ParameterList requirements) {
                for (Parameter requirement : requirements) {
                    if (requirement instanceof Function function
                            && function.getParameters().size() >= 3
                            && function.getParameters().get(0) instanceof Numeral x
                            && function.getParameters().get(1) instanceof Numeral y
                            && function.getParameters().get(2) instanceof Identifier type) {
                        blockTypes.add(type.getValue());
                        offsets.add(new InternalMap.Position(
                                x.getValue().intValue(), y.getValue().intValue()));
                    }
                }
            }
            return new TaskInfo(name, deadline, List.copyOf(blockTypes), List.copyOf(offsets));
        }

        private int groupSize() {
            return offsets.size() <= 1 ? 1 : offsets.size() + 1;
        }

        private boolean activeAt(int step) {
            return deadline < 0 || step <= deadline;
        }

        private long remainingAt(int step) {
            return deadline < 0 ? Long.MAX_VALUE : (long) deadline - step;
        }
    }
    private final List<TaskInfo> currentTasks = new ArrayList<>();
    private TaskInfo currentTaskInfo;
    private String teamName = "";
    private final Map<String, Boolean> knownAgentGroupState = new HashMap<>();
    private final Map<String, String> knownAgentGroupLeader = new HashMap<>();
    private final Set<String> currentGroupMembers = new HashSet<>();
    private final Set<String> pendingGroupInvitations = new HashSet<>();
    private final Set<String> rejectedGroupInviteTargets = new HashSet<>();
    private String currentGroupInviteTarget = null;
    private boolean groupGoalZoneConfirmed = false;
    private boolean groupGoalRelocationPending;
    private int currentTaskBlockCount = 1;
    private int desiredGroupSize = 1;
    private String currentGroupLeader = "";
    private boolean groupLeaderMode = false;
    private boolean groupFormationActive = false;
    private String groupTaskName = "";
    private String deliveryBlockType = null;
    private InternalMap.Position requiredBlockOffset;

    private final Set<String> requiredDispenserTypes = new HashSet<>();
    private final Set<InternalMap.Position> assembledBlockPositions = new HashSet<>();
    private final Map<InternalMap.Position, String> assembledBlockTypes = new HashMap<>();
    private final Map<String, GoalReservationSnapshot> goalReservationSnapshots = new HashMap<>();
    private final Map<String, Integer> latestGoalReservationRevisions = new HashMap<>();
    private final Map<InternalMap.Position, PendingAssemblyAttachment> pendingAssemblyDeliveries = new HashMap<>();
    private final Set<String> pendingAssemblyDetaches = new HashSet<>();
    private final List<String> attachedBlockDirections = new ArrayList<>();
    private final Set<String> deferredGroupDissolveMembers = new HashSet<>();
    private boolean assemblyCleanupRequested;
    private final List<String> taskBlockTypes = new ArrayList<>();
    private final List<InternalMap.Position> taskRequirementOffsets = new ArrayList<>();
    private final Map<String, InternalMap.Position> knownAgents = new HashMap<>();
    private final Map<String, String> knownAgentSources = new HashMap<>();
    private final Map<String, PendingTeammateConfirmation> pendingTeammateConfirmations = new HashMap<>();
    private final Map<String, PendingTeammateConfirmation> pendingTeammateAcceptances = new HashMap<>();
    private final Map<String, InternalMap.Position> knownTargets = new HashMap<>();
    private final List<VisibleThing> currentVisibleThings = new ArrayList<>();
    private final List<PendingTeammateRequest> pendingTeammateRequests = new ArrayList<>();
    private final InternalMap internalMap = new InternalMap();

    /** A currently visible thing represented relative to this agent. */
    private record VisibleThing(int x, int y, String type, String details) {}

    /** Teammate identity request queued until the current percept batch is processed. */
    private record PendingTeammateRequest(String sender, String senderLeaderName, int x, int y,
            int senderX, int senderY, int receiverX, int receiverY,
            List<VisibleThing> senderVisibleThings) {}

    /** Coordinate data retained while a teammate identity exchange is confirmed. */
    private record PendingTeammateConfirmation(String senderLeaderName, int senderX, int senderY,
            int relativeX, int relativeY) {}

    /** A teammate-delivered block awaiting connection to the leader's assembly. */
    private record PendingAssemblyAttachment(String member, String blockType,
            InternalMap.Position targetPosition) {}

    /** An assembly connection currently being coordinated with a teammate. */
    private record PendingAssemblyConnection(String partner, String blockType,
            InternalMap.Position targetPosition, int leaderBlockX, int leaderBlockY) {}

    /** Versioned snapshot of goal cells reserved by one agent. */
    private record GoalReservationSnapshot(int revision, int updatedAtStep,
            Set<InternalMap.Position> positions) {

        private GoalReservationSnapshot {
            positions = Set.copyOf(positions);
        }
    }

    private InternalMap.Position explorationTarget;
    private InternalMap.Position goalPosition;
    private InternalMap.Position carriedBlockPosition;
    private String retrieveBlockDirection;
    private boolean blockRequested;
    private boolean blockRetrieved;
    private String carriedBlockType;
    private boolean blockPlaced;
    private boolean detachRequested;
    private boolean successorGroupFormationAfterDetach;
    private boolean explorationFinished;
    private Intention currentIntention;
    private String pendingAction;
    private String pendingDirection;
    private String pendingRotation;
    private String clearDirection;
    private PendingAssemblyAttachment pendingAssemblyAttachment;
    private PendingAssemblyConnection pendingAssemblyConnection;


    private final AStarPathPlanner pathPlanner = new AStarPathPlanner();
    private final ExplorationTargetSelector explorationTargetSelector = new ExplorationTargetSelector();

    /**
     * Creates an agent with the given identity and message service.
     *
     * @param name agent name
     * @param mailbox service used to send and receive agent messages
     */
    public BasicAgent(String name, MailService mailbox) {
        super(name, mailbox);
        leaderName = name;
    }

    private void applyTeammateConfirmation(String sender, PendingTeammateConfirmation confirmation) {
        if (nameNumber(leaderName) > nameNumber(confirmation.senderLeaderName())) {
                int offsetX = confirmation.senderX() + confirmation.relativeX() - internalMap.getAgentX();
                int offsetY = confirmation.senderY() + confirmation.relativeY() - internalMap.getAgentY();
            switchLeader(confirmation.senderLeaderName(), offsetX, offsetY);
        }
        rememberKnownAgent(sender,
                new InternalMap.Position(
                        internalMap.getAgentX() - confirmation.relativeX(),
                        internalMap.getAgentY() - confirmation.relativeY()), getName());
    }

    @Override
    public void handlePercept(Percept percept) {
    }

    /**
     * Processes a teammate message and updates coordination state.
     *
     * @param message message percept
     * @param sender name of the sending agent
     */
    @Override
    public void handleMessage(Percept message, String sender) {
        if (message.getName().equals("mapMerge")
                && message.getParameters().size() >= 1) {
            internalMap.mergeObservations(message.getParameters().get(0));
            sendMergedMapUpdates();
            return;
        }

        if (message.getName().equals("newLeader")
                && message.getParameters().size() >= 5
                && message.getParameters().get(0) instanceof Identifier previousLeader
                && message.getParameters().get(1) instanceof Identifier newLeader
                && message.getParameters().get(2) instanceof Numeral offsetX
                && message.getParameters().get(3) instanceof Numeral offsetY
                && message.getParameters().get(4) instanceof ParameterList map
                && leaderName.equals(previousLeader.getValue())) {
            translateWorld(offsetX.getValue().intValue(), offsetY.getValue().intValue());
            internalMap.setObservations(map);
            if (message.getParameters().size() > 5) {
                mergeKnownAgents(message.getParameters().get(5), sender);
            }
            leaderName = newLeader.getValue();
            notifyKnownForNewLeader(previousLeader.getValue(), newLeader.getValue(),
                    offsetX.getValue().intValue(), offsetY.getValue().intValue(), sender);
            sendMergedMapUpdates();
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
            internalMap.mergeObservations(message.getParameters().get(4));
            rememberKnownAgent(sender,
                    new InternalMap.Position(senderX.getValue().intValue(), senderY.getValue().intValue()), sender);
            updateKnownAgentGroupState(sender, message);
            if (message.getParameters().size() > 6) {
                mergeKnownAgents(message.getParameters().get(6), sender);
            }
            if (message.getParameters().size() > 9) {
                mergeGoalReservationSnapshots(message.getParameters().get(9));
            }
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
                        internalMap.getAgentX(), internalMap.getAgentY(),
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

            pendingTeammateAcceptances.put(sender,
                    new PendingTeammateConfirmation(
                        senderLeaderName.getValue(), senderX.getValue().intValue(),
                        senderY.getValue().intValue(), x.getValue().intValue(),
                        y.getValue().intValue()));
            sendMessage(new Percept("teammateConfirm", new Identifier(getName())),
                    sender, getName());
            return;
        }

        if (message.getName().equals("teammateConfirm")
                && message.getParameters().size() >= 1
                && message.getParameters().get(0) instanceof Identifier identifier
                && sender.equals(identifier.getValue())) {
            PendingTeammateConfirmation confirmation = pendingTeammateConfirmations.remove(sender);
            if (confirmation != null) {
                applyTeammateConfirmation(sender, confirmation);
                sendMessage(new Percept("teammateConfirmed", new Identifier(getName())),
                        sender, getName());
            }
            return;
        }

        if (message.getName().equals("teammateConfirmed")
                && message.getParameters().size() >= 1
                && message.getParameters().get(0) instanceof Identifier identifier
                && sender.equals(identifier.getValue())) {
            PendingTeammateConfirmation confirmation = pendingTeammateAcceptances.remove(sender);
            if (confirmation != null) {
                applyTeammateConfirmation(sender, confirmation);
                sendMapMerge(sender);
            }
            return;
        }

        if (message.getName().equals("groupInvite")
                && message.getParameters().size() >= 3
                && message.getParameters().get(0) instanceof Identifier leader
                && message.getParameters().get(1) instanceof Identifier task
                && message.getParameters().get(2) instanceof Numeral size) {
            if (currentTask == null
                    || !currentTask.equals(task.getValue())
                    || !isTaskActive()
                    || DEFAULT_ROLE.equals(currentRole)
                    || Boolean.TRUE.equals(knownAgentGroupState.get(getName()))
                    || groupFormationActive) {
                sendMessage(new Percept("groupInviteRejected",
                        new Identifier(getName())), leader.getValue(), getName());
                return;
            }

            currentGroupLeader = leader.getValue();
            groupTaskName = task.getValue();
            desiredGroupSize = size.getValue().intValue();
            groupFormationActive = true;
            groupLeaderMode = false;
            knownAgentGroupState.put(getName(), true);
            knownAgentGroupLeader.put(getName(), leader.getValue());
            currentGroupMembers.clear();
            currentGroupMembers.add(leader.getValue());
            currentGroupMembers.add(getName());
            sendMessage(new Percept("groupJoinAccepted",
                    new Identifier(getName()),
                    new Identifier(groupTaskName),
                    new Numeral(desiredGroupSize)), leader.getValue(), getName());
            return;
        }

        if (message.getName().equals("groupInviteRejected")
                && message.getParameters().size() >= 1
                && message.getParameters().get(0) instanceof Identifier rejectedAgent
                && groupLeaderMode) {
            pendingGroupInvitations.remove(rejectedAgent.getValue());
            currentGroupInviteTarget = null;
            rejectedGroupInviteTargets.add(rejectedAgent.getValue());
            return;
        }

        if (message.getName().equals("groupJoinAccepted")
                && message.getParameters().size() >= 2
                && message.getParameters().get(0) instanceof Identifier agent
                && message.getParameters().get(1) instanceof Identifier
                && groupLeaderMode) {
            pendingGroupInvitations.remove(agent.getValue());
            currentGroupInviteTarget = null;
            knownAgentGroupState.put(agent.getValue(), true);
            knownAgentGroupLeader.put(agent.getValue(), getName());
            currentGroupMembers.add(agent.getValue());
            if (currentGroupMembers.size() >= desiredGroupSize) {
                recruitNextGroupLeader();
            }
            return;
        }

        if (message.getName().equals("groupStart")
                && message.getParameters().isEmpty()) {
            if (!DEFAULT_ROLE.equals(currentRole)
                    && !groupFormationActive
                    && (!knownAgentGroupState.getOrDefault(getName(), false)
                    || currentGroupLeader.equals(sender))) {
                startGroupFormationAsSuccessor();
            }
            return;
        }

        if ((message.getName().equals("groupBlockTask")
                || message.getName().equals("blockTask")
                || message.getName().equals("retrieveBlockTask"))
                && message.getParameters().size() >= 3
                && message.getParameters().get(0) instanceof Identifier blockType
                && message.getParameters().get(1) instanceof Numeral targetX
                && message.getParameters().get(2) instanceof Numeral targetY) {
            String newBlockType = blockType.getValue();
            InternalMap.Position newGoalPosition = new InternalMap.Position(
                    targetX.getValue().intValue(), targetY.getValue().intValue());
            boolean sameAssignment = newBlockType.equalsIgnoreCase(deliveryBlockType)
                    && newGoalPosition.equals(goalPosition);
            deliveryBlockType = blockType.getValue();
            goalPosition = newGoalPosition;
            if (message.getParameters().size() >= 4
                    && message.getParameters().get(3) instanceof Identifier expectedDirection) {
                retrieveBlockDirection = expectedDirection.getValue();
            }
            if (!sameAssignment || (!blockRequested && !blockRetrieved)) {
                prepareForBlockAssignment();
            }
            currentIntention = null;
            return;
        }

        if (message.getName().equals("groupBlockDelivered")
                && message.getParameters().size() >= 3
                && message.getParameters().get(0) instanceof Identifier blockType
                && message.getParameters().get(1) instanceof Numeral targetX
                && message.getParameters().get(2) instanceof Numeral targetY
            && isCurrentGroupLeader()) {
            InternalMap.Position target = new InternalMap.Position(
                    targetX.getValue().intValue(), targetY.getValue().intValue());
            pendingAssemblyDeliveries.put(target,
                    new PendingAssemblyAttachment(sender, blockType.getValue(), target));
            if (goalPosition == null) {
                goalPosition = target;
            }
            startNextPendingAssemblyConnection();
            return;
        }

        if (message.getName().equals("groupConnectRequest")
                && message.getParameters().size() >= 5
                && message.getParameters().get(0) instanceof Numeral leaderBlockX
                && message.getParameters().get(1) instanceof Numeral leaderBlockY
                && message.getParameters().get(2) instanceof Numeral targetX
                && message.getParameters().get(3) instanceof Numeral targetY
                && message.getParameters().get(4) instanceof Identifier blockType
                && sender.equals(currentGroupLeader)) {
            pendingAssemblyConnection = new PendingAssemblyConnection(
                    sender,
                    blockType.getValue(),
                    new InternalMap.Position(targetX.getValue().intValue(), targetY.getValue().intValue()),
                    leaderBlockX.getValue().intValue(), leaderBlockY.getValue().intValue());
            if (carriedBlockPosition == null && retrieveBlockDirection != null) {
                int[] offset = directionOffset(retrieveBlockDirection);
                carriedBlockPosition = new InternalMap.Position(
                        internalMap.getAgentX() + offset[0],
                        internalMap.getAgentY() + offset[1]);
            }
            pendingAction = null;
            pendingDirection = null;
            currentIntention = new Intention(Desire.RETRIEVE_BLOCK, List.of("connect"), 0);
            return;
        }

        if (message.getName().equals("groupConnectCompleted")
                && message.getParameters().size() >= 3
                && message.getParameters().get(0) instanceof Identifier blockType
                && message.getParameters().get(1) instanceof Numeral targetX
                && message.getParameters().get(2) instanceof Numeral targetY
                && sender.equals(currentGroupLeader)
                && pendingAssemblyConnection != null
                && pendingAssemblyConnection.blockType().equalsIgnoreCase(blockType.getValue())
                && pendingAssemblyConnection.targetPosition().equals(new InternalMap.Position(
                    targetX.getValue().intValue(), targetY.getValue().intValue()))) {
            pendingAssemblyConnection = null;
            currentIntention = null;
            blockPlaced = true;
            return;
        }

        if (message.getName().equals("groupDetachBlock")
                && !currentGroupLeader.isEmpty()
                && sender.equals(currentGroupLeader)) {
            detachRequested = true;
            currentIntention = null;
            return;
        }

        if (message.getName().equals("groupBlockDetached")
            && isCurrentGroupLeader()) {
            pendingAssemblyDetaches.remove(sender);
            finishAssemblyCleanupIfReady();
            return;
        }

        if (message.getName().equals("newRetrieveBlockLocation")
                && message.getParameters().size() >= 3
                && message.getParameters().get(0) instanceof Identifier blockType
                && message.getParameters().get(1) instanceof Numeral targetX
                && message.getParameters().get(2) instanceof Numeral targetY) {
            if (currentGroupLeader.isEmpty() || sender.equals(currentGroupLeader)) {
                deliveryBlockType = blockType.getValue();
                goalPosition = new InternalMap.Position(
                        targetX.getValue().intValue(), targetY.getValue().intValue());
                blockPlaced = false;
                currentIntention = null;
            }
            return;
        }

        if (message.getName().equals("retrieveBlockLocationUnavailable")
                && isCurrentGroupLeader()) {
            if (message.getParameters().size() >= 3
                    && message.getParameters().get(0) instanceof Identifier blockType
                    && message.getParameters().get(1) instanceof Numeral targetX
                    && message.getParameters().get(2) instanceof Numeral targetY
                    && deliveryBlockType != null
                    && deliveryBlockType.equalsIgnoreCase(blockType.getValue())) {
                goalPosition = new InternalMap.Position(
                        targetX.getValue().intValue(), targetY.getValue().intValue());
                currentIntention = null;
                notifyGroupOfNewRetrieveBlockLocation();
            } else {
                if (goalPosition != null) {
                    internalMap.forgetGoalZone(goalPosition);
                }
                refreshGroupGoalLocation();
            }
            return;
        }

        if (message.getName().equals("groupDissolve")) {
            resetRetrieveAssignment();
            pendingGroupInvitations.clear();
            currentGroupInviteTarget = null;
            knownAgentGroupState.put(getName(), false);
            knownAgentGroupLeader.remove(getName());
            currentGroupMembers.clear();
            groupLeaderMode = false;
            groupFormationActive = false;
            currentGroupLeader = "";
            groupTaskName = "";
            explorationFinished = false;
            return;
        }

    }

    /** Replies only when exactly one queued teammate request matches this agent's view. */
    private void processTeammateRequests() {
        List<PendingTeammateRequest> requests;
        synchronized (pendingTeammateRequests) {
            requests = new ArrayList<>(pendingTeammateRequests);
            pendingTeammateRequests.clear();
        }

        List<PendingTeammateRequest> matchingRequests = requests.stream()
            .filter(request -> isVisibleTeammateAt(
                -adjustedRelativeX(request), -adjustedRelativeY(request)))
                .filter(request -> !knownAgents.containsKey(request.sender()))
            .filter(request -> isConsistentWithOwnView(
                -adjustedRelativeX(request), -adjustedRelativeY(request),
                        request.senderVisibleThings()))
                .toList();

        if (matchingRequests.size() != 1) {
            return;
        }

        PendingTeammateRequest request = matchingRequests.get(0);
        int relativeX = adjustedRelativeX(request);
        int relativeY = adjustedRelativeY(request);
        pendingTeammateConfirmations.put(request.sender(),
                new PendingTeammateConfirmation(
                    request.senderLeaderName(), request.senderX(), request.senderY(),
                    relativeX, relativeY));
        sendMessage(new Percept("teammateReply",
                new Identifier(getName()),
            new Numeral(-relativeX),
            new Numeral(-relativeY),
                new Numeral(internalMap.getAgentX()),
                new Numeral(internalMap.getAgentY()),
                new Identifier(leaderName),
                currentVisibleThingsParameters()), request.sender(), getName());
    }

    private int adjustedRelativeX(PendingTeammateRequest request) {
        return request.x() - (internalMap.getAgentX() - request.receiverX());
    }

    private int adjustedRelativeY(PendingTeammateRequest request) {
        return request.y() - (internalMap.getAgentY() - request.receiverY());
    }

    private void updateCurrentVisibleThings(List<Percept> percepts) {
        currentVisibleThings.clear();
        Set<InternalMap.Position> attachedPositions = new HashSet<>();
        for (Percept percept : percepts) {
            if (percept.getName().equals("attached")
                && percept.getParameters().size() >= 2
                && percept.getParameters().get(0) instanceof Numeral x
                && percept.getParameters().get(1) instanceof Numeral y) {
                    attachedPositions.add(new InternalMap.Position(x.getValue().intValue(), y.getValue().intValue()));
            } else if (percept.getName().equals("thing")
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
            } else if ((percept.getName().equals("goalZone")
                    || percept.getName().equals("roleZone"))
                    && percept.getParameters().size() >= 2
                    && percept.getParameters().get(0) instanceof Numeral x
                    && percept.getParameters().get(1) instanceof Numeral y) {
                        currentVisibleThings.add(new VisibleThing(x.getValue().intValue(), y.getValue().intValue(), percept.getName(), ""));
            }
        }
            currentVisibleThings.removeIf(thing ->
                thing.type().equals("block")
                    && attachedPositions.contains(new InternalMap.Position(thing.x(), thing.y())));
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

    /** Checks shared visible things before accepting a claimed teammate identity. */
    private boolean isConsistentWithOwnView(int relativeX, int relativeY,
            List<VisibleThing> reportedThings) {
        int matchingVerificationThings = 0;

        for (VisibleThing thing : reportedThings) {
            int ownX = relativeX + thing.x();
            int ownY = relativeY + thing.y();

            if (Math.abs(ownX) + Math.abs(ownY) > VISION_RANGE) {
                continue;
            }

            boolean matches = currentVisibleThings.stream().anyMatch(own ->
                    own.x() == ownX && own.y() == ownY
                        && own.type().equals(thing.type())
                        && own.details().equals(thing.details()));

            if (!matches) {
                return false;
            }

            if (!thing.type().equals("entity")) {
                matchingVerificationThings++;
            }
        }
        return matchingVerificationThings >= MIN_SHARED_VERIFICATION_THINGS;
    }

    private void sendMapMerge(String recipient) {
        sendMessage(new Percept("mapMerge", mapParameters()), recipient, getName());
    }

    private ParameterList knownAgentsParameters() {
        ParameterList agents = new ParameterList();
        for (Map.Entry<String, InternalMap.Position> entry : knownAgents.entrySet()) {
            if (entry.getKey().equals(getName())) {
                continue;
            }
            InternalMap.Position position = entry.getValue();
            boolean inGroup = Boolean.TRUE.equals(knownAgentGroupState.getOrDefault(entry.getKey(), false));
            String groupLeader = knownAgentGroupLeader.getOrDefault(entry.getKey(), "");
            agents.add(new Function("knownAgent",
                    new Identifier(entry.getKey()),
                    new Numeral(position.x()),
                    new Numeral(position.y()),
                    new Identifier(knownAgentSources.getOrDefault(entry.getKey(), "unknown")),
                    new Identifier(inGroup ? "grouped" : "free"),
                    new Identifier(groupLeader)));
        }
        return agents;
    }

    private void mergeKnownAgents(Parameter parameter, String sender) {
        if (!(parameter instanceof ParameterList agents)) {
            return;
        }
        for (Parameter entry : agents) {
            if (!(entry instanceof Function knownAgent)
                    || !knownAgent.getName().equals("knownAgent")
                    || knownAgent.getParameters().size() < 4
                    || !(knownAgent.getParameters().get(0) instanceof Identifier agent)
                    || !(knownAgent.getParameters().get(1) instanceof Numeral x)
                    || !(knownAgent.getParameters().get(2) instanceof Numeral y)
                    || !(knownAgent.getParameters().get(3) instanceof Identifier source)
                    || agent.getValue().equals(getName())) {
                continue;
            }

            boolean inGroup = false;
            String groupLeader = "";
            if (knownAgent.getParameters().size() >= 6
                    && knownAgent.getParameters().get(4) instanceof Identifier groupState
                    && knownAgent.getParameters().get(5) instanceof Identifier leaderIdentifier) {
                inGroup = isGroupedState(groupState);
                groupLeader = leaderIdentifier.getValue();
            } else if (knownAgent.getParameters().size() >= 5
                    && knownAgent.getParameters().get(4) instanceof Identifier groupState) {
                inGroup = isGroupedState(groupState);
            }

            rememberKnownAgent(agent.getValue(),
                    new InternalMap.Position(x.getValue().intValue(), y.getValue().intValue()),
                    sender + ">" + source.getValue(), inGroup, groupLeader);
        }
    }

    private void updateKnownAgentGroupState(String agent, Percept message) {
        if (message.getParameters().size() < 9
                || !(message.getParameters().get(7) instanceof Identifier groupState)
                || !(message.getParameters().get(8) instanceof Identifier groupLeader)) {
            return;
        }

        boolean inGroup = isGroupedState(groupState);
        knownAgentGroupState.put(agent, inGroup);
        if (inGroup) {
            knownAgentGroupLeader.put(agent, groupLeader.getValue());
        } else {
            knownAgentGroupLeader.remove(agent);
            rejectedGroupInviteTargets.remove(agent);
        }
    }

    private boolean isGroupedState(Identifier groupState) {
        return "grouped".equalsIgnoreCase(groupState.getValue())
                || "true".equalsIgnoreCase(groupState.getValue());
    }

    private ParameterList mapParameters() {
        return internalMap.toParameterList();
    }

    private void switchLeader(String newLeaderName, int deltaX, int deltaY) {
        String previousLeader = leaderName;
        translateWorld(deltaX, deltaY);
        leaderName = newLeaderName;
        notifyKnownForNewLeader(previousLeader, newLeaderName, deltaX, deltaY, null);
    }

    private void notifyKnownForNewLeader(String previousLeader, String newLeader,
            int offsetX, int offsetY, String excludedAgent) {
        for (String agent : new ArrayList<>(knownAgents.keySet())) {
            if (agent.equals(getName()) || agent.equals(excludedAgent)) {
                continue;
            }
            sendMessage(new Percept("newLeader",
                    new Identifier(previousLeader),
                    new Identifier(newLeader),
                    new Numeral(offsetX),
                    new Numeral(offsetY),
                    mapParameters(),
                    knownAgentsParameters()), agent, getName());
        }
    }

    private void translateKnownPositions(int offsetX, int offsetY) {
        for (Map.Entry<String, InternalMap.Position> entry : knownAgents.entrySet()) {
            InternalMap.Position position = entry.getValue();
            entry.setValue(new InternalMap.Position(
                    position.x() + offsetX, position.y() + offsetY));
        }
        for (Map.Entry<String, InternalMap.Position> entry : knownTargets.entrySet()) {
            InternalMap.Position position = entry.getValue();
            entry.setValue(new InternalMap.Position(
                    position.x() + offsetX, position.y() + offsetY));
        }
        for (Map.Entry<String, GoalReservationSnapshot> entry : goalReservationSnapshots.entrySet()) {
            GoalReservationSnapshot snapshot = entry.getValue();
            Set<InternalMap.Position> translatedPositions = new HashSet<>();
            for (InternalMap.Position position : snapshot.positions()) {
                translatedPositions.add(new InternalMap.Position(
                        position.x() + offsetX, position.y() + offsetY));
            }
            entry.setValue(new GoalReservationSnapshot(snapshot.revision(),
                    snapshot.updatedAtStep(), translatedPositions));
        }
        if (explorationTarget != null) {
            explorationTarget = new InternalMap.Position(
                    explorationTarget.x() + offsetX, explorationTarget.y() + offsetY);
        }
        if (goalPosition != null) {
            goalPosition = new InternalMap.Position(
                    goalPosition.x() + offsetX, goalPosition.y() + offsetY);
        }
    }

    private void translateWorld(int offsetX, int offsetY) {
        internalMap.translate(offsetX, offsetY);
        translateKnownPositions(offsetX, offsetY);
    }

    private void updateAgentPosition(List<Percept> percepts) {
        internalMap.updateAgentPositionFromPercepts(percepts);
    }

    /** Updates task, role, step, and deactivation beliefs from the current percept batch. */
    private void updateBeliefs(List<Percept> percepts) {
        currentTasks.clear();
        boolean taskPerceptReceived = false;
        boolean wasDeactivated = deactivated;
        String previousAction = null;
        String previousActionResult = null;
        String previousTask = currentTask;

        for (Percept percept : percepts) {
            if (percept.getParameters().isEmpty()) {
                continue;
            }

            switch (percept.getName()) {
                case "step" -> currentStep = numberValue(percept, currentStep);
                case "role" -> {
                    if (percept.getParameters().size() == 1
                            && percept.getParameters().get(0) instanceof Identifier identifier) {
                        currentRole = identifier.getValue();
                    }
                }
                case "team" -> teamName = identifierValue(percept, teamName);
                case "deactivated" -> deactivated = identifierValue(percept, "false").equals("true");
                case "lastAction" -> previousAction = identifierValue(percept, null);
                case "lastActionResult" -> previousActionResult = identifierValue(percept, null);
                case "task" -> {
                    taskPerceptReceived = true;
                    currentTasks.add(TaskInfo.fromPercept(percept));
                }
                default -> {
                }
            }
        }
        boolean previousSubmitSucceeded = "submit".equals(previousAction) && "success".equals(previousActionResult);

        boolean currentTaskAvailable = currentTasks.stream().anyMatch(task -> task.name().equals(currentTask));
        boolean selectNewTask = currentTask == null || !currentTaskAvailable || !isTaskActive();
        if (selectNewTask) {
            TaskInfo selectedTask = currentTasks.stream()
                    .filter(task -> task.activeAt(currentStep))
                    .max((first, second) -> Long.compare(
                            first.remainingAt(currentStep), second.remainingAt(currentStep)))
                    .orElse(null);
            if (selectedTask != null) {
                applyTaskInfo(selectedTask);
            } else if (!currentTaskAvailable || !isTaskActive()) {
                currentTask = null;
                currentTaskInfo = null;
                requiredDispenserTypes.clear();
            }
        }

        if (!taskPerceptReceived) {
            if (previousTask != null) {
                resetGroupStateForNewTask();
                resetRetrieveAssignment();
                clearPendingActionState();
            }
            if (previousSubmitSucceeded) {
                finishAssemblyAfterSuccessfulSubmit();
            } else if (blockRetrieved) {
                detachRequested = true;
            } else if (previousTask != null) {
                resetCarriedBlockTracking();
            }
            currentTask = null;
            currentTaskInfo = null;
            requiredDispenserTypes.clear();
            currentTaskBlockCount = 1;
            desiredGroupSize = 1;
            requiredBlockOffset = null;
            deliveryBlockType = null;
            goalPosition = null;
        } else if (!Objects.equals(currentTask, previousTask)) {
            resetGroupStateForNewTask();
            deliveryBlockType = null;
            goalPosition = null;
            resetRetrieveAssignment();
            if (previousSubmitSucceeded) {
                finishAssemblyAfterSuccessfulSubmit();
            } else if (blockRetrieved) {
                detachRequested = true;
            }
            clearPendingActionState();
        } else if (!isTaskActive()) {
            deliveryBlockType = null;
            goalPosition = null;
            resetRetrieveAssignment();
            if (previousSubmitSucceeded) {
                finishAssemblyAfterSuccessfulSubmit();
            } else if (blockRetrieved) {
                detachRequested = true;
            }
            clearPendingActionState();
        }

        if (deactivated && !wasDeactivated) {
            resetAssemblyStateAfterDeactivation();
        } else if (deactivated) {
            resetCarriedBlockTracking();
        }
    }

    private void resetAssemblyStateAfterDeactivation() {
        attachedBlockDirections.clear();
        pendingAssemblyDetaches.clear();
        assemblyCleanupRequested = false;
        groupGoalRelocationPending = false;
        resetCarriedBlockTracking();
        resetGroupStateForNewTask();
        resetRetrieveAssignment();
        clearPendingActionState();
    }

    private void applyTaskInfo(TaskInfo taskInfo) {
        currentTaskInfo = taskInfo;
        currentTask = taskInfo == null ? null : taskInfo.name();
        requiredDispenserTypes.clear();
        requiredDispenserTypes.addAll(taskInfo.blockTypes());
        taskBlockTypes.clear();
        taskBlockTypes.addAll(taskInfo.blockTypes());
        taskRequirementOffsets.clear();
        taskRequirementOffsets.addAll(taskInfo.offsets());
        currentTaskBlockCount = taskInfo.blockTypes().size();
        requiredBlockOffset = taskInfo.offsets().isEmpty() ? null : taskInfo.offsets().get(0);
        if (currentTaskBlockCount <= 0) {
            currentTaskBlockCount = 1;
        }
        desiredGroupSize = taskInfo.groupSize();
    }

    private boolean isVisibleTeammateAt(int x, int y) {
        return internalMap.getVisibleTeammates().contains(new InternalMap.Position(x, y));
    }

    private String serverAgentName(String agentName) {
        if (agentName == null || agentName.startsWith(SERVER_AGENT_PREFIX)) {
            return agentName;
        }
        return SERVER_AGENT_PREFIX + agentName;
    }

    private void exchangeTeammateNames() {
        for (InternalMap.Position teammate : internalMap.getVisibleTeammates()) {
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
        sendMapUpdates(internalMap.currentPercepts(percepts, currentStep));
    }

    private void sendMapUpdates(ParameterList content) {
        ParameterList knownAgentsContent = knownAgentsParameters();
        InternalMap.Position target = explorationTarget;
        Percept update = new Percept(
                "mapUpdate",
                new Numeral(internalMap.getAgentX()),
                new Numeral(internalMap.getAgentY()),
                new Numeral(target == null ? Integer.MIN_VALUE : target.x()),
                new Numeral(target == null ? Integer.MIN_VALUE : target.y()),
                content,
                new Identifier(leaderName),
                knownAgentsContent,
                new Identifier(Boolean.TRUE.equals(knownAgentGroupState.get(getName()))
                        ? "grouped" : "free"),
                new Identifier(knownAgentGroupLeader.getOrDefault(getName(), "")),
                currentGoalReservationParameters());
        for (String agent : knownAgents.keySet()) {
            if (agent.equals(getName())) {
                continue;
            }
            sendMessage(update, agent, getName());
        }
    }

    private void sendMergedMapUpdates() {
        sendMapUpdates(mapParameters());
    }

    private ParameterList currentGoalReservationParameters() {
        publishOwnGoalReservationSnapshot();
        expireGoalReservationSnapshots();

        ParameterList reservations = new ParameterList();
        for (Map.Entry<String, GoalReservationSnapshot> entry : goalReservationSnapshots.entrySet()) {
            GoalReservationSnapshot snapshot = entry.getValue();
            ParameterList positions = new ParameterList();
            for (InternalMap.Position position : snapshot.positions()) {
                positions.add(new Function("reservedPosition",
                        new Numeral(position.x()), new Numeral(position.y())));
            }
            reservations.add(new Function("goalReservation",
                    new Identifier(entry.getKey()),
                    new Numeral(snapshot.revision()),
                    new Numeral(snapshot.updatedAtStep()),
                    positions));
        }
        return reservations;
    }

    private void publishOwnGoalReservationSnapshot() {
        GoalReservationSnapshot ownSnapshot = goalReservationSnapshots.get(getName());
        if (ownSnapshot != null && currentStep > ownSnapshot.updatedAtStep()) {
            updateOwnGoalReservationSnapshot(ownSnapshot.positions());
        }
    }

    private void mergeGoalReservationSnapshots(Parameter parameter) {
        if (!(parameter instanceof ParameterList reservations)) {
            return;
        }
        for (Parameter entry : reservations) {
            if (!(entry instanceof Function reservation)
                    || !reservation.getName().equals("goalReservation")
                    || reservation.getParameters().size() < 4
                    || !(reservation.getParameters().get(0) instanceof Identifier owner)
                    || !(reservation.getParameters().get(1) instanceof Numeral revision)
                    || !(reservation.getParameters().get(2) instanceof Numeral updatedAtStep)
                    || !(reservation.getParameters().get(3) instanceof ParameterList positions)
                    || owner.getValue().equals(getName())) {
                continue;
            }

            String ownerName = owner.getValue();
            int snapshotRevision = revision.getValue().intValue();
            if (snapshotRevision <= latestGoalReservationRevisions.getOrDefault(ownerName, 0)) {
                continue;
            }

            Set<InternalMap.Position> reservedPositions = new HashSet<>();
            for (Parameter position : positions) {
                if (position instanceof Function reservedPosition
                        && reservedPosition.getName().equals("reservedPosition")
                        && reservedPosition.getParameters().size() >= 2
                        && reservedPosition.getParameters().get(0) instanceof Numeral x
                        && reservedPosition.getParameters().get(1) instanceof Numeral y) {
                    reservedPositions.add(new InternalMap.Position(
                            x.getValue().intValue(), y.getValue().intValue()));
                }
            }

            int snapshotStep = updatedAtStep.getValue().intValue();
            latestGoalReservationRevisions.put(ownerName, snapshotRevision);
            GoalReservationSnapshot snapshot = new GoalReservationSnapshot(
                    snapshotRevision, snapshotStep, reservedPositions);
            if (isGoalReservationSnapshotFresh(snapshot)) {
                goalReservationSnapshots.put(ownerName, snapshot);
            } else {
                goalReservationSnapshots.remove(ownerName);
            }
        }
    }

    private void updateOwnGoalReservationSnapshot(Set<InternalMap.Position> positions) {
        int revision = latestGoalReservationRevisions.getOrDefault(getName(), 0) + 1;
        latestGoalReservationRevisions.put(getName(), revision);
        goalReservationSnapshots.put(getName(),
                new GoalReservationSnapshot(revision, currentStep, positions));
    }

    private void expireGoalReservationSnapshots() {
        goalReservationSnapshots.entrySet().removeIf(entry ->
                !entry.getKey().equals(getName())
                        && !isGoalReservationSnapshotFresh(entry.getValue()));
    }

    private boolean isGoalReservationSnapshotFresh(GoalReservationSnapshot snapshot) {
        return currentStep < 0 || currentStep - snapshot.updatedAtStep() <= GOAL_RESERVATION_MAX_AGE;
    }

    private int calculateDesiredGroupSize() {
        if (currentTaskBlockCount <= 1) {
            return 1;
        }
        return currentTaskBlockCount + 1;
    }

    private boolean isTaskActive() {
        return currentTaskInfo == null || currentTaskInfo.deadline() < 0 || currentStep <= currentTaskInfo.deadline();
    }

    private void resetGroupStateForNewTask() {
        groupGoalRelocationPending = false;
        requestAssemblyCleanup();
        for (String member : new ArrayList<>(currentGroupMembers)) {
            if (!member.equals(getName()) && !assemblyCleanupRequested) {
                sendMessage(new Percept("groupDissolve"), member, getName());
            } else if (!member.equals(getName())) {
                deferredGroupDissolveMembers.add(member);
            }
        }
        for (String agent : knownAgents.keySet()) {
            knownAgentGroupState.put(agent, false);
            knownAgentGroupLeader.remove(agent);
        }
        knownAgentGroupState.put(getName(), false);
        knownAgentGroupLeader.remove(getName());
        currentGroupMembers.clear();
        pendingGroupInvitations.clear();
        currentGroupInviteTarget = null;
        groupLeaderMode = false;
        groupFormationActive = false;
        currentGroupLeader = "";
        groupTaskName = "";
    }

    private void startGroupFormation() {
        if (!leaderName.equals(getName())) {
            return;
        }
        startGroupFormationCore();
    }

    private void startGroupFormationAsSuccessor() {
        if (isCarryingBlock() || !attachedBlockDirections.isEmpty()) {
            successorGroupFormationAfterDetach = true;
            if (attachedBlockDirections.isEmpty()) {
                detachRequested = true;
            } else {
                assemblyCleanupRequested = true;
            }
            currentIntention = null;
            return;
        }
        startGroupFormationCore();
    }

    private boolean canFormTaskGroup(TaskInfo taskInfo) {
        if (!taskInfo.activeAt(currentStep)) {
            return false;
        }

        int freeAgents = 1;
        for (String agent : knownAgents.keySet()) {
            if (!agent.equals(getName())
                    && !Boolean.TRUE.equals(knownAgentGroupState.get(agent))) {
                freeAgents++;
            }
        }
        return taskInfo.groupSize() <= freeAgents;
    }

    private boolean hasFormableGroupTask() {
        return currentTasks.stream().anyMatch(task -> canFormTaskGroup(task) && task.groupSize() > 1);
    }

    /** Selects an available group task and begins inviting known free agents. */
    private void startGroupFormationCore() {
        TaskInfo selectedTask = currentTasks.stream()
                .filter(this::canFormTaskGroup)
                .max((first, second) -> Long.compare(
                        first.remainingAt(currentStep), second.remainingAt(currentStep)))
                .orElse(null);
        if (selectedTask == null) {
            return;
        }

        applyTaskInfo(selectedTask);

        if (desiredGroupSize <= 1
                || groupFormationActive
                || !explorationFinished
                || currentTask == null
                || currentTask.isEmpty()
                || DEFAULT_ROLE.equals(currentRole)
                || !isTaskActive()) {
            return;
        }

        InternalMap.Observation selectedGoalZone = findNearestGoalZone();
        if (selectedGoalZone == null) {
            return;
        }

        pendingGroupInvitations.clear();
        rejectedGroupInviteTargets.clear();
        currentGroupInviteTarget = null;
        desiredGroupSize = calculateDesiredGroupSize();
        groupTaskName = currentTask;
        groupLeaderMode = true;
        groupFormationActive = true;
        currentGroupLeader = getName();
        currentGroupMembers.clear();
        currentGroupMembers.add(getName());
        goalPosition = new InternalMap.Position(selectedGoalZone.x(), selectedGoalZone.y());
        reserveGroupGoalPositions(goalPosition);
        groupGoalZoneConfirmed = true;
        knownAgentGroupState.put(getName(), true);
        knownAgentGroupLeader.put(getName(), getName());
        inviteKnownAgentsToGroup();
    }

    private void inviteKnownAgentsToGroup() {
        if (!groupLeaderMode || !groupFormationActive) {
            return;
        }
        if (currentGroupMembers.size() >= desiredGroupSize) {
            return;
        }
        if (!pendingGroupInvitations.isEmpty() || currentGroupInviteTarget != null) {
            return;
        }

        Set<String> invitationCandidates = new HashSet<>(knownAgents.keySet());
        invitationCandidates.addAll(knownAgentGroupState.keySet());
        InternalMap.Position leaderPosition = currentPosition();
        List<String> sortedInvitationCandidates = new ArrayList<>(invitationCandidates);
        sortedInvitationCandidates.sort((first, second) -> {
            InternalMap.Position firstPosition = knownAgents.get(first);
            InternalMap.Position secondPosition = knownAgents.get(second);
            int firstDistance = firstPosition == null
                    ? Integer.MAX_VALUE
                    : distanceTo(firstPosition.x(), firstPosition.y(),
                        leaderPosition.x(), leaderPosition.y());
            int secondDistance = secondPosition == null
                    ? Integer.MAX_VALUE
                    : distanceTo(secondPosition.x(), secondPosition.y(),
                        leaderPosition.x(), leaderPosition.y());
            int distanceComparison = Integer.compare(firstDistance, secondDistance);
            return distanceComparison != 0
                    ? distanceComparison
                    : first.compareTo(second);
        });
        for (String agent : sortedInvitationCandidates) {
            if (agent.equals(getName())
                    || Boolean.TRUE.equals(knownAgentGroupState.get(agent))
                    || rejectedGroupInviteTargets.contains(agent)) {
                continue;
            }
            currentGroupInviteTarget = agent;
            pendingGroupInvitations.add(agent);
            sendMessage(new Percept("groupInvite",
                    new Identifier(getName()),
                    new Identifier(groupTaskName),
                    new Numeral(desiredGroupSize)), agent, getName());
            return;
        }
    }

    private void recruitNextGroupLeader() {
        if (!groupLeaderMode || !groupFormationActive || currentGroupMembers.size() < desiredGroupSize) {
            return;
        }

        assignBlocksToCurrentGroup();
        groupFormationActive = false;

        for (String agent : knownAgents.keySet()) {
            if (agent.equals(getName())
                    || currentGroupMembers.contains(agent)
                    || Boolean.TRUE.equals(knownAgentGroupState.get(agent))
                    || rejectedGroupInviteTargets.contains(agent)) {
                continue;
            }
            sendMessage(new Percept("groupStart"), agent, getName());
            groupLeaderMode = false;
            currentGroupLeader = getName();
            return;
        }
    }

    private void assignBlocksToCurrentGroup() {
        InternalMap.Position assemblyAnchor = goalPosition;
        if (assemblyAnchor == null) {
            InternalMap.Observation selectedGoalZone = findNearestGoalZone();
            if (selectedGoalZone == null) {
                return;
            }
            assemblyAnchor = new InternalMap.Position(selectedGoalZone.x(), selectedGoalZone.y());
        }
        assignBlocksToCurrentGroupAt(assemblyAnchor);
    }

    /**
     * Reserves the assembly area and assigns each fetcher its required block
     * and delivery position relative to the chosen anchor.
     */
    private void assignBlocksToCurrentGroupAt(InternalMap.Position leaderGoalAnchor) {
        List<String> members = new ArrayList<>(currentGroupMembers);
        members.sort(String::compareTo);

        clearAssemblyState();

        goalPosition = leaderGoalAnchor;
        reserveGroupGoalPositions(leaderGoalAnchor);
        groupGoalZoneConfirmed = isKnownGoalZone(leaderGoalAnchor);
        currentIntention = null;

        List<String> fetchers = new ArrayList<>(members);
        fetchers.remove(getName());

        int assignmentCount = Math.min(fetchers.size(),
            Math.min(taskRequirementOffsets.size(), taskBlockTypes.size()));
        for (int index = 0; index < assignmentCount; index++) {
            String member = fetchers.get(index);
            String blockType = taskBlockTypes.get(index);
            InternalMap.Position requirementOffset = taskRequirementOffsets.get(index);
            String requiredDirection = offsetToDirection(requirementOffset);
            InternalMap.Position deliveryTarget = new InternalMap.Position(
                    leaderGoalAnchor.x() + requirementOffset.x(),
                    leaderGoalAnchor.y() + requirementOffset.y());
            sendMessage(new Percept("groupBlockTask",
                    new Identifier(blockType),
                    new Numeral(deliveryTarget.x()),
                    new Numeral(deliveryTarget.y()),
                    new Identifier(requiredDirection),
                    new Numeral(leaderGoalAnchor.x()),
                    new Numeral(leaderGoalAnchor.y())), member, getName());
        }

    }

    /** Keeps the assembly anchor on a reachable, available goal zone. */
    private void refreshGroupGoalLocation() {
        if (goalPosition == null || !isCurrentGroupLeader()) {
            return;
        }

        if (isKnownGoalZone(goalPosition)
            && !hasOpponentOnReservedAssemblyPosition(goalPosition)) {
            groupGoalZoneConfirmed = true;
            groupGoalRelocationPending = false;
            return;
        }

        if (deliveryBlockType == null && !groupGoalZoneConfirmed) {
            return;
        }

        InternalMap.Position replacement = findReachableGoalZone(currentPosition(), goalPosition);
        if (replacement == null) {
            groupGoalRelocationPending = true;
            currentIntention = new Intention(Desire.WAIT, List.of(), 0);
            return;
        }

        groupGoalRelocationPending = false;
        relocateAssemblyGoal(replacement);
    }

    private void tryStartPendingAssemblyConnection(PendingAssemblyAttachment delivery) {
        if (!isCurrentGroupLeader() || goalPosition == null
            || pendingAssemblyConnection != null) {
            return;
        }

        InternalMap.Position bridge = findAssembledBridge(delivery.targetPosition());
        if (bridge == null) {
            return;
        }

        int leaderBlockX = bridge.x() - internalMap.getAgentX();
        int leaderBlockY = bridge.y() - internalMap.getAgentY();
        pendingAssemblyConnection = new PendingAssemblyConnection(
                delivery.member(), delivery.blockType(), delivery.targetPosition(), leaderBlockX, leaderBlockY);
        pendingAssemblyDeliveries.remove(delivery.targetPosition());
        currentIntention = new Intention(Desire.RETRIEVE_BLOCK,
                List.of("connect:" + delivery.member() + ":" + leaderBlockX + ":" + leaderBlockY), 0);
        sendMessage(new Percept("groupConnectRequest",
                new Numeral(leaderBlockX),
                new Numeral(leaderBlockY),
                new Numeral(delivery.targetPosition().x()),
                new Numeral(delivery.targetPosition().y()),
                new Identifier(delivery.blockType())), delivery.member(), getName());
    }

    private void startNextPendingAssemblyConnection() {
        if (goalPosition == null || pendingAssemblyConnection != null || !isAtGoalPosition()) {
            return;
        }
        for (PendingAssemblyAttachment delivery
                : new ArrayList<>(pendingAssemblyDeliveries.values())) {
            if (distanceTo(delivery.targetPosition().x(), delivery.targetPosition().y(),
                    goalPosition.x(), goalPosition.y()) == 1) {
                String attachDirection = directionTo(delivery.targetPosition(), goalPosition);
                pendingAssemblyAttachment = new PendingAssemblyAttachment(
                    delivery.member(), delivery.blockType(), delivery.targetPosition());
                pendingAssemblyDeliveries.remove(delivery.targetPosition());
                currentIntention = new Intention(Desire.RETRIEVE_BLOCK,
                        List.of("attach:" + attachDirection), 0);
                return;
            }
            if (findAssembledBridge(delivery.targetPosition()) != null) {
                tryStartPendingAssemblyConnection(delivery);
                return;
            }
        }
    }

    private InternalMap.Position findAssembledBridge(InternalMap.Position target) {
        for (String direction : List.of("n", "e", "s", "w")) {
            int[] offset = directionOffset(direction);
            InternalMap.Position candidate = new InternalMap.Position(
                    target.x() + offset[0], target.y() + offset[1]);
            if (assembledBlockPositions.contains(candidate)) {
                return candidate;
            }
        }
        return null;
    }

    private boolean isKnownGoalZone(InternalMap.Position position) {
        return internalMap.getObservations().stream().anyMatch(observation ->
                observation.type().equals("goalZone")
                        && observation.x() == position.x()
                        && observation.y() == position.y());
    }

    private void notifyGroupOfNewRetrieveBlockLocation() {
        if (deliveryBlockType == null || goalPosition == null) {
            return;
        }
        Set<String> recipients = new HashSet<>(currentGroupMembers);
        for (Map.Entry<String, String> entry : knownAgentGroupLeader.entrySet()) {
            if (getName().equals(entry.getValue())) {
                recipients.add(entry.getKey());
            }
        }
        for (String member : recipients) {
            if (!member.equals(getName())) {
                sendMessage(new Percept("newRetrieveBlockLocation",
                        new Identifier(deliveryBlockType),
                        new Numeral(goalPosition.x()),
                        new Numeral(goalPosition.y())), member, getName());
            }
        }
    }

    private void dissolveCurrentGroup() {
        if (!groupLeaderMode || !groupFormationActive) {
            return;
        }

        for (String member : new ArrayList<>(currentGroupMembers)) {
            sendMessage(new Percept("groupDissolve"), member, getName());
            knownAgentGroupState.put(member, false);
            knownAgentGroupLeader.remove(member);
        }
        currentGroupMembers.clear();
        pendingGroupInvitations.clear();
        rejectedGroupInviteTargets.clear();
        currentGroupInviteTarget = null;
        groupLeaderMode = false;
        groupFormationActive = false;
        currentGroupLeader = "";
        groupTaskName = "";
        resetRetrieveAssignment();
    }

    private void resetRetrieveAssignment() {
        if (assemblyCleanupRequested) {
            return;
        }
        clearRetrieveAssignmentState();
    }

    private void clearRetrieveAssignmentState() {
        releaseGroupGoalPositions();
        deliveryBlockType = null;
        goalPosition = null;
        pendingAssemblyAttachment = null;
        pendingAssemblyDeliveries.clear();
        clearAssemblyState();
        attachedBlockDirections.clear();
        currentIntention = null;
        if (!blockRetrieved) {
            resetCarriedBlockTracking();
        } else {
            blockRequested = false;
            blockPlaced = false;
        }
    }

    private void requestAssemblyCleanup() {
        if (!isAssemblyLeader()) {
            return;
        }
        if (attachedBlockDirections.isEmpty() && pendingAssemblyDetaches.isEmpty()) {
            return;
        }
        assemblyCleanupRequested = true;
        for (String member : new ArrayList<>(pendingAssemblyDetaches)) {
            sendMessage(new Percept("groupDetachBlock"), member, getName());
        }
    }

    private void finishAssemblyCleanupIfReady() {
        if (!assemblyCleanupRequested
                || !attachedBlockDirections.isEmpty()
                || !pendingAssemblyDetaches.isEmpty()) {
            return;
        }
        assemblyCleanupRequested = false;
        clearRetrieveAssignmentState();
        for (String member : new ArrayList<>(deferredGroupDissolveMembers)) {
            sendMessage(new Percept("groupDissolve"), member, getName());
        }
        deferredGroupDissolveMembers.clear();
        if (successorGroupFormationAfterDetach) {
            successorGroupFormationAfterDetach = false;
            startGroupFormationCore();
        }
    }

    private void finishAssemblyAfterSuccessfulSubmit() {
        assemblyCleanupRequested = false;
        attachedBlockDirections.clear();
        pendingAssemblyDetaches.clear();
        detachRequested = false;
        successorGroupFormationAfterDetach = false;
        clearRetrieveAssignmentState();
        resetCarriedBlockTracking();
        for (String member : new ArrayList<>(deferredGroupDissolveMembers)) {
            sendMessage(new Percept("groupDissolve"), member, getName());
        }
        deferredGroupDissolveMembers.clear();
    }

    private boolean isAssemblyLeader() {
        return isCurrentGroupLeader();
    }

    private void updateGroupState() {
        resetGroupStateAfterTaskChange();

        if (desiredGroupSize > 1
                && goalPosition == null
                && currentTask != null
                && !currentTask.isEmpty()
                && isTaskActive()
                && isCurrentGroupLeader()
                && !groupFormationActive
                && currentGroupMembers.size() >= desiredGroupSize) {
            InternalMap.Observation goalZone = findNearestGoalZone();
            if (goalZone != null) {
                assignBlocksToCurrentGroupAt(
                        new InternalMap.Position(goalZone.x(), goalZone.y()));
            }
        }

        if (desiredGroupSize <= 1
                && deliveryBlockType == null
                && goalPosition == null
                && currentTask != null
                && !currentTask.isEmpty()
                && isTaskActive()
                && !groupFormationActive
                && !groupLeaderMode) {
            InternalMap.Observation goalZone = findNearestGoalZone();
            if (goalZone != null && !taskBlockTypes.isEmpty()) {
                deliveryBlockType = taskBlockTypes.get(0);
                goalPosition = new InternalMap.Position(goalZone.x(), goalZone.y());
                prepareForBlockAssignment();
            }
        }

        if (leaderName.equals(getName())
                && !groupFormationActive
                && explorationFinished
                && !Boolean.TRUE.equals(knownAgentGroupState.get(getName()))
                && hasFormableGroupTask()) {
            startGroupFormation();
        }

        if (groupLeaderMode && groupFormationActive
                && currentGroupMembers.size() < desiredGroupSize
                && currentGroupInviteTarget == null
                && pendingGroupInvitations.isEmpty()) {
            inviteKnownAgentsToGroup();
        }

        if (groupLeaderMode && groupFormationActive) {
            if (!isTaskActive() || currentTask == null || currentTask.isEmpty()
                    || !currentTask.equals(groupTaskName)) {
                dissolveCurrentGroup();
            } else if (currentGroupMembers.size() >= desiredGroupSize) {
                recruitNextGroupLeader();
                groupFormationActive = false;
                groupLeaderMode = false;
                currentGroupLeader = getName();
            }
        }
    }

    private void resetGroupStateAfterTaskChange() {
        if (!groupFormationActive) {
            return;
        }
        if (currentTask != null
                && !currentTask.isEmpty()
                && currentTask.equals(groupTaskName)) {
            return;
        }

        if (groupLeaderMode) {
            dissolveCurrentGroup();
        } else {
            knownAgentGroupState.put(getName(), false);
            knownAgentGroupLeader.remove(getName());
            currentGroupMembers.clear();
            pendingGroupInvitations.clear();
            currentGroupInviteTarget = null;
            groupFormationActive = false;
            currentGroupLeader = "";
            groupTaskName = "";
            resetRetrieveAssignment();
        }

        for (String agent : knownAgents.keySet()) {
            knownAgentGroupState.put(agent, false);
            knownAgentGroupLeader.remove(agent);
        }
        rejectedGroupInviteTargets.clear();
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
            if (percept.getName().equals("actionID")
                    && !percept.getParameters().isEmpty()
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

    /** Advances or repairs the active intention using the previous action result. */
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

        if ("success".equals(lastActionResult)) {
            currentIntention = currentIntention.advance();

            if ("request".equals(lastAction)) {
                blockRequested = true;
            } else if ("rotate".equals(lastAction) && pendingRotation != null) {
                boolean clockwise = "cw".equals(pendingRotation);
                if (retrieveBlockDirection != null) {
                    retrieveBlockDirection = rotateDirection(
                            retrieveBlockDirection, clockwise);
                } else {
                    currentIntention = null;
                }
                if (isAssemblyLeader()) {
                    for (int index = 0; index < attachedBlockDirections.size(); index++) {
                        attachedBlockDirections.set(index,
                                rotateDirection(attachedBlockDirections.get(index), clockwise));
                    }
                }
            } else if ("attach".equals(lastAction)) {
                if (pendingAssemblyAttachment != null) {
                    if (isAssemblyLeader()) {
                        attachedBlockDirections.add(directionTo(
                                pendingAssemblyAttachment.targetPosition(), currentPosition()));
                    }
                    assembledBlockPositions.add(pendingAssemblyAttachment.targetPosition());
                    assembledBlockTypes.put(relativeAssemblyPosition(
                            pendingAssemblyAttachment.targetPosition()),
                            pendingAssemblyAttachment.blockType());
                    pendingAssemblyDetaches.add(pendingAssemblyAttachment.member());
                    sendMessage(new Percept("groupDetachBlock"),
                            pendingAssemblyAttachment.member(), getName());
                    pendingAssemblyDeliveries.remove(pendingAssemblyAttachment.targetPosition());
                    pendingAssemblyAttachment = null;
                    currentIntention = null;
                    resetCarriedBlockTracking();
                    startNextPendingAssemblyConnection();
                    return;
                }
                blockRetrieved = true;
                carriedBlockType = deliveryBlockType;
                if (isAssemblyLeader() && retrieveBlockDirection != null) {
                    attachedBlockDirections.add(retrieveBlockDirection);
                }
            } else if ("connect".equals(lastAction)
                    && pendingAssemblyConnection != null) {
                if (isCurrentGroupLeader()
                        && currentGroupMembers.contains(pendingAssemblyConnection.partner())) {
                    pendingAssemblyDetaches.add(pendingAssemblyConnection.partner());
                    sendMessage(new Percept("groupDetachBlock"),
                            pendingAssemblyConnection.partner(), getName());
                }
                assembledBlockPositions.add(pendingAssemblyConnection.targetPosition());
                assembledBlockTypes.put(relativeAssemblyPosition(
                        pendingAssemblyConnection.targetPosition()),
                        pendingAssemblyConnection.blockType());
                pendingAssemblyDeliveries.remove(pendingAssemblyConnection.targetPosition());
                if (isCurrentGroupLeader()) {
                    sendMessage(new Percept("groupConnectCompleted",
                            new Identifier(pendingAssemblyConnection.blockType()),
                            new Numeral(pendingAssemblyConnection.targetPosition().x()),
                            new Numeral(pendingAssemblyConnection.targetPosition().y())),
                        pendingAssemblyConnection.partner(), getName());
                }
                pendingAssemblyConnection = null;
                currentIntention = null;
                blockPlaced = true;
                startNextPendingAssemblyConnection();
            } else if ("detach".equals(lastAction)) {
                detachRequested = false;
                if (assemblyCleanupRequested && !attachedBlockDirections.isEmpty()) {
                    attachedBlockDirections.remove(attachedBlockDirections.size() - 1);
                    currentIntention = null;
                    resetCarriedBlockTracking();
                    finishAssemblyCleanupIfReady();
                    return;
                }
                if (!currentGroupLeader.isEmpty()) {
                    sendMessage(new Percept("groupBlockDetached", new Identifier(getName())),
                            currentGroupLeader, getName());
                }
                resetCarriedBlockTracking();
                currentIntention = null;
                if (successorGroupFormationAfterDetach) {
                    successorGroupFormationAfterDetach = false;
                    startGroupFormationCore();
                }
            } else if ("submit".equals(lastAction)) {
                finishAssemblyAfterSuccessfulSubmit();
            }
        } else {
            if ("clear".equals(lastAction)
                    && "failed_random".equals(lastActionResult)
                    && clearDirection != null) {
                currentIntention = new Intention(
                        currentIntention.desire(),
                        currentIntention.plan(),
                        currentIntention.nextAction());
            } else if ("clear".equals(lastAction) && clearDirection != null) {
                int[] offset = directionOffset(clearDirection);
                internalMap.forgetObservationsAt(
                        internalMap.getAgentX() + offset[0],
                        internalMap.getAgentY() + offset[1]);
                currentIntention = null;
            } else if ("clear".equals(lastAction)
                    && blockRetrieved
                    && currentIntention.desire() == Desire.RETRIEVE_BLOCK
                    && currentIntention.nextAction() + 1 < currentIntention.plan().size()
                    && currentIntention.plan().get(currentIntention.nextAction() + 1).startsWith("rotate:")) {
                currentIntention = currentIntention.advance();
            } else if ("attach".equals(lastAction) && pendingAssemblyAttachment != null) {
                currentIntention = new Intention(
                        Desire.ATTACH_ASSEMBLY,
                        List.of("attach:" + directionTo(
                                pendingAssemblyAttachment.targetPosition(), currentPosition())), 0);
            } else if ("attach".equals(lastAction) && retrieveBlockDirection != null) {
                currentIntention = createRetrieveBlockIntention();
            } else if ("detach".equals(lastAction) && detachRequested) {
                currentIntention = createDetachIntention();
            } else if ("rotate".equals(lastAction) && retrieveBlockDirection != null) {
                rememberFailedRotationTarget();
                currentIntention = createRetrieveBlockIntention();
            } else if ("connect".equals(lastAction)
                    && pendingAssemblyConnection != null) {
                currentIntention = new Intention(Desire.RETRIEVE_BLOCK,
                        currentIntention.plan(), currentIntention.nextAction());
                } else if ("move".equals(lastAction)
                    && isMovementDirection(pendingDirection)) {
                int[] offset = directionOffset(pendingDirection);
                int blockedX = internalMap.getAgentX() + offset[0];
                int blockedY = internalMap.getAgentY() + offset[1];
                internalMap.rememberFailedPath(blockedX, blockedY, currentStep);
                if (blockRetrieved && retrieveBlockDirection != null) {
                    String mirroredSide = oppositeDirection(pendingDirection);
                    String rotation = chooseRotationForCarryRecovery(mirroredSide);
                    currentIntention = new Intention(
                            Desire.RETRIEVE_BLOCK,
                            List.of("clear:" + mirroredSide, "rotate:" + rotation, pendingDirection),
                            0);
                } else {
                    currentIntention = new Intention(
                            Desire.CLEAR_OBSTACLE, List.of(pendingDirection), 0);
                }
            } else {
                currentIntention = null;
            }
        }

        pendingAction = null;
        pendingDirection = null;
        pendingRotation = null;
        clearDirection = null;
    }

    private void verifyCarriedBlock(List<Percept> percepts) {
        if (!blockRetrieved || retrieveBlockDirection == null) {
            return;
        }

        InternalMap.Position attachedPosition = findAttachedBlockPosition(percepts, retrieveBlockDirection);
        if (attachedPosition == null) {
            resetCarriedBlockTracking();
            return;
        }

        carriedBlockPosition = attachedPosition;
        retrieveBlockDirection = directionTo(
                attachedPosition,
                new InternalMap.Position(internalMap.getAgentX(), internalMap.getAgentY()));
        recordAttachedAssemblyRequirement(attachedPosition, carriedBlockType);
    }

    private void updateBlockDeliveryStatus() {
        if (!blockRetrieved || currentTaskBlockCount <= 1 || goalPosition == null) {
            return;
        }

        boolean atDeliveryTarget = carriedBlockPosition != null && carriedBlockPosition.equals(goalPosition);
        if (!atDeliveryTarget) {
            blockPlaced = false;
            return;
        }

        if (!blockPlaced && !currentGroupLeader.isEmpty()
                && !currentGroupLeader.equals(getName())) {
            sendMessage(new Percept("groupBlockDelivered",
                    new Identifier(deliveryBlockType),
                    new Numeral(goalPosition.x()),
                    new Numeral(goalPosition.y())), currentGroupLeader, getName());
        }
        blockPlaced = true;
    }

    private void recordAttachedAssemblyRequirement(InternalMap.Position blockPosition,
            String blockType) {
        if (!isCurrentGroupLeader() || currentTaskBlockCount <= 1
                || goalPosition == null || blockType == null) {
            return;
        }

        InternalMap.Position requirementOffset = relativeAssemblyPosition(blockPosition);
        for (int index = 0; index < taskRequirementOffsets.size(); index++) {
            if (taskRequirementOffsets.get(index).equals(requirementOffset)
                    && taskBlockTypes.get(index).equalsIgnoreCase(blockType)) {
                assembledBlockPositions.add(blockPosition);
                assembledBlockTypes.put(requirementOffset, blockType);
                return;
            }
        }
    }

    private void resetCarriedBlockTracking() {
        blockRequested = false;
        blockRetrieved = false;
        carriedBlockType = null;
        blockPlaced = false;
        detachRequested = false;
        carriedBlockPosition = null;
        retrieveBlockDirection = null;
    }

    private void clearPendingActionState() {
        pendingAction = null;
        pendingDirection = null;
        pendingRotation = null;
        clearDirection = null;
    }

    private void prepareForBlockAssignment() {
        blockPlaced = false;
        currentIntention = null;
        blockRequested = blockRetrieved
                && carriedBlockType != null
                && deliveryBlockType != null
                && carriedBlockType.equalsIgnoreCase(deliveryBlockType);
    }

    private InternalMap.Position findAttachedBlockPosition(List<Percept> percepts, String preferredDirection) {
        if (preferredDirection != null) {
            int[] preferredOffset = directionOffset(preferredDirection);
            for (Percept percept : percepts) {
                if (!percept.getName().equals("attached")
                        || percept.getParameters().size() < 2
                        || !(percept.getParameters().get(0) instanceof Numeral x)
                        || !(percept.getParameters().get(1) instanceof Numeral y)) {
                    continue;
                }
                if (x.getValue().intValue() == preferredOffset[0]
                        && y.getValue().intValue() == preferredOffset[1]) {
                    return new InternalMap.Position(
                            internalMap.getAgentX() + preferredOffset[0],
                            internalMap.getAgentY() + preferredOffset[1]);
                }
            }
        }
        for (Percept percept : percepts) {
            if (!percept.getName().equals("attached")
                    || percept.getParameters().size() < 2
                    || !(percept.getParameters().get(0) instanceof Numeral x)
                    || !(percept.getParameters().get(1) instanceof Numeral y)) {
                continue;
            }

            int relativeX = x.getValue().intValue();
            int relativeY = y.getValue().intValue();
            if (Math.abs(relativeX) + Math.abs(relativeY) != 1) {
                continue;
            }

            return new InternalMap.Position(
                    internalMap.getAgentX() + relativeX,
                    internalMap.getAgentY() + relativeY);
        }
        return null;
    }

    /** Derives currently eligible behaviors from the agent's beliefs and state. */
    private Set<Desire> generateDesires() {
        Set<Desire> desires = EnumSet.noneOf(Desire.class);

        if (deactivated) {
            desires.add(Desire.WAIT);
            return desires;
        }
        if (assemblyCleanupRequested) {
            if (attachedBlockDirections.isEmpty()) {
                desires.add(Desire.WAIT);
            } else {
                desires.add(Desire.DETACH_ASSEMBLY);
            }
            return desires;
        }
        if (pendingAssemblyConnection != null) {
            desires.add(Desire.CONNECT_ASSEMBLY);
            return desires;
        }
        if (pendingAssemblyAttachment != null) {
            desires.add(Desire.ATTACH_ASSEMBLY);
            return desires;
        }
        if (currentTask != null && !isTaskActive()) {
            desires.add(Desire.EXPLORE);
            return desires;
        }
        if (detachRequested && detachDirection() != null) {
            desires.add(Desire.RETRIEVE_BLOCK);
            return desires;
        }
        if (DEFAULT_ROLE.equals(currentRole)) {
            if (isAtRoleZone()) {
                desires.add(Desire.ADAPT_ROLE);
                return desires;
            }
            InternalMap.Observation roleZone = findNearestAvailableRoleZone();
            if (roleZone != null
                    && (explorationFinished
                    || distanceTo(roleZone.x(), roleZone.y(), internalMap.getAgentX(), internalMap.getAgentY())
                            <= ROLE_ZONE_MAX_DISTANCE)) {
                desires.add(Desire.REACH_ROLE_ZONE);
                return desires;
            }

        }

        if (isCurrentGroupLeader()
                && currentTask != null
                && isTaskActive()
                && currentTaskBlockCount > 1
                && goalPosition != null
                && isAtGoalZone()
                && isAssemblyComplete()) {
            desires.add(Desire.SUBMIT);
            return desires;
        }

        if (currentTask != null && isTaskActive() && currentTaskBlockCount > 1
                && (groupLeaderMode || currentGroupLeader.equals(getName()))
                && goalPosition != null) {
            desires.add(groupGoalRelocationPending || isAtGoalPosition()
                    ? Desire.WAIT : Desire.REACH_GOAL_ZONE);
            return desires;
        }

        if (!currentRole.isEmpty() && !DEFAULT_ROLE.equals(currentRole)
                && isTaskActive()
                && goalPosition != null && deliveryBlockType != null) {
            if (!blockPlaced) {
                desires.add(Desire.RETRIEVE_BLOCK);
                return desires;
            }
            if (currentTaskBlockCount > 1) {
                desires.add(Desire.WAIT);
                return desires;
            }
            desires.add(isAtGoalZone() ? Desire.WAIT : Desire.REACH_GOAL_ZONE);
            return desires;
        }

        if (desiredGroupSize > 1
                && currentTask != null
                && isTaskActive()
                && explorationFinished
                && Boolean.TRUE.equals(knownAgentGroupState.get(getName()))
                && !groupFormationActive) {
            desires.add(Desire.WAIT);
            return desires;
        }

        desires.add(Desire.EXPLORE);
        return desires;
    }


    private boolean isAssemblyComplete() {
        if (goalPosition == null || taskRequirementOffsets.size() != taskBlockTypes.size()
                || taskRequirementOffsets.isEmpty()
                || assembledBlockTypes.size() != taskRequirementOffsets.size()
                || !pendingAssemblyDetaches.isEmpty()) {
            return false;
        }

        for (int index = 0; index < taskRequirementOffsets.size(); index++) {
            InternalMap.Position offset = taskRequirementOffsets.get(index);
                String actualType = assembledBlockTypes.get(offset);
            if (actualType == null
                    || !actualType.equalsIgnoreCase(taskBlockTypes.get(index))) {
                return false;
            }
        }
        return true;
    }

    private InternalMap.Position relativeAssemblyPosition(InternalMap.Position absolutePosition) {
        return new InternalMap.Position(
            absolutePosition.x() - internalMap.getAgentX(),
            absolutePosition.y() - internalMap.getAgentY());
    }

    private boolean hasObservation(String type) {
        return internalMap.getObservations().stream().anyMatch(observation -> observation.type().equals(type));
    }

    private boolean hasAllRequiredDispensers() {
        Set<String> dispenserTypes = new HashSet<>();
        for (InternalMap.Observation observation : internalMap.getObservations()) {
            if (observation.type().equals("dispenser")) {
                dispenserTypes.add(observation.details());
            }
        }
        return !requiredDispenserTypes.isEmpty() && dispenserTypes.containsAll(requiredDispenserTypes);
    }

    private void rememberKnownAgent(String agent, InternalMap.Position position, String source) {
        knownAgents.put(agent, position);
        knownAgentSources.put(agent, source);
        knownAgentGroupState.putIfAbsent(agent, false);
        knownAgentGroupLeader.putIfAbsent(agent, "");
    }

    private void rememberKnownAgent(String agent, InternalMap.Position position, String source,
            boolean inGroup, String groupLeader) {
        knownAgents.put(agent, position);
        knownAgentSources.put(agent, source);
        boolean alreadyGrouped = Boolean.TRUE.equals(knownAgentGroupState.get(agent));
        if (!alreadyGrouped || inGroup) {
            knownAgentGroupState.put(agent, inGroup);
            if (inGroup && groupLeader != null && !groupLeader.isEmpty()) {
                knownAgentGroupLeader.put(agent, groupLeader);
            } else {
                knownAgentGroupLeader.put(agent, "");
            }
        }
        if (!inGroup) {
            rejectedGroupInviteTargets.remove(agent);
        }
    }

    private boolean isAtGoalZone() {
        int agentX = internalMap.getAgentX();
        int agentY = internalMap.getAgentY();
        return internalMap.getObservations().stream().anyMatch(observation ->
                observation.type().equals("goalZone") && observation.x() == agentX && observation.y() == agentY);
    }

    private boolean isAtGoalPosition() {
        return goalPosition != null
                && internalMap.getAgentX() == goalPosition.x()
                && internalMap.getAgentY() == goalPosition.y();
    }

    private String offsetToDirection(InternalMap.Position offset) {
        if (offset.x() == 1) return "e";
        if (offset.x() == -1) return "w";
        if (offset.y() == 1) return "s";
        if (offset.y() == -1) return "n";
        return "n";
    }

    /** Chooses the highest-priority intention among the currently eligible desires. */
    private Intention selectIntention(Set<Desire> desires) {
        if (desires.contains(Desire.WAIT)) {
            return new Intention(Desire.WAIT, List.of(), 0);
        }
        if (desires.contains(Desire.CONNECT_ASSEMBLY)) {
            return new Intention(Desire.CONNECT_ASSEMBLY, List.of("connect"), 0);
        }
        if (desires.contains(Desire.ATTACH_ASSEMBLY)) {
            String direction = directionTo(pendingAssemblyAttachment.targetPosition(), currentPosition());
            return new Intention(Desire.ATTACH_ASSEMBLY, List.of("attach:" + direction), 0);
        }
        if (desires.contains(Desire.DETACH_ASSEMBLY)) {
            return new Intention(Desire.DETACH_ASSEMBLY,
                    List.of("detach:" + attachedBlockDirections.get(attachedBlockDirections.size() - 1)), 0);
        }
        if (desires.contains(Desire.CLEAR_OBSTACLE)) {
            return new Intention(Desire.CLEAR_OBSTACLE, List.of(), 0);
        }
        if (desires.contains(Desire.ADAPT_ROLE)) {
            return new Intention(Desire.ADAPT_ROLE, List.of(WORKER_ROLE), 0);
        }
        if (desires.contains(Desire.SUBMIT)) {
            return new Intention(Desire.SUBMIT, List.of("submit:" + currentTask), 0);
        }
        if (detachRequested && desires.contains(Desire.RETRIEVE_BLOCK)) {
            return createDetachIntention();
        }
        if (desires.contains(Desire.REACH_ROLE_ZONE)) {
            return createRoleZoneIntention();
        }
        if (desires.contains(Desire.REACH_GOAL_ZONE)) {
            return createGoalIntention();
        }
        if (desires.contains(Desire.RETRIEVE_BLOCK)) {
            return createRetrieveBlockIntention();
        }
        if (desires.contains(Desire.EXPLORE)) {
            return createExploreIntention();
        }
        return null;
    }

    private Intention createRoleZoneIntention() {
        InternalMap.Observation roleZone = findNearestAvailableRoleZone();
        if (roleZone == null) {
            return null;
        }

        List<String> path = findPathForCurrentState(
                currentPosition(),
                new InternalMap.Position(roleZone.x(), roleZone.y()),
            blockedPositionsForMovement(),
            occupiedPositionsForMovement());
        if (nextMoveIsBlocked(path)) {
            return new Intention(Desire.CLEAR_OBSTACLE, List.of(path.get(0)), 0);
        }
        return new Intention(Desire.REACH_ROLE_ZONE, path, 0);
    }

    private Intention createGoalIntention() {
        InternalMap.Position start = currentPosition();
        InternalMap.Position target = goalPosition;
        if (target == null) {
            InternalMap.Observation goal = findNearestGoalZone();
            if (goal == null) {
                return new Intention(Desire.WAIT, List.of(), 0);
            }
            target = new InternalMap.Position(goal.x(), goal.y());
        }

        List<String> path = findPathForCurrentState(start, target,
                blockedPositionsForMovement(), occupiedPositionsForMovement());
        if (isCurrentGroupLeader()
                && currentTaskBlockCount > 1
                && (!pathExists(path) || isPhysicallyOccupied(target))) {
            InternalMap.Position replacement = findReachableGoalZone(start, target);
            if (replacement != null) {
                relocateAssemblyGoal(replacement);
                target = replacement;
                path = findPathForCurrentState(start, target,
                        blockedPositionsForMovement(), occupiedPositionsForMovement());
            }
        }
        if (nextMoveIsBlocked(path)) {
            return new Intention(Desire.CLEAR_OBSTACLE, List.of(path.get(0)), 0);
        }
        return new Intention(Desire.REACH_GOAL_ZONE, path, 0);
    }

    private boolean pathExists(List<String> path) {
        return path != null && !path.isEmpty();
    }

    /** Plans movement to a target using the current carrying and occupancy state. */
    private List<String> findPathForCurrentState(InternalMap.Position start,
            InternalMap.Position goal, List<InternalMap.Position> blockedPositions,
            Set<InternalMap.Position> occupiedPositions) {
        if (isCarryingBlock()) {
            return pathPlanner.findCarryingPath(start, goal, retrieveBlockDirection,
                    blockedPositions, occupiedPositions, reservedGoalPositionsForMovement());
        }
        return pathPlanner.findPath(start, goal, blockedPositions, occupiedPositions,
                reservedGoalPositionsForMovement());
    }

    /** Plans to an agent position while carrying a block in the required direction. */
    private List<String> findPathForCurrentState(InternalMap.Position start,
            InternalMap.Position goal, String requiredBlockDirection,
            List<InternalMap.Position> blockedPositions,
            Set<InternalMap.Position> occupiedPositions) {
        if (isCarryingBlock() && requiredBlockDirection != null) {
            return pathPlanner.findCarryingPathToAgentPosition(start, goal,
                    retrieveBlockDirection, requiredBlockDirection,
                    blockedPositions, occupiedPositions, reservedGoalPositionsForMovement());
        }
        return findPathForCurrentState(start, goal, blockedPositions, occupiedPositions);
    }

    private boolean isCarryingBlock() {
        return blockRetrieved && retrieveBlockDirection != null;
    }

    private boolean isPhysicallyOccupied(InternalMap.Position position) {
        return internalMap.getPhysicalOccupiedEntityPositions().contains(position)
            || internalMap.getBlockedPositions().contains(position);
    }

    private InternalMap.Position findReachableGoalZone(InternalMap.Position start,
            InternalMap.Position currentTarget) {
        return internalMap.getObservations().stream()
                .filter(observation -> observation.type().equals("goalZone"))
                .map(observation -> new InternalMap.Position(observation.x(), observation.y()))
                .filter(candidate -> !candidate.equals(currentTarget))
                .filter(this::isGoalReservationAvailable)
                .filter(candidate -> !hasOpponentOnReservedAssemblyPosition(candidate))
                .filter(candidate -> candidate.equals(start) || !isPhysicallyOccupied(candidate))
                .filter(candidate -> candidate.equals(start)
                    || pathExists(findPathForCurrentState(start, candidate,
                        blockedPositionsForMovement(), occupiedPositionsForMovement())))
                .min((first, second) -> Integer.compare(
                    distanceTo(first.x(), first.y(), currentTarget.x(), currentTarget.y()),
                    distanceTo(second.x(), second.y(), currentTarget.x(), currentTarget.y())))
                .orElse(null);
    }

    private boolean hasOpponentOnReservedAssemblyPosition(InternalMap.Position assemblyAnchor) {
        if (teamName.isEmpty()) {
            return false;
        }

        Set<InternalMap.Position> reservedPositions = new HashSet<>();
        reservedPositions.add(assemblyAnchor);
        for (InternalMap.Position offset : taskRequirementOffsets) {
            reservedPositions.add(new InternalMap.Position(
                    assemblyAnchor.x() + offset.x(), assemblyAnchor.y() + offset.y()));
        }

        int agentX = internalMap.getAgentX();
        int agentY = internalMap.getAgentY();
        for (VisibleThing thing : currentVisibleThings) {
            if (thing.type().equals("entity")
                    && !thing.details().isEmpty()
                    && !thing.details().equals(teamName)
                    && reservedPositions.contains(new InternalMap.Position(
                            agentX + thing.x(), agentY + thing.y()))) {
                return true;
            }
        }
        return false;
    }

    private void relocateAssemblyGoal(InternalMap.Position replacement) {
        groupGoalRelocationPending = false;
        goalPosition = replacement;
        currentIntention = null;
        clearAssemblyState();
        assignBlocksToCurrentGroupAt(replacement);
    }

    private void clearAssemblyState() {
        assembledBlockPositions.clear();
        assembledBlockTypes.clear();
        pendingAssemblyDetaches.clear();
        pendingAssemblyConnection = null;
    }

    private void reserveGroupGoalPositions(InternalMap.Position assemblyAnchor) {
        releaseGroupGoalPositions();
        if (!isCurrentGroupLeader()) {
            return;
        }

        Set<InternalMap.Position> positions = new HashSet<>();
        positions.add(assemblyAnchor);
        for (InternalMap.Position offset : taskRequirementOffsets) {
            positions.add(new InternalMap.Position(
                    assemblyAnchor.x() + offset.x(), assemblyAnchor.y() + offset.y()));
        }
        updateOwnGoalReservationSnapshot(positions);
    }

    private void releaseGroupGoalPositions() {
        GoalReservationSnapshot ownSnapshot = goalReservationSnapshots.get(getName());
        if (ownSnapshot != null && !ownSnapshot.positions().isEmpty()) {
            updateOwnGoalReservationSnapshot(Set.of());
        }
    }

    private boolean isReservedForAnotherLeader(InternalMap.Position position) {
        for (Map.Entry<String, GoalReservationSnapshot> entry : goalReservationSnapshots.entrySet()) {
            if (!entry.getKey().equals(getName())
                    && isGoalReservationSnapshotFresh(entry.getValue())
                    && entry.getValue().positions().contains(position)) {
                return true;
            }
        }
        return false;
    }

    private boolean isGoalReservationAvailable(InternalMap.Position assemblyAnchor) {
        if (isReservedForAnotherLeader(assemblyAnchor)) {
            return false;
        }
        for (InternalMap.Position offset : taskRequirementOffsets) {
            if (isReservedForAnotherLeader(new InternalMap.Position(
                    assemblyAnchor.x() + offset.x(), assemblyAnchor.y() + offset.y()))) {
                return false;
            }
        }
        return true;
    }

    /** Builds the next action plan for obtaining and delivering the assigned block. */
    private Intention createRetrieveBlockIntention() {
        if (goalPosition == null || deliveryBlockType == null) {
            return new Intention(Desire.WAIT, List.of(), 0);
        }

        String blockType = deliveryBlockType;
        InternalMap.Position deliveryTarget = goalPosition;

        if (blockRetrieved && carriedBlockType != null
                && !carriedBlockType.equalsIgnoreCase(blockType)) {
            return new Intention(Desire.RETRIEVE_BLOCK,
                    List.of("detach:" + retrieveBlockDirection), 0);
        }

        if (!blockRequested) {
            InternalMap.Observation dispenser = findNearestDispenser(blockType);
            if (dispenser == null) {
                return createExploreIntention();
            }

            InternalMap.Position dispenserPosition =
                    new InternalMap.Position(dispenser.x(), dispenser.y());
            String adjacentDirection = adjacentDirectionTo(dispenserPosition);
            if (adjacentDirection != null) {
                retrieveBlockDirection = adjacentDirection;
                if (isFreeBlockAt(adjacentDirection, blockType)) {
                    return new Intention(Desire.RETRIEVE_BLOCK,
                            List.of("attach:" + adjacentDirection), 0);
                }
                return new Intention(Desire.RETRIEVE_BLOCK,
                        List.of("request:" + adjacentDirection), 0);
            }

            List<String> path = pathToAdjacentPosition(dispenserPosition);
            if (!path.isEmpty()) {
                if (nextMoveIsBlocked(path)) {
                    return new Intention(Desire.CLEAR_OBSTACLE, List.of(path.get(0)), 0);
                }
                return new Intention(Desire.RETRIEVE_BLOCK, path, 0);
            }
            return createExploreIntention();
        }

        if (!blockRetrieved) {
            return new Intention(Desire.RETRIEVE_BLOCK,
                    List.of("attach:" + retrieveBlockDirection), 0);
        }

        if (currentTaskBlockCount == 1) {
            String requiredDirection = requiredBlockDirection();
            if (requiredDirection == null) {
                return new Intention(Desire.WAIT, List.of(), 0);
            }
            if (isAtGoalZone()
                    && requiredDirection.equals(retrieveBlockDirection)) {
                return new Intention(Desire.RETRIEVE_BLOCK,
                        List.of("submit:" + currentTask), 0);
            }

            List<String> path = findPathForCurrentState(currentPosition(), deliveryTarget,
                    requiredDirection, blockedPositionsForMovement(),
                    occupiedPositionsForMovement());
            if (path.isEmpty()) {
                return new Intention(Desire.WAIT, List.of(), 0);
            }
            return new Intention(Desire.RETRIEVE_BLOCK, path, 0);
        }

        if (carriedBlockPosition != null && carriedBlockPosition.equals(deliveryTarget)) {
            if (!blockPlaced && !currentGroupLeader.isEmpty()
                    && !currentGroupLeader.equals(getName())) {
                sendMessage(new Percept("groupBlockDelivered",
                        new Identifier(deliveryBlockType),
                        new Numeral(deliveryTarget.x()),
                        new Numeral(deliveryTarget.y())), currentGroupLeader, getName());
            }
            blockPlaced = true;
            return new Intention(Desire.WAIT, List.of(), 0);
        }
        List<String> path = findPathForCurrentState(currentPosition(), deliveryTarget,
                blockedPositionsForMovement(),
                occupiedPositionsForMovement());
        if (path.isEmpty()) {
            return new Intention(Desire.WAIT, List.of(), 0);
        }
        return new Intention(Desire.RETRIEVE_BLOCK, path, 0);
    }

    private Intention createDetachIntention() {
        String direction = detachDirection();
        if (direction == null) {
            return new Intention(Desire.WAIT, List.of(), 0);
        }
        return new Intention(Desire.RETRIEVE_BLOCK, List.of("detach:" + direction), 0);
    }

    private String detachDirection() {
        if (!blockRetrieved) {
            return null;
        }
        if (retrieveBlockDirection != null) {
            return retrieveBlockDirection;
        }
        if (carriedBlockPosition != null) {
            int deltaX = carriedBlockPosition.x() - internalMap.getAgentX();
            int deltaY = carriedBlockPosition.y() - internalMap.getAgentY();
            if (Math.abs(deltaX) + Math.abs(deltaY) == 1) {
                return directionTo(carriedBlockPosition, currentPosition());
            }
        }
        return null;
    }

    private String requiredBlockDirection() {
        if (requiredBlockOffset == null
                || Math.abs(requiredBlockOffset.x()) + Math.abs(requiredBlockOffset.y()) != 1) {
            return null;
        }
        if (requiredBlockOffset.x() == 1) return "e";
        if (requiredBlockOffset.x() == -1) return "w";
        if (requiredBlockOffset.y() == 1) return "s";
        return "n";
    }

    private List<String> pathToAdjacentPosition(InternalMap.Position target) {
        List<String> bestPath = List.of();
        for (String direction : List.of("n", "e", "s", "w")) {
            int[] offset = directionOffset(direction);
            InternalMap.Position candidate = new InternalMap.Position(
                    target.x() + offset[0], target.y() + offset[1]);
            if (currentPosition().equals(candidate)) {
                continue;
            }
            List<String> path = findPathForCurrentState(currentPosition(), candidate,
                    blockedPositionsForMovement(), occupiedPositionsForMovement());
            if (!path.isEmpty() && (bestPath.isEmpty() || path.size() < bestPath.size())) {
                bestPath = path;
            }
        }
        return bestPath;
    }

    private String adjacentDirectionTo(InternalMap.Position target) {
        int deltaX = target.x() - internalMap.getAgentX();
        int deltaY = target.y() - internalMap.getAgentY();
        if (AgentUtils.manhattanDistance(deltaX, deltaY, 0, 0) != 1) {
            return null;
        }
        return directionTo(target, currentPosition());
    }

    private InternalMap.Observation nearestObservation(String type,
            Predicate<InternalMap.Observation> isAvailable) {
        int agentX = internalMap.getAgentX();
        int agentY = internalMap.getAgentY();
        return internalMap.getObservations().stream()
                .filter(observation -> observation.type().equals(type))
                .filter(isAvailable)
                .min(Comparator.comparingInt(observation ->
                        distanceTo(observation.x(), observation.y(), agentX, agentY)))
                .orElse(null);
    }

    private InternalMap.Observation findNearestDispenser(String blockType) {
        InternalMap.Position target = goalPosition == null ? currentPosition() : goalPosition;
        int agentX = internalMap.getAgentX();
        int agentY = internalMap.getAgentY();
        return internalMap.getObservations().stream()
            .filter(observation -> observation.type().equals("dispenser"))
            .filter(observation -> observation.details().equalsIgnoreCase(blockType))
            .min(Comparator
                .comparingInt((InternalMap.Observation observation) ->
                    distanceTo(observation.x(), observation.y(), target.x(), target.y()))
                .thenComparingInt(observation ->
                    distanceTo(observation.x(), observation.y(), agentX, agentY)))
            .orElse(null);
    }

    private String directionTo(InternalMap.Position target, InternalMap.Position from) {
        int x = target.x() - from.x();
        int y = target.y() - from.y();
        if (x == 1) return "e";
        if (x == -1) return "w";
        if (y == 1) return "s";
        if (y == -1) return "n";
        throw new IllegalArgumentException("Positions are not adjacent");
    }

    private String chooseRotationForCarryRecovery(String mirroredSide) {
        String targetDirection = mirroredSide;
        for (boolean clockwise : List.of(true, false)) {
            String rotated = rotateDirection(retrieveBlockDirection, clockwise);
            if (rotated.equals(targetDirection)) {
                return clockwise ? "cw" : "ccw";
            }
        }
        return "cw";
    }

    private Intention createExploreIntention() {
        InternalMap.Position start = currentPosition();
        InternalMap.Position target = explorationTargetSelector.selectTarget(internalMap, knownTargets, getName());
        explorationTarget = target;

        List<String> path = findPathForCurrentState(start, target,
            blockedPositionsForMovement(), occupiedPositionsForMovement());
        if (nextMoveIsBlocked(path)) {
            return new Intention(Desire.CLEAR_OBSTACLE, List.of(path.get(0)), 0);
        }
        return new Intention(Desire.EXPLORE, path, 0);
    }

    private InternalMap.Position currentPosition() {
        return new InternalMap.Position(internalMap.getAgentX(), internalMap.getAgentY());
    }

    private List<InternalMap.Position> blockedPositionsForMovement() {
        return internalMap.getBlockedPositions();
    }

    private Set<InternalMap.Position> occupiedPositionsForMovement() {
        Set<InternalMap.Position> occupied =
                new HashSet<>(internalMap.getPhysicalOccupiedEntityPositions());
        if (carriedBlockPosition != null) {
            occupied.remove(carriedBlockPosition);
        }
        return occupied;
    }

    private Set<InternalMap.Position> reservedGoalPositionsForMovement() {
        Set<InternalMap.Position> reserved = new HashSet<>();
        for (Map.Entry<String, GoalReservationSnapshot> entry : goalReservationSnapshots.entrySet()) {
            if (!entry.getKey().equals(getName())
                    && isGoalReservationSnapshotFresh(entry.getValue())) {
                reserved.addAll(entry.getValue().positions());
            }
        }
        return reserved;
    }

    private boolean isCurrentGroupLeader() {
        return groupLeaderMode || getName().equals(currentGroupLeader);
    }

    private boolean allExplorationRequirementsKnown() {
        return hasObservation("roleZone")
                && hasObservation("goalZone")
                && hasAllRequiredDispensers();
    }

    private boolean nextMoveIsBlocked(List<String> plan) {
        if (plan == null || plan.isEmpty()) {
            return false;
        }

        String direction = plan.get(0);
        if (!isMovementDirection(direction)) {
            return false;
        }
        int[] offset = directionOffset(direction);
        int nextX = internalMap.getAgentX() + offset[0];
        int nextY = internalMap.getAgentY() + offset[1];
        InternalMap.Position nextPosition = new InternalMap.Position(nextX, nextY);

        return blockedPositionsForMovement().contains(nextPosition)
            || occupiedPositionsForMovement().contains(nextPosition)
            || reservedGoalPositionsForMovement().contains(nextPosition);
    }

    private InternalMap.Observation findNearestGoalZone() {
        return nearestObservation("goalZone", observation -> isGoalReservationAvailable(
                new InternalMap.Position(observation.x(), observation.y())));
    }

    private InternalMap.Observation findNearestAvailableRoleZone() {
        return nearestObservation("roleZone",
                observation -> !isOccupiedByAnotherAgent(observation.x(), observation.y()));
    }

    private boolean isOccupiedByAnotherAgent(int x, int y) {
        return (x != internalMap.getAgentX() || y != internalMap.getAgentY())
                && (occupiedPositionsForMovement().contains(new InternalMap.Position(x, y))
                || reservedGoalPositionsForMovement().contains(new InternalMap.Position(x, y)));
    }

    private boolean isAtRoleZone() {
        int agentX = internalMap.getAgentX();
        int agentY = internalMap.getAgentY();
        return internalMap.getObservations().stream().anyMatch(observation ->
                observation.type().equals("roleZone")
                && observation.x() == agentX
                && observation.y() == agentY);
    }

    private boolean shouldInterruptForRole() {
        if (!DEFAULT_ROLE.equals(currentRole)) {
            return false;
        }
        if (currentIntention != null
                && currentIntention.desire() == Desire.CLEAR_OBSTACLE) {
            return false;
        }
        if (isAtRoleZone()) {
            return currentIntention == null
                    || currentIntention.desire() != Desire.ADAPT_ROLE;
        }

        InternalMap.Observation roleZone = findNearestAvailableRoleZone();
        boolean roleZoneIsRelevant = roleZone != null
                && (explorationFinished
                    || distanceTo(roleZone.x(), roleZone.y(), internalMap.getAgentX(), internalMap.getAgentY())
                        <= ROLE_ZONE_MAX_DISTANCE);
        return roleZoneIsRelevant
                && (currentIntention == null
                    || currentIntention.desire() != Desire.REACH_ROLE_ZONE);
    }

    /** Recomputes movement plans when the map or coordination state has changed. */
    private void replanMovementIntention() {
        if (currentIntention == null || currentIntention.finished()) {
            return;
        }

        if (groupGoalRelocationPending) {
            currentIntention = new Intention(Desire.WAIT, List.of(), 0);
            return;
        }

        if (pendingAssemblyConnection != null) {
            return;
        }

        if (currentTaskBlockCount > 1
                && currentGroupLeader.equals(getName())
                && goalPosition != null
                && pendingAssemblyAttachment == null
                && !detachRequested) {
            currentIntention = isAtGoalPosition()
                    ? new Intention(Desire.WAIT, List.of(), 0)
                    : createGoalIntention();
            return;
        }

        if (currentIntention.desire() == Desire.RETRIEVE_BLOCK
                && currentIntention.plan().size() > 1
                && currentIntention.plan().stream().anyMatch(step -> step.startsWith("clear:")
                    || step.startsWith("rotate:"))) {
            return;
        }

        switch (currentIntention.desire()) {
            case REACH_ROLE_ZONE -> currentIntention = createRoleZoneIntention();
            case REACH_GOAL_ZONE -> currentIntention = createGoalIntention();
            case RETRIEVE_BLOCK -> {
                if (detachRequested) {
                    currentIntention = createDetachIntention();
                } else if (blockRequested && !blockRetrieved) {
                    currentIntention = createRetrieveBlockIntention();
                } else if (blockRetrieved) {
                    currentIntention = createRetrieveBlockIntention();
                }
            }
            case EXPLORE -> {
                if (explorationTarget != null) {
                    if (internalMap.isKnownPosition(
                            explorationTarget.x(), explorationTarget.y())) {
                        explorationTarget = null;
                        currentIntention = null;
                        return;
                    }
                    List<String> path = findPathForCurrentState(
                            currentPosition(), explorationTarget,
                            blockedPositionsForMovement(),
                            occupiedPositionsForMovement());
                    currentIntention = nextMoveIsBlocked(path)
                            ? new Intention(Desire.CLEAR_OBSTACLE, List.of(path.get(0)), 0)
                            : new Intention(Desire.EXPLORE, path, 0);
                }
            }
            default -> {
            }
        }
    }

    private int distanceTo(int firstX, int firstY, int secondX, int secondY) {
        return AgentUtils.manhattanDistance(firstX, firstY, secondX, secondY);
    }

    private Action move(String direction) {
        if (direction.equals("n") || direction.equals("s") || direction.equals("e") || direction.equals("w")) {
            return new Action("move", new Identifier(direction));
        }
        throw new IllegalArgumentException("Invalid direction: " + direction);
    }

    /** Executes the next step of the active intention. */
    private Action executeIntention() {
        if (currentIntention == null || currentIntention.finished()) {
            return skip();
        }
        if (currentIntention.desire() == Desire.WAIT) {
            return skip();
        }
        if (currentIntention.plan().isEmpty()) {
            return skip();
        }
        String currentStep = currentIntention.plan().get(currentIntention.nextAction());
        if (isBlockActionStep(currentStep)
                && currentIntention.desire() != Desire.CLEAR_OBSTACLE
                && currentIntention.desire() != Desire.ADAPT_ROLE
                && currentIntention.desire() != Desire.SUBMIT) {
            return executeRetrieveBlock();
        }
        if (currentIntention.desire() == Desire.CLEAR_OBSTACLE) {
            return executeClear();
        }
        if (currentIntention.desire() == Desire.ADAPT_ROLE) {
            return executeAdapt();
        }
        if (currentIntention.desire() == Desire.SUBMIT) {
            return executeSubmit();
        }
        if (currentIntention.desire() == Desire.RETRIEVE_BLOCK
                || currentIntention.desire() == Desire.ATTACH_ASSEMBLY
                || currentIntention.desire() == Desire.CONNECT_ASSEMBLY
                || currentIntention.desire() == Desire.DETACH_ASSEMBLY) {
            return executeRetrieveBlock();
        }
        return executeMove();
    }

    private boolean isBlockActionStep(String step) {
        return step.equals("connect")
                || step.startsWith("connect:")
                || step.startsWith("request:")
                || step.startsWith("attach:")
                || step.startsWith("detach:")
                || step.startsWith("submit:")
                || step.startsWith("rotate:")
                || step.startsWith("clear:");
    }

    private Action executeAdapt() {
        pendingAction = "adapt";
        return new Action("adapt", new Identifier(currentIntention.plan().get(0)));
    }

    private Action executeSubmit() {
        pendingAction = "submit";
        return new Action("submit", new Identifier(currentTask));
    }

    private Action executeMove() {
        if (currentIntention.plan().isEmpty()) {
            return skip();
        }
        String direction = currentIntention.plan().get(currentIntention.nextAction());
        int[] offset = directionOffset(direction);
        InternalMap.Position nextPosition = new InternalMap.Position(
                internalMap.getAgentX() + offset[0], internalMap.getAgentY() + offset[1]);
        if (isReservedForAnotherLeader(nextPosition)) {
            currentIntention = null;
            return skip();
        }
        if (nextMoveIsBlocked(List.of(direction))) {
            currentIntention = new Intention(
                    Desire.CLEAR_OBSTACLE, List.of(direction), 0);
            return executeClear();
        }
        pendingAction = "move";
        pendingDirection = direction;
        return move(direction);
    }

    private Action executeRetrieveBlock() {
        String step = currentIntention.plan().get(currentIntention.nextAction());
        if ("connect".equals(step) || step.startsWith("connect:")) {
            if (pendingAssemblyConnection == null) {
                return skip();
            }
            int blockX;
            int blockY;
            if (isCurrentGroupLeader()) {
                blockX = pendingAssemblyConnection.leaderBlockX();
                blockY = pendingAssemblyConnection.leaderBlockY();
            } else if (carriedBlockPosition != null) {
                blockX = carriedBlockPosition.x() - internalMap.getAgentX();
                blockY = carriedBlockPosition.y() - internalMap.getAgentY();
            } else {
                return skip();
            }
            if (isCurrentGroupLeader()) {
                sendMessage(new Percept("groupConnectRequest",
                        new Numeral(pendingAssemblyConnection.leaderBlockX()),
                        new Numeral(pendingAssemblyConnection.leaderBlockY()),
                        new Numeral(pendingAssemblyConnection.targetPosition().x()),
                        new Numeral(pendingAssemblyConnection.targetPosition().y()),
                        new Identifier(pendingAssemblyConnection.blockType())),
                    pendingAssemblyConnection.partner(), getName());
            }
            pendingAction = "connect";
            return new Action("connect",
                    new Identifier(serverAgentName(pendingAssemblyConnection.partner())),
                    new Numeral(blockX), new Numeral(blockY));
        }
        if (step.startsWith("request:") || step.startsWith("attach:") || step.startsWith("detach:")) {
            String action = step.substring(0, step.indexOf(':'));
            String direction = step.substring(step.indexOf(':') + 1);
            if ("attach".equals(action)
                    && pendingAssemblyAttachment == null
                    && !isVisibleRequestedBlockAt(direction)) {
                blockRequested = false;
                currentIntention = createRetrieveBlockIntention();
                return skip();
            }
            pendingAction = action;
            return new Action(action, new Identifier(direction));
        }
        if (step.startsWith("submit:")) {
            pendingAction = "submit";
            return new Action("submit", new Identifier(step.substring(step.indexOf(':') + 1)));
        }
        if (step.startsWith("rotate:")) {
            String direction = step.substring(step.indexOf(':') + 1);
            if (!rotationPossible(direction)) {
                if (retrieveBlockDirection == null) {
                    resetCarriedBlockTracking();
                    currentIntention = null;
                    return skip();
                } else {
                    rememberFailedRotationTarget(direction);
                    String rotatedDirection = rotateDirection(
                            retrieveBlockDirection, "cw".equals(direction));
                    currentIntention = new Intention(
                            Desire.CLEAR_OBSTACLE, List.of(rotatedDirection), 0);
                    return executeClear();
                }
            }
            pendingAction = "rotate";
            pendingRotation = direction;
            return new Action("rotate", new Identifier(direction));
        }
        if (step.startsWith("clear:")) {
            String direction = step.substring(step.indexOf(':') + 1);
            pendingAction = "clear";
            clearDirection = direction;
            int[] offset = directionOffset(direction);
            return new Action("clear", new Numeral(offset[0]), new Numeral(offset[1]));
        }
        return executeMove();
    }

    private boolean isVisibleRequestedBlockAt(String direction) {
        int[] offset = directionOffset(direction);
        return currentVisibleThings.stream().anyMatch(thing ->
                thing.x() == offset[0]
                        && thing.y() == offset[1]
                        && thing.type().equals("block")
                        && deliveryBlockType != null
                        && thing.details().equalsIgnoreCase(deliveryBlockType));
    }

    private boolean isFreeBlockAt(String direction, String blockType) {
        int[] offset = directionOffset(direction);
        return currentVisibleThings.stream().anyMatch(thing ->
                thing.x() == offset[0]
                        && thing.y() == offset[1]
                        && thing.type().equals("block")
                        && thing.details().equalsIgnoreCase(blockType));
    }

    private boolean rotationPossible(String rotation) {
        if (retrieveBlockDirection == null) {
            return false;
        }
        String rotatedDirection = rotateDirection(
                retrieveBlockDirection, "cw".equals(rotation));
        int[] offset = directionOffset(rotatedDirection);
        InternalMap.Position rotatedBlockPosition = new InternalMap.Position(
                internalMap.getAgentX() + offset[0], internalMap.getAgentY() + offset[1]);
        return !internalMap.getBlockedPositions().contains(rotatedBlockPosition);
    }

    private void rememberFailedRotationTarget() {
        if (pendingRotation != null) {
            rememberFailedRotationTarget(pendingRotation);
        }
    }

    private void rememberFailedRotationTarget(String rotation) {
        if (retrieveBlockDirection == null) {
            return;
        }
        String rotatedDirection = rotateDirection(
                retrieveBlockDirection, "cw".equals(rotation));
        int[] offset = directionOffset(rotatedDirection);
        internalMap.rememberFailedPath(
                internalMap.getAgentX() + offset[0],
                internalMap.getAgentY() + offset[1], currentStep);
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

    /**
     * Processes the current simulation step and returns the action to execute.
     * Returns {@code null} when the percept batch does not represent a new action cycle.
     *
     * @return the selected action, or {@code null} when no action is due
     */
    @Override
    public Action step() {
        List<Percept> percepts = getPercepts();

        if (!isNewActionCycle(percepts)) {
            return null;
        }

        updateAgentPosition(percepts);
        updateBeliefs(percepts);
        internalMap.updateFromPercepts(percepts, teamName);
        updateCurrentVisibleThings(percepts);
        refreshGroupGoalLocation();
        processTeammateRequests();
        exchangeTeammateNames();

        System.out.println("Name: " + getName() + ", Percepts: " + percepts.toString());

        updateIntentionAfterAction(percepts);
        verifyCarriedBlock(percepts);
        updateBlockDeliveryStatus();
        if (assemblyCleanupRequested) {
            currentIntention = null;
        }

        if (!explorationFinished && allExplorationRequirementsKnown()) {
            explorationFinished = true;
            if (currentIntention != null && currentIntention.desire() == Desire.EXPLORE) {
                currentIntention = null;
                explorationTarget = null;
            }
        }

        updateGroupState();
        if (isCurrentGroupLeader() && !assemblyCleanupRequested) {
            startNextPendingAssemblyConnection();
        }
        if (shouldInterruptForRole()) {
            currentIntention = null;
            explorationTarget = null;
        }

        replanMovementIntention();

        if (currentIntention == null || currentIntention.finished()) {
            Set<Desire> desires = generateDesires();
            currentIntention = selectIntention(desires);
        }

        exchangeMapUpdates(percepts);

        return executeIntention();
    }
}