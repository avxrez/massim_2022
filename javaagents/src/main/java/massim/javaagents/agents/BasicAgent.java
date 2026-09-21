package massim.javaagents.agents;

import eis.iilang.*;
import massim.javaagents.MailService;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
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
        RETRIEVE_BLOCK,
        WAIT,
        REACH_ROLE_ZONE,
        ADAPT_ROLE,
        SUBMIT
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

    private static final int VISION_RANGE = 5;
    private static final int MIN_SHARED_VERIFICATION_THINGS = 3;
    private static final int ROLE_ZONE_MAX_DISTANCE = 20;
    private static final int MIN_GROUP_DISTANCE = 10;
    private static final String DEFAULT_ROLE = "default";
    private static final String WORKER_ROLE = "worker";
    private static final String SERVER_AGENT_PREFIX = "agent";

    private String leaderName = "";
    private int lastID = -1;
    private int currentStep = -1;
    private int taskDeadline = -1;
    private int energy = -1;
    private boolean deactivated;
    private String currentRole = "";
    private String currentTask;
    private String teamName = "";
            private final Map<String, Boolean> knownAgentGroupState = new HashMap<>();
            private final Map<String, String> knownAgentGroupLeader = new HashMap<>();
    private final Set<String> currentGroupMembers = new HashSet<>();
    private final Set<String> pendingGroupInvitations = new HashSet<>();
    private final Set<String> rejectedGroupInviteTargets = new HashSet<>();
    private String currentGroupInviteTarget = null;
    private boolean groupInviteRetryRequested = false;
    private boolean waitingForNextTask = false;
    private boolean groupGoalZoneConfirmed = false;
    private int currentTaskBlockCount = 1;
    private int desiredGroupSize = 1;
    private String currentGroupLeader = "";
        private boolean groupLeaderMode = false;
    private boolean groupFormationActive = false;
    private boolean blocksAssignedForCurrentGroup = false;
    private String groupTaskName = "";
    private String deliveryBlockType = null;
    private InternalMap.Position requiredBlockOffset;

    private final Set<String> requiredDispenserTypes = new HashSet<>();
    private final Set<InternalMap.Position> assembledBlockPositions = new HashSet<>();
    private final Map<InternalMap.Position, String> assembledBlockTypes = new HashMap<>();
    private final Map<InternalMap.Position, PendingAssemblyAttachment> pendingAssemblyDeliveries = new HashMap<>();
    private final Set<String> pendingAssemblyDetaches = new HashSet<>();
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

    private record VisibleThing(int x, int y, String type, String details) {}

        private record PendingTeammateRequest(String sender, String senderLeaderName, int x, int y,
            int senderX, int senderY, int receiverX, int receiverY,
            List<VisibleThing> senderVisibleThings) {}

        private record PendingTeammateConfirmation(String senderLeaderName, int senderX, int senderY,
            int relativeX, int relativeY) {}

    private record PendingAssemblyAttachment(String member, String blockType,
            InternalMap.Position targetPosition, String detachDirection) {}

        private record PendingAssemblyConnection(String partner, String blockType,
            InternalMap.Position targetPosition, int leaderBlockX, int leaderBlockY) {}

    // ============================================================
    // CURRENT INTENTION
    // ============================================================

    private InternalMap.Position explorationTarget;
    private InternalMap.Position goalPosition;
    private InternalMap.Position carriedBlockPosition;
    private String retrieveBlockDirection;
    private boolean blockRequested;
    private boolean blockRetrieved;
    private String carriedBlockType;
    private boolean blockPlaced;
    private boolean detachRequested;
    private boolean attachmentCheckPending;
    private boolean explorationFinished;
    private Intention currentIntention;
    private String pendingAction;
    private String pendingDirection;
    private String pendingRotation;
    private String clearDirection;
    private PendingAssemblyAttachment pendingAssemblyAttachment;
    private PendingAssemblyConnection pendingAssemblyConnection;


    // ============================================================
    // PATHFINDING
    // ============================================================

    private final AStarPathPlanner pathPlanner = new AStarPathPlanner();
    private final ExplorationTargetSelector explorationTargetSelector = new ExplorationTargetSelector();

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
            internalMap.replaceReservationSnapshot(message.getParameters().get(0));
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
            internalMap.replaceReservationSnapshot(map);
            internalMap.setObservations(map);
            if (message.getParameters().size() > 5) {
                mergeKnownAgents(message.getParameters().get(5), sender);
            }
            System.out.println("Received new leader message from " + sender + " to switch from "
                    + previousLeader.getValue() + " to " + newLeader.getValue());
            leaderName = newLeader.getValue();
                notifyKnownForNewLeader(previousLeader.getValue(), newLeader.getValue(),
                    offsetX.getValue().intValue(), offsetY.getValue().intValue(), sender);
            sendMergedMapUpdates();
            return;
        }

        if (message.getName().equals("mapUpdate")
                && message.getParameters().size() >= 5) {
            internalMap.replaceReservationSnapshot(message.getParameters().get(4));
        }

        if (message.getName().equals("mapUpdate")
                && message.getParameters().size() >= 6
                && message.getParameters().get(0) instanceof Numeral senderX
                && message.getParameters().get(1) instanceof Numeral senderY
                && message.getParameters().get(2) instanceof Numeral targetX
                && message.getParameters().get(3) instanceof Numeral targetY
                && message.getParameters().get(5) instanceof Identifier senderLeaderName
                && leaderName.equals(senderLeaderName.getValue())) {
            internalMap.replaceReservationSnapshot(message.getParameters().get(4));
            internalMap.mergeObservations(message.getParameters().get(4));
                rememberKnownAgent(sender,
                    new InternalMap.Position(
                        senderX.getValue().intValue(), senderY.getValue().intValue()), sender);
            updateKnownAgentGroupState(sender, message);
            if (message.getParameters().size() > 6) {
                mergeKnownAgents(message.getParameters().get(6), sender);
            }
            if (targetX.getValue().intValue() == Integer.MIN_VALUE) {
                knownTargets.remove(sender);
            } else {
                knownTargets.put(sender,
                        new InternalMap.Position(
                                targetX.getValue().intValue(), targetY.getValue().intValue()));
            }
            return;
        }else if (message.getName().equals("mapUpdate")) {
            System.out.println("missupdated from " + sender + " with leader " + message.getParameters().get(5) + " and my leader is " + leaderName);
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
                    if (nameNumber(leaderName) > nameNumber(confirmation.senderLeaderName())) {
                        int offsetX = confirmation.senderX() + confirmation.relativeX()
                            - internalMap.getAgentX();
                        int offsetY = confirmation.senderY() + confirmation.relativeY()
                            - internalMap.getAgentY();
                        switchLeader(confirmation.senderLeaderName(), offsetX, offsetY);
                    }
                    rememberKnownAgent(sender,
                        new InternalMap.Position(
                            internalMap.getAgentX() - confirmation.relativeX(),
                            internalMap.getAgentY() - confirmation.relativeY()), getName());
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
                        if (nameNumber(leaderName) > nameNumber(confirmation.senderLeaderName())) {
                            int offsetX = confirmation.senderX() + confirmation.relativeX()
                                - internalMap.getAgentX();
                            int offsetY = confirmation.senderY() + confirmation.relativeY()
                                - internalMap.getAgentY();
                            switchLeader(confirmation.senderLeaderName(), offsetX, offsetY);
                        }
                        rememberKnownAgent(sender,
                            new InternalMap.Position(
                                internalMap.getAgentX() - confirmation.relativeX(),
                                internalMap.getAgentY() - confirmation.relativeY()), getName());
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
                System.out.println(getName() + " declines invite from " + leader.getValue()
                        + " because it is already in a group.");
                sendMessage(new Percept("groupInviteRejected",
                        new Identifier(getName())), leader.getValue(), getName());
                return;
            }

            System.out.println(getName() + " received group invite from " + leader.getValue()
                    + " for task " + task.getValue() + " with target size " + size.getValue().intValue());
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
            System.out.println(getName() + " accepted invite and joined group of leader " + currentGroupLeader);
            return;
        }

        if (message.getName().equals("groupInviteRejected")
                && message.getParameters().size() >= 1
                && message.getParameters().get(0) instanceof Identifier rejectedAgent
                && groupLeaderMode) {
            pendingGroupInvitations.remove(rejectedAgent.getValue());
            currentGroupInviteTarget = null;
            rejectedGroupInviteTargets.add(rejectedAgent.getValue());
            System.out.println(getName() + " received rejection from " + rejectedAgent.getValue()
                    + ". Skipping this agent for the current group and waiting for the next free agent.");
            if (currentGroupMembers.size() < desiredGroupSize) {
                groupInviteRetryRequested = true;
            }
            return;
        }

        if (message.getName().equals("groupJoinAccepted")
                && message.getParameters().size() >= 2
                && message.getParameters().get(0) instanceof Identifier agent
                && message.getParameters().get(1) instanceof Identifier task
                && groupLeaderMode) {
            pendingGroupInvitations.remove(agent.getValue());
            currentGroupInviteTarget = null;
            System.out.println(getName() + " accepted agent " + agent.getValue()
                    + " into group for task " + task.getValue()
                    + ". Current group size: " + currentGroupMembers.size() + "/" + desiredGroupSize);
            knownAgentGroupState.put(agent.getValue(), true);
            knownAgentGroupLeader.put(agent.getValue(), getName());
            currentGroupMembers.add(agent.getValue());
            if (currentGroupMembers.size() >= desiredGroupSize) {
                System.out.println(getName() + " group is full (" + currentGroupMembers.size() + "/" + desiredGroupSize + "), selecting next group leader if needed.");
                recruitNextGroupLeader();
            } else {
                groupInviteRetryRequested = true;
            }
            return;
        }

        if (message.getName().equals("groupStart")
                && message.getParameters().size() >= 2
                && message.getParameters().get(0) instanceof Identifier task
                && message.getParameters().get(1) instanceof Numeral size) {
            if (currentTask != null
                && currentTask.equals(task.getValue())
                    && isTaskActive()
                && !DEFAULT_ROLE.equals(currentRole)
                && !groupFormationActive
                && (!knownAgentGroupState.getOrDefault(getName(), false)
                    || currentGroupLeader.equals(sender))) {
                System.out.println(getName() + " received start signal for next group on task "
                        + task.getValue() + " with target size " + size.getValue().intValue());
                groupTaskName = task.getValue();
                desiredGroupSize = size.getValue().intValue();
                currentGroupLeader = getName();
                groupLeaderMode = true;
                groupFormationActive = true;
                currentGroupMembers.clear();
                currentGroupMembers.add(getName());
                knownAgentGroupState.put(getName(), true);
                knownAgentGroupLeader.put(getName(), getName());
                System.out.println(getName() + " starts new group as leader for task " + groupTaskName
                        + " (size " + desiredGroupSize + ")");
                inviteKnownAgentsToGroup();
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
            if (message.getParameters().size() >= 6
                    && message.getParameters().get(4) instanceof Numeral originX
                    && message.getParameters().get(5) instanceof Numeral originY) {
                reserveAssemblyGoalPosition(new InternalMap.Position(
                        originX.getValue().intValue(), originY.getValue().intValue()));
            }
            if (message.getParameters().size() >= 4
                    && message.getParameters().get(3) instanceof Identifier expectedDirection) {
                retrieveBlockDirection = expectedDirection.getValue();
            }
            if (!sameAssignment || (!blockRequested && !blockRetrieved)) {
                prepareForBlockAssignment();
            }
            System.out.println(getName() + " received custom group block task: fetch " + deliveryBlockType
                    + " and deliver it to absolute target (" + goalPosition.x() + ", " + goalPosition.y()
                    + ") with required side " + retrieveBlockDirection); 
            currentIntention = null;
            return;
        }

        if (message.getName().equals("groupBlockDelivered")
                && message.getParameters().size() >= 3
                && message.getParameters().get(0) instanceof Identifier blockType
                && message.getParameters().get(1) instanceof Numeral targetX
                && message.getParameters().get(2) instanceof Numeral targetY
                && leaderName.equals(getName())) {
            InternalMap.Position target = new InternalMap.Position(
                    targetX.getValue().intValue(), targetY.getValue().intValue());
            if (goalPosition == null) {
                goalPosition = target;
            }
                InternalMap.Position assemblyPosition = goalPosition;
                if (!isAtGoalPosition()) {
                    pendingAssemblyDeliveries.put(target,
                            new PendingAssemblyAttachment(sender, blockType.getValue(), target, null));
                    return;
                }
                int targetDistance = distanceTo(target.x(), target.y(),
                    assemblyPosition.x(), assemblyPosition.y());
                if (targetDistance != 1) {
                    PendingAssemblyAttachment delivery = new PendingAssemblyAttachment(
                            sender, blockType.getValue(), target, null);
                    pendingAssemblyDeliveries.put(target, delivery);
                    tryStartPendingAssemblyConnection(delivery);
                return;
                }
                String attachDirection = directionTo(target, assemblyPosition);
            deliveryBlockType = blockType.getValue();
            pendingAssemblyAttachment = new PendingAssemblyAttachment(
                    sender,
                    deliveryBlockType,
                    target,
                    attachDirection);
            currentIntention = new Intention(Desire.RETRIEVE_BLOCK, List.of("attach:" + attachDirection), 0);
            System.out.println(getName() + " leader accepts delivered block " + deliveryBlockType
                    + " at target (" + target.x() + ", " + target.y() + ") and will attach it");
            return;
        }

        if (message.getName().equals("groupConnectRequest")
            && message.getParameters().size() >= 5
                && message.getParameters().get(0) instanceof Numeral leaderBlockX
                && message.getParameters().get(1) instanceof Numeral leaderBlockY
                && message.getParameters().get(2) instanceof Numeral targetX
                && message.getParameters().get(3) instanceof Numeral targetY
                && message.getParameters().get(4) instanceof Identifier blockType
                && (sender.equals(currentGroupLeader) || sender.equals(leaderName))) {
            pendingAssemblyConnection = new PendingAssemblyConnection(
                    sender,
                    blockType.getValue(),
                    new InternalMap.Position(targetX.getValue().intValue(), targetY.getValue().intValue()),
                    leaderBlockX.getValue().intValue(), leaderBlockY.getValue().intValue());
            if (currentGroupLeader.isEmpty()) {
                currentGroupLeader = sender;
            }
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

        if (message.getName().equals("groupDetachBlock")
                && !currentGroupLeader.isEmpty()
                && sender.equals(currentGroupLeader)) {
            detachRequested = true;
            currentIntention = null;
            System.out.println(getName() + " received detach instruction for delivered block");
            return;
        }

        if (message.getName().equals("groupBlockDetached")
                && leaderName.equals(getName())) {
            pendingAssemblyDetaches.remove(sender);
            System.out.println(getName() + " received detach confirmation from " + sender
                    + "; pending detaches: " + pendingAssemblyDetaches);
            return;
        }

        if (message.getName().equals("newRetrieveBlockLocation")
                && message.getParameters().size() >= 3
                && message.getParameters().get(0) instanceof Identifier blockType
                && message.getParameters().get(1) instanceof Numeral targetX
                && message.getParameters().get(2) instanceof Numeral targetY) {
            if (deliveryBlockType == null
                    || deliveryBlockType.equalsIgnoreCase(blockType.getValue())) {
                deliveryBlockType = blockType.getValue();
                internalMap.clearReservedAssemblyPositions(getName());
                goalPosition = new InternalMap.Position(
                        targetX.getValue().intValue(), targetY.getValue().intValue());
                blockPlaced = false;
                currentIntention = null;
                System.out.println(getName() + " updated retrieve-block target to ("
                        + goalPosition.x() + ", " + goalPosition.y() + ")");
            }
            return;
        }

        if (message.getName().equals("retrieveBlockLocationUnavailable")
                && leaderName.equals(getName())
                && (groupLeaderMode || currentGroupLeader.equals(getName()))) {
            if (message.getParameters().size() >= 3
                    && message.getParameters().get(0) instanceof Identifier blockType
                    && message.getParameters().get(1) instanceof Numeral targetX
                    && message.getParameters().get(2) instanceof Numeral targetY
                    && deliveryBlockType != null
                    && deliveryBlockType.equalsIgnoreCase(blockType.getValue())) {
                internalMap.clearReservedAssemblyPositions(getName());
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
            System.out.println(getName() + " received group dissolve notice from leader " + sender);
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
                } else if ((percept.getName().equals("goalZone")
                    || percept.getName().equals("roleZone"))
                    && percept.getParameters().size() >= 2
                    && percept.getParameters().get(0) instanceof Numeral x
                    && percept.getParameters().get(1) instanceof Numeral y) {
                currentVisibleThings.add(new VisibleThing(
                    x.getValue().intValue(), y.getValue().intValue(),
                    percept.getName(), ""));
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

    /** Verschiebt alle bekannten Agenten-, Ziel- und Erkundungspositionen um den angegebenen Offset. */
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
        if (explorationTarget != null) {
            explorationTarget = new InternalMap.Position(
                    explorationTarget.x() + offsetX, explorationTarget.y() + offsetY);
        }
        if (goalPosition != null) {
            goalPosition = new InternalMap.Position(
                    goalPosition.x() + offsetX, goalPosition.y() + offsetY);
        }
    }

    /** Verschiebt die interne Karte sowie alle bekannten Positionen um den angegebenen Offset. */
    private void translateWorld(int offsetX, int offsetY) {
        internalMap.translate(offsetX, offsetY);
        translateKnownPositions(offsetX, offsetY);
    }

    private ParameterList currentMapPercepts(List<Percept> percepts) {
        return internalMap.currentPercepts(percepts, currentStep);
    }

    /**
     * Updates the internal agent position using the result of the previous
     * move action.
     */
    private void updateAgentPosition(List<Percept> percepts) {
        internalMap.updateAgentPositionFromPercepts(percepts);
    }

    // ============================================================
    // BELIEFS
    // ============================================================

    /**
     * Updates scalar beliefs from the current percepts.
     */
    private void updateBeliefs(List<Percept> percepts) {
        requiredDispenserTypes.clear();
        boolean taskPerceptReceived = false;
        String previousTask = currentTask;

        for (Percept percept : percepts) {
            if (percept.getParameters().isEmpty()) {
                continue;
            }

            switch (percept.getName()) {
                case "step" -> currentStep = numberValue(percept, currentStep);
                case "energy" -> energy = numberValue(percept, energy);
                case "role" -> {
                    if (percept.getParameters().size() == 1
                            && percept.getParameters().get(0) instanceof Identifier identifier) {
                        currentRole = identifier.getValue();
                    }
                }
                case "team" -> teamName = identifierValue(percept, teamName);
                case "deactivated" -> deactivated = identifierValue(percept, "false").equals("true");
                case "task" -> {
                    taskPerceptReceived = true;
                    currentTask = identifierValue(percept, currentTask);
                    if (percept.getParameters().size() > 1
                            && percept.getParameters().get(1) instanceof Numeral deadline) {
                        taskDeadline = deadline.getValue().intValue();
                    }
                    rememberTaskRequirements(percept);
                }
                default -> {
                    // Not a scalar belief.
                }
            }
        }

        if (!taskPerceptReceived) {
            resetGroupStateForNewTask();
            currentTask = null;
            taskDeadline = -1;
            currentTaskBlockCount = 1;
            desiredGroupSize = 1;
            requiredBlockOffset = null;
            deliveryBlockType = null;
            goalPosition = null;
            currentIntention = null;
            resetCarriedBlockTracking();
            internalMap.clearReservedAssemblyPositions(getName());
        } else if (!Objects.equals(currentTask, previousTask)) {
            waitingForNextTask = false;
            resetGroupStateForNewTask();
            deliveryBlockType = null;
            goalPosition = null;
            currentIntention = null;
            resetRetrieveAssignment();
        } else if (!isTaskActive()) {
            deliveryBlockType = null;
            goalPosition = null;
            currentIntention = null;
            resetRetrieveAssignment();
        }

        if (deactivated) {
            resetCarriedBlockTracking();
        }
    }

    private boolean isVisibleTeammateAt(int x, int y) {
        return internalMap.getVisibleTeammates().contains(new InternalMap.Position(x, y));
    }

    private int nameNumber(String name) {
        String number = name.replaceAll("[^0-9]", "");
        return number.isEmpty() ? -1 : Integer.parseInt(number);
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
        ParameterList content = currentMapPercepts(percepts);
        ParameterList knownAgentsContent = knownAgentsParameters();
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
                    content,
                    new Identifier(leaderName),
                        knownAgentsContent,
                        new Identifier(Boolean.TRUE.equals(knownAgentGroupState.get(getName()))
                            ? "grouped" : "free"),
                        new Identifier(knownAgentGroupLeader.getOrDefault(getName(), ""))), agent, getName());
        }
    }

    private void sendMergedMapUpdates() {
        ParameterList content = mapParameters();
        ParameterList knownAgentsContent = knownAgentsParameters();
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
                    content,
                    new Identifier(leaderName),
                        knownAgentsContent,
                        new Identifier(Boolean.TRUE.equals(knownAgentGroupState.get(getName()))
                            ? "grouped" : "free"),
                        new Identifier(knownAgentGroupLeader.getOrDefault(getName(), ""))), agent, getName());
        }
    }

    /**
     * Extracts the dispenser requirements from the current task.
     */
    private void rememberTaskRequirements(Percept taskPercept) {
        if (taskPercept.getParameters().size() < 4
                || !(taskPercept.getParameters().get(3) instanceof ParameterList requirements)) {
            return;
        }

        requiredDispenserTypes.clear();
        taskBlockTypes.clear();
        taskRequirementOffsets.clear();
        currentTaskBlockCount = 0;
        requiredBlockOffset = null;
        for (Parameter requirement : requirements) {
            if (requirement instanceof Function function
                    && function.getParameters().size() >= 3
                    && function.getParameters().get(0) instanceof Numeral requiredX
                    && function.getParameters().get(1) instanceof Numeral requiredY
                    && function.getParameters().get(2) instanceof Identifier type) {
                requiredDispenserTypes.add(type.getValue());
                taskBlockTypes.add(type.getValue());
                InternalMap.Position offset = new InternalMap.Position(
                        requiredX.getValue().intValue(), requiredY.getValue().intValue());
                taskRequirementOffsets.add(offset);
                if (currentTaskBlockCount == 0) {
                    requiredBlockOffset = offset;
                }
                currentTaskBlockCount++;
            }
        }
        if (currentTaskBlockCount <= 0) {
            currentTaskBlockCount = 1;
        }
        desiredGroupSize = calculateDesiredGroupSize();
    }

    private int calculateDesiredGroupSize() {
        if (currentTaskBlockCount <= 1) {
            return 1;
        }
        return currentTaskBlockCount + 1;
    }

    static boolean hasAllRequiredBlockTypes(List<String> availableBlockTypes, List<String> requiredBlockTypes) {
        if (requiredBlockTypes == null || requiredBlockTypes.isEmpty()) {
            return true;
        }
        Set<String> available = new HashSet<>(availableBlockTypes);
        for (String requiredType : requiredBlockTypes) {
            if (!available.contains(requiredType)) {
                return false;
            }
        }
        return true;
    }

    private boolean isTaskActive() {
        return taskDeadline < 0 || currentStep <= taskDeadline;
    }

    private void resetGroupStateForNewTask() {
        for (String member : new ArrayList<>(currentGroupMembers)) {
            if (!member.equals(getName())) {
                sendMessage(new Percept("groupDissolve"), member, getName());
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
        groupInviteRetryRequested = false;
        groupLeaderMode = false;
        groupFormationActive = false;
        currentGroupLeader = "";
        groupTaskName = "";
    }

    private void startGroupFormation() {
        if (desiredGroupSize <= 1
                || !leaderName.equals(getName())
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
        System.out.println(getName() + " starts group formation after exploration for task " + currentTask
                + " with desired size " + calculateDesiredGroupSize());
        desiredGroupSize = calculateDesiredGroupSize();
        groupTaskName = currentTask;
        groupLeaderMode = true;
        groupFormationActive = true;
        blocksAssignedForCurrentGroup = false;
        currentGroupLeader = getName();
        currentGroupMembers.clear();
        currentGroupMembers.add(getName());
        goalPosition = new InternalMap.Position(selectedGoalZone.x(), selectedGoalZone.y());
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
            System.out.println(getName() + " invites " + agent + " to group for task " + groupTaskName
                    + " (current members: " + currentGroupMembers.size() + "/" + desiredGroupSize + ")");
            sendMessage(new Percept("groupInvite",
                    new Identifier(getName()),
                    new Identifier(groupTaskName),
                    new Numeral(desiredGroupSize)), agent, getName());
            return;
        }
        System.out.println(getName() + " found no free known agent for group task " + groupTaskName
            + " (known agents: " + invitationCandidates
            + ", rejected: " + rejectedGroupInviteTargets + ")");
    }

    private void recruitNextGroupLeader() {
        if (!groupLeaderMode || !groupFormationActive || currentGroupMembers.size() < desiredGroupSize) {
            return;
        }

        assignBlocksToCurrentGroup();
        groupFormationActive = false;

        System.out.println(getName() + " attempts to start the next group after reaching full size "
                + currentGroupMembers.size() + "/" + desiredGroupSize);
        for (String agent : knownAgents.keySet()) {
            if (agent.equals(getName())
                    || currentGroupMembers.contains(agent)
                    || Boolean.TRUE.equals(knownAgentGroupState.get(agent))
                    || rejectedGroupInviteTargets.contains(agent)) {
                continue;
            }
            System.out.println(getName() + " appoints " + agent + " as next group leader for task " + groupTaskName);
            sendMessage(new Percept("groupStart",
                    new Identifier(groupTaskName),
                    new Numeral(desiredGroupSize)), agent, getName());
                    groupLeaderMode = false;
                    currentGroupLeader = getName();
            return;
        }
        System.out.println(getName() + " has no free known agent left for a new group.");
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

        private void assignBlocksToCurrentGroupAt(InternalMap.Position leaderGoalAnchor) {
        List<String> members = new ArrayList<>(currentGroupMembers);
        members.sort(String::compareTo);

        assembledBlockPositions.clear();
        assembledBlockTypes.clear();
        pendingAssemblyDeliveries.clear();
        pendingAssemblyDetaches.clear();
        pendingAssemblyConnection = null;

        goalPosition = leaderGoalAnchor;
        groupGoalZoneConfirmed = isKnownGoalZone(leaderGoalAnchor);
        reserveAssemblyGoalPosition(leaderGoalAnchor);
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
            System.out.println(getName() + " assigns " + blockType + " to " + member
                + " with delivery target (" + deliveryTarget.x() + ", " + deliveryTarget.y()
                + ") anchored to leader position (" + leaderGoalAnchor.x() + ", " + leaderGoalAnchor.y()
                + ") and required side " + requiredDirection);
        }

        if (assignmentCount < fetchers.size()) {
            System.out.println(getName() + " cannot assign all group members: received "
                    + taskRequirementOffsets.size() + " task offsets for " + fetchers.size() + " fetchers.");
        }
        blocksAssignedForCurrentGroup = true;
    }

    private void refreshGroupGoalLocation() {
        if (goalPosition == null || !isCurrentGroupLeader()) {
            return;
        }

        if (isKnownGoalZone(goalPosition)) {
            groupGoalZoneConfirmed = true;
            return;
        }

        if (deliveryBlockType == null && !groupGoalZoneConfirmed) {
            return;
        }

        InternalMap.Observation replacement = findNearestGoalZone();
        if (replacement == null) {
            dissolveGroupAndResumeExploration();
            return;
        }

        goalPosition = new InternalMap.Position(replacement.x(), replacement.y());
        currentIntention = null;
        assembledBlockPositions.clear();
        assembledBlockTypes.clear();
        pendingAssemblyDeliveries.clear();
        pendingAssemblyDetaches.clear();
        pendingAssemblyConnection = null;
        assignBlocksToCurrentGroupAt(goalPosition);
    }

        private void tryStartPendingAssemblyConnection(PendingAssemblyAttachment delivery) {
        if (!leaderName.equals(getName()) || goalPosition == null
            || pendingAssemblyConnection != null) {
            return;
        }

        InternalMap.Position bridge = findAssembledBridge(delivery.targetPosition());
        if (bridge == null) {
            System.out.println(getName() + " waits for bridge block at ("
                + delivery.targetPosition().x() + ", " + delivery.targetPosition().y()
                + ") before connecting block at ("
                + delivery.targetPosition().x() + ", " + delivery.targetPosition().y() + ")");
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
                        delivery.member(), delivery.blockType(), delivery.targetPosition(), attachDirection);
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

    private void dissolveGroupAndResumeExploration() {
        Set<String> members = new HashSet<>(currentGroupMembers);
        for (Map.Entry<String, String> entry : knownAgentGroupLeader.entrySet()) {
            if (getName().equals(entry.getValue())) {
                members.add(entry.getKey());
            }
        }
        for (String member : members) {
            if (!member.equals(getName())) {
                sendMessage(new Percept("groupDissolve"), member, getName());
            }
        }
        resetGroupStateForNewTask();
        explorationFinished = false;
        resetRetrieveAssignment();
    }

    private void dissolveCurrentGroup() {
        if (!groupLeaderMode || !groupFormationActive) {
            return;
        }

        System.out.println(getName() + " dissolves current group for task " + groupTaskName
                + " because the task changed or the group is no longer required.");
        for (String member : new ArrayList<>(currentGroupMembers)) {
            sendMessage(new Percept("groupDissolve"), member, getName());
            knownAgentGroupState.put(member, false);
            knownAgentGroupLeader.remove(member);
        }
        currentGroupMembers.clear();
        pendingGroupInvitations.clear();
        rejectedGroupInviteTargets.clear();
        currentGroupInviteTarget = null;
        groupInviteRetryRequested = false;
        groupLeaderMode = false;
        groupFormationActive = false;
        currentGroupLeader = "";
        groupTaskName = "";
        resetRetrieveAssignment();
    }

    private void resetRetrieveAssignment() {
        blocksAssignedForCurrentGroup = false;
        deliveryBlockType = null;
        goalPosition = null;
        pendingAssemblyAttachment = null;
        pendingAssemblyConnection = null;
        assembledBlockPositions.clear();
        assembledBlockTypes.clear();
        pendingAssemblyDeliveries.clear();
        pendingAssemblyDetaches.clear();
        internalMap.clearReservedAssemblyPositions(getName());
        currentIntention = null;
        if (!blockRetrieved) {
            resetCarriedBlockTracking();
        } else {
            blockRequested = false;
            blockPlaced = false;
            attachmentCheckPending = false;
        }
    }

    private void updateGroupState() {
        resetGroupStateAfterTaskChange();

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
                System.out.println(getName() + " self-assigns solo block task " + deliveryBlockType
                        + " to goal zone (" + goalPosition.x() + ", " + goalPosition.y() + ")");
            }
        }

        if (desiredGroupSize > 1
                && leaderName.equals(getName())
                && !groupFormationActive
                && explorationFinished
                && !waitingForNextTask
                && currentTask != null
            && !currentTask.isEmpty()
            && deliveryBlockType == null
            && !Boolean.TRUE.equals(knownAgentGroupState.get(getName()))
            && isTaskActive()) {
            startGroupFormation();
        }

        if (groupLeaderMode && groupFormationActive
                && currentGroupMembers.size() < desiredGroupSize
                && currentGroupInviteTarget == null
                && pendingGroupInvitations.isEmpty()) {
            groupInviteRetryRequested = false;
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
            System.out.println(getName() + " leaves old group " + groupTaskName
                    + " because the task is no longer active.");
            knownAgentGroupState.put(getName(), false);
            knownAgentGroupLeader.remove(getName());
            currentGroupMembers.clear();
            pendingGroupInvitations.clear();
            currentGroupInviteTarget = null;
            groupInviteRetryRequested = false;
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

    // ============================================================
    // INTENTION FEEDBACK
    // ============================================================

    /**
     * Checks whether the environment has advanced to a new action cycle.
     */
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

    /**
     * Updates the current intention according to the result of the
     * previously executed action.
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

        if ("success".equals(lastActionResult)) {
            // Action succeeded.
            currentIntention = currentIntention.advance();

                if ("clear".equals(lastAction)
                    && "failed_target".equals(lastActionResult)
                    && clearDirection != null) {
                int[] offset = directionOffset(clearDirection);
                internalMap.forgetObservationsAt(
                        internalMap.getAgentX() + offset[0], internalMap.getAgentY() + offset[1]);
            }
            if ("request".equals(lastAction)) {
                blockRequested = true;
            } else if ("rotate".equals(lastAction) && pendingRotation != null) {
                retrieveBlockDirection = rotateDirection(
                        retrieveBlockDirection, "cw".equals(pendingRotation));
            } else if ("attach".equals(lastAction)) {
                if (pendingAssemblyAttachment != null) {
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
                attachmentCheckPending = true;
            } else if ("connect".equals(lastAction)
                    && pendingAssemblyConnection != null) {
                if (getName().equals(leaderName)) {
                    pendingAssemblyDetaches.add(pendingAssemblyConnection.partner());
                    sendMessage(new Percept("groupDetachBlock"),
                            pendingAssemblyConnection.partner(), getName());
                }
                assembledBlockPositions.add(pendingAssemblyConnection.targetPosition());
                assembledBlockTypes.put(relativeAssemblyPosition(
                    pendingAssemblyConnection.targetPosition()),
                    pendingAssemblyConnection.blockType());
                pendingAssemblyDeliveries.remove(pendingAssemblyConnection.targetPosition());
                pendingAssemblyConnection = null;
                currentIntention = null;
                blockPlaced = true;
                startNextPendingAssemblyConnection();
            } else if ("detach".equals(lastAction)) {
                detachRequested = false;
                if (!currentGroupLeader.isEmpty()) {
                    sendMessage(new Percept("groupBlockDetached", new Identifier(getName())),
                            currentGroupLeader, getName());
                }
                resetCarriedBlockTracking();
                currentIntention = null;
                if (deliveryBlockType != null && currentGroupLeader != null
                        && !currentGroupLeader.isEmpty()) {
                    System.out.println(getName() + " detached successfully and resumes retrieving "
                            + deliveryBlockType + " for group leader " + currentGroupLeader);
                }
            } else if ("submit".equals(lastAction)) {
                System.out.println(getName() + " submitted completed task " + currentTask
                        + "; resetting assembly map");
                resetRetrieveAssignment();
                waitingForNextTask = true;
            }
        } else {
            if ("clear".equals(lastAction) && clearDirection != null) {
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
            } else if ("move".equals(lastAction) && pendingDirection != null) {
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
        attachmentCheckPending = false;

        InternalMap.Position attachedPosition = findAttachedBlockPosition(percepts, retrieveBlockDirection);
        if (attachedPosition == null) {
            resetCarriedBlockTracking();
            return;
        }

        carriedBlockPosition = attachedPosition;
        retrieveBlockDirection = directionTo(
                attachedPosition,
                new InternalMap.Position(internalMap.getAgentX(), internalMap.getAgentY()));
    }

    private void resetCarriedBlockTracking() {
        blockRequested = false;
        blockRetrieved = false;
        carriedBlockType = null;
        blockPlaced = false;
        attachmentCheckPending = false;
        carriedBlockPosition = null;
        retrieveBlockDirection = null;
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

    // ============================================================
    // DESIRE GENERATION
    // ============================================================

    /**
     * Generates the current desires from the agent's beliefs.
     *
     * Important: This method only decides WHAT the agent wants to do.
     * It does not calculate paths or actions.
     */
    private Set<Desire> generateDesires() {
        Set<Desire> desires = EnumSet.noneOf(Desire.class);

        // Highest priority: wait when deactivated.
        if (deactivated) {
            desires.add(Desire.WAIT);
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

        if (leaderName.equals(getName())
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
            desires.add(isAtGoalPosition() ? Desire.WAIT : Desire.REACH_GOAL_ZONE);
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

        // Default behaviour for now.
        desires.add(Desire.EXPLORE);
        return desires;
    }

    // ============================================================
    // DESIRE CONDITIONS
    // ============================================================

    private boolean isAssemblyComplete() {
        if (goalPosition == null || taskRequirementOffsets.size() != taskBlockTypes.size()
                || taskRequirementOffsets.isEmpty()
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

    private boolean hasDispenser(String blockType) {
        return internalMap.getObservations().stream().anyMatch(observation ->
            observation.type().equals("dispenser")
                && observation.details().equalsIgnoreCase(blockType));
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

    private void printDispenserContents() {
        System.out.println("Known dispensers:");
        for (InternalMap.Observation observation : internalMap.getObservations()) {
            if (observation.type().equals("dispenser")) {
                System.out.println("- content=" + observation.details()
                        + ", position=(" + observation.x() + ", " + observation.y() + ")"
                        + ", lastSeenStep=" + observation.lastSeenStep());
            }
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

    // ============================================================
    // INTENTION SELECTION
    // ============================================================

    /**
     * Converts a desire into an intention.
     *
     * This method decides HOW the desired behaviour should currently be
     * achieved.
     */
    private Intention selectIntention(Set<Desire> desires) {
        if (desires.contains(Desire.WAIT)) {
            return new Intention(Desire.WAIT, List.of(), 0);
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

        List<String> path = pathPlanner.findPath(
                currentPosition(),
                new InternalMap.Position(roleZone.x(), roleZone.y()),
            blockedPositionsForMovement(),
            occupiedPositionsForMovement());
        if (nextMoveIsBlocked(path)) {
            return new Intention(Desire.CLEAR_OBSTACLE, List.of(path.get(0)), 0);
        }
        return new Intention(Desire.REACH_ROLE_ZONE, path, 0);
    }

    /**
     * Creates an intention for reaching the goal.
     */
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

        List<String> path = pathPlanner.findPath(start, target,
            blockedPositionsForMovement(), occupiedPositionsForMovement());
        if (isCurrentGroupLeader()
                && currentTaskBlockCount > 1
                && (!pathExists(path) || isPhysicallyOccupied(target))) {
            InternalMap.Position replacement = findReachableGoalZone(start, target);
            if (replacement != null) {
                relocateAssemblyGoal(replacement);
                target = replacement;
                path = pathPlanner.findPath(start, target,
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

    private boolean isPhysicallyOccupied(InternalMap.Position position) {
        return internalMap.getPhysicalOccupiedEntityPositions().contains(position)
                || internalMap.getPhysicalBlockedPositions().contains(position);
    }

    private InternalMap.Position findReachableGoalZone(InternalMap.Position start,
            InternalMap.Position currentTarget) {
        return internalMap.getObservations().stream()
                .filter(observation -> observation.type().equals("goalZone"))
                .map(observation -> new InternalMap.Position(observation.x(), observation.y()))
                .filter(candidate -> !candidate.equals(currentTarget))
                .filter(candidate -> !isPhysicallyOccupied(candidate))
                .filter(candidate -> pathExists(pathPlanner.findPath(start, candidate,
                        blockedPositionsForMovement(), occupiedPositionsForMovement())))
                .min((first, second) -> Integer.compare(
                        distanceTo(first.x(), first.y(), start.x(), start.y()),
                        distanceTo(second.x(), second.y(), start.x(), start.y())))
                .orElse(null);
    }

    private void relocateAssemblyGoal(InternalMap.Position replacement) {
        goalPosition = replacement;
        currentIntention = null;
        assembledBlockPositions.clear();
        assembledBlockTypes.clear();
        pendingAssemblyDeliveries.clear();
        pendingAssemblyDetaches.clear();
        pendingAssemblyConnection = null;
        assignBlocksToCurrentGroupAt(replacement);
    }

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
                return new Intention(Desire.WAIT, List.of(), 0);
            }

            InternalMap.Position dispenserPosition =
                    new InternalMap.Position(dispenser.x(), dispenser.y());
                String adjacentDirection = adjacentDirectionTo(dispenserPosition);
                if (adjacentDirection != null) {
                retrieveBlockDirection = adjacentDirection;
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
            return new Intention(Desire.WAIT, List.of(), 0);
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

            List<String> path = pathPlanner.findCarryingPathToAgentPosition(currentPosition(), deliveryTarget,
                    retrieveBlockDirection, requiredDirection, blockedPositionsForMovement(),
                    occupiedPositionsForMovement());
            if (path.isEmpty()) {
                return new Intention(Desire.WAIT, List.of(), 0);
            }
            return new Intention(Desire.RETRIEVE_BLOCK, path, 0);
        }

        if (deliveryTarget != null && carriedBlockPosition != null && carriedBlockPosition.equals(deliveryTarget)) {
            if (!currentGroupLeader.isEmpty() && !currentGroupLeader.equals(getName())) {
                sendMessage(new Percept("groupBlockDelivered",
                        new Identifier(deliveryBlockType),
                        new Numeral(deliveryTarget.x()),
                        new Numeral(deliveryTarget.y())), currentGroupLeader, getName());
            }
            blockPlaced = true;
            return new Intention(Desire.WAIT, List.of(), 0);
        }
        List<String> path = pathPlanner.findCarryingPath(currentPosition(), deliveryTarget,
            retrieveBlockDirection, blockedPositionsForMovement(),
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
            List<String> path = pathPlanner.findPath(currentPosition(), candidate,
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
        if (Math.abs(deltaX) + Math.abs(deltaY) != 1) {
            return null;
        }
        return directionTo(target, currentPosition());
    }

    private InternalMap.Observation findNearestDispenser(String blockType) {
        return internalMap.getObservations().stream()
                .filter(observation -> observation.type().equals("dispenser")
                && observation.details().equalsIgnoreCase(blockType))
                .min((first, second) -> Integer.compare(
                        distanceTo(first.x(), first.y(), internalMap.getAgentX(), internalMap.getAgentY()),
                        distanceTo(second.x(), second.y(), internalMap.getAgentX(), internalMap.getAgentY())))
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

    private String oppositeDirection(String direction) {
        return switch (direction) {
            case "n" -> "s";
            case "e" -> "w";
            case "s" -> "n";
            case "w" -> "e";
            default -> throw new IllegalArgumentException("Invalid direction: " + direction);
        };
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

    private String rotateDirection(String direction, boolean clockwise) {
        return switch (direction) {
            case "n" -> clockwise ? "e" : "w";
            case "e" -> clockwise ? "s" : "n";
            case "s" -> clockwise ? "w" : "e";
            case "w" -> clockwise ? "n" : "s";
            default -> throw new IllegalArgumentException("Invalid direction: " + direction);
        };
    }

    /**
     * Creates the exploration intention.
     *
     * Exploration target selection will be implemented separately.
     */
    private Intention createExploreIntention() {
        InternalMap.Position start = currentPosition();
        InternalMap.Position target = explorationTargetSelector.selectTarget(internalMap, knownTargets, getName());
        explorationTarget = target;

        List<String> path = pathPlanner.findPath(start, target,
            blockedPositionsForMovement(), occupiedPositionsForMovement());
        if (nextMoveIsBlocked(path)) {
            return new Intention(Desire.CLEAR_OBSTACLE, List.of(path.get(0)), 0);
        }
        return new Intention(Desire.EXPLORE, path, 0);
    }

    private InternalMap.Position currentPosition() {
        return new InternalMap.Position(internalMap.getAgentX(), internalMap.getAgentY());
    }

    private void reserveAssemblyGoalPosition(InternalMap.Position originGoalPosition) {
        Set<InternalMap.Position> reserved = new HashSet<>();
        reserved.add(originGoalPosition);
        for (InternalMap.Position offset : taskRequirementOffsets) {
            reserved.add(new InternalMap.Position(
                    originGoalPosition.x() + offset.x(),
                    originGoalPosition.y() + offset.y()));
        }
        if (overlapsForeignReservationDistance(reserved)) {
            System.out.println(getName() + " cannot reserve assembly near another group at "
                    + originGoalPosition);
            return;
        }
        internalMap.setReservedAssemblyPositions(getName(), reserved);
    }

    private boolean overlapsForeignReservationDistance(Set<InternalMap.Position> positions) {
        for (Map.Entry<String, Set<InternalMap.Position>> entry
                : internalMap.getReservationGroups().entrySet()) {
            if (isReservationForCurrentGroup(entry.getKey())) {
                continue;
            }
            for (InternalMap.Position candidate : positions) {
                for (InternalMap.Position reserved : entry.getValue()) {
                    if (distanceTo(candidate.x(), candidate.y(), reserved.x(), reserved.y())
                            < MIN_GROUP_DISTANCE) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    private List<InternalMap.Position> blockedPositionsForMovement() {
        List<InternalMap.Position> blocked = new ArrayList<>(internalMap.getBlockedPositions());
        if (carriedBlockPosition != null) {
            blocked.remove(carriedBlockPosition);
        }
        if (goalPosition != null
                && internalMap.isReservedAssemblyPosition(goalPosition)
                && !internalMap.getPhysicalBlockedPositions().contains(goalPosition)) {
            blocked.remove(goalPosition);
        }
        return blocked;
    }

    private Set<InternalMap.Position> occupiedPositionsForMovement() {
        Set<InternalMap.Position> occupied = new HashSet<>(
                internalMap.getPhysicalOccupiedEntityPositions());
        if (carriedBlockPosition != null) {
            occupied.remove(carriedBlockPosition);
        }
        InternalMap.Position current = currentPosition();

        for (Map.Entry<String, Set<InternalMap.Position>> entry
                : internalMap.getReservationGroups().entrySet()) {
            if (isReservationForCurrentGroup(entry.getKey())
                    && (isCurrentGroupLeader() || entry.getKey().equals(getName()))) {
                continue;
            }
            occupied.addAll(entry.getValue());
        }
        if (goalPosition != null
                && !internalMap.getPhysicalOccupiedEntityPositions().contains(goalPosition)) {
            occupied.remove(goalPosition);
        }
        return occupied;
    }

    private boolean isReservationForCurrentGroup(String owner) {
        if (owner.equals(getName())) {
            return true;
        }
        if (!currentGroupLeader.isEmpty()
                && owner.equals(currentGroupLeader)) {
            return true;
        }
        return !currentGroupLeader.isEmpty()
                && currentGroupLeader.equals(knownAgentGroupLeader.get(owner));
    }

    private boolean isCurrentGroupLeader() {
        return groupLeaderMode || getName().equals(currentGroupLeader);
    }

    private boolean explorationTargetSeen() {
        return explorationTarget != null
                && internalMap.isKnownPosition(explorationTarget.x(), explorationTarget.y());
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
        int[] offset = directionOffset(direction);
        int nextX = internalMap.getAgentX() + offset[0];
        int nextY = internalMap.getAgentY() + offset[1];
        InternalMap.Position nextPosition = new InternalMap.Position(nextX, nextY);

        if (!isCurrentGroupLeader()
                && goalPosition != null
                && nextPosition.equals(goalPosition)
                && internalMap.isReservedAssemblyPosition(goalPosition)) {
            return true;
        }

        return blockedPositionsForMovement().contains(nextPosition)
            || occupiedPositionsForMovement().contains(nextPosition);
    }

    private InternalMap.Observation findNearestGoalZone() {
        int agentX = internalMap.getAgentX();
        int agentY = internalMap.getAgentY();

        return internalMap.getObservations().stream()
                .filter(observation -> observation.type().equals("goalZone"))
                .min((first, second) -> Integer.compare(
                        distanceTo(first.x(), first.y(), agentX, agentY),
                        distanceTo(second.x(), second.y(), agentX, agentY)))
                .orElse(null);
    }

            private InternalMap.Observation findNearestRoleZone() {
            int agentX = internalMap.getAgentX();
            int agentY = internalMap.getAgentY();

            return internalMap.getObservations().stream()
                .filter(observation -> observation.type().equals("roleZone"))
                .min((first, second) -> Integer.compare(
                    distanceTo(first.x(), first.y(), agentX, agentY),
                    distanceTo(second.x(), second.y(), agentX, agentY)))
                .orElse(null);
            }

            private InternalMap.Observation findNearestAvailableRoleZone() {
            int agentX = internalMap.getAgentX();
            int agentY = internalMap.getAgentY();

            return internalMap.getObservations().stream()
                .filter(observation -> observation.type().equals("roleZone"))
                .filter(observation -> !isOccupiedByAnotherAgent(observation.x(), observation.y()))
                .min((first, second) -> Integer.compare(
                    distanceTo(first.x(), first.y(), agentX, agentY),
                    distanceTo(second.x(), second.y(), agentX, agentY)))
                .orElse(null);
            }

            private boolean isOccupiedByAnotherAgent(int x, int y) {
            return (x != internalMap.getAgentX() || y != internalMap.getAgentY())
                && occupiedPositionsForMovement().contains(new InternalMap.Position(x, y));
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

        /** Replans movement after every perception cycle using the current map. */
        private void replanMovementIntention() {
            if (currentIntention == null || currentIntention.finished()) {
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
                        List<String> path = pathPlanner.findPath(
                                currentPosition(), explorationTarget,
                            blockedPositionsForMovement(),
                            occupiedPositionsForMovement());
                        currentIntention = nextMoveIsBlocked(path)
                                ? new Intention(Desire.CLEAR_OBSTACLE, List.of(path.get(0)), 0)
                                : new Intention(Desire.EXPLORE, path, 0);
                    }
                }
                default -> {
                    // State-changing actions must finish before replanning.
                }
            }
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
     * This method does not make decisions. It only translates the
     * intention into an action.
     */
    private Action executeIntention() {
        if (currentIntention == null || currentIntention.finished()) {
            return skip();
        }
        if (currentIntention.desire() == Desire.WAIT) {
            return skip();
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
        if (currentIntention.desire() == Desire.RETRIEVE_BLOCK) {
            return executeRetrieveBlock();
        }
        return executeMove();
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
            if (getName().equals(leaderName)) {
                blockX = pendingAssemblyConnection.leaderBlockX();
                blockY = pendingAssemblyConnection.leaderBlockY();
            } else if (carriedBlockPosition != null) {
                blockX = carriedBlockPosition.x() - internalMap.getAgentX();
                blockY = carriedBlockPosition.y() - internalMap.getAgentY();
            } else {
                return skip();
            }
            System.out.println(getName() + " sends connect(" + serverAgentName(pendingAssemblyConnection.partner())
                    + ", " + blockX + ", " + blockY + ") for target ("
                    + pendingAssemblyConnection.targetPosition().x() + ", "
                    + pendingAssemblyConnection.targetPosition().y() + ")");
                if (getName().equals(leaderName)) {
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
            if ("attach".equals(action) && !isVisibleRequestedBlockAt(direction)) {
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
            pendingDirection = direction;
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

    private boolean rotationPossible(String rotation) {
        if (retrieveBlockDirection == null) {
            return false;
        }
        String rotatedDirection = rotateDirection(
                retrieveBlockDirection, "cw".equals(rotation));
        int[] offset = directionOffset(rotatedDirection);
        InternalMap.Position rotatedBlockPosition = new InternalMap.Position(
                internalMap.getAgentX() + offset[0], internalMap.getAgentY() + offset[1]);
        return !internalMap.getPhysicalBlockedPositions().contains(rotatedBlockPosition);
    }

    private void rememberFailedRotationTarget() {
        if (pendingDirection != null) {
            rememberFailedRotationTarget(pendingDirection);
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

    // ============================================================
    // UTILITY
    // ============================================================

    private int[] directionOffset(String direction) {
        return switch (direction) {
            case "n" -> new int[]{0, -1};
            case "e" -> new int[]{1, 0};
            case "s" -> new int[]{0, 1};
            case "w" -> new int[]{-1, 0};
            default -> throw new IllegalArgumentException("Invalid direction: " + direction);
        };
    }

    private void printGroupState() {
        StringBuilder knownStates = new StringBuilder();
        for (String agent : knownAgents.keySet()) {
            if (knownStates.length() > 0) {
                knownStates.append(", ");
            }
            knownStates.append(agent)
                    .append("=")
                    .append(Boolean.TRUE.equals(knownAgentGroupState.get(agent)) ? "grouped" : "free")
                    .append("/")
                    .append(knownAgentGroupLeader.getOrDefault(agent, "-"));
        }

        System.out.println(getName() + " GROUP STATE: task=" + currentTask
                + ", deadline=" + taskDeadline
                + ", active=" + isTaskActive()
                + ", groupTask=" + groupTaskName
                + ", groupLeader=" + currentGroupLeader
                + ", leaderMode=" + groupLeaderMode
                + ", formationActive=" + groupFormationActive
                + ", members=" + currentGroupMembers
                + ", size=" + currentGroupMembers.size() + "/" + desiredGroupSize
                + ", inviteTarget=" + currentGroupInviteTarget
                + ", pendingInvites=" + pendingGroupInvitations
                + ", retryRequested=" + groupInviteRetryRequested);
        System.out.println(getName() + " KNOWN GROUP STATES: [" + knownStates + "]");
    }

    // ============================================================
    // BDI CYCLE
    // ============================================================

    @Override
    public Action step() {
        // --------------------------------------------------------
        // 1. Perception
        // --------------------------------------------------------

        List<Percept> percepts = getPercepts();

        if (!isNewActionCycle(percepts)) {
            return null;
        }

        System.out.println(getName() + " - Step: " + currentStep + ", Leader: " + leaderName+ ", Position: (" + internalMap.getAgentX() + ", " + internalMap.getAgentY() + ")");System.out.println((""+ " has goal zone: ") + hasObservation("goalZone") + ", has all required dispensers: " + hasAllRequiredDispensers() + ", is at goal zone: " + isAtGoalZone());
        
        //printKnownAgentRelations();
        System.out.println(goalPosition);
        System.out.println(currentIntention);
        //printDispenserContents();
        

        // --------------------------------------------------------
        // 2. Update beliefs / world model
        // --------------------------------------------------------

        updateAgentPosition(percepts);
        updateBeliefs(percepts);
        internalMap.updateFromPercepts(percepts, teamName);
        updateCurrentVisibleThings(percepts);
        refreshGroupGoalLocation();
        processTeammateRequests();
        exchangeTeammateNames();

        // --------------------------------------------------------
        // 3. Process previous intention
        // --------------------------------------------------------

        updateIntentionAfterAction(percepts);
        verifyCarriedBlock(percepts);

        if (!explorationFinished && allExplorationRequirementsKnown()) {
            explorationFinished = true;
            if (currentIntention != null && currentIntention.desire() == Desire.EXPLORE) {
                currentIntention = null;
                explorationTarget = null;
            }
        }

        updateGroupState();
        printGroupState();

        if (shouldInterruptForRole()) {
            currentIntention = null;
            explorationTarget = null;
        }

        replanMovementIntention();

        // --------------------------------------------------------
        // 4. BDI decision cycle
        // --------------------------------------------------------

        if (currentIntention == null || currentIntention.finished()) {
            Set<Desire> desires = generateDesires();
            currentIntention = selectIntention(desires);
        }

        exchangeMapUpdates(percepts);

        // --------------------------------------------------------
        // 5. Execute intention
        // --------------------------------------------------------

        return executeIntention();
    }
}