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
        ADAPT_ROLE
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
    private static final String DEFAULT_ROLE = "default";
    private static final String WORKER_ROLE = "worker";

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
    private int currentTaskBlockCount = 1;
    private int desiredGroupSize = 1;
    private String currentGroupLeader = "";
        private boolean groupLeaderMode = false;
    private boolean groupFormationActive = false;
    private String groupTaskName = "";
    private String deliveryBlockType = null;
    private InternalMap.Position requiredBlockOffset;

    private final Set<String> requiredDispenserTypes = new HashSet<>();
    private final List<String> taskBlockTypes = new ArrayList<>();
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
    private boolean attachmentCheckPending;
    private boolean explorationFinished;
    private Intention currentIntention;
    private String pendingAction;
    private String pendingDirection;
    private String clearDirection;


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
            System.out.println("Received new leader message from " + sender + " to switch from "
                    + previousLeader.getValue() + " to " + newLeader.getValue());
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
                && !groupFormationActive
                && !knownAgentGroupState.getOrDefault(getName(), false)) {
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
            deliveryBlockType = blockType.getValue();
            goalPosition = new InternalMap.Position(targetX.getValue().intValue(), targetY.getValue().intValue());
            prepareForBlockAssignment();
            System.out.println(getName() + " received custom group block task: fetch " + deliveryBlockType
                    + " and deliver it to absolute target (" + goalPosition.x() + ", " + goalPosition.y() + ")");
            currentIntention = null;
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
            currentTask = null;
            taskDeadline = -1;
            currentTaskBlockCount = 1;
            desiredGroupSize = 1;
            requiredBlockOffset = null;
            deliveryBlockType = null;
            goalPosition = null;
            currentIntention = null;
            resetCarriedBlockTracking();
        } else if (!Objects.equals(currentTask, previousTask)) {
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
        for (String agent : knownAgents.keySet()) {
            if (agent.equals(getName())) {
                continue;
            }
            if(currentStep % 10 == 0){
                content = currentMapPercepts(percepts);

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
                        knownAgentsParameters(),
                        new Identifier(Boolean.TRUE.equals(knownAgentGroupState.get(getName()))
                            ? "grouped" : "free"),
                        new Identifier(knownAgentGroupLeader.getOrDefault(getName(), ""))), agent, getName());
        }
    }

    private void sendMergedMapUpdates() {
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
                    mapParameters(),
                    new Identifier(leaderName),
                        knownAgentsParameters(),
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
                if (currentTaskBlockCount == 0) {
                    requiredBlockOffset = new InternalMap.Position(
                            requiredX.getValue().intValue(), requiredY.getValue().intValue());
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
        return Math.max(1, currentTaskBlockCount);
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
        if (!leaderName.equals(getName())
                || groupFormationActive
                || !explorationFinished
                || currentTask == null
                || currentTask.isEmpty()
                || !isTaskActive()) {
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
        currentGroupLeader = getName();
        currentGroupMembers.clear();
        currentGroupMembers.add(getName());
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

        for (String agent : knownAgents.keySet()) {
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
                + " (known agents: " + knownAgents.keySet() + ")");
    }

    private void recruitNextGroupLeader() {
        if (!groupLeaderMode || !groupFormationActive || currentGroupMembers.size() < desiredGroupSize) {
            return;
        }

        assignBlocksToCurrentGroup();

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
            knownAgentGroupState.put(agent, true);
            knownAgentGroupLeader.put(agent, agent);
            sendMessage(new Percept("groupStart",
                    new Identifier(groupTaskName),
                    new Numeral(desiredGroupSize)), agent, getName());
                    groupFormationActive = false;
                    groupLeaderMode = false;
                    currentGroupLeader = "";
            return;
        }
        System.out.println(getName() + " has no free known agent left for a new group.");
    }

    private void assignBlocksToCurrentGroup() {
        List<String> members = new ArrayList<>(currentGroupMembers);
        members.sort(String::compareTo);

        InternalMap.Observation goalZone = findNearestGoalZone();

        for (int index = 0; index < members.size() && index < taskBlockTypes.size(); index++) {
            String member = members.get(index);
            String blockType = taskBlockTypes.get(index);
            sendMessage(new Percept("groupBlockTask",
                    new Identifier(blockType),
                new Numeral(goalZone.x()),
                new Numeral(goalZone.y())), member, getName());
            System.out.println(getName() + " assigns " + blockType + " to " + member
                + " with delivery target (" + goalZone.x() + ", " + goalZone.y() + ")");
        }
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
        deliveryBlockType = null;
        goalPosition = null;
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

        if (leaderName.equals(getName())
                && !groupFormationActive
                && explorationFinished
                && currentTask != null
            && !currentTask.isEmpty()
            && deliveryBlockType == null
            && !Boolean.TRUE.equals(knownAgentGroupState.get(getName()))
            && isTaskActive()) {
            startGroupFormation();
        }

        if (groupLeaderMode && groupFormationActive && groupInviteRetryRequested
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

            if ("clear".equals(lastAction) && clearDirection != null) {
                int[] offset = directionOffset(clearDirection);
                internalMap.forgetObservationsAt(
                        internalMap.getAgentX() + offset[0], internalMap.getAgentY() + offset[1]);
            }
            if ("request".equals(lastAction)) {
                blockRequested = true;
            } else if ("attach".equals(lastAction)) {
                blockRetrieved = true;
                carriedBlockType = deliveryBlockType;
                attachmentCheckPending = true;
            } else if ("detach".equals(lastAction)) {
                resetCarriedBlockTracking();
            } else if ("submit".equals(lastAction)) {
                if (currentTaskBlockCount == 1 && isTaskActive()) {
                    resetCarriedBlockTracking();
                    currentIntention = null;
                } else {
                    blockPlaced = true;
                }
            }
        } else {
            if ("clear".equals(lastAction)
                    && blockRetrieved
                    && currentIntention.desire() == Desire.RETRIEVE_BLOCK
                    && currentIntention.nextAction() + 1 < currentIntention.plan().size()
                    && currentIntention.plan().get(currentIntention.nextAction() + 1).startsWith("rotate:")) {
                currentIntention = currentIntention.advance();
            } else if ("attach".equals(lastAction) && retrieveBlockDirection != null) {
                currentIntention = createRetrieveBlockIntention();
            } else if ("rotate".equals(lastAction) && retrieveBlockDirection != null) {
                rememberFailedRotationTarget();
                currentIntention = createRetrieveBlockIntention();
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
        clearDirection = null;
    }

    private void verifyCarriedBlock(List<Percept> percepts) {
        if (!blockRetrieved || retrieveBlockDirection == null) {
            return;
        }
        attachmentCheckPending = false;

        InternalMap.Position attachedPosition = findAttachedBlockPosition(percepts);
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

    private InternalMap.Position findAttachedBlockPosition(List<Percept> percepts) {
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
        if (DEFAULT_ROLE.equals(currentRole)) {
            if (isAtRoleZone()) {
                desires.add(Desire.ADAPT_ROLE);
                return desires;
            }
            InternalMap.Observation roleZone = findNearestRoleZone();
            if (roleZone != null
                    && (explorationFinished
                        || distanceTo(roleZone.x(), roleZone.y(), internalMap.getAgentX(), internalMap.getAgentY())
                            <= ROLE_ZONE_MAX_DISTANCE)) {
                desires.add(Desire.REACH_ROLE_ZONE);
                return desires;
            }
        }

        if (!currentRole.isEmpty() && !DEFAULT_ROLE.equals(currentRole)
            && isTaskActive()
            && goalPosition != null && deliveryBlockType != null) {
            if (!blockPlaced) {
                desires.add(Desire.RETRIEVE_BLOCK);
                return desires;
            }
            desires.add(isAtGoalZone() ? Desire.WAIT : Desire.REACH_GOAL_ZONE);
            return desires;
        }

        if (currentTask != null && isTaskActive() && explorationFinished
                && Boolean.TRUE.equals(knownAgentGroupState.get(getName()))) {
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
        InternalMap.Observation roleZone = findNearestRoleZone();
        if (roleZone == null) {
            return null;
        }

        List<String> path = pathPlanner.findPath(
                currentPosition(),
                new InternalMap.Position(roleZone.x(), roleZone.y()),
            internalMap.getBlockedPositions(),
            internalMap.getOccupiedEntityPositions());
        if (nextMoveIsBlocked(path)) {
            return new Intention(Desire.CLEAR_OBSTACLE, List.of(path.get(0)), 0);
        }
        return new Intention(Desire.REACH_ROLE_ZONE, path, 0);
    }

    /**
     * Creates an intention for reaching the goal.
     */
    private Intention createGoalIntention() {
        InternalMap.Observation goal = findNearestGoalZone();
        InternalMap.Position start = currentPosition();
        InternalMap.Position target = new InternalMap.Position(goal.x(), goal.y());

        List<String> path = pathPlanner.findPath(start, target,
            internalMap.getBlockedPositions(), internalMap.getOccupiedEntityPositions());
        if (nextMoveIsBlocked(path)) {
            return new Intention(Desire.CLEAR_OBSTACLE, List.of(path.get(0)), 0);
        }
        return new Intention(Desire.REACH_GOAL_ZONE, path, 0);
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
                    retrieveBlockDirection, requiredDirection, internalMap.getBlockedPositions(),
                    internalMap.getOccupiedEntityPositions());
            if (path.isEmpty()) {
                return new Intention(Desire.WAIT, List.of(), 0);
            }
            return new Intention(Desire.RETRIEVE_BLOCK, path, 0);
        }

        if (deliveryTarget != null && carriedBlockPosition != null && carriedBlockPosition.equals(deliveryTarget)) {
            blockPlaced = true;
            return new Intention(Desire.WAIT, List.of(), 0);
        }
        int[] blockOffset = directionOffset(retrieveBlockDirection);
        InternalMap.Position targetPosition = new InternalMap.Position(
            deliveryTarget.x() - blockOffset[0], deliveryTarget.y() - blockOffset[1]);
        List<String> path = pathPlanner.findCarryingPath(currentPosition(), targetPosition,
            retrieveBlockDirection, internalMap.getBlockedPositions(),
            internalMap.getOccupiedEntityPositions());
        if (path.isEmpty()) {
            return new Intention(Desire.WAIT, List.of(), 0);
        }
        return new Intention(Desire.RETRIEVE_BLOCK, path, 0);
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
                    internalMap.getBlockedPositions(), internalMap.getOccupiedEntityPositions());
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
            internalMap.getBlockedPositions(), internalMap.getOccupiedEntityPositions());
        if (nextMoveIsBlocked(path)) {
            return new Intention(Desire.CLEAR_OBSTACLE, List.of(path.get(0)), 0);
        }
        return new Intention(Desire.EXPLORE, path, 0);
    }

    private InternalMap.Position currentPosition() {
        return new InternalMap.Position(internalMap.getAgentX(), internalMap.getAgentY());
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

        return internalMap.getBlockedPositions().contains(nextPosition);
    }

    private InternalMap.Observation findNearestGoalZone() {
        int agentX = internalMap.getAgentX();
        int agentY = internalMap.getAgentY();

        return internalMap.getObservations().stream()
                .filter(observation -> observation.type().equals("goalZone"))
                .min((first, second) -> Integer.compare(
                        distanceTo(first.x(), first.y(), agentX, agentY),
                        distanceTo(second.x(), second.y(), agentX, agentY)))
                .orElseThrow();
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

                InternalMap.Observation roleZone = findNearestRoleZone();
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
                    if (blockRequested && !blockRetrieved) {
                        currentIntention = createRetrieveBlockIntention();
                    } else if (blockRetrieved) {
                        currentIntention = createRetrieveBlockIntention();
                    }
                }
                case EXPLORE -> {
                    if (explorationTarget != null) {
                        List<String> path = pathPlanner.findPath(
                                currentPosition(), explorationTarget,
                            internalMap.getBlockedPositions(),
                            internalMap.getOccupiedEntityPositions());
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
        if (currentIntention.desire() == Desire.RETRIEVE_BLOCK) {
            return executeRetrieveBlock();
        }
        return executeMove();
    }

    private Action executeAdapt() {
        pendingAction = "adapt";
        return new Action("adapt", new Identifier(currentIntention.plan().get(0)));
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
        return !internalMap.getBlockedPositions().contains(rotatedBlockPosition);
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
        //System.out.println(goalPosition);
        System.out.println(currentIntention);
        //printDispenserContents();
        

        // --------------------------------------------------------
        // 2. Update beliefs / world model
        // --------------------------------------------------------

        updateAgentPosition(percepts);
        updateBeliefs(percepts);
        internalMap.updateFromPercepts(percepts, teamName);
        updateCurrentVisibleThings(percepts);
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