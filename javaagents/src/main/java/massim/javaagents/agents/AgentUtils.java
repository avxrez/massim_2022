package massim.javaagents.agents;

import java.util.List;

public final class AgentUtils {

    public static final List<String> CARDINAL_DIRECTIONS = List.of("n", "e", "s", "w");

    private AgentUtils() {
        // utility class
    }

    public static int[] directionOffset(String direction) {
        return switch (direction) {
            case "n" -> new int[]{0, -1};
            case "e" -> new int[]{1, 0};
            case "s" -> new int[]{0, 1};
            case "w" -> new int[]{-1, 0};
            default -> throw new IllegalArgumentException("Invalid direction: " + direction);
        };
    }

    public static String rotateDirection(String direction, boolean clockwise) {
        return switch (direction) {
            case "n" -> clockwise ? "e" : "w";
            case "e" -> clockwise ? "s" : "n";
            case "s" -> clockwise ? "w" : "e";
            case "w" -> clockwise ? "n" : "s";
            default -> throw new IllegalArgumentException("Invalid direction: " + direction);
        };
    }

    public static String oppositeDirection(String direction) {
        return switch (direction) {
            case "n" -> "s";
            case "e" -> "w";
            case "s" -> "n";
            case "w" -> "e";
            default -> throw new IllegalArgumentException("Invalid direction: " + direction);
        };
    }

    public static boolean isMovementDirection(String direction) {
        return "n".equals(direction) || "e".equals(direction)
                || "s".equals(direction) || "w".equals(direction);
    }

    public static int manhattanDistance(InternalMap.Position first, InternalMap.Position second) {
        return manhattanDistance(first.x(), first.y(), second.x(), second.y());
    }

    public static int manhattanDistance(int firstX, int firstY, int secondX, int secondY) {
        return Math.abs(firstX - secondX) + Math.abs(firstY - secondY);
    }

    public static String directionFrom(InternalMap.Position from, InternalMap.Position to) {
        if (to.x() > from.x()) return "e";
        if (to.x() < from.x()) return "w";
        if (to.y() > from.y()) return "s";
        return "n";
    }

    public static int nameNumber(String name) {
        String number = name == null ? "" : name.replaceAll("[^0-9]", "");
        return number.isEmpty() ? -1 : Integer.parseInt(number);
    }
}
