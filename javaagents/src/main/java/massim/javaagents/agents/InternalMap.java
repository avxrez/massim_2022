package massim.javaagents.agents;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class InternalMap {

	/*
	 * Position des Agenten auf der internen Karte.
	 */
	private int agentX = 0;
	private int agentY = 0;

	/*
	 * Alle aktuell bekannten Beobachtungen.
	 *
	 * Der Key stellt sicher, dass wir pro Position
	 * nicht mehrere alte Informationen behalten.
	 */
	private final Map<ObservationKey, Observation> observations = new HashMap<>();
	private final Set<Position> occupiedEntityPositions = new HashSet<>();


	// -------------------------------------------------------------------------
	// Data classes
	// -------------------------------------------------------------------------

	public record Observation(String type, int x, int y, String details, int lastSeenStep) {

    }

	public record Position(int x, int y) {

    }

	private record ObservationKey(String type, int x, int y, String details ) {

    }


	// -------------------------------------------------------------------------
	// Agent position
	// -------------------------------------------------------------------------

	public int getAgentX() {
		return agentX;
	}

	public int getAgentY() {
		return agentY;
	}

	/**
	 * Aktualisiert die Position des Agenten nach einer erfolgreichen Bewegung.
	 */
	public void updateAgentPosition(String direction) {
        switch (direction) {
            case "n" -> agentY--;
		    case "s" -> agentY++;
		    case "e" -> agentX++;
		    case "w" -> agentX--;
		    default -> throw new IllegalArgumentException("Invalid direction: " + direction);
        }
	}


	// -------------------------------------------------------------------------
	// Remember observations
	// -------------------------------------------------------------------------

	/**
	 * Speichert eine beobachtete Zelle.
	 *
	 * Die übergebenen Koordinaten sind relativ zum Agenten.
	 * Intern werden sie in absolute Koordinaten umgerechnet.
	 */
	public void rememberObservation(String type, int relativeX, int relativeY, String details, int step) {
		int absoluteX = agentX + relativeX;
		int absoluteY = agentY + relativeY;

		String observationDetails =details == null ? "" : details;
        removeObservationsForUpdate(absoluteX, absoluteY, type);

		ObservationKey key = new ObservationKey(type, absoluteX, absoluteY, observationDetails);

		observations.put(key, new Observation(type, absoluteX, absoluteY, observationDetails, step));
	}

	/**
	 * Speichert eine bekannte freie Zelle.
	 */
	public void rememberFreeCell(int relativeX, int relativeY, int step) {
        rememberObservation("free", relativeX, relativeY, "", step);
	}

	public void clearOccupiedEntityPositions() {
		occupiedEntityPositions.clear();
	}

	public void rememberOccupiedEntity(int relativeX, int relativeY) {
		occupiedEntityPositions.add(new Position(agentX + relativeX, agentY + relativeY));
	}


	// -------------------------------------------------------------------------
	// Query map
	// -------------------------------------------------------------------------

	/**
	 * Gibt alle aktuell bekannten Beobachtungen zurück.
	 */
	public List<Observation> getObservations() {
		return Collections.unmodifiableList(new ArrayList<>(observations.values()));
	}

	/**
	 * Fügt Beobachtungen einer bereits bekannten Karte mit absoluten
	 * Koordinaten in diese Karte ein.
	 */
	public void mergeObservations(List<Observation> observationsToMerge) {
		for (Observation observation : observationsToMerge) {
			removeObservationsForUpdate(observation.x(), observation.y(), observation.type());

			String details = observation.details() == null ? "" : observation.details();
			ObservationKey key = new ObservationKey(observation.type(), observation.x(), observation.y(), details);
			observations.put(key, new Observation(
					observation.type(), observation.x(), observation.y(), details, observation.lastSeenStep()));
		}
	}

	/**
	 * Gibt alle Positionen zurück, die aktuell als blockiert gelten.
	 *
	 * obstacle:
	 *     tatsächlich beobachtetes Hindernis
	 *
	 * failedPath:
	 *     Position, die beim letzten Versuch nicht betreten
	 *     werden konnte
	 */
	public List<Position> getBlockedPositions() {
		Set<Position> blockedPositions = new HashSet<>(occupiedEntityPositions);
		observations.values().stream()
				.filter(observation -> observation.type().equals("obstacle")
						|| observation.type().equals("failedPath"))
				.map(observation -> new Position(observation.x(), observation.y()))
				.forEach(blockedPositions::add);
		return List.copyOf(blockedPositions);
	}

	/**
	 * Prüft, ob eine absolute Position bereits bekannt ist.
	 */
	public boolean isKnownPosition(int x, int y) {
		return observations.values().stream().anyMatch(observation -> observation.x() == x && observation.y() == y);
	}


	// -------------------------------------------------------------------------
	// Failed paths
	// -------------------------------------------------------------------------

	/**
	 * Merkt sich eine Position, die nicht betreten werden konnte.
	 */
	public void rememberFailedPath(int x, int y, int step) {
		removeBlockedObservationsAt(x, y);
        ObservationKey key =new ObservationKey("failedPath",x,y,"");
		observations.put(key, new Observation("failedPath",x,y,"",step));
	}


	// -------------------------------------------------------------------------
	// Remove / update information
	// -------------------------------------------------------------------------

	/**
	 * Entfernt alle bisherigen Informationen über eine Position
	 * und markiert die Position anschließend als frei.
	 *
	 * Wird beispielsweise nach erfolgreichem Clear verwendet.
	 */
	public void forgetObservationsAt(int x, int y) {
		removeBlockedObservationsAt(x, y);
		rememberFreeCell(x - agentX, y - agentY, 0 );
	}

	/**
	 * Entfernt alle Beobachtungen an einer absoluten Position.
	 */
	private void removeObservationsAt(int x, int y) {
		observations.keySet().removeIf( key -> key.x() == x && key.y() == y);
	}

	private void removeBlockedObservationsAt(int x, int y) {
		observations.keySet().removeIf(key ->
				key.x() == x && key.y() == y && isBlocked(key.type()));
	}

	private void removeObservationsForUpdate(int x, int y, String newType) {
		observations.keySet().removeIf(key ->
				key.x() == x
						&& key.y() == y
						&& !mustKeepTogether(key.type(), newType));
	}

	private boolean mustKeepTogether(String firstType, String secondType) {
		return isZone(firstType) || isZone(secondType);
	}

	private boolean isZone(String type) {
		return type.equals("goalZone") || type.equals("roleZone");
	}

	private boolean isBlocked(String type) {
		return type.equals("obstacle") || type.equals("failedPath");
	}

	/**
 * Verschiebt die gesamte interne Karte um einen Offset.
 *
 * Alle bekannten Koordinaten und auch die eigene
 * Agentenposition werden entsprechend verschoben.
 */
	public void translate(int offsetX, int offsetY) {
	
	    // Agentenposition verschieben
	    agentX += offsetX;
	    agentY += offsetY;
	
	    // Beobachtungen verschieben
	    Map<ObservationKey, Observation> translatedObservations = new HashMap<>();
	
	    for (Observation observation : observations.values()) {
		
	        int newX = observation.x() + offsetX;
	        int newY = observation.y() + offsetY;
		
	        Observation translated = new Observation(
	                observation.type(),
	                newX,
	                newY,
	                observation.details(),
	                observation.lastSeenStep()
	        );
		
	        ObservationKey key = new ObservationKey(
	                translated.type(),
	                newX,
	                newY,
	                translated.details()
	        );
		
	        translatedObservations.put(key, translated);
	    }
	
	    observations.clear();
	    observations.putAll(translatedObservations);
	
	    // Besetzte Entity-Positionen ebenfalls verschieben
	    Set<Position> translatedEntities = new HashSet<>();
	
	    for (Position position : occupiedEntityPositions) {
		
	        translatedEntities.add(
	                new Position(
	                        position.x() + offsetX,
	                        position.y() + offsetY
	                )
	        );
	    }
	
	    occupiedEntityPositions.clear();
	    occupiedEntityPositions.addAll(translatedEntities);
	}
}


