/**
 * 
 */
package roadgraph;

import java.util.HashSet;

import geography.GeographicPoint;

/**
 * @author mschmitz
 *
 */
public class MapNode {
	private GeographicPoint location;
	private HashSet<MapEdge> edges;
	private double searchDistance;	// used for comparing distance in a search
	private double estimatedDistanceFromGoal;	// used for comparing distance in a A* search algorithm
	private double pathTime;	// used for comparing time in a search
	private double estimatedTimeFromGoal;	// used for comparing time in a A* search algorithm

	public MapNode(GeographicPoint loc) {
		this.setLocation(loc);
		this.edges = new HashSet<MapEdge>();
		this.estimatedDistanceFromGoal=0; // Initialize to 0 for compatibility with Dijkstra search algorithm
	}
	
	public GeographicPoint getLocation(){
		return this.location;
	}
	
	public void setLocation(GeographicPoint p) {
		this.location = p;
	}
	
	public HashSet<MapEdge> getEdges() {
		return edges;
	}
	
	public void addEdge(MapEdge e) {
		if(!edges.contains(e)) {
			edges.add(e);
		}
	}
	
	public double getSearchDistance() {
		return searchDistance;
	}
	
	public void setSearchDistance(double d) {
		this.searchDistance = d;
	}

	public double getPathTime() {
		return pathTime;
	}
	
	public void setPathTime(double d) {
		this.pathTime = d;
	}
	
	public double getEstimatedDistanceFromGoal() {
		return estimatedDistanceFromGoal;
	}
	
	public void setEstimatedDistanceFromGoal(double d) {
		this.estimatedDistanceFromGoal = d;
	}


	public double getEstimatedTimeFromGoal() {
		return estimatedTimeFromGoal;
	}
	
	public void setEstimatedTimeFromGoal(double d) {
		this.estimatedTimeFromGoal = d;
	}
	
	public String toString() {
		String s = "Node: " + this.getLocation();
		s += ", searchDistance: "+this.getSearchDistance();
		s += ", estimatedDistanceFromGoal: "+this.getEstimatedDistanceFromGoal();
		return s;
	}
}