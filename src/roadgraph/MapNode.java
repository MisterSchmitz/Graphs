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
public class MapNode implements Comparable<MapNode> {
	private GeographicPoint location;
	private HashSet<MapEdge> edges;
	private double searchDistance;	// used for comparing distance in a search

	public MapNode(GeographicPoint loc) {
		this.setLocation(loc);
		this.edges = new HashSet<MapEdge>();
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
	

	public String toString() {
		String s = "Node: " + this.getLocation();
		s += ", searchDistance: "+this.getSearchDistance();
		return s;
	}
	
	public int compareTo(MapNode other) {
		if(this.searchDistance < other.searchDistance) return -1;
		if(this.searchDistance > other.searchDistance) return 1;
		return 0;
	}
}