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
	
}
