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
	private HashSet<MapEdge> edgesOut;
	private HashSet<MapEdge> edgesIn;
	
	public MapNode(GeographicPoint loc) {
		this.setLocation(loc);
	}
	
	public GeographicPoint getLocation(){
		return this.location;
	}
	
	public void setLocation(GeographicPoint p) {
		this.location = p;
	}
	
	public HashSet<MapEdge> getEdgesOut() {
		return edgesOut;
	}
	
	public void addEdgeOut(MapEdge e) {
		if(!edgesOut.contains(e)) {
			edgesOut.add(e);
		}
	}
	
	public HashSet<MapEdge> getEdgesIn() {
		return edgesIn;
	}
	
	public void addEdgeIn(MapEdge e) {
		if(!edgesIn.contains(e)) {
			edgesIn.add(e);
		}
	}
}
