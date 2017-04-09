/**
 * 
 */
package roadgraph;

import geography.GeographicPoint;

/**
 * @author mschmitz
 *
 */
public class MapEdge {
	private GeographicPoint locationStart;
	private GeographicPoint locationEnd;
	private MapNode nodeStart;
	private MapNode nodeEnd;
	private String roadName;
	private String roadType;
	private double distance;
	
	public MapEdge(GeographicPoint start, GeographicPoint end, double dist) {
		this.setLocationStart(start);
		this.setLocationEnd(end);
		this.setDistance(dist);
	}

	public MapEdge(GeographicPoint start, GeographicPoint end, String name, String type, double dist) {
		this.setLocationStart(start);
		this.setLocationEnd(end);
		this.setRoadName(name);
		this.setRoadType(type);
		this.setDistance(dist);
	}
	
	public GeographicPoint getLocationStart() {
		return this.locationStart;
	}
	
	public void setLocationStart(GeographicPoint p) {
		this.locationStart = p;
	}
	
	public GeographicPoint getLocationEnd() {
		return this.locationEnd;
	}
	
	public void setLocationEnd(GeographicPoint p) {
		this.locationEnd = p;
	}
	
	public String getRoadName() {
		return this.roadName;
	}
	
	public void setRoadName(String s) {
		this.roadName = s;
	}
	
	public String getRoadType() {
		return this.roadType;
	}
	
	public void setRoadType(String s) {
		this.roadType = s;
	}

	public double getDistance() {
		return this.distance;
	}
	
	public void setDistance(double d) {
		this.distance = d;
	}
	
}