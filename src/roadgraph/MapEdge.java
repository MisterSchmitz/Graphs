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
	private String roadName;
	private String roadType;
	private double distance;
	private double roadSpeed;
	
	public MapEdge(GeographicPoint start, GeographicPoint end, double dist) {
		this.setLocationStart(start);
		this.setLocationEnd(end);
		this.setDistance(dist);
		this.setRoadSpeed(40);
	}

	public MapEdge(GeographicPoint start, GeographicPoint end, String name, String type, double dist) {
		this.setLocationStart(start);
		this.setLocationEnd(end);
		this.setRoadName(name);
		this.setRoadType(type);
		this.setDistance(dist);
		if (type.equals("motorway")) {
			this.setRoadSpeed(115);
		} else if (type.equals("primary")) {
			this.setRoadSpeed(90);
		} else if (type.equals("secondary")) {
			this.setRoadSpeed(60);
		} else {
			this.setRoadSpeed(40);
		}
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

	public double getRoadSpeed() {
		return this.roadSpeed;
	}
	
	public void setRoadSpeed(double d) {
		this.roadSpeed = d;
	}
	
	public double getDistance() {
		return this.distance;
	}
	
	public void setDistance(double d) {
		this.distance = d;
	}
	
	public double getRoadTime() {
		return this.distance / this.roadSpeed;
	}
	
	public String toString() {
		String s = "Edge: " + this.getRoadName();
		s += ", Start: "+this.getLocationStart();
		s += ", End: "+this.getLocationEnd();
		s += ", Distance: "+this.getDistance();
		s += ", Type: "+this.getRoadType();
		return s;
	}
}