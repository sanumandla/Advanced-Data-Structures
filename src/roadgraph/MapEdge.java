/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import geography.GeographicPoint;

/**
 * @author Sridhar Anumandla
 * 
 * A class which represents an edge between two geographic points along with 
 * roadName, roadType and length
 *  
 */
public class MapEdge {
	
	private double length;
	private String roadName;
	private String roadType;
	private MapNode from;
	private MapNode to;	
	
	public MapEdge(MapNode from, MapNode to, String roadName, String roadType, double length)
	{
		this.setFrom(from);
		this.setTo(to);
		this.setRoadName(roadName);
		this.setRoadType(roadType);
		this.setLength(length);
	}

	public double getLength() {
		return length;
	}

	public void setLength(double length) {
		this.length = length;
	}

	public String getRoadName() {
		return roadName;
	}

	public void setRoadName(String roadName) {
		this.roadName = roadName;
	}

	public String getRoadType() {
		return roadType;
	}

	public void setRoadType(String roadType) {
		this.roadType = roadType;
	}

	public MapNode getFrom() {
		return from;
	}

	public void setFrom(MapNode from) {
		this.from = from;
	}

	public MapNode getTo() {
		return to;
	}

	public void setTo(MapNode to) {
		this.to = to;
	}	
	
	public MapNode getOtherNode(MapNode original) {
		if (original.equals(to)) {
			return from;
		} else if (original.equals(from)) {
			return to;
		} else {
			throw new IllegalArgumentException("No nodes found connecting to " + original);
		}
	}
	
}
