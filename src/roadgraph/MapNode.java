/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import geography.GeographicPoint;


/**
 * @author Sridhar Anumandla
 * 
 * A class to represent each node in a graph
 *  
 */
public class MapNode implements Comparable<MapNode> {
	
	private GeographicPoint location;
	private Set<MapEdge> edges;	
	
	private double originalDistance;
	
	private double predictedDistace;
	
	public MapNode(GeographicPoint location) {
		this.location = location;
		edges = new HashSet<>();
		originalDistance = Double.POSITIVE_INFINITY;
		predictedDistace = Double.POSITIVE_INFINITY;			
	}
	
	public GeographicPoint getLocation() {
		return location;
	}
	
	public Set<MapNode> getNeighbors() {
		Set<MapNode> neighbors = new HashSet<>();
		
		Set<MapEdge> edges = getEdges();
		for (MapEdge edge : edges) {
			neighbors.add(edge.getOtherNode(this));
		}
		
		return neighbors;
	}
	
	public Set<MapEdge> getEdges() {
		return edges;
	}
	
	public void addEdge(MapEdge edge) {
		edges.add(edge);		
	}

	public double getOriginalDistance() {
		return originalDistance;
	}

	public void setOriginalDistance(double originalDistance) {
		this.originalDistance = originalDistance;
	}

	public double getPredictedDistace() {
		return predictedDistace;
	}

	public void setPredictedDistance(double predictedDistace) {
		this.predictedDistace = predictedDistace;
	}
	
	public double getEdgeLength(MapNode to) {
		Set<MapEdge> edges = getEdges();
		for(MapEdge edge : edges) {
			if (edge.getFrom().equals(this) && edge.getTo().equals(to)) {
				return edge.getLength();
			}
		}
		
		return Double.POSITIVE_INFINITY;
	}
	
	@Override
	public boolean equals(Object o) {
		if (!(o instanceof MapNode) || (o == null)) {
			return false;
		}
		
		MapNode other = (MapNode)o;
		return other.getLocation().equals(this.location);		
	}
	
	/** Because we compare nodes using their location, we also 
	 * may use their location for HashCode.
	 * @return The HashCode for this node, which is the HashCode for the 
	 * underlying point
	 */
	@Override
	public int hashCode() {
		return location.hashCode();
	}
	
	// This is for Dijkstra
//	@Override
//	public int compareTo(MapNode o) {
//		return ((Double) this.originalDistance).compareTo(o.getOriginalDistance());		
//	}

	// This is for AStarSearch
	@Override
	public int compareTo(MapNode o) {
		return ((Double) (this.originalDistance + this.predictedDistace))
				.compareTo(o.getOriginalDistance() + o.getPredictedDistace());		
	}
		
}
