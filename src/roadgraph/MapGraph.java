/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 *         A class which represents a graph of geographic locations Nodes in the
 *         graph are intersections between
 *
 */
public class MapGraph {

	int edges;
	Map<GeographicPoint, MapNode> map;
	
	public static int count = 0;

	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph() {
		map = new HashMap<>();
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * 
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		return map.values().size();
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * 
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		return map.keySet();
	}

	/**
	 * Get the number of road segments in the graph
	 * 
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		return edges;
	}

	/**
	 * Add a node corresponding to an intersection at a Geographic Point If the
	 * location is already in the graph or null, this method does not change the
	 * graph.
	 * 
	 * @param location
	 *            The location of the intersection
	 * @return true if a node was added, false if it was not (the node was
	 *         already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		if (location == null || map.containsKey(location)) {
			return false;
		}

		map.put(location, new MapNode(location));
		return true;
	}

	/**
	 * Adds a directed edge to the graph from pt1 to pt2. Precondition: Both
	 * GeographicPoints have already been added to the graph
	 * 
	 * @param from
	 *            The starting point of the edge
	 * @param to
	 *            The ending point of the edge
	 * @param roadName
	 *            The name of the road
	 * @param roadType
	 *            The type of the road
	 * @param length
	 *            The length of the road, in km
	 * @throws IllegalArgumentException
	 *             If the points have not already been added as nodes to the
	 *             graph, if any of the arguments is null, or if the length is
	 *             less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length)
			throws IllegalArgumentException {
		validateAddEdge(from, to, roadName, roadType, length);

		MapEdge edge = new MapEdge(map.get(from), map.get(to), roadName, roadType, length);
		map.get(from).addEdge(edge);
		++edges; // increment the counter on adding every edge
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return bfs(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		if ((start == null || !map.containsKey(start)) || (goal == null || !map.containsKey(goal))) {
			return path;
		}
		
		MapNode startNode = map.get(start);
		MapNode goalNode = map.get(goal);

		Map<MapNode, MapNode> parentNodeMap = new HashMap<>();
		boolean found = searchPathUsingBFS(startNode, goalNode, parentNodeMap, nodeSearched);
		if (!found) {
			System.out.println("No path exists");
			return path;
		}

		path = constructPath(startNode, goalNode, parentNodeMap);

		return path;
	}

	/**
	 * Search for a path from start to goal and keep track of the path
	 * 
	 * @param start
	 *            start node
	 * @param goal
	 *            end node
	 * @param parentNodeMap
	 *            mapping between parent and child nodes
	 * @return
	 */
	private boolean searchPathUsingBFS(MapNode start, MapNode goal, 
			Map<MapNode, MapNode> parentNodeMap, Consumer<GeographicPoint> nodeSearched) {
		boolean found = false;

		// Initialize
		LinkedList<MapNode> queue = new LinkedList<>();
		Set<MapNode> visited = new HashSet<>();

		// Add start node
		visited.add(start);
		queue.addLast(start);

		// BFS - for each node check if it's the goal
		// if not add the children to the queue
		// during this process make sure the node is not already visited
		while (!queue.isEmpty()) {
			MapNode current = queue.removeFirst();
			System.out.println("BFS - Visited node: " + current.getLocation());
			if (current.equals(goal)) {
				found = true;
				break;
			}

			// Hook for visualization. See writeup.
			nodeSearched.accept(current.getLocation());

			Set<MapEdge> edges = map.get(current.getLocation()).getEdges();
			if (edges != null) {
				for (MapEdge edge : edges) {
					MapNode child = edge.getTo();
					if (!visited.contains(child)) {
						visited.add(child);
						queue.addLast(child);
						parentNodeMap.put(child, current);
					}
				}
			}
		}

		return found;
	}

	/**
	 * Construct a path from start to goal node
	 * 
	 * @param parentNodeMap
	 *            collection with mapping between nodes to traverse the path
	 *            from start to goal
	 * @return path from start to goal
	 */
	private List<GeographicPoint> constructPath(MapNode start, MapNode goal,
			Map<MapNode, MapNode> parentNodeMap) {
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();

		// start from goal and traverse back by looking for its parents
		MapNode current = goal;
		while (current != start) {
			path.addFirst(current.getLocation());
			current = parentNodeMap.get(current);
		}

		// add the first node
		path.addFirst(start.getLocation());

		return path;
	}

	/**
	 * Print the map
	 * 
	 * @param map
	 *            with points and edges
	 */
	private void printMap() {
		Set<GeographicPoint> points = map.keySet();

		for (GeographicPoint point : points) {
			System.out.print(point + " ==>> ");

			Set<MapEdge> edges = map.get(point).getEdges();
			if (edges != null) {
				for (MapEdge edge : edges) {
					System.out.print(edge.getTo().getLocation() + " , ");
				}
			}

			System.out.println();
		}
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return dijkstra(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		count = 0;
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		if ((start == null || !map.containsKey(start)) || (goal == null || !map.containsKey(goal))) {
			return path;
		}

		MapNode startNode = map.get(start);
		MapNode goalNode = map.get(goal);
		
		Map<MapNode, MapNode> parentNodeMap = new HashMap<>();		
		boolean found = searchPathUsingDijkstra(startNode, goalNode, parentNodeMap, nodeSearched);
		if (!found) {
			System.out.println("No path exists");
			return path;
		}

		System.out.println("Number of nodes visited by Dijkstra: " + count);
		path = constructPath(startNode, goalNode, parentNodeMap);

		return path;
	}

	/**
	 * Search for a path from start to goal and keep track of the path
	 * 
	 * @param start
	 *            start node
	 * @param goal
	 *            end node
	 * @param parentNodeMap
	 *            mapping between parent and child nodes
	 * @return
	 */
	private boolean searchPathUsingDijkstra(MapNode start, MapNode goal,
			Map<MapNode, MapNode> parentNodeMap, Consumer<GeographicPoint> nodeSearched) {
		boolean found = false;

		// Initialize
		PriorityQueue<MapNode> queue = new PriorityQueue<>();
		Set<MapNode> visited = new HashSet<>();

		// Add start node
		MapNode startNode = map.get(start.getLocation());
		startNode.setOriginalDistance(0);  // init start length to 0
		
		queue.add(startNode);

		// Dijkstra - for each node check if it's the goal
		// if not loop through the children and compute distances from start to
		// each child
		// update the distances for each node and add the entry into priority
		// queue
		// during this process make sure the node is not already visited
		while (!queue.isEmpty()) {
			MapNode current = queue.remove();
			System.out.println("Dijkstra - Visited node: " + current.getLocation());
			++count;
			
			// Add node to visited list
			if (!visited.contains(current)) {
				visited.add(current);

				if (current.equals(goal)) {
					found = true;
					break;
				}

				// Hook for visualization. See writeup.
				nodeSearched.accept(current.getLocation());

				Set<MapNode> neighbors = current.getNeighbors();
				if (neighbors != null) {
					for (MapNode neighbor : neighbors) {
						if (!visited.contains(neighbor)) {
							// check path from current to neighbor 
							// distance of current node + length of edge from current to neighbor
							double distance = current.getOriginalDistance() + current.getEdgeLength(neighbor);							
							if (distance < neighbor.getOriginalDistance()) {
								neighbor.setOriginalDistance(distance);
								
								parentNodeMap.put(neighbor, current);							
								queue.add(neighbor);
							}
						}										
					}
				}
			}
		}

		return found;
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return aStarSearch(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		count = 0;
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		if ((start == null || !map.containsKey(start)) || (goal == null || !map.containsKey(goal))) {
			return path;
		}

		MapNode startNode = map.get(start);
		MapNode goalNode = map.get(goal);
		
		Map<MapNode, MapNode> parentNodeMap = new HashMap<>();
		boolean found = searchPathUsingAStarSearch(startNode, goalNode, parentNodeMap, nodeSearched);
		if (!found) {
			System.out.println("No path exists");
			return path;
		}
		
		System.out.println("Number of nodes visited by AStarSearch: " + count);
		path = constructPath(startNode, goalNode, parentNodeMap);

		return path;
	}
	
	/**
	 * Search for a path from start to goal and keep track of the path
	 * 
	 * @param start
	 *            start node
	 * @param goal
	 *            end node
	 * @param parentNodeMap
	 *            mapping between parent and child nodes
	 * @return
	 */
	private boolean searchPathUsingAStarSearch(MapNode start, MapNode goal,
			Map<MapNode, MapNode> parentNodeMap, Consumer<GeographicPoint> nodeSearched) {
		boolean found = false;

		// Initialize
		PriorityQueue<MapNode> queue = new PriorityQueue<>();
		Set<MapNode> visited = new HashSet<>();

		// Add start node
		MapNode startNode = map.get(start.getLocation());
		startNode.setOriginalDistance(0);  // init start length to 0
		startNode.setPredictedDistance(0);  // init start length to 0
		
		queue.add(startNode);

		// Dijkstra - for each node check if it's the goal
		// if not loop through the children and compute distances from start to
		// each child
		// update the distances for each node and add the entry into priority
		// queue
		// during this process make sure the node is not already visited
		while (!queue.isEmpty()) {
			MapNode current = queue.remove();
			System.out.println("AStarSearch - Visited node: " + current.getLocation());
			++count;

			// Add node to visited list
			if (!visited.contains(current)) {
				visited.add(current);

				if (current.equals(goal)) {
					found = true;
					break;
				}

				// Hook for visualization. See writeup.
				nodeSearched.accept(current.getLocation());

				Set<MapNode> neighbors = current.getNeighbors();
				if (neighbors != null) {
					for (MapNode neighbor : neighbors) {
						if (!visited.contains(neighbor)) {
							// check path from current to neighbor and predicted distance from neighbor to goal 
							// distance of current node + length of edge from current to neighbor
							// distance from neighbor to goal
							double originalDistance = current.getOriginalDistance() + current.getEdgeLength(neighbor);
							double predictedDistance = neighbor.getLocation().distance(goal.getLocation());							
							double distance = originalDistance + predictedDistance;							
							if (distance < neighbor.getOriginalDistance() + neighbor.getPredictedDistace()) {
								neighbor.setOriginalDistance(distance);
								neighbor.setPredictedDistance(predictedDistance);
								
								parentNodeMap.put(neighbor, current);							
								queue.add(neighbor);
							}
						}										
					}
				}
			}
		}

		return found;
	}

	/**
	 * This method validates whether there are any null parameters and / or
	 * length < 0 and / or the points don't exist in the graph
	 * 
	 * @param from
	 *            starting point
	 * @param to
	 *            ending point
	 * @param roadName
	 *            name of the road
	 * @param roadType
	 *            type of the road
	 * @param length
	 *            road length
	 */
	private void validateAddEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType,
			double length) {
		if ((from == null || !map.containsKey(from)) || (to == null || !map.containsKey(to)) || (roadName == null)
				|| (roadType == null) || length < 0) {
			throw new IllegalArgumentException("Please check if all the parameters are not null, distance >= 0 and both"
					+ " the points exist in the map");
		}
	}

	public static void main(String[] args) {
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		
//		System.out.print("Making a new map...");
//		MapGraph theMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
//		System.out.println("DONE.");
//
//		System.out.println("\n\n\nPrint map");
//		theMap.printMap();
//		System.out.println("=======");
//
//		System.out.println("Number of vertices = " + theMap.getNumVertices() + ". Number of edges = "
//				+ theMap.getNumEdges() + "\n");
//
//		System.out.println("List of vertices");
//		Set<GeographicPoint> vertices = theMap.getVertices();
//		for (GeographicPoint vertex : vertices) {
//			System.out.print(vertex + " , ");
//		}
//		
//		 List<GeographicPoint> vertexList = new ArrayList<>(vertices);
//		 GeographicPoint start = vertexList.get(0); 
//		 GeographicPoint goal = vertexList.get(5);
//		 
//		 System.out.println("\nPath from " + start + " to " + goal);
//
//		 List<GeographicPoint> path = theMap.aStarSearch(start, goal); 
//		 for(GeographicPoint node : path) { System.out.print(node + " , "); }

		/*
		 * List<GeographicPoint> vertexList = new ArrayList<>(vertices);
		 * GeographicPoint start = vertexList.get(0); GeographicPoint goal =
		 * vertexList.get(7);
		 * 
		 * System.out.println("\nPath from " + start + " to " + goal);
		 * 
		 * List<GeographicPoint> path = theMap.bfs(start, goal); for
		 * (GeographicPoint node : path) { System.out.print(node + " , "); }
		 */

		// You can use this method for testing.

		/*
		 * Use this code in Week 3 End of Week Quiz MapGraph theMap = new
		 * MapGraph(); System.out.print("DONE. \nLoading the map...");
		 * GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		 * System.out.println("DONE.");
		 * 
		 * GeographicPoint start = new GeographicPoint(32.8648772,
		 * -117.2254046); GeographicPoint end = new GeographicPoint(32.8660691,
		 * -117.217393);
		 * 
		 * 
		 * List<GeographicPoint> route = theMap.dijkstra(start,end);
		 * List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		 * 
		 */

	}

}
