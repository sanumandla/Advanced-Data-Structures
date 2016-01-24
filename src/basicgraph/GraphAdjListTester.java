package basicgraph;

import static org.junit.Assert.*;

import java.util.List;

import org.junit.Before;
import org.junit.Test;

/**
 * Unit tests
 */
public class GraphAdjListTester {

	GraphAdjList instance;
	
	private int[][] matrix;
	
	@Before
	public void setup() {
		instance = new GraphAdjList();			
	}
	
	@Test
	public void testGetDistance2() {
		instance.addVertex();
		instance.addVertex();
		instance.addVertex();
		instance.addVertex();
		
		instance.addEdge(0, 1);
		instance.addEdge(0, 2);
		instance.addEdge(1, 3);
		instance.addEdge(2, 3);
		
		List<Integer> dist2Vertices = instance.getDistance2(0);
		assertTrue(dist2Vertices.size() == 2);
		assertTrue(dist2Vertices.contains(3));
		assertTrue(dist2Vertices.contains(3));
	}
		
}
