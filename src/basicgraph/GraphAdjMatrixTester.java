package basicgraph;

import static org.junit.Assert.*;

import java.util.List;

import org.junit.Before;
import org.junit.Test;

/**
 * Unit tests
 */
public class GraphAdjMatrixTester {

	GraphAdjMatrix instance;
	
	private int[][] matrix;
	
	@Before
	public void setup() {
		instance = new GraphAdjMatrix();
		
		matrix = new int[4][4];
		matrix[0][0] = 0;
		matrix[0][1] = 1;
		matrix[0][2] = 1;
		matrix[0][3] = 0;
		matrix[1][0] = 0;
		matrix[1][1] = 0;
		matrix[1][2] = 0;
		matrix[1][3] = 1;
		matrix[2][0] = 0;
		matrix[2][1] = 1;
		matrix[2][2] = 0;
		matrix[2][3] = 1;
		matrix[3][0] = 0;
		matrix[3][1] = 0;
		matrix[3][2] = 0;
		matrix[3][3] = 0;
	}
	
	@Test
	public void testGetDistance2() {
		instance.implementAddEdge(0,  1);
		instance.implementAddEdge(0,  2);
		instance.implementAddEdge(1,  3);
		instance.implementAddEdge(2,  1);
		instance.implementAddEdge(2,  3);
		
		List<Integer> dist2Vertices = instance.getDistance2(0);
		assertTrue(dist2Vertices.size() == 3);
		assertTrue(dist2Vertices.contains(1));
		assertTrue(dist2Vertices.contains(3));
	}
	
	@Test
	public void testMatrixMultiplication() {
		printMatrix(matrix);
		System.out.println();
		
		int retMatrix[][] = instance.matrixMultiplication(matrix);
		printMatrix(retMatrix);
		
		assertTrue(retMatrix[0][0] == 0);
		assertTrue(retMatrix[0][1] == 1);
		assertTrue(retMatrix[0][2] == 0);
		assertTrue(retMatrix[0][3] == 2);
		assertTrue(retMatrix[1][0] == 0);
		assertTrue(retMatrix[1][1] == 0);
		assertTrue(retMatrix[1][2] == 0);
		assertTrue(retMatrix[1][3] == 0);
		assertTrue(retMatrix[2][0] == 0);
		assertTrue(retMatrix[2][1] == 0);
		assertTrue(retMatrix[2][2] == 0);
		assertTrue(retMatrix[2][3] == 1);
		assertTrue(retMatrix[3][0] == 0);
		assertTrue(retMatrix[3][1] == 0);
		assertTrue(retMatrix[3][2] == 0);
		assertTrue(retMatrix[3][3] == 0);		
	}
	
	private void printMatrix(int[][] matrix) {
		for(int i = 0; i < matrix.length; i++) {
			for(int j = 0; j < matrix[0].length; j++) {
				System.out.print(" " + matrix[i][j]);
			}
			System.out.println();
		}
	}
		
}
