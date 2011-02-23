package edu.umich.soarrobot.SoarRobotTablet.layout;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.nio.ShortBuffer;

import javax.microedition.khronos.opengles.GL10;

public class GLUtil {
	
	// X = left / right
	// Y = up / down
	// Z = forward / backward
	
	/*
	 *        6----------7
	 *       /|          |
	 *      / |          |
	 *     2  |      (3) |
	 *     |  |          |
	 *     |  4----------5
	 *     | /          /
	 *     |/          /
	 *     0----------1
	 */
	
	private static float[] cubeCoords = {
		0.0f, 0.0f, 0.0f,
		1.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f,
		1.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 1.0f,
		1.0f, 0.0f, 1.0f,
		0.0f, 1.0f, 1.0f,
		1.0f, 1.0f, 1.0f,
	};
	
	private static short[] cubeIndeces = {
		// Back
		0, 2, 1,
		1, 2, 3,
		
		// Front
		4, 5, 6,
		5, 7, 6,
		
		// Left side
		0, 4, 2,
		4, 6, 2,
		
		// Right side
		1, 3, 5,
		5, 3, 7,
		
		// Top side
		2, 6, 3,
		6, 7, 3,
		
		// Bottom side
		0, 1, 4,
		1, 5, 4,
	};
	
	private static float[] cubeColors = new float[8 * 4];
	
	private static FloatBuffer cubeCoordsBuffer, cubeColorsBuffer;
	private static ShortBuffer cubeIndecesBuffer;
	
	static {
        // float has 4 bytes, coordinate * 4 bytes
        ByteBuffer vbb = ByteBuffer.allocateDirect(cubeCoords.length * 4);
        vbb.order(ByteOrder.nativeOrder());
        cubeCoordsBuffer = vbb.asFloatBuffer();
     
        // short has 2 bytes, indices * 2 bytes
        ByteBuffer ibb = ByteBuffer.allocateDirect(cubeIndeces.length * 2);
        ibb.order(ByteOrder.nativeOrder());
        cubeIndecesBuffer = ibb.asShortBuffer();
     
        // float has 4 bytes, colors (RGBA) * 4 bytes
        ByteBuffer cbb = ByteBuffer.allocateDirect(cubeColors.length * 4);
        cbb.order(ByteOrder.nativeOrder());
        cubeColorsBuffer = cbb.asFloatBuffer();
        
        for (int i = 0; i < cubeColors.length; ++i)
		{
			cubeColors[i] = i / (float) cubeColors.length;
		}
     
        cubeCoordsBuffer.put(cubeCoords);
        cubeIndecesBuffer.put(cubeIndeces);
        cubeColorsBuffer.put(cubeColors);
     
        cubeCoordsBuffer.position(0);
        cubeIndecesBuffer.position(0);
        cubeColorsBuffer.position(0);
	}
	
	/**
	 * Current matrix should be MODELVIEW
	 * @param gl
	 * @param x
	 * @param y
	 * @param r
	 * @param g
	 * @param b
	 */
	public static void drawCube(GL10 gl, float x, float y, float z, float w, float h, float d, float r, float g, float b)
	{
		for (int i = 0; i < cubeColors.length / 4; ++i)
		{
			cubeColors[i * 4] = r;
			cubeColors[i * 4 + 1] = g;
			cubeColors[i * 4 + 2] = b;
			cubeColors[i * 4 + 3] = 1.0f;
		}

		cubeColorsBuffer.put(cubeColors);
        cubeColorsBuffer.position(0);
        
        gl.glPushMatrix();
        gl.glScalef(w, h, d);
        gl.glTranslatef(-x / w, -y / h, -z / d);

        gl.glVertexPointer(3, GL10.GL_FLOAT, 0, cubeCoordsBuffer);
        gl.glColorPointer(4, GL10.GL_FLOAT, 0, cubeColorsBuffer);
        gl.glDrawElements(GL10.GL_TRIANGLES, 12 * 3, GL10.GL_UNSIGNED_SHORT, cubeIndecesBuffer);
        
        gl.glPopMatrix();
	}
}
