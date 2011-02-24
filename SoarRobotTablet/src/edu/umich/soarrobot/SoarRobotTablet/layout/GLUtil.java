package edu.umich.soarrobot.SoarRobotTablet.layout;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;

import javax.microedition.khronos.opengles.GL10;

import android.graphics.Color;

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
	
	private static short[] rectIndeces = {
		0, 2, 1,
		1, 2, 3,
	};
	
	private static float[] whiteColors = new float[8 * 4];
	private static float[] redColors = new float[8 * 4];
	private static float[] greenColors = new float[8 * 4];
	private static float[] blueColors = new float[8 * 4];
	private static float[] yellowColors = new float[8 * 4];
	
	private static FloatBuffer cubeCoordsBuffer, whiteColorsBuffer, redColorsBuffer, greenColorsBuffer, blueColorsBuffer, yellowColorsBuffer;
	private static ShortBuffer cubeIndecesBuffer, rectIndecesBuffer;
	
	static {
        // float has 4 bytes, coordinate * 4 bytes
        ByteBuffer vbb = ByteBuffer.allocateDirect(cubeCoords.length * 4);
        vbb.order(ByteOrder.nativeOrder());
        cubeCoordsBuffer = vbb.asFloatBuffer();
     
        // short has 2 bytes, indices * 2 bytes
        ByteBuffer ibb = ByteBuffer.allocateDirect(cubeIndeces.length * 2);
        ibb.order(ByteOrder.nativeOrder());
        cubeIndecesBuffer = ibb.asShortBuffer();
        
        // short has 2 bytes, indices * 2 bytes
        ByteBuffer ribb = ByteBuffer.allocateDirect(rectIndeces.length * 2);
        ribb.order(ByteOrder.nativeOrder());
        rectIndecesBuffer = ribb.asShortBuffer();
     
        // float has 4 bytes, colors (RGBA) * 4 bytes
        ByteBuffer cbb = ByteBuffer.allocateDirect(whiteColors.length * 4);
        cbb.order(ByteOrder.nativeOrder());
        whiteColorsBuffer = cbb.asFloatBuffer();
        
        // float has 4 bytes, colors (RGBA) * 4 bytes
        cbb = ByteBuffer.allocateDirect(whiteColors.length * 4);
        cbb.order(ByteOrder.nativeOrder());
        redColorsBuffer = cbb.asFloatBuffer();
        
        // float has 4 bytes, colors (RGBA) * 4 bytes
        cbb = ByteBuffer.allocateDirect(whiteColors.length * 4);
        cbb.order(ByteOrder.nativeOrder());
        greenColorsBuffer = cbb.asFloatBuffer();
        
        // float has 4 bytes, colors (RGBA) * 4 bytes
        cbb = ByteBuffer.allocateDirect(whiteColors.length * 4);
        cbb.order(ByteOrder.nativeOrder());
        blueColorsBuffer = cbb.asFloatBuffer();
        
        // float has 4 bytes, colors (RGBA) * 4 bytes
        cbb = ByteBuffer.allocateDirect(whiteColors.length * 4);
        cbb.order(ByteOrder.nativeOrder());
        yellowColorsBuffer = cbb.asFloatBuffer();
        
		assignColors(whiteColors, 1.0f, 1.0f, 1.0f);
		assignColors(redColors, 1.0f, 0.0f, 0.0f);
		assignColors(greenColors, 0.0f, 1.0f, 0.0f);
		assignColors(blueColors, 0.0f, 0.0f, 1.0f);
		assignColors(yellowColors, 0.8f, 0.8f, 0.0f);
     
        cubeCoordsBuffer.put(cubeCoords);
        cubeIndecesBuffer.put(cubeIndeces);
        whiteColorsBuffer.put(whiteColors);
        redColorsBuffer.put(redColors);
        greenColorsBuffer.put(greenColors);
        blueColorsBuffer.put(blueColors);
        yellowColorsBuffer.put(yellowColors);
        rectIndecesBuffer.put(rectIndeces);
     
        cubeCoordsBuffer.position(0);
        cubeIndecesBuffer.position(0);
        whiteColorsBuffer.position(0);
        rectIndecesBuffer.position(0);
        redColorsBuffer.position(0);
        greenColorsBuffer.position(0);
        blueColorsBuffer.position(0);
        yellowColorsBuffer.position(0);
	}
	
	private static void assignColors(float[] ar, float r, float g, float b) {
		for (int i = 0; i < ar.length / 4; ++i) {
			ar[i * 4] = r;
			ar[i * 4 + 1] = g;
			ar[i * 4 + 2] = b;
			ar[i * 4 + 3] = 1.0f;
		}
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
	public static void drawCube(GL10 gl, float x, float y, float z, float w, float h, float d, int color)
	{
        gl.glPushMatrix();
        
        gl.glTranslatef(x, y, z);
        gl.glScalef(w, h, d);

        gl.glVertexPointer(3, GL10.GL_FLOAT, 0, cubeCoordsBuffer);
        gl.glColorPointer(4, GL10.GL_FLOAT, 0, getColor(color));
        gl.glDrawElements(GL10.GL_TRIANGLES, 12 * 3, GL10.GL_UNSIGNED_SHORT, cubeIndecesBuffer);
        
        gl.glPopMatrix();
	}
	
	/*************
	 * Rectangle *
	 *************/
	
	public static void drawRect(GL10 gl, float x, float y, float z, float w, float h, int color)
	{        
        gl.glPushMatrix();
        
        gl.glTranslatef(x, y, z);
        gl.glScalef(w, h, 1.0f);

        gl.glVertexPointer(3, GL10.GL_FLOAT, 0, cubeCoordsBuffer);
        gl.glColorPointer(4, GL10.GL_FLOAT, 0, getColor(color));
        gl.glDrawElements(GL10.GL_TRIANGLES, 2 * 3, GL10.GL_UNSIGNED_SHORT, rectIndecesBuffer);
        
        gl.glPopMatrix();
	}
	
	private static FloatBuffer getColor(int color) {
		switch (color) {
		case Color.WHITE:
			return whiteColorsBuffer;
		case Color.RED:
			return redColorsBuffer;
		case Color.GREEN:
			return greenColorsBuffer;
		case Color.BLUE:
			return blueColorsBuffer;
		case Color.YELLOW:
			return yellowColorsBuffer;
		}
		return whiteColorsBuffer;
	}
}
