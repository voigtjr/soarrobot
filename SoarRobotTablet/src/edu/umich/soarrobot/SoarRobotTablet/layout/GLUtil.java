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
	
	private static float[] cubeNormals = {
		0.0f, 0.0f, 1.0f,
		0.0f, 0.0f, 1.0f,
		0.0f, 0.0f, -1.0f,
		0.0f, 0.0f, -1.0f,
		-1.0f, 0.0f, 0.0f,
		-1.0f, 0.0f, 0.0f,
		1.0f, 0.0f, 0.0f,
		1.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f,
		0.0f, 1.0f, 0.0f,
		0.0f, -1.0f, 0.0f,
		0.0f, -1.0f, 0.0f,
	};
		
	private static short[] rectIndeces = {
		0, 2, 1,
		1, 2, 3,
	};
	
	private static FloatBuffer cubeCoordsBuffer, whiteColorsBuffer, redColorsBuffer, greenColorsBuffer, blueColorsBuffer, yellowColorsBuffer, blackColorsBuffer, grayColorsBuffer;
	private static ShortBuffer cubeIndecesBuffer, rectIndecesBuffer;
	private static FloatBuffer cubeNormalsBuffer, rectNormalsBuffer;
	
	static {
        // float has 4 bytes, coordinate * 4 bytes
        ByteBuffer bb = ByteBuffer.allocateDirect(cubeCoords.length * 4);
        bb.order(ByteOrder.nativeOrder());
        cubeCoordsBuffer = bb.asFloatBuffer();
     
        // short has 2 bytes, indices * 2 bytes
        bb = ByteBuffer.allocateDirect(cubeIndeces.length * 2);
        bb.order(ByteOrder.nativeOrder());
        cubeIndecesBuffer = bb.asShortBuffer();
        
        // short has 2 bytes, indices * 2 bytes
        bb = ByteBuffer.allocateDirect(rectIndeces.length * 2);
        bb.order(ByteOrder.nativeOrder());
        rectIndecesBuffer = bb.asShortBuffer();

        whiteColorsBuffer = makeColorBuffer(1.0f, 1.0f, 1.0f);
        redColorsBuffer = makeColorBuffer(1.0f, 0.0f, 0.0f);
        greenColorsBuffer = makeColorBuffer(0.0f, 1.0f, 0.0f);
        blueColorsBuffer = makeColorBuffer(0.0f, 0.0f, 1.0f);
        yellowColorsBuffer = makeColorBuffer(1.0f, 1.0f, 0.0f);
        blackColorsBuffer = makeColorBuffer(0.0f, 0.0f, 0.0f);
        grayColorsBuffer = makeColorBuffer(0.5f, 0.5f, 0.5f);
     
        cubeCoordsBuffer.put(cubeCoords);
        cubeIndecesBuffer.put(cubeIndeces);
        rectIndecesBuffer.put(rectIndeces);
     
        cubeCoordsBuffer.position(0);
        cubeIndecesBuffer.position(0);
        rectIndecesBuffer.position(0);
        
        bb = ByteBuffer.allocateDirect(cubeNormals.length * 4);
        bb.order(ByteOrder.nativeOrder());
        cubeNormalsBuffer = bb.asFloatBuffer();
        cubeNormalsBuffer.put(cubeNormals);
        cubeNormalsBuffer.position(0);

	}
	
	private static FloatBuffer makeColorBuffer(float r, float g, float b) {
        float[] colors = new float[8 * 4];
        assignColors(colors, r, g, b);
		ByteBuffer cbb = ByteBuffer.allocateDirect(colors.length * 4);
        cbb.order(ByteOrder.nativeOrder());
        FloatBuffer buffer = cbb.asFloatBuffer();
        buffer.put(colors);
        buffer.position(0);
        return buffer;
	}
	
	private static void assignColors(float[] ar, float r, float g, float b) {
		r += 0.2f;
		g += 0.2f;
		b += 0.2f;
		if (r > 1.0f) r = 1.0f;
		if (g > 1.0f) g = 1.0f;
		if (b > 1.0f) b = 1.0f;
		for (int i = 0; i < ar.length / 8; ++i) {
			ar[i * 4] = r;
			ar[i * 4 + 1] = g;
			ar[i * 4 + 2] = b;
			ar[i * 4 + 3] = 1.0f;
		}
		r -= 0.4f;
		g -= 0.4f;
		b -= 0.4f;
		if (r < 0.0f) r = 0.0f;
		if (g < 0.0f) g = 0.0f;
		if (b < 0.0f) b = 0.0f;
		for (int i = ar.length / 8; i < ar.length / 4; ++i) {
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
	public static void drawCube(GL10 gl, float x, float y, float z, float w, float h, float d, int color, float theta)
	{
        gl.glPushMatrix();

        gl.glTranslatef(x, y, z);
        gl.glRotatef(theta, 0.0f, 0.0f, 1.0f);
        gl.glScalef(w, h, d);
        gl.glTranslatef(-0.5f, -0.5f, -0.5f);
        
        gl.glVertexPointer(3, GL10.GL_FLOAT, 0, cubeCoordsBuffer);
        gl.glColorPointer(4, GL10.GL_FLOAT, 0, getColor(color));
        gl.glNormalPointer(GL10.GL_FLOAT, 0, cubeNormalsBuffer);
        gl.glDrawElements(GL10.GL_TRIANGLES, 12 * 3, GL10.GL_UNSIGNED_SHORT, cubeIndecesBuffer);
        
        gl.glPopMatrix();
	}
	
	public static void drawCube(GL10 gl, float x, float y, float z, float w, float h, float d, int color)
	{
		drawCube(gl, x, y, z, w, h, d, color, 0.0f);
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
        gl.glNormalPointer(GL10.GL_FLOAT, 0, cubeNormalsBuffer);
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
		case Color.BLACK:
			return blackColorsBuffer;
		case Color.GRAY:
			return grayColorsBuffer;
		}
		return whiteColorsBuffer;
	}
}
