/*
 * Copyright (c) 2011, Regents of the University of Michigan
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
package edu.umich.robot.metamap;

/**
 * @author voigtjr@gmail.com
 */
public class AreaState
{
    private boolean isRoomLightBroken = false;
    private boolean roomLightOn = true;
    private boolean hasWindows = false;
    private boolean areWindowShadesBroken = false;
    private boolean windowShadesActivated = false;
    
    void setRoomLightBroken(boolean broken)
    {
        this.isRoomLightBroken = broken;
    }
    
    void setRoomLightOn(boolean on)
    {
        if (isRoomLightBroken)
            return;
        this.roomLightOn = on;
    }
    
    void setHasWindows(boolean hasWindows)
    {
        this.hasWindows = hasWindows;
    }
    
    void setWindowShadesBroken(boolean broken)
    {
        this.areWindowShadesBroken = broken;
    }
    
    void setWindowShadesActivated(boolean activated)
    {
        if (areWindowShadesBroken)
            return;
        this.windowShadesActivated = activated;
    }
    
    public boolean hasWindows()
    {
        return hasWindows;
    }
    
    public boolean isLit(boolean daytime)
    {
        return roomLightOn || (daytime && hasWindows && !windowShadesActivated);
    }

    public boolean isRoomLightOn()
    {
        return roomLightOn;
    }
}
