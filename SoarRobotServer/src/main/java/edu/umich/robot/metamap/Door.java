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

import java.util.List;

import com.google.common.collect.ImmutableList;

/**
 * @author voigtjr@gmail.com
 */
public class Door
{
    public enum State
    {
        OPEN, CLOSED, LOCKED
    }
    
    private State state = State.OPEN;
    
    private int code = -1;
    
    private final List<Double> xywh;
    
    private final int id;
    
    public Door(int id, List<Double> xywh)
    {
        this.id = id;
        this.xywh = new ImmutableList.Builder<Double>()
        .addAll(xywh)
        .build();
    }
    
    Door copy()
    {
        Door ret = new Door(id, xywh);
        ret.state = state;
        ret.code = code;
        return ret;
    }
    
    public synchronized State getState()
    {
        return state;
    }
    
    synchronized void setState(State state)
    {
        this.state = state;
    }
    
    public synchronized int getCode()
    {
        return code;
    }
    
    synchronized void setCode(int code)
    {
        this.code = code;
    }
    
    public synchronized List<Double> getxywh()
    {
        return xywh;
    }

    public synchronized int getId()
    {
        return id;
    }
    
}
