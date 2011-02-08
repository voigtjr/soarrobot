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
package edu.umich.robot.superdroid;

import java.net.SocketException;
import java.net.UnknownHostException;

import edu.umich.grrc.GrrcSuperdroid;
import edu.umich.robot.util.Updatable;

/**
 * @author voigtjr@gmail.com
 */
public class SuperdroidInterface implements Updatable
{
    private final GrrcSuperdroid sd;

    private float lv;

    private float av;

    /**
     * "141.212.203.164" "141.212.205.136" 3192
     * 
     * @param hostname
     * @param port
     * @throws SocketException
     * @throws UnknownHostException
     */
    public SuperdroidInterface(String hostname, int port)
            throws UnknownHostException, SocketException
    {
        sd = new GrrcSuperdroid(hostname, port);
    }

    public void setLinear(float lv)
    {
        this.lv = lv;
    }

    public void setAngular(float av)
    {
        this.av = av;
    }

    public void update(double dt)
    {
        sd.setVelocities(lv, av);
    }
    
    public void shutdown()
    {
        sd.shutdown();
    }
}
