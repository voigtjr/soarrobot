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
package edu.umich.robot.laser;

import java.util.concurrent.locks.ReentrantLock;

import april.lcmtypes.laser_t;
import april.util.TimeUtil;

/**
 * @author voigtjr@gmail.com
 */
class LaserMerger
{
    private final ReentrantLock simlock = new ReentrantLock();
    private laser_t sim;
    private long simstamp = -1;

    private final ReentrantLock sicklock = new ReentrantLock();
    private laser_t sick;
    private long sickstamp = -1;
    private volatile boolean changed = false;

    private laser_t auth;
    private Laser cached;
    
    private final long cacheTime;
    
    private int checks = 0;
    private int calls = 0;
    private int cacheReturned = 0;
    
    public LaserMerger(long cacheTime)
    {
        this.cacheTime = cacheTime;
    }

    Laser getLaser()
    {
        ++calls;
        changed = false;
        long now = TimeUtil.mstime();
        
        laser_t m = getSim(now);
        laser_t k = getSick(now);

        if (m == null && k == null)
        {
            ++cacheReturned;
            return cached;
        }
        
        auth = m == null ? k : m;
        
        Laser.Builder builder = new Laser.Builder(auth.rad0, auth.radstep);
        for (int i = 0; i < auth.nranges; ++i)
            builder.add(getRange(i, m, k));
        cached = builder.build();
        return cached;
    }
    
    private laser_t getSim(long now)
    {
        simlock.lock();
        try
        {
            if (sim != null)
            {
                if (now - simstamp > cacheTime)
                    sim = null;
                else
                    return sim;
            }
        }
        finally
        {
            simlock.unlock();
        }
        return null;
    }
    
    private laser_t getSick(long now)
    {
        sicklock.lock();
        try
        {
            if (sick != null)
            {
                if (now - sickstamp > cacheTime)
                    sick = null;
                else
                    return sick;
            }
        }
        finally
        {
            sicklock.unlock();
        }
        return null;
    }

    private float getRange(int index, laser_t m, laser_t k)
    {
        if (index < 0)
            return Float.MAX_VALUE;

        if (k == null)
        {
            if (index < auth.ranges.length)
                return auth.ranges[index];
            return Float.MAX_VALUE;
        }

        float simr = index < m.ranges.length ? m.ranges[index]
                : Float.MAX_VALUE;
        float sickr = index < k.ranges.length ? k.ranges[index]
                : Float.MAX_VALUE;

        // want min here, the closest range reported
        return Math.min(simr, sickr);
    }
    
    void newSim(laser_t laser)
    {
        simlock.lock();
        try 
        {
            if (sim != null)
                if (laser.utime <= sim.utime)
                    return;
            changed = true;
            sim = laser;
            simstamp = TimeUtil.mstime();
        }
        finally
        {
            simlock.unlock();
        }
    }

    void newSick(laser_t laser)
    {
        sicklock.lock();
        try
        {
            if (sick != null)
                if (laser.utime <= sick.utime)
                    return;
            changed = true;
            sick = laser;
            sickstamp = TimeUtil.mstime();
        }
        finally
        {
            sicklock.unlock();
        }
    }

    boolean isChanged()
    {
        ++checks;
        return changed;
    }

}
