package edu.umich.robot.laser;

import java.io.IOException;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;
import april.lcmtypes.urg_range_t;

import com.google.common.collect.ImmutableList;
import com.google.common.collect.Lists;

public class UrgBinner
{
    private static final String channel = "TODO"; // TODO

    private final LCM lcm = LCM.getSingleton();
    
    private final int nbins;
    
    private final List<Float> binned = Lists.newArrayList();
    
    private urg_range_t urg;
    
    private long last = -1;
    
    private final double fov0;
    
    private final double fovstep;
    
    private final ReentrantLock lock = new ReentrantLock();
    
    public UrgBinner(int bins, double fov)
    {
        if (bins <= 0 || fov <= 0)
            throw new IllegalArgumentException("bins or fov less than zero");
        
        if (fov > Math.PI * 2)
            throw new IllegalArgumentException("fov > 2pi");
        
        this.nbins = bins;
        this.fov0 = fov / -2;
        this.fovstep = fov / bins;
        
        for (int i = 0; i < bins; ++i)
            binned.add(Float.valueOf(0));
        
        lcm.subscribe(channel, subscriber);
    }
    
    private final LCMSubscriber subscriber = new LCMSubscriber()
    {
        @Override
        public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
        {
            try 
            {
                urg = new urg_range_t(ins);
            }
            catch (IOException e)
            {
                e.printStackTrace();
            }
        }
    };
    
    public void update()
    {
        update(urg);
    }
    
    void update(urg_range_t urg)
    {
        // shadowed this.urg on purpose

        if (urg == null)
            return;
        
        if (last >= urg.utime)
            return;
        
        double theta = urg.rad0;
        double nextbin = fov0;
        int bin = -1;
        float value = Float.MAX_VALUE;
        
        for (int i = 0; i < urg.num_ranges; ++i, theta += urg.radstep)
        {
            if (theta >= nextbin)
            {
                if (bin >= 0)
                {
                    lock.lock();
                    try 
                    {
                        binned.set(bin, value);
                    }
                    finally
                    {
                        lock.unlock();
                    }
                }
                
                nextbin += fovstep;
                bin += 1;
                if (bin >= nbins)
                    break;
            }
            
            if (bin < 0)
                continue;
            
            if (urg.ranges[i] < 0)
                continue;
            
            value = Math.min(value, urg.ranges[i]);
        }
        
        if (bin >= 0 && bin < nbins)
        {
            lock.lock();
            try
            {
                binned.set(bin, value);
            }
            finally
            {
                lock.unlock();
            }
        }
    }
    
    public List<Float> getBinned()
    {
        return new ImmutableList.Builder<Float>().addAll(binned).build();
    }
    
}
