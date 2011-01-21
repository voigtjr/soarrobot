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

import java.io.IOException;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import april.lcmtypes.laser_t;

import com.google.common.util.concurrent.MoreExecutors;

/**
 * Merges messages from a SICK and/or simulated laser sensor in to a single,
 * "lower resolution" reading of usually 5 ranges. Merges by taking the minimum
 * reading from the set of sensors.
 * 
 * Will use an old reading up to a configurable duration so that the sensor
 * doesn't "jitter" when old and new readings aren't available at the same time.
 * 
 * @author voigtjr@gmail.com
 */
public class Lidar
{
    private static final Log logger = LogFactory.getLog(Lidar.class);

    private static final LCM lcm = LCM.getSingleton();

    private static final String SIM_LASER_CHANNEL_BASE = "SIM_LIDAR_FRONT_";

    private static final String SICK_LASER_CHANNEL_BASE = "SICK_LIDAR_FRONT_";

    private static final String LASER_LOWRES_CHANNEL_BASE = "LIDAR_LOWRES_";

    private static final int RANGE_COUNT = 5; // TODO: make configurable

    private final String simChannel;

    private final String sickChannel;

    private final String lowresChannel;
    
    private final ScheduledExecutorService schexec = MoreExecutors.getExitingScheduledExecutorService(new ScheduledThreadPoolExecutor(1));

    private final LaserMerger merger = new LaserMerger(TimeUnit.NANOSECONDS
            .convert(1, TimeUnit.SECONDS));

    private Laser cached;
    
    public Lidar(String name)
    {
        simChannel = SIM_LASER_CHANNEL_BASE + name;
        sickChannel = SICK_LASER_CHANNEL_BASE + name;
        lowresChannel = LASER_LOWRES_CHANNEL_BASE + name;

        lcm.subscribe(simChannel, subscriber);
        lcm.subscribe(sickChannel, subscriber);

        schexec.scheduleAtFixedRate(broadcast, 0, 100, TimeUnit.MILLISECONDS);
    }

    private final Runnable broadcast = new Runnable()
    {
        public void run()
        {
            if (merger.isChanged())
                updateCached();
            if (cached == null)
                return;
            lcm.publish(lowresChannel, cached.toLcm());
        }
    };

    private final LCMSubscriber subscriber = new LCMSubscriber()
    {
        public void messageReceived(LCM lcm, String channel,
                LCMDataInputStream ins)
        {
            try
            {
                if (channel.equals(simChannel))
                    merger.newSim(new laser_t(ins));
                else if (channel.equals(sickChannel))
                    merger.newSick(new laser_t(ins));
                else
                    logger.error("Unknown message channel: " + channel);
            }
            catch (IOException e)
            {
                logger.error("Error decoding " + channel + ": "
                        + e.getMessage());
            }
        }
    };

    private void updateCached()
    {
        Laser ml = merger.getLaser();
        if (ml == null)
            return;

        int chunkRanges = ml.getRanges().size() / RANGE_COUNT;
        float rad0 = ml.getRad0() + ml.getRadStep() * (chunkRanges / 2.0f);
        float radstep = ml.getRanges().size() * ml.getRadStep() / RANGE_COUNT;

        Laser.Builder lowres = new Laser.Builder(rad0, radstep);
        for (int slice = 0, index = 0; slice < RANGE_COUNT; ++slice)
        {
            float distance = Float.MAX_VALUE;

            for (; index < (chunkRanges * slice) + chunkRanges; ++index)
                distance = Math.min(ml.getRanges().get(index), distance);

            if (logger.isTraceEnabled())
                logger.trace(String.format("%d: %1.3f", slice, distance));

            lowres.add(distance);
        }

        cached = lowres.build();
    }

    public Laser getLaserLowRes()
    {
        return cached;
    }
    
    public void shutdown()
    {
        schexec.shutdown();
    }
}
