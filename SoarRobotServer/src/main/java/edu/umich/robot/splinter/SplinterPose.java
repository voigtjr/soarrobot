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
package edu.umich.robot.splinter;

import java.io.IOException;
import java.util.concurrent.TimeUnit;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import april.jmat.LinAlg;
import april.jmat.MathUtil;
import april.lcmtypes.pose_t;

/**
 * @author voigtjr@gmail.com
 */
public class SplinterPose
{
    private static final Log logger = LogFactory.getLog(SplinterPose.class);

    private static final double MIN_ELABORATION_INTERVAL_SEC = 0.010; 

    public static final String POSE_CHANNEL_BASE = "POSE_";

    private pose_t pose;

    private pose_t elaboratedPose = new pose_t();

    SplinterPose(String id)
    {
        String channel = POSE_CHANNEL_BASE + id;
        LCM.getSingleton().subscribe(channel, new LCMSubscriber()
        {
            public void messageReceived(LCM lcm, String channel,
                    LCMDataInputStream ins)
            {
                try
                {
                    pose = new pose_t(ins);
                }
                catch (IOException e)
                {
                    logger.error("Error decoding pose_t message: "
                            + e.getMessage());
                }
            }
        });
    }

    pose_t getPose()
    {
        pose_t tp = pose; // grab a reference once

        if (tp != null && tp.utime > elaboratedPose.utime)
        {
            if (elaboratedPose.utime == 0)
            {
                elaboratedPose = tp;
                return elaboratedPose;
            }
            // elaborate to get velocities from odometry

            // (new usec - old usec) * 1 / usec in sec
            double dt = (tp.utime - elaboratedPose.utime) / (double)TimeUnit.MICROSECONDS.convert(1, TimeUnit.SECONDS);
            if (dt < MIN_ELABORATION_INTERVAL_SEC)
                return elaboratedPose;

            // TODO: may need modification for smoothing
            tp.vel[0] = (tp.pos[0] - elaboratedPose.pos[0]) * (1.0 / dt);
            tp.vel[1] = (tp.pos[1] - elaboratedPose.pos[1]) * (1.0 / dt);

            double newTheta = LinAlg.quatToRollPitchYaw(tp.orientation)[2];
            newTheta = MathUtil.mod2pi(newTheta);
            double oldTheta = LinAlg
                    .quatToRollPitchYaw(elaboratedPose.orientation)[2];
            oldTheta = MathUtil.mod2pi(oldTheta);
            tp.rotation_rate[2] = MathUtil.mod2pi(newTheta - oldTheta)
                    * (1.0 / dt);

            if (logger.isTraceEnabled())
            {
                double xvel = LinAlg.rotate2(tp.vel, -newTheta)[0];
                logger.trace(String.format(
                        "dt%1.5f vx%1.3f vy%1.3f r%1.3f xv%1.3f", dt,
                        tp.vel[0], tp.vel[1], tp.rotation_rate[2], xvel));
            }

            elaboratedPose = tp;
        }

        return elaboratedPose.copy();
    }
}
