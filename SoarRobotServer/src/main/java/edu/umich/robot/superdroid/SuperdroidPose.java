package edu.umich.robot.superdroid;

import java.io.IOException;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import april.jmat.LinAlg;
import april.lcmtypes.pose_t;
import edu.umich.robot.util.PoseProvider;
import edu.umich.robot.util.Poses;

public class SuperdroidPose implements PoseProvider
{
    private static final Log logger = LogFactory.getLog(Poses.class);

    public static final String POSE_CHANNEL_BASE = "POSE_";

    private pose_t pose;

    private pose_t elaboratedPose = new pose_t();

    private double[] offset = { 0, 0, 0 };
    
    SuperdroidPose(String id)
    {
        String channel = POSE_CHANNEL_BASE + id;
        LCM lcm = LCM.getSingleton();
        lcm.subscribe(channel, new LCMSubscriber()
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

        // TODO: This isn't switched by name. If there are multiple bots, they'll all teleport.
        lcm.subscribe("POSE_TELEPORT", new LCMSubscriber()
        {
            @Override
            public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
            {
                try
                {
                    pose_t teleport = new pose_t(ins);
                    LinAlg.subtract(teleport.pos, pose.pos, offset);
                }
                catch (IOException e)
                {
                    e.printStackTrace();
                }
            }
        });
        
    }
    
    public void setOffset(double[] offset)
    {
        this.offset = offset;
    }
    
    public pose_t getPose()
    {
        pose_t newPose = pose; // grab a reference once

        if (newPose != null && newPose.utime > elaboratedPose.utime)
        {
            elaboratedPose = Poses.elaborate(elaboratedPose, newPose);
            LinAlg.add(elaboratedPose.pos, offset, elaboratedPose.pos);
        }
        
        return elaboratedPose.copy();
    }

}
