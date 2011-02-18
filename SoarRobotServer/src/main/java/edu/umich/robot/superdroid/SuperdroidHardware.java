package edu.umich.robot.superdroid;

import java.io.IOException;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import april.jmat.LinAlg;
import april.lcmtypes.differential_drive_command_t;
import april.lcmtypes.pose_t;
import april.util.TimeUtil;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;
import edu.umich.grrc.GrrcSuperdroid;

public class SuperdroidHardware
{
    private final GrrcSuperdroid sd;

    private final LCM lcm = LCM.getSingleton();
    
    private final ScheduledExecutorService schexec = Executors.newSingleThreadScheduledExecutor();

    private final double[] rpy = { 0, 0, 0 };
    
    private final String poseChannel;
    
    public SuperdroidHardware(String name, String hostname, int port) throws UnknownHostException, SocketException
    {
        poseChannel = SuperdroidPose.POSE_CHANNEL_BASE + name;
        final String velChannel = SuperdroidVelocities.VELOCITIES_CHANNEL_BASE + name;
        
        lcm.subscribe(velChannel, new LCMSubscriber()
        {
            public void messageReceived(LCM lcm, String channel,
                    LCMDataInputStream ins)
            {
                try
                {
                    differential_drive_command_t vels = new differential_drive_command_t(ins);
                    sd.setVelocities((float)vels.left, (float)vels.right);
                }
                catch (IOException e)
                {
                    e.printStackTrace();
                }
            }
        });
        sd = new GrrcSuperdroid(hostname, port);

        schexec.scheduleAtFixedRate(update, 0, (long)100, TimeUnit.MILLISECONDS);
    }
    
    private Runnable update = new Runnable()
    {
        public void run()
        {
            pose_t pose = new pose_t();
            rpy[2] = sd.getTheta();
            sd.getPos(pose.pos);
            pose.orientation = LinAlg.rollPitchYawToQuat(rpy);
            pose.utime = TimeUtil.utime();
            lcm.publish(poseChannel, pose);
        }
    };

    public void shutdown()
    {
        sd.shutdown();
        schexec.shutdown();
    }

}
