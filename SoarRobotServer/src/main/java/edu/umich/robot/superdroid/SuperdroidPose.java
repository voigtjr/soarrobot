package edu.umich.robot.superdroid;

import april.jmat.LinAlg;
import april.lcmtypes.pose_t;
import april.util.TimeUtil;
import edu.umich.grrc.GrrcSuperdroid;
import edu.umich.robot.util.Poses;
import edu.umich.robot.util.Updatable;

public class SuperdroidPose implements Updatable
{
    private final GrrcSuperdroid sd;

    private final double[] rpy = new double[] { 0, 0, 0 };
    
    private pose_t pose;
    
    private pose_t elaboratedPose = new pose_t();

    public SuperdroidPose(GrrcSuperdroid sd)
    {
        this.sd = sd;
    }

    public void update(double dt)
    {
        pose_t newPose = new pose_t();
        rpy[2] = sd.getTheta();
        sd.getPos(newPose.pos);
        newPose.orientation = LinAlg.rollPitchYawToQuat(rpy);
        newPose.utime = TimeUtil.utime();
        pose = newPose;
    }
    
    public pose_t getPose()
    {
        pose_t newPose = pose;

        if (newPose != null && newPose.utime > elaboratedPose.utime)
            elaboratedPose = Poses.elaborate(elaboratedPose, newPose);
        
        return elaboratedPose.copy();
    }
}
