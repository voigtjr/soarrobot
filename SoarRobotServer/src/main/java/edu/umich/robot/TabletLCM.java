package edu.umich.robot;

import java.io.IOException;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;

public class TabletLCM
{
    private final LCM input;
    private final LCM output;
    
    public TabletLCM(String connectionString) throws IOException, IllegalArgumentException
    {
        output = new LCM(connectionString);
        input = LCM.getSingleton();
        input.subscribe("POSE_seek", subscriber);
    }
    
    private final LCMSubscriber subscriber = new LCMSubscriber() 
    {
        public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins) 
        {
            try
            {
                output.publish(channel, ins.getBuffer(), ins.getBufferOffset(), ins.available());
            }
            catch (IOException e)
            {
                e.printStackTrace();
            }
            
        }
    };

    public void close()
    {
        input.unsubscribe("POSE_seek", subscriber);
        output.close();
    }
}
