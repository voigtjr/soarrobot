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
package edu.umich.robot.packbot;

import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

import april.util.TimeUtil;

import com.google.common.util.concurrent.MoreExecutors;

import edu.umich.robot.Robot;
import edu.umich.robot.RobotConfiguration;
import edu.umich.robot.RobotOutput;
import edu.umich.robot.events.control.AbstractControlEvent;
import edu.umich.robot.events.control.AbstractDriveEvent;
import edu.umich.robot.events.control.DriveAngularEvent;
import edu.umich.robot.events.control.DriveEStopEvent;
import edu.umich.robot.events.control.DriveLinearEvent;
import edu.umich.robot.events.feedback.AbstractFeedbackEvent;
import edu.umich.robot.laser.Laser;
import edu.umich.robot.metamap.AreaDescription;
import edu.umich.robot.metamap.AreaState;
import edu.umich.robot.metamap.VirtualObject;
import edu.umich.robot.radio.Radio;
import edu.umich.robot.util.Pose;
import edu.umich.robot.util.Updatable;
import edu.umich.robot.util.events.RobotEventListener;
import edu.umich.robot.util.properties.PropertyManager;

/**
 * @author voigtjr@gmail.com
 */
public class PackBot implements Robot
{
    private final String name;

    private final int port;

    private final PackBotInterface velocities;

    private final ScheduledExecutorService schexec = MoreExecutors.getExitingScheduledExecutorService(new ScheduledThreadPoolExecutor(1));

    private Updatable controller;

    private long last;

    public PackBot(String name, String hostname, int port,
            PropertyManager properties) throws UnknownHostException,
            SocketException
    {
        this.name = name;
        this.port = port == 0 ? properties.get(PackBotProperties.PORT) : port;

        velocities = new PackBotInterface(hostname, this.port);

        long period = properties.get(PackBotProperties.UPDATE_PERIOD);
        schexec.scheduleAtFixedRate(command, 0, period, TimeUnit.MILLISECONDS);
    }

    private final Runnable command = new Runnable()
    {
        public void run()
        {
            synchronized (this)
            {
                if (controller != null)
                {
                    long now = TimeUtil.mstime();
                    if (last != 0)
                    {
                        double dt = (now - last) / (double)TimeUnit.MILLISECONDS.convert(1, TimeUnit.SECONDS);
                        controller.update(dt);
                    }
                    last = now;
                }
            }
        }
    };

    public RobotOutput getOutput()
    {
        return output;
    }

    private final RobotOutput output = new RobotOutput()
    {
        public AreaDescription getAreaDescription()
        {
            throw new UnsupportedOperationException("Not implemented");
        }

        public Laser getLaser()
        {
            throw new UnsupportedOperationException("Not implemented");
        }

        public Radio getRadio()
        {
            throw new UnsupportedOperationException("Not implemented");
        }

        public List<VirtualObject> getVisibleObjects()
        {
            throw new UnsupportedOperationException("Not implemented");
        }

        public Pose getPose()
        {
            throw new UnsupportedOperationException("Not implemented");
        }

        public VirtualObject getCarriedObject()
        {
            throw new UnsupportedOperationException("Not implemented");
        }

        public <T extends AbstractFeedbackEvent> void addFeedbackEventListener(
                Class<T> klass, RobotEventListener listener)
        {
            throw new UnsupportedOperationException("Not implemented");
        }

        public RobotConfiguration getRobotConfiguration()
        {
            return new RobotConfiguration.Builder()
            .build();
        }

        public <T extends AbstractFeedbackEvent> void removeFeedbackEventListener(
                Class<T> klass, RobotEventListener listener)
        {
            throw new UnsupportedOperationException("Not implemented");
        }

        public AreaState getAreaState()
        {
            throw new UnsupportedOperationException("Not implemented");
        }

        public double getBatteryLife()
        {
            throw new UnsupportedOperationException("Not implemented");
        }

        public boolean isHeadlightOn()
        {
            throw new UnsupportedOperationException("Not implemented");
        }

    };
    
    public void handleControlEvent(AbstractControlEvent event)
    {
        if (event instanceof AbstractDriveEvent)
        {
            if (event instanceof DriveEStopEvent)
            {
                estop();
            }
            else if (event instanceof DriveAngularEvent)
            {
                DriveAngularEvent d = (DriveAngularEvent)event;
                setAngular(d.getAngularVelocity());
            }
            else if (event instanceof DriveLinearEvent)
            {
                DriveLinearEvent d = (DriveLinearEvent)event;
                setLinear(d.getLinearVelocity());
            } 
            else 
            {
                throw new UnsupportedOperationException("Packbot does not support " + event);
            }
        }
        else 
        {
            throw new UnsupportedOperationException("Packbot does not support " + event);
        }
    }

    private void setAngular(double av)
    {
        synchronized (this)
        {
            if (controller != velocities)
                controller = velocities;
            velocities.setAngular((float) av);
        }
    }

    private void setLinear(double lv)
    {
        synchronized (this)
        {
            if (controller != velocities)
                controller = velocities;
            velocities.setLinear((float) lv);
        }
    }

    private void estop()
    {
        synchronized (this)
        {
            if (controller != velocities)
                controller = velocities;
            velocities.setAngular(0);
            velocities.setLinear(0);
        }
    }

    public String getName()
    {
        return name;
    }

    @Override
    public String toString()
    {
        return "Packbot: " + getName();
    }

    public void setTimeScale(int multiplier)
    {
        // TODO Auto-generated method stub
        
    }

    public void shutdown()
    {
        velocities.shutdown();
        schexec.shutdown();
    }

}
