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
package edu.umich.robot;

import java.io.File;
import java.io.IOException;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import sml.Kernel;
import sml.smlSystemEventId;
import sml.Kernel.SystemEventInterface;
import april.config.Config;
import april.sim.SimLaser;
import april.sim.SimObject;
import april.sim.SimSplinter;
import april.sim.Simulator;
import april.util.TimeUtil;

import com.google.common.collect.Maps;

import edu.umich.robot.events.AbstractProgramEvent;
import edu.umich.robot.events.AfterResetEvent;
import edu.umich.robot.events.BeforeResetEvent;
import edu.umich.robot.events.RobotAddedEvent;
import edu.umich.robot.events.RobotRemovedEvent;
import edu.umich.robot.events.SoarStartedEvent;
import edu.umich.robot.events.SoarStoppedEvent;
import edu.umich.robot.events.TimeScaleChangedEvent;
import edu.umich.robot.events.control.AbstractControlEvent;
import edu.umich.robot.events.control.AbstractDriveEvent;
import edu.umich.robot.events.control.DriveEStopEvent;
import edu.umich.robot.gp.Gamepad;
import edu.umich.robot.metamap.Metamap;
import edu.umich.robot.metamap.MetamapFactory;
import edu.umich.robot.radio.Radio;
import edu.umich.robot.radio.SimRadio;
import edu.umich.robot.soar.Soar;
import edu.umich.robot.soar.SoarAgent;
import edu.umich.robot.soar.SoarDataCollector;
import edu.umich.robot.soar.SoarException;
import edu.umich.robot.splinter.Splinter;
import edu.umich.robot.util.Misc;
import edu.umich.robot.util.Pose;
import edu.umich.robot.util.events.RobotEventListener;
import edu.umich.robot.util.events.RobotEventManager;
import edu.umich.robot.util.properties.PropertyManager;

/**
 * @author voigtjr@gmail.com
 */
public class Controller
{
    private static final String GAMEPAD_NAME = "gamepad";

    private final RobotEventManager events = new RobotEventManager();

    private final RobotManager robots = new RobotManager(events);

    private final Map<String, RobotController> rcmap = new HashMap<String, RobotController>();

    private final Gamepad gp;

    private final Soar soar;

    private final Simulator sim;

    private final Radio radio = new SimRadio();

    private final Metamap metamap;

    private final ScalableRobotController gprc = new ScalableRobotController(GAMEPAD_NAME);

    private boolean gamepadOverride = false;

    private String selectedRobot;
    
    private static class SplinterData
    {
        public SplinterData(String name, Pose pose, boolean collisions)
        {
            this.name = name;
            this.initialPose = pose;
            this.collisions = collisions;
        }
        
        final String name;
        final Pose initialPose;
        final boolean collisions;
        SimSplinter ss;
        SimLaser sl;
    }
    
    private final Map<String, SplinterData> simSplinters = Maps.newConcurrentMap();

    public Controller(Config config, Gamepad gp)
    {
        this.gp = gp;
        if (gp != null)
            rcmap.put(gprc.getName(), gprc);
        
        soar = new Soar(config.getChild("soar"));
        soar.registerForSystemEvent(smlSystemEventId.smlEVENT_SYSTEM_START, soarHandler, null);
        soar.registerForSystemEvent(smlSystemEventId.smlEVENT_SYSTEM_STOP, soarHandler, null);
        
        sim = new Simulator(config);
        metamap = new MetamapFactory(config).build();
        events.addListener(RobotAddedEvent.class, metamap);
        events.addListener(RobotRemovedEvent.class, metamap);

        events.addListener(BeforeResetEvent.class, soar);
        events.addListener(AfterResetEvent.class, soar);
    }
    
    private final SystemEventInterface soarHandler = new Kernel.SystemEventInterface()
    {
        public void systemEventHandler(int eventId, Object arg1, Kernel arg2)
        {
            if (eventId == smlSystemEventId.smlEVENT_SYSTEM_START.swigValue())
                events.fireEvent(new SoarStartedEvent());
            else if (eventId == smlSystemEventId.smlEVENT_SYSTEM_STOP.swigValue())
                events.fireEvent(new SoarStoppedEvent());
        }
    };

    public void initializeGamepad()
    {
        if (gp != null)
            gp.initializeGamepad(this);
    }

    public void createSplinterRobot(String robotName, Pose pose, boolean collisions)
    {
        SplinterData sd = new SplinterData(robotName, pose, collisions);
        addSimSplinter(sd);
        simSplinters.put(robotName, sd);
        Robot splinter = new Splinter(robotName, radio, metamap);
        robots.addRobot(splinter);
    }

    private void addSimSplinter(SplinterData sd)
    {
        
        Config rconfig = new Config();
        rconfig.setString("class", "april.sim.SimSplinter");
        rconfig.setDoubles("initialPosition", Misc.toPrimitiveArray(sd.initialPose.getPos()));
        rconfig.setBoolean("wallCollisions", sd.collisions);
        SimObject ss = sim.addObject(sd.name, rconfig);
        if (ss != null && ss instanceof SimSplinter)
            sd.ss = (SimSplinter) ss;

        Config lconfig = new Config();
        lconfig.setString("class", "april.sim.SimLaser");
        lconfig.setString("pose", sd.name);
        lconfig.setDoubles("position", new double[] { 0, 0, 0.4 });
        lconfig.setDoubles("rollpitchyaw_degrees", new double[] { 0, 0, 0 });
        lconfig.setInts("color", new int[] { 1, 0, 0 });
        lconfig.setInt("degree0", -90);
        lconfig.setInt("degree_step", 1);
        lconfig.setInt("nranges", 180);
        lconfig.setDouble("range_noise_m", 0.01);
        lconfig.setDouble("theta_noise_degrees", 0.25);
        lconfig.setInt("max_range_m", 30);
        lconfig.setInt("hz", 7);
        SimObject sl = sim.addObject(sd.name + "lidar", lconfig);
        if (sl != null && sl instanceof SimLaser)
            sd.sl = (SimLaser) sl;
        
    }

    public void createSoarController(String rcName, String robotName, String productions, Config properties)
    {
        RobotOutput output = robots.getOutput(robotName);
        if (output == null)
            throw new IllegalArgumentException("No such robot: " + robotName);

        try
        {
            synchronized (rcmap)
            {
                RobotController rc = soar.createRobotController(rcName, output,
                        productions, properties);

                rcmap.put(rcName, rc);
                // remove gamepad
                if (robotName.equals(selectedRobot))
                {
                    if (gamepadOverride)
                    {
                        robots.popController(selectedRobot);
                        gamepadOverride = false;
                    }
                }
                robots.pushController(robotName, rc);
            }
        }
        catch (SoarException e)
        {
            e.printStackTrace();
            return;
        }
    }

    public void setDeadZonePercent(String component, float deadZonePercent)
    {
        gp.setDeadZonePercent(component, deadZonePercent);
    }

    public void selectRobot(String name)
    {
        synchronized (rcmap)
        {
            if (selectedRobot != null)
            {
                if (selectedRobot.equals(name))
                {
                    return;
                }
                if (gamepadOverride)
                {
                    robots.popController(selectedRobot);
                    gamepadOverride = false;
                }
            }
            selectedRobot = name;
            System.out.println(selectedRobot);
        }
    }

    public void toggleGamepadOverride()
    {
        if (gp == null)
            return;
        
        if (selectedRobot == null)
        {
            if (robots.getAll().isEmpty())
                return;

            for (Robot robot : robots.getAll())
            {
                selectRobot(robot.getName());
                break;
            }
        }

        synchronized (rcmap)
        {
            if (gamepadOverride)
            {
                fireGamepadControlEvent(DriveEStopEvent.INSTANCE);
                robots.popController(selectedRobot);
                gp.setRobotOutput(null);
                gamepadOverride = false;
            }
            else
            {
                robots.pushController(selectedRobot, gprc);
                gp.setRobotOutput(robots.getOutput(selectedRobot));
                gamepadOverride = true;
            }
        }
    }

    public void toggleSoarRunState()
    {
        soar.toggleRunState();
    }
    
    public void startSoar()
    {
        soar.startSoar(-1);
    }
    
    public void startSoar(int cycleLimit)
    {
        soar.startSoar(cycleLimit);
    }
    
    public void stopSoar()
    {
        soar.stopSoar();
    }
    
    private int rate = 1;
    public void toggleRate()
    {
        rate = rate * 2;
        if (rate > 4)
            rate = 1;
        
        TimeUtil.setTimeScale(rate);
        for(SplinterData sd : simSplinters.values())
            sd.ss.setTimeScale(rate);
        for (Robot r : robots.getAll())
            r.setTimeScale(rate);
        events.fireEvent(new TimeScaleChangedEvent(rate));
    }
    
    public int getRate()
    {
        return rate;
    }

    public void fireGamepadControlEvent(AbstractControlEvent event)
    {
        if (event instanceof AbstractDriveEvent)
            gprc.fireEvent(event, AbstractDriveEvent.class);
        else
            gprc.fireEvent(event, event.getClass());
    }

    public <T extends AbstractProgramEvent> void addListener(Class<T> klass,
            RobotEventListener listener)
    {
        events.addListener(klass, listener);
    }

    public <T extends AbstractProgramEvent> void removeListener(Class<T> klass,
            RobotEventListener listener)
    {
        events.removeListener(klass, listener);
    }

    public Collection<? extends Robot> getAllRobots()
    {
        return robots.getAll();
    }

    public void shutdown()
    {
        soar.shutdown();
        if (gp != null)
            gp.shutdown();
        robots.shutdown();
        sim.shutdown();
        metamap.shutdown();
    }

    public String getSelectedRobotName()
    {
	return selectedRobot;
    }

    public List<String> getObjectNames()
    {
        return metamap.getObjectNames();
    }

    public void addObject(String name, double[] pos)
    {
        metamap.addObject(name, pos);
    }

    public void reset()
    {
        events.fireEvent(new BeforeResetEvent());
        
        for (SplinterData sd : simSplinters.values())
        {
            sim.removeObject(sd.ss);
            sim.removeObject(sd.sl);
        }

        for (SplinterData sd : simSplinters.values())
            addSimSplinter(sd);
        
        metamap.reset();

        events.fireEvent(new AfterResetEvent());
    }

    public void saveMap(File selectedFile) throws IOException
    {
        ConfigSaver cs = new ConfigSaver(metamap, soar.getProperties());
        
        for (Robot r : robots.getAll())
        {
            if (r instanceof Splinter)
            {
                RobotController rc = rcmap.get(r.getName());
                if (rc != null && rc instanceof SoarAgent)
                    cs.addSplinter(r.getName(), r.getOutput().getPose().getPos(), (SoarAgent)rc);
                else
                    cs.addSplinter(r.getName(), r.getOutput().getPose().getPos(), null);
            }
        }
        
        cs.write(selectedFile);
    }

    public void soarToggle()
    {
        soar.toggleRunState();
    }

    public void soarStep()
    {
        soar.step();
    }

    public PropertyManager getSoarAgentProperties(String name)
    {
        return soar.getAgentProperties(name);
    }
    
    public PropertyManager getSoarProperties()
    {
        return soar.getProperties();
    }
    
    public SoarDataCollector getSoarDataCollector()
    {
        return soar.getSoarDataCollector();
    }

    public boolean hasSoarAgents()
    {
        return soar.hasSoarAgents();
    }

}
