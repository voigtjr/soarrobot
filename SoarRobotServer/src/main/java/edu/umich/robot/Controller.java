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
import edu.umich.robot.network.Server;
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
 * <p>
 * Main container and hub for components and events.
 * 
 * @author voigtjr@gmail.com
 */
public class Controller
{
    /**
     * <p>
     * Used to identify controller in controller list.
     */
    private static final String GAMEPAD_NAME = "gamepad";

    /**
     * <p>
     * Event manager producing events extending AbstractProgramEvent
     */
    private final RobotEventManager events = new RobotEventManager();

    /**
     * <p>
     * Manages robots and controllers for the robots. Provides a stack-like
     * interface for switching controllers.
     */
    private final RobotManager robots = new RobotManager(events);

    /**
     * <p>
     * List of robot controller names, mapped to the controllers.
     */
    private final Map<String, RobotController> rcmap = new HashMap<String, RobotController>();

    private final Gamepad gp;

    private final Soar soar;

    private final Simulator sim;

    private final Radio radio = new SimRadio();

    private final Metamap metamap;

    /**
     * <p>
     * The gamepad device robot controller.
     */
    private final ScalableRobotController gprc = new ScalableRobotController(GAMEPAD_NAME);

    /**
     * <p>
     * True when the gamepad is enabled and overriding whatever robot it is
     * currently selected to control.
     */
    private boolean gamepadOverride = false;

    private String selectedRobot;
    
    private Server server;
    
    /**
     * <p>
     * Data structure representing a splinter, its initial conditions, and
     * references to its simulator objects. These simulator objects references
     * are saved so that they can be removed later.
     * 
     * @author voigtjr
     * 
     */
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
    
    /**
     * <p>
     * Maps names to splinter information.
     */
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
        
        try {
			server = new Server(12122);
			server.setController(this);
			server.start();
		} catch (IOException e) {
			e.printStackTrace();
		}
    }
    
    /**
     * <p>
     * Causes Soar start and stop events to fire program control events.
     */
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

    /**
     * <p>
     * Creates a robot and adds it to the simulation.
     * 
     * @param robotName
     *            The name of the robot, must be unique among all
     * @param pose
     *            Initial starting position
     * @param collisions
     *            True to allow collisions with walls.
     */
    public void createSplinterRobot(String robotName, Pose pose, boolean collisions)
    {
        SplinterData sd = new SplinterData(robotName, pose, collisions);
        addSimSplinter(sd);
        simSplinters.put(robotName, sd);
        Robot splinter = new Splinter(robotName, radio, metamap);
        robots.addRobot(splinter);
    }

    /**
     * <p>
     * Elaborates a splinter configuration and adds the splinter to the
     * simulator using that configuration.
     * 
     * <p>
     * Also creates a configuration and uses it to create a simulated laser
     * sensor for the robot.
     * 
     * @param sd
     */
    private void addSimSplinter(SplinterData sd)
    {

        Config rconfig = new Config();
        rconfig.setString("class", "april.sim.SimSplinter");
        rconfig.setDoubles("initialPosition", Misc
                .toPrimitiveDoubleArray(sd.initialPose.getPos()));
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

    /**
     * <p>
     * Create a robot controller with a name for a certain robot (by name) using
     * productions (or not) and with other specified controller/agent
     * properties.
     * 
     * @param rcName
     *            The name for the new robot controller, must be unique among
     *            robot controllers but can share names with the robots. Usually
     *            the same name as the robot it is controlling.
     * @param robotName
     *            The robot's name to control.
     * @param productions
     *            Productions for the agent, or null for none.
     * @param properties
     *            Properties to be passed to the controller, see
     *            soar.createRobotController
     */
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

    /**
     * <p>
     * Tweak dead zone for a gamepad axis. See Gamepad.
     * 
     * @param component
     * @param deadZonePercent
     */
    public void setDeadZonePercent(String component, float deadZonePercent)
    {
        gp.setDeadZonePercent(component, deadZonePercent);
    }

    /**
     * <p>
     * Select a robot by name. Does nothing if the currently selected robot is
     * selected again. Deactivates gamepad if a new robot is selected.
     * 
     * @param name
     *            Robot to select.
     */
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

    /**
     * <p>
     * Toggle whether or not the gamepad is controlling the currently selected
     * robot.
     */
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

    /**
     * <p>
     * Toggles the current Soar running state.
     * @return True if Soar is now running.
     */
    public boolean toggleSoarRunState()
    {
        return soar.toggleRunState();
    }
    
    /**
     * <p>
     * Start Soar, run forever.
     */
    public void startSoar()
    {
        soar.startSoar(-1);
    }
    
    /**
     * <p>
     * Start Soar and run a limited number of cycles.
     * 
     * @param cycleLimit
     *            Number of cycles to run
     */
    public void startSoar(int cycleLimit)
    {
        soar.startSoar(cycleLimit);
    }
    
    /**
     * <p>
     * Request to stop Soar.
     */
    public void stopSoar()
    {
        soar.stopSoar();
    }
    
    /**
     * <p>
     * Time scale rate. 1 = real time, 2 = 2x real time.
     */
    private int rate = 1;
    
    /**
     * <p>
     * Cycle the rate 1 to 2 to 4 back to 1.
     */
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
    
    /**
     * <p>
     * Get the current time scale rate.
     * 
     * @return 1, 2, or 4
     */
    public int getRate()
    {
        return rate;
    }

    /**
     * <p>
     * Used to fire gamepad control events.
     * 
     * @param event
     *            The event instance to fire.
     */
    public void fireGamepadControlEvent(AbstractControlEvent event)
    {
        if (event instanceof AbstractDriveEvent)
            gprc.fireEvent(event, AbstractDriveEvent.class);
        else
            gprc.fireEvent(event, event.getClass());
    }

    /**
     * <p>
     * Add a listener for an event type.
     * 
     * @param <T>
     *            The event type.
     * @param klass
     *            The event type.
     * @param listener
     *            The listener callback.
     */
    public <T extends AbstractProgramEvent> void addListener(Class<T> klass,
            RobotEventListener listener)
    {
        events.addListener(klass, listener);
    }

    /**
     * <p>
     * Remove a previously registered listener by reference id.
     * 
     * @param <T>
     *            The event type.
     * @param klass
     *            The event type.
     * @param listener
     *            The listener to remove.
     */
    public <T extends AbstractProgramEvent> void removeListener(Class<T> klass,
            RobotEventListener listener)
    {
        events.removeListener(klass, listener);
    }

    /**
     * <p>
     * Return an unmodifiable collection of all robots.
     * 
     * @return Unmodifiable collection of robots.
     */
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

    /**
     * <p>
     * Returns the name of the currently selected robot, or null.
     * 
     * @return The name of the currently selected robot, or null.
     */
    public String getSelectedRobotName()
    {
        return selectedRobot;
    }

    /**
     * <p>
     * Returns a list of all currently registered object prototypes.
     * 
     * @return A list of all currently registered object prototypes.
     */
    public List<String> getObjectNames()
    {
        return metamap.getObjectNames();
    }

    /**
     * <p>
     * Add an object instance by name.
     * 
     * @param name The object type, see getObjectNames
     * @param pos Where to put the new object.
     */
    public void addObject(String name, double[] pos)
    {
        metamap.addObject(name, pos);
    }

    /**
     * <p>
     * Deletes splinters and recreates them in their initial locations and fires
     * reset events.
     */
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

    /**
     * <p>
     * Take the currently configured map and write it out to a file.
     * 
     * @param selectedFile The file to overwrite if it exists.
     * @throws IOException If there was some file error.
     */
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

    /**
     * <p>
     * Retrieve a Soar agent's properties.
     * 
     * @param name
     *            The agent to retrieve.
     * @return The property manager for the agent.
     */
    public PropertyManager getSoarAgentProperties(String name)
    {
        return soar.getAgentProperties(name);
    }

    /**
     * <p>
     * Retrieve Soar (kernel-level) properties.
     * 
     * @return The property manager for the Soar instance.
     */
    public PropertyManager getSoarProperties()
    {
        return soar.getProperties();
    }
    
    /**
     * <p>
     * Retrieve the data collector class for Soar.
     * 
     * @return The data collector class.
     */
    public SoarDataCollector getSoarDataCollector()
    {
        return soar.getSoarDataCollector();
    }

    /**
     * Check to see if there are Soar agents.
     * 
     * @return true if there are Soar agents.
     */
    public boolean hasSoarAgents()
    {
        return soar.hasSoarAgents();
    }

}
