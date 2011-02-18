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

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import edu.umich.robot.events.RobotAddedEvent;
import edu.umich.robot.events.RobotRemovedEvent;
import edu.umich.robot.util.events.RobotEventManager;

/**
 * @author voigtjr@gmail.com
 */
public class RobotManager
{
    private final Map<String, Robot> robots = new HashMap<String, Robot>();

    private final Map<String, OutputEventForwarder> forwarders = new HashMap<String, OutputEventForwarder>();

    private final RobotEventManager events;

    public RobotManager(RobotEventManager events)
    {
        this.events = events;
    }

    public void addRobot(Robot robot)
    {
        robots.put(robot.getName(), robot);
        events.fireEvent(new RobotAddedEvent(robot));
    }

    public void removeRobot(String name)
    {
        Robot robot = robots.remove(name);
        events.fireEvent(new RobotRemovedEvent(robot));
    }

    public void pushController(String name, RobotController controller)
    {
        OutputEventForwarder fw = forwarders.get(name);
        if (fw == null)
        {
            if (!robots.containsKey(name))
                throw new IllegalStateException("no such robot: " + name);
            fw = new OutputEventForwarder(robots.get(name), events);
            forwarders.put(name, fw);
        }
        fw.pushController(controller);
    }

    public void popController(String name)
    {
        OutputEventForwarder fw = forwarders.get(name);
        if (fw == null)
            throw new IllegalStateException("no such robot: " + name);
        fw.popController();
    }

    public Collection<? extends Robot> getAll()
    {
        return Collections.unmodifiableCollection(robots.values());
    }
    
    Robot get(String name)
    {
        return robots.get(name);
    }

    RobotOutput getOutput(String name)
    {
        Robot robot = robots.get(name);
        if (robot == null)
            throw new IllegalStateException("no such robot: " + name);
        return robot.getOutput();
    }
    
    public void shutdown()
    {
        for (Robot robot : robots.values())
            robot.shutdown();
    }
}
