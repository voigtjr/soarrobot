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
package edu.umich.robot.metamap;

import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

import lcm.lcm.LCM;
import april.lcmtypes.map_metadata_t;
import april.util.TimeUtil;

import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.util.concurrent.MoreExecutors;

import edu.umich.robot.RelativePose;
import edu.umich.robot.Robot;
import edu.umich.robot.events.RobotAddedEvent;
import edu.umich.robot.events.RobotRemovedEvent;
import edu.umich.robot.metamap.Door.State;
import edu.umich.robot.util.Pose;
import edu.umich.robot.util.events.RobotEvent;
import edu.umich.robot.util.events.RobotEventListener;

/**
 * @author voigtjr@gmail.com
 */
public class Metamap implements RobotEventListener
{
    private static final double HALF_FOV = Math.PI / 2;

    private final List<AreaDescription> areaList;

    private final VirtualObjectManager objects;
    
    private final Map<Integer, AreaState> areaStates = Maps.newConcurrentMap();
    
    private final Map<Integer, Door> doors = Maps.newConcurrentMap();
    private final Map<Integer, Door> initialDoorState = Maps.newConcurrentMap();
    
    private final List<VirtualObject> robots = Lists.newArrayList();
    
    private final ScheduledExecutorService schexec = MoreExecutors.getExitingScheduledExecutorService(new ScheduledThreadPoolExecutor(1));
    
    private final String imagePath;
    private final double metersPerPixel;
    private final int[] origin;
   
    private final IdGenerator idg;
    
    Metamap(String imagePath, double metersPerPixel, int[] origin, 
            List<AreaDescription> areaList, VirtualObjectManager objects, Map<Integer, Door> doors, IdGenerator idg)
    {
        this.idg = idg;
        this.imagePath = imagePath;
        this.metersPerPixel = metersPerPixel;
        this.origin = Arrays.copyOf(origin, origin.length);
        
        this.areaList = areaList;
        this.objects = objects;
        this.doors.putAll(doors);

        for (Door door : doors.values())
            initialDoorState.put(door.getId(), door.copy());
        broadcastDoors();
        
        final map_metadata_t meta = new map_metadata_t();
        meta.nareas = areaList.size();
        meta.areas = new double[areaList.size()][4];
        List<Gateway> gatewayList = Lists.newArrayList();
        for (int i = 0; i < areaList.size(); ++i)
        {
            AreaDescription ad = areaList.get(i);
            meta.areas[i][0] = ad.getPose().getPos(0);
            meta.areas[i][1] = ad.getPose().getPos(1);
            meta.areas[i][2] = ad.getPose().getVel().get(0);
            meta.areas[i][3] = ad.getPose().getVel().get(1);
            gatewayList.addAll(ad.getGateways());
        }
        
        meta.ngateways = gatewayList.size();
        meta.gateway_ids = new int[gatewayList.size()];
        meta.gateways = new double[gatewayList.size()][2];
        for (int i = 0; i < gatewayList.size(); ++i)
        {
            meta.gateway_ids[i] = gatewayList.get(i).getId();
            meta.gateways[i][0] = gatewayList.get(i).getPose().getPos(0);
            meta.gateways[i][1] = gatewayList.get(i).getPose().getPos(1);
        }

        schexec.scheduleAtFixedRate(new Runnable()
        {
            final LCM lcm = LCM.getSingleton();
            
            public void run()
            {
                meta.utime = TimeUtil.utime();
                lcm.publish("AREA_DESCRIPTIONS", meta);
            }
        }, 0, 1, TimeUnit.SECONDS);
    }

    private void broadcastDoors()
    {
        for (Door door : doors.values())
            if (door.getState() != State.OPEN)
                objects.getBroadcaster().addDoor(door);
    }

    public AreaDescription getAreaFromId(int id)
    {
        if (id >= areaList.size())
            return null;
        return areaList.get(id);
    }
    
    public AreaDescription getArea(Pose pose)
    {
        // TODO optimize
        for (AreaDescription area : areaList)
        {
            Pose ap = area.getPose();
            if (pose.getX() < ap.getX())
                continue;
            if (pose.getY() < ap.getY())
                continue;
            if (pose.getX() >= ap.getX() + ap.getVX())
                continue;
            if (pose.getY() >= ap.getY() + ap.getVY())
                continue;
            return area;
        }
        return null;
    }

    public AreaState getAreaState(int id)
    {
        AreaState ret = areaStates.get(id);
        if (ret == null)
        {
            ret = new AreaState();
            areaStates.put(id, ret);
        }
        return ret;
    }

    public void setRoomLight(int id, boolean on)
    {
        AreaState as = areaStates.get(id);
        if (as == null)
            return;
        as.setRoomLightOn(on);
    }
    
    private int getAreaId(Pose pose)
    {
        AreaDescription area = getArea(pose);
        if (area == null)
            return -1;
        return area.getId();
    }

    public List<VirtualObject> getVisibleObjects(Robot robot)
    {
        AreaDescription area = getArea(robot.getOutput().getPose());
        if (area == null)
            return Lists.newArrayListWithCapacity(0);
        
        List<VirtualObjectImpl> placed = objects.getObjects(area.getId());
        AreaDescription ra = getArea(robot.getOutput().getPose());
        AreaState rs = getAreaState(ra.getId());
        
        List<VirtualObject> ret = Lists.newArrayList();

        // TODO night
        if (!rs.isLit(true) && !robot.getOutput().isHeadlightOn())
            return ret;
        
        // TODO optimize
        for (VirtualObject vo : placed)
        {
            if (isVisible(robot.getOutput().getPose(), vo.getPose()))
                ret.add(vo);
        }

        for (VirtualObject vo : robots)
        {
            if (isVisible(robot.getOutput().getPose(), vo.getPose()))
                ret.add(vo);
        }
        
        return ret;
    }
    
    private boolean isVisible(Pose myPose, Pose otherPose)
    {
        int myAreaId = getAreaId(myPose);
        int otherAreaId = getAreaId(otherPose);
        if (otherAreaId == myAreaId)
        {
            RelativePose rp = myPose.getRelative(otherPose);
            // ignore self
            if (Double.compare(rp.getDistance(), Double.valueOf(0)) == 0)
                    return false;
            double ayaw = Math.abs(rp.getRelativeYaw());
            return HALF_FOV >= ayaw;
        }
        return false;
    }

    public VirtualObject getCarried(Robot robot)
    {
        return objects.getCarried(robot);
    }

    public boolean cancelEffector(Robot robot)
    {
        // nothing to do, everything happens instantaneously right now
        return true;
    }

    public boolean pickupObject(Robot robot, int id)
    {
        return objects.pickupObject(robot, id);
    }

    public boolean dropObject(Robot robot)
    {
        return objects.dropObject(robot);
    }

    public boolean diffuseObject(Robot robot, int id)
    {
        return objects.diffuseObject(robot, id);
    }

    public boolean diffuseObjectByWire(Robot robot, int id, String color)
    {
        return objects.diffuseObjectByWire(robot, id, color);
    }

    public void doorOpen(Robot robot, int id)
    {
        Door door = doors.get(id);
        if (door == null)
            return;

        switch (door.getState())
        {
        case CLOSED:
            door.setState(State.OPEN);
            objects.getBroadcaster().removeDoor(door);
            break;
            
        case LOCKED:
        case OPEN:
            break;
        }
    }

    public void doorClose(Robot robot, int id)
    {
        Door door = doors.get(id);
        if (door == null)
            return;

        switch (door.getState())
        {
        case OPEN:
            door.setState(State.CLOSED);
            objects.getBroadcaster().addDoor(door);
            break;
            
        case LOCKED:
        case CLOSED:
            break;
        }
    }

    public void doorUnlock(Robot robot, int id, int code)
    {
        Door door = doors.get(id);
        if (door == null)
            return;

        switch (door.getState())
        {
        case LOCKED:
            if (door.getCode() == code)
                door.setState(State.CLOSED);
            break;
            
        case OPEN:
        case CLOSED:
            break;
        }
    }

    public void doorLock(Robot robot, int id, int code)
    {
        Door door = doors.get(id);
        if (door == null)
            return;

        switch (door.getState())
        {
        case CLOSED:
            door.setCode(code);
            door.setState(State.LOCKED);
            break;
            
        case OPEN:
        case LOCKED:
            break;
        }
    }

    public List<String> getObjectNames()
    {
        return objects.getObjectNames();
    }

    public void addObject(String name, double[] pos)
    {
        objects.placeNew(name, new Pose(pos));
    }

    public void onEvent(RobotEvent event)
    {
        if (event instanceof RobotAddedEvent)
        {
            RobotAddedEvent e = (RobotAddedEvent)event;
            
            robots.add(new VirtualObjectRobot(e.getRobot(), idg.getId()));
        }
        else if (event instanceof RobotRemovedEvent)
        {
            RobotAddedEvent e = (RobotAddedEvent)event;
            
            for (Iterator<VirtualObject> iter = robots.iterator(); iter.hasNext();)
            {
                VirtualObject o = iter.next();
                if (o.getProperties().get("name").equals(e.getRobot().getName()))
                {
                    iter.remove();
                    break;
                }
            }
        }
    }
    
    public void reset()
    {
        areaStates.clear();
        objects.reset();

        for (Door door : doors.values())
        {
            Door other = initialDoorState.get(door.getId());
            door.setState(other.getState());
            if (other.getState() == Door.State.LOCKED)
                door.setCode(other.getCode());
        }
        
        broadcastDoors();
    }

    public VirtualObject getTemplate(String name)
    {
        return objects.getTemplate(name);
    }

    public List<VirtualObject> getPlacedObjects()
    {
        return objects.getPlacedObjects();
    }

    public String getImagePath()
    {
        return imagePath;
    }

    public double getMetersPerPixel()
    {
        return metersPerPixel;
    }

    public int[] getImageOrigin()
    {
        return Arrays.copyOf(origin, origin.length);
    }
    
    public void shutdown()
    {
        schexec.shutdown();
        objects.shutdown();
    }
}
