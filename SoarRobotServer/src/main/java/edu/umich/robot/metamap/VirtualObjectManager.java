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

import java.util.Collections;
import java.util.List;
import java.util.Map;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import april.config.Config;
import april.jmat.LinAlg;
import april.lcmtypes.pose_t;

import com.google.common.collect.ImmutableList;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;

import edu.umich.robot.Robot;
import edu.umich.robot.util.Pose;

/**
 * @author voigtjr@gmail.com
 */
class VirtualObjectManager
{
    private static final Log logger = LogFactory.getLog(VirtualObjectManager.class);

    private static final double MANIPULATION_DISTANCE = 1; // TODO pull from properties?
    
    private static final double SQUARED_MANIPULATION_DISTANCE = LinAlg.sq(MANIPULATION_DISTANCE);

    private final Map<String, VirtualObjectTemplate> templates = Maps.newHashMap();

    private final Map<Integer, VirtualObjectImpl> instances = Maps.newHashMap();
    
    private final List<VirtualObjectImpl> placed = Lists.newArrayList();
    
    private final Map<Robot, VirtualObjectImpl> carried = Maps.newHashMap();
    
    private final ObstacleBroadcaster obc = new ObstacleBroadcaster();
    
    private final Config config;
    
    private final IdGenerator idg;
    
    VirtualObjectManager(Config config, IdGenerator idg)
    {
        this.idg = idg;
        this.config = (config != null) ? config.copy() : null;
        initialize();
    }
    
    private void initialize()
    {
        String[] placedString = null;
        for (String key : config.getKeys())
        {
            if (key.equals("placed"))
            {
                placedString = config.requireStrings("placed");
                continue;
            }
            
            String objectNickname = key.split("\\.", 2)[0];
            if (templates.containsKey(objectNickname))
                continue;
            addTemplate(objectNickname, config.getChild(objectNickname));
        }
        
        placeInstances(placedString);
    }
    
    private void addTemplate(String name, Config config)
    {
        templates.put(name, new VirtualObjectTemplate(name, config));
    }
    
    List<String> getObjectNames()
    {
        return Lists.newArrayList(templates.keySet());
    }
    
    private void placeInstances(String[] placedString)
    {
        if (placedString == null)
            return;
        
        try
        {
            for (int i = 0; i < placedString.length - 2; i += 3)
            {
                Pose pose = new Pose(new double[] { 
                        Double.parseDouble(placedString[i + 1]), 
                        Double.parseDouble(placedString[i + 2]) });
                placeNew(placedString[i], pose);
            }
        } 
        catch (NumberFormatException e)
        {
            logger.error("Number format exception placing object.");
        }
    }
    
    void placeNew(String name, Pose pose)
    {
        VirtualObjectTemplate template = templates.get(name);
        if (template == null)
        {
            logger.error("No template for " + name);
            return;
        }
        
        VirtualObjectImpl vo = new VirtualObjectImpl(name, template, idg.getId());
        vo.setPose(pose);
        
        instances.put(vo.getId(), vo);
        addPlaced(vo);
    }
    
    private void addPlaced(VirtualObjectImpl voi)
    {
        logger.debug("Placed " + voi.toString());

        placed.add(voi);
        if (obc != null)
            obc.addVirtualObject(voi);
    }
    
    private void removePlaced(VirtualObjectImpl voi)
    {
        placed.remove(voi);
        if (obc != null)
            obc.removeVirtualObject(voi);
    }
    
    List<VirtualObjectImpl> getObjects(int area)
    {
        return Collections.unmodifiableList(placed);
    }
    
    private boolean isInRange(Robot robot, VirtualObject object)
    {
        pose_t rp = robot.getOutput().getPose().asLcmType();
        pose_t vp = object.getPose().asLcmType();
        
        double squaredDistance = LinAlg.squaredDistance(rp.pos, vp.pos);
        
        // decrease the distance by object's radius
        // estimating radius by largest side
        double diameter = Math.max(object.getSize().get(0), object.getSize().get(1));
        
        return (squaredDistance - LinAlg.sq(diameter / 2)) <= SQUARED_MANIPULATION_DISTANCE;
    }

    boolean pickupObject(Robot robot, int id)
    {
        if (!carried.containsKey(robot))
        {
            VirtualObjectImpl voi = instances.get(Integer.valueOf(id));
            if (voi != null && placed.contains(voi))
            {
                if (isInRange(robot, voi))
                {
                    removePlaced(voi);
                    carried.put(robot, voi);
                    return true;
                }
            }
        }
        
        return false;   
    }

    boolean dropObject(Robot robot)
    {
        // TODO placement restrictions
        
        VirtualObjectImpl voi = carried.get(robot);
        if (voi != null)
        {
            pose_t rp = robot.getOutput().getPose().asLcmType();
            pose_t vp = voi.getPose().asLcmType();
            double[] dpos = LinAlg.quatRotate(rp.orientation, new double[] { MANIPULATION_DISTANCE, 0, 0 });
            vp.pos = LinAlg.add(rp.pos, dpos);
            voi.setPose(new Pose(vp));
            addPlaced(voi);
            carried.remove(robot);
            return true;
        }
        
        return false;
    }

    boolean diffuseObject(Robot robot, int id)
    {
        VirtualObject vo = instances.get(id);
        if (vo != null && placed.contains(vo) && isInRange(robot, vo))
        {
            VirtualObjectImpl voi = (VirtualObjectImpl) vo;
            if (voi.isDiffusable())
                voi.setDiffused(true);
        }
        return false;
    }

    boolean diffuseObjectByWire(Robot robot, int id, String color)
    {
        VirtualObject vo = instances.get(id);
        if (vo != null && placed.contains(vo) && isInRange(robot, vo))
        {
            VirtualObjectImpl voi = (VirtualObjectImpl)vo;
            if (voi.isDiffusableByColor(color))
                voi.setDiffused(true);
        }
        return false;
    }

    VirtualObject getCarried(Robot robot)
    {
        return carried.get(robot);
    }

    void reset()
    {
        obc.reset();
        templates.clear();
        placed.clear();
        carried.clear();
        instances.clear();
        idg.rewind();
        initialize();
    }

    ObstacleBroadcaster getBroadcaster()
    {
        return obc;
    }

    public VirtualObject getTemplate(String name)
    {
        return templates.get(name);
    }

    public List<VirtualObject> getPlacedObjects()
    {
        ImmutableList.Builder<VirtualObject> b = new ImmutableList.Builder<VirtualObject>();
        b.addAll(placed);
        return b.build();
    }
    
    public void shutdown()
    {
        obc.shutdown();
    }
}
