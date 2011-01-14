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
package edu.umich.robot.soar;

import java.util.List;

import lcm.lcm.LCM;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import sml.Identifier;
import april.lcmtypes.waypoint_list_t;
import april.lcmtypes.waypoint_t;
import april.util.TimeUtil;

import com.google.common.collect.Lists;

import edu.umich.robot.RobotOutput;
import edu.umich.robot.metamap.AreaDescription;
import edu.umich.robot.metamap.AreaState;
import edu.umich.robot.metamap.Gateway;
import edu.umich.robot.metamap.Wall;
import edu.umich.soar.IntWme;
import edu.umich.soar.StringWme;

/**
 * Input link management for area description data.
 * 
 * @author voigtjr
 */
public class AreaDescriptionIL extends InputLinkElement
{
    private static final Log logger = LogFactory
            .getLog(AreaDescriptionIL.class);

    private final SoarAgent agent;

    private final IntWme idwme;

    private final StringWme typewme;

    private final StringWme lightwme;

    private final RobotOutput output;

    private final List<PointWithDistanceIL> pointDataList = Lists.newArrayList();

    private final List<Identifier> destroyList = Lists.newArrayList();

    private final String channel;
    
    private static final LCM lcm = LCM.getSingleton();
    
    private boolean warned = false;

    public AreaDescriptionIL(SoarAgent agent)
    {
        super(agent, IOConstants.AREA_DESCRIPTION, agent.getSoarAgent().GetInputLink());

        this.agent = agent;
        this.channel = "WAYPOINTS_" + agent.getName();

        idwme = IntWme.newInstance(getRoot(), IOConstants.ID, -1);
        typewme = StringWme.newInstance(getRoot(), IOConstants.TYPE);
        lightwme = StringWme.newInstance(getRoot(), IOConstants.LIGHT);
        output = agent.getRobotOutput();

        update();
    }

    private void newArea(AreaDescription ad)
    {
        idwme.update(ad.getId());
        logger.trace(idwme);
        
        String type = ad.getProperties().get("type");
        typewme.update(type == null ? IOConstants.ROOM : type);
        logger.trace(typewme);
        
        waypoint_list_t lcmwps = new waypoint_list_t();
        lcmwps.utime = TimeUtil.utime();
        List<waypoint_t> lcmtemplist = Lists.newArrayList();
        
        // gateways
        for (int i = 0; i < ad.getGateways().size(); ++i)
        {
            Gateway gateway = ad.getGateways().get(i);
            String dir = gateway.getDirection(ad).toString().toLowerCase();
            if (logger.isTraceEnabled())
                logger.trace(dir + ":" + gateway);
            
            waypoint_t wp = new waypoint_t();
            wp.utime = lcmwps.utime;
            wp.xLocal = gateway.getPose().getX();
            wp.yLocal = gateway.getPose().getY();
            wp.tLocal = gateway.getPose().getYaw();
            lcmtemplist.add(wp);

            Identifier gatewaywme = agent.getSoarAgent().CreateIdWME(getRoot(), IOConstants.GATEWAY);
            
            Identifier doorwme = agent.getSoarAgent().CreateIdWME(gatewaywme, IOConstants.DOOR);
            IntWme.newInstance(doorwme, IOConstants.ID, gateway.getDoor().getId());
            StringWme.newInstance(doorwme, IOConstants.STATE, gateway.getDoor().getState().toString().toLowerCase());
            
            PointWithDistanceIL pointData = new PointWithDistanceIL(gatewaywme, agent, gateway.getPose());
            StringWme.newInstance(gatewaywme, IOConstants.DIRECTION, dir.toString().toLowerCase());

            IntWme.newInstance(gatewaywme, IOConstants.ID, gateway.getId());
            for (Integer to : gateway.getTo())
                IntWme.newInstance(gatewaywme, IOConstants.TO, to);

            pointDataList.add(pointData);
            destroyList.add(gatewaywme);
        }

        // walls
        for (Wall w : ad.getWalls())
        {
            waypoint_t wp = new waypoint_t();
            wp.utime = lcmwps.utime;
            wp.xLocal = w.getMidpoint().getX();
            wp.yLocal = w.getMidpoint().getY();
            wp.tLocal = w.getMidpoint().getYaw();
            lcmtemplist.add(wp);

            Identifier wallwme = agent.getSoarAgent().CreateIdWME(getRoot(), IOConstants.WALL);
            PointWithDistanceIL pointData = new PointWithDistanceIL(wallwme, agent, w.getMidpoint());

            String dir = w.getDirection().toString().toLowerCase();
            StringWme.newInstance(wallwme, IOConstants.DIRECTION, dir);

            boolean open = false;
            for (Integer id : w.getTo())
            {
                open = true;
                IntWme.newInstance(wallwme, IOConstants.TO, id);
            }
            StringWme.newInstance(wallwme, IOConstants.OPEN, Boolean.toString(open));
            
            pointDataList.add(pointData);
            destroyList.add(wallwme);
        }

        lcmwps.waypoints = lcmtemplist.toArray(new waypoint_t[lcmtemplist.size()]);
        lcmwps.nwaypoints = lcmwps.waypoints.length;
        
        lcm.publish(channel, lcmwps);
    }

    @Override
    public void update()
    {
        AreaDescription ad = output.getAreaDescription();
        AreaState as = output.getAreaState();

        if (ad != null)
        {
            warned = false;
            if (ad.getId() != idwme.getValue())
            {
                for (Identifier old : destroyList)
                    old.DestroyWME();
                destroyList.clear();
                pointDataList.clear();

                newArea(ad);
            }
            
            updateLight(as);
        }
        else
        {
            if (logger.isTraceEnabled() && !warned)
            {
                logger.trace("No area description");
                warned = true;
            }
        }

        for (PointWithDistanceIL pointData : pointDataList)
            pointData.update();
    }
    
    private void updateLight(AreaState as)
    {
        lightwme.update(as.isLit(false) ? IOConstants.TRUE : IOConstants.FALSE);
    }

}