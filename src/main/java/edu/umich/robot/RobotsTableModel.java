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

import java.util.ArrayList;
import java.util.List;

import javax.swing.SwingUtilities;
import javax.swing.table.AbstractTableModel;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import edu.umich.robot.events.ControllerActivatedEvent;
import edu.umich.robot.events.ControllerDeactivatedEvent;
import edu.umich.robot.events.RobotAddedEvent;
import edu.umich.robot.events.RobotRemovedEvent;
import edu.umich.robot.util.events.RobotEvent;
import edu.umich.robot.util.events.RobotEventListener;

/**
 * @author voigtjr@gmail.com
 */
public class RobotsTableModel extends AbstractTableModel
{
    private static final Log logger = LogFactory.getLog(RobotsTableModel.class);

    private static final long serialVersionUID = 1466373161707709390L;

    private final Controller controller;

    private final List<Robot> robots = new ArrayList<Robot>();

    public static final int ROBOT_COL = 0;

    public static final int CONTROL_COL = 1;

    /**
     * synchronized by field robots
     */
    private final List<RobotController> controllers = new ArrayList<RobotController>();

    public RobotsTableModel(Controller controller)
    {
        this.controller = controller;
        this.controller.addListener(RobotAddedEvent.class, listener);
        this.controller.addListener(RobotRemovedEvent.class, listener);
        this.controller.addListener(ControllerActivatedEvent.class, listener);
        this.controller.addListener(ControllerDeactivatedEvent.class, listener);

        robots.addAll(controller.getAllRobots());
        for (Robot robot : robots)
        {
            robot.getName(); // shuts up unused var warning
            controllers.add(NullController.INSTANCE);
        }
    }

    private void handleRobotAdded(Robot robot)
    {
        int row = 0;
        synchronized (robots)
        {
            row = robots.size();
            robots.add(robot);
            controllers.add(NullController.INSTANCE);
        }
        fireTableRowsInserted(row, row);
    }

    private void handleRobotRemoved(Robot robot)
    {
        int row = 0;
        synchronized (robots)
        {
            row = robots.indexOf(robot);
            if (row == -1)
            {
                return;
            }
            robots.remove(row);
            controllers.remove(row);
        }
        fireTableRowsDeleted(row, row);
    }

    private void handleControllerDeactivated(Robot robot)
    {
        int row = -1;
        synchronized (robots)
        {
            row = robots.indexOf(robot);
            if (row == -1)
            {
                logger.error("handleControllerDeactivated unknown robot "
                        + robot);
                return;
            }
            controllers.set(row, NullController.INSTANCE);
        }
        fireTableRowsUpdated(row, row);
    }

    private void handleControllerActivated(Robot robot,
            RobotController controller)
    {
        int row = -1;
        synchronized (robots)
        {
            row = robots.indexOf(robot);
            if (row == -1)
            {
                logger.error("handleControllerDeactivated unknown robot "
                        + robot);
                return;
            }
            controllers.set(row, controller);
        }
        fireTableRowsUpdated(row, row);
    }

    RobotEventListener listener = new RobotEventListener()
    {
        public void onEvent(final RobotEvent event)
        {
            Runnable runnable = new Runnable()
            {
                public void run()
                {
                    if (event instanceof RobotAddedEvent)
                    {
                        handleRobotAdded(((RobotAddedEvent) event).getRobot());
                    }
                    else if (event instanceof RobotRemovedEvent)
                    {
                        handleRobotRemoved(((RobotRemovedEvent) event)
                                .getRobot());
                    }
                    else if (event instanceof ControllerActivatedEvent)
                    {
                        ControllerActivatedEvent e = (ControllerActivatedEvent) event;
                        handleControllerActivated(e.getRobot(), e
                                .getController());
                    }
                    else if (event instanceof ControllerDeactivatedEvent)
                    {
                        ControllerDeactivatedEvent e = (ControllerDeactivatedEvent) event;
                        handleControllerDeactivated(e.getRobot());
                    }
                    else
                    {
                        throw new AssertionError("unhandled event " + event);
                    }
                }

            };
            if (SwingUtilities.isEventDispatchThread())
            {
                runnable.run();
            }
            else
            {
                SwingUtilities.invokeLater(runnable);
            }
        }
    };
    
    public int getRowByName(String robotName)
    {
        for (int i = 0; i < robots.size(); ++i)
        {
            if (robots.get(i).getName().equals(robotName))
                return i;
        }
        return -1;
    }

    @Override
    public Class<?> getColumnClass(int c)
    {
        switch (c)
        {
        case ROBOT_COL:
            return Robot.class;
        case CONTROL_COL:
            return RobotController.class;
        }
        return super.getColumnClass(c);
    }

    @Override
    public String getColumnName(int c)
    {
        switch (c)
        {
        case ROBOT_COL:
            return "Name";
        case CONTROL_COL:
            return "Controller";
        }
        return super.getColumnName(c);
    }

    public int getColumnCount()
    {
        return 2;
    }

    public int getRowCount()
    {
        synchronized (robots)
        {
            return robots.size();
        }
    }

    public Object getValueAt(int rowIndex, int columnIndex)
    {
        if (rowIndex < 0|| columnIndex < 0)
            return null;
        
        synchronized (robots)
        {
            switch (columnIndex)
            {
            case ROBOT_COL:
                return robots.get(rowIndex);
            case CONTROL_COL:
                return controllers.get(rowIndex);
            }
        }
        return null;
    }

}
