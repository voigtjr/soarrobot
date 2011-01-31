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

import java.awt.BorderLayout;
import java.awt.Component;
import java.awt.Rectangle;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.FocusAdapter;
import java.awt.event.FocusEvent;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.IOException;
import java.util.Map;
import java.util.prefs.BackingStoreException;
import java.util.prefs.Preferences;

import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JDialog;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JOptionPane;
import javax.swing.JPopupMenu;
import javax.swing.JSeparator;
import javax.swing.JSplitPane;
import javax.swing.JTextField;
import javax.swing.JToolBar;
import javax.swing.KeyStroke;
import javax.swing.ToolTipManager;
import javax.swing.filechooser.FileFilter;
import javax.swing.filechooser.FileNameExtensionFilter;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import april.config.Config;
import april.config.ConfigFile;
import april.config.ConfigUtil;
import april.jmat.geom.GRay3D;
import april.viewer.ViewRobot;
import april.viewer.ViewTrajectory;
import april.viewer.Viewer;
import april.viewer.ViewRobot.FollowMode;
import april.vis.VisCanvas;
import april.vis.VisCanvasEventAdapter;

import com.google.common.collect.Maps;
import com.jgoodies.forms.layout.CellConstraints;
import com.jgoodies.forms.layout.FormLayout;

import edu.umich.robot.actions.ActionManager;
import edu.umich.robot.actions.AddObjectAction;
import edu.umich.robot.actions.CreateSplinterRobotAction;
import edu.umich.robot.actions.DisableFollowAction;
import edu.umich.robot.actions.ExitAction;
import edu.umich.robot.actions.FollowPositionAction;
import edu.umich.robot.actions.FollowPositionAndThetaAction;
import edu.umich.robot.actions.MoveCameraAboveAction;
import edu.umich.robot.actions.MoveCameraBehindAction;
import edu.umich.robot.actions.ResetAction;
import edu.umich.robot.actions.ResetPreferencesAction;
import edu.umich.robot.actions.SaveMapAction;
import edu.umich.robot.actions.SimSpeedAction;
import edu.umich.robot.actions.SoarDataAction;
import edu.umich.robot.actions.SoarParametersAction;
import edu.umich.robot.actions.SoarStepAction;
import edu.umich.robot.actions.SoarToggleAction;
import edu.umich.robot.events.AfterResetEvent;
import edu.umich.robot.events.ControllerActivatedEvent;
import edu.umich.robot.events.ControllerDeactivatedEvent;
import edu.umich.robot.events.RobotAddedEvent;
import edu.umich.robot.events.RobotRemovedEvent;
import edu.umich.robot.gp.Gamepad;
import edu.umich.robot.util.Configs;
import edu.umich.robot.util.Pose;
import edu.umich.robot.util.events.RobotEvent;
import edu.umich.robot.util.events.RobotEventListener;
import edu.umich.robot.util.properties.PropertyManager;

/**
 * @author voigtjr@gmail.com
 */
public class GuiApplication
{
    private static final Log logger = LogFactory.getLog(GuiApplication.class);

    private final Preferences PREFERENCES = Preferences.userRoot().node("edu/umich/robot");

    private final JFrame frame = new JFrame();

    private final PropertyManager properties = new PropertyManager();

    private final Controller controller;

    private final Viewer viewer;
    
    private boolean resetPreferencesAtExit = false;
    
    private final ViewerView viewerView;
    
    private final RobotsView robotsView;

    private final ConsoleView consoleView;
    
    private static class RobotData
    {
        public RobotData(ViewRobot vr, ViewTrajectory vt)
        {
            this.vr = vr;
            this.vt = vt;
        }
        
        final ViewRobot vr;
        final ViewTrajectory vt;
    }

    private final Map<String, RobotData> robotData = Maps.newConcurrentMap();
    
    private final StatusBar status;
    
    private String objectToAdd;
    
    private ActionManager actionManager;
        
    private Config promptForConfig()
    {
        JFileChooser fc = new JFileChooser("config");
        fc.setFileSelectionMode(JFileChooser.FILES_ONLY);
        FileFilter filter = new FileNameExtensionFilter("Text Config File", "txt");
        fc.setFileFilter(filter);
        fc.setMultiSelectionEnabled(false);
        int ret = fc.showOpenDialog(frame);
        if (ret == JFileChooser.APPROVE_OPTION)
        {
            try
            {
                return new ConfigFile(fc.getSelectedFile().getAbsolutePath());
            }
            catch (IOException e)
            {
                logger.error(e.getMessage());
            }
        }
        return null;
    }
    
    /**
     * @param args
     */
    public GuiApplication(String[] args)
    {
        JPopupMenu.setDefaultLightWeightPopupEnabled(false);
        ToolTipManager.sharedInstance().setLightWeightPopupEnabled(false);
        
        Config config = (args.length > 0) ? ConfigUtil.getDefaultConfig(args) : promptForConfig();
        if (config == null)
            System.exit(1);

        Application.setupSimulatorConfig(config);
        setupViewerConfig(config);
        
        Configs.toLog(logger, config);
        
        controller = new Controller(config, new Gamepad());
        controller.initializeGamepad();

        viewer = new Viewer(config);
        viewer.getVisCanvas().getViewManager().setInterfaceMode(3);

        controller.addListener(RobotAddedEvent.class, listener);
        controller.addListener(RobotRemovedEvent.class, listener);
        controller.addListener(AfterResetEvent.class, listener);
        controller.addListener(ControllerActivatedEvent.class, listener);
        controller.addListener(ControllerDeactivatedEvent.class, listener);

        actionManager = new ActionManager(this);
        initActions();
        
        frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
        frame.addWindowListener(new WindowAdapter()
        {
            @Override
            public void windowClosed(WindowEvent e)
            {
                controller.shutdown();
                frame.dispose();

                try
                {
                    Thread.sleep(500);
                }
                catch (InterruptedException ignored)
                {
                }
                System.exit(0); // No way to shut down april threads
            }
        });
        frame.setLayout(new BorderLayout());

        JMenuBar menuBar = new JMenuBar();
        JMenu fileMenu = new JMenu("File");
        fileMenu.add(actionManager.getAction(CreateSplinterRobotAction.class));
        fileMenu.add(actionManager.getAction(ResetPreferencesAction.class));

        //fileMenu.add(new JSeparator());
        //fileMenu.add(actionManager.getAction(ForwardToTabletAction.class));
        
        fileMenu.add(new JSeparator());
        fileMenu.add(actionManager.getAction(SaveMapAction.class));
        fileMenu.add(new JSeparator());
        fileMenu.add(actionManager.getAction(ExitAction.class));
        menuBar.add(fileMenu);

        JMenu cameraMenu = new JMenu("Camera");
        cameraMenu.add(actionManager.getAction(DisableFollowAction.class));
        cameraMenu.add(actionManager.getAction(FollowPositionAction.class));
        cameraMenu.add(actionManager.getAction(FollowPositionAndThetaAction.class));
        cameraMenu.add(new JSeparator());
        cameraMenu.add(actionManager.getAction(MoveCameraBehindAction.class));
        cameraMenu.add(actionManager.getAction(MoveCameraAboveAction.class));
        menuBar.add(cameraMenu);
        
        JMenu objectMenu = new JMenu("Objects");
        boolean added = false;
        for (String objectName : controller.getObjectNames())
        {
            added = true;
            objectMenu.add(new AddObjectAction(this, objectName));
        }
        if (!added)
            objectMenu.add(new JLabel("No objects available"));
        menuBar.add(objectMenu);

        menuBar.revalidate();
        frame.setJMenuBar(menuBar);

        JToolBar toolBar = new JToolBar();
        toolBar.setFloatable(false);
        toolBar.setRollover(true);
        toolBar.add(actionManager.getAction(SoarParametersAction.class));
        toolBar.add(actionManager.getAction(SoarDataAction.class));
        toolBar.add(actionManager.getAction(ResetAction.class));
        toolBar.add(actionManager.getAction(SoarToggleAction.class));
        toolBar.add(actionManager.getAction(SoarStepAction.class));
        toolBar.add(actionManager.getAction(SimSpeedAction.class));
        frame.add(toolBar, BorderLayout.PAGE_START);

        viewerView = new ViewerView(viewer.getVisCanvas());
        robotsView = new RobotsView(this);
        
        final JSplitPane splitPane = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, 
                viewerView, robotsView); 
        
        viewer.addRobotSelectionChangedListener(robotsView);
        viewer.getVisCanvas().setDrawGround(true);
        viewer.getVisCanvas().addEventHandler(new VisCanvasEventAdapter() {

            public String getName() {
                return "Place Object";
            }

            @Override
            public boolean mouseClicked(VisCanvas vc, GRay3D ray, MouseEvent e) {
                boolean ret = false;
                synchronized (GuiApplication.this)
                {
                    if (objectToAdd != null && controller != null)
                    {
                        controller.addObject(objectToAdd, ray.intersectPlaneXY());
                        objectToAdd = null;
                        ret = true;
                    } 
                }
                status.setMessage(String.format("%3.1f,%3.1f", ray.intersectPlaneXY()[0], ray.intersectPlaneXY()[1]));
                return ret;
            }
        });
        
        consoleView = new ConsoleView();
        
        final JSplitPane splitPane2 = new JSplitPane(JSplitPane.VERTICAL_SPLIT, 
                splitPane, consoleView); 
        
        frame.add(splitPane2, BorderLayout.CENTER);
        
        status = new StatusBar();
        frame.add(status, BorderLayout.SOUTH);

        frame.addWindowListener(new WindowAdapter() {
            @Override
            public void windowClosing(WindowEvent e)
            {
                final Preferences windowPrefs = getWindowPreferences();
                final Rectangle r = frame.getBounds();
                if(frame.getExtendedState() == JFrame.NORMAL)
                {
                    windowPrefs.putInt("x", r.x);
                    windowPrefs.putInt("y", r.y);
                    windowPrefs.putInt("width", r.width);
                    windowPrefs.putInt("height", r.height);
                    windowPrefs.putInt("divider", splitPane.getDividerLocation());
                }
                
                exit();
            }});

        Preferences windowPrefs = getWindowPreferences();
        if (windowPrefs.get("x", null) != null)
        {
            frame.setBounds(
                    windowPrefs.getInt("x", 0), 
                    windowPrefs.getInt("y", 0), 
                    windowPrefs.getInt("width", 800), 
                    windowPrefs.getInt("height", 800));
            splitPane.setDividerLocation(windowPrefs.getInt("divider", 500));
        }
        else
        {
            frame.setBounds(
                    windowPrefs.getInt("x", 0), 
                    windowPrefs.getInt("y", 0), 
                    windowPrefs.getInt("width", 600), 
                    windowPrefs.getInt("height", 600));
            splitPane.setDividerLocation(0.75);
            frame.setLocationRelativeTo(null); // center
        }

        frame.getRootPane().registerKeyboardAction(new ActionListener()
                {
                    public void actionPerformed(ActionEvent e)
                    {
                        frame.dispose();
                    }
                }, KeyStroke.getKeyStroke(KeyEvent.VK_ESCAPE, 0),
                JComponent.WHEN_IN_FOCUSED_WINDOW);

        frame.pack();
        
        frame.setVisible(true);

        String[] splinters = config.getStrings("splinters", new String[0]);
        for (String s : splinters)
        {
            double[] pos = config.getDoubles(s + ".position");
            if (pos == null)
            {
                logger.error("Splinter indexed in config file but no position defined: " + s);
                continue;
            }
            
            Pose pose = new Pose(pos);
            String prods = config.getString(s + ".productions");
            boolean collisions = config.getBoolean(s + ".wallCollisions", true);
            
            controller.createSplinterRobot(s, pose, collisions);
            if (prods != null)
            {
                controller.createSoarController(s, s, prods, config.getChild(s + ".properties"));
                PREFERENCES.put("lastProductions", prods);
            }
        }
        
    }
    
    private void initActions()
    {
        new ResetAction(actionManager);
        new SaveMapAction(actionManager);
        new SoarDataAction(actionManager);
        new SoarParametersAction(actionManager);
        new SoarToggleAction(actionManager);
        new SoarStepAction(actionManager);
        new CreateSplinterRobotAction(actionManager);
        new ResetPreferencesAction(actionManager);
        new ExitAction(actionManager);
        new DisableFollowAction(actionManager);
        new FollowPositionAction(actionManager);
        new FollowPositionAndThetaAction(actionManager);
        new MoveCameraAboveAction(actionManager);
        new MoveCameraBehindAction(actionManager);
        new SimSpeedAction(actionManager);
        //new ForwardToTabletAction(actionManager);
    }
    
    public Controller getController()
    {
        return controller;
    }
    
    private ViewRobot getRobot()
    {
        String name = controller.getSelectedRobotName();
        if (name == null)
            return null;

        return robotData.get(name).vr;
    }
    
    public void setFollowMode(FollowMode mode)
    {
        ViewRobot vr = getRobot();
        if (vr == null)
            return;
        vr.setFollowMode(mode, viewer.getVisCanvas());
    }

    public void snapCamera(int mode)
    {
        ViewRobot robot = getRobot();
        if (robot != null) 
            robot.moveCameraToRobot(mode, viewer.getVisCanvas());
    }

    public void dispose()
    {
        frame.dispose();
    }
    
    private void exit()
    {
        if (resetPreferencesAtExit)
        {
            try
            {
                PREFERENCES.removeNode();
            }
            catch (BackingStoreException e)
            {
                logger.error(e);
            }
        }
    }
    
    public Preferences getWindowPreferences()
    {
        return PREFERENCES.node("window");
    }
    
    public PropertyManager getProperties()
    {
        return properties;
    }

    public void createSplinterRobotDialog()
    {
        final Pose pose = new Pose();
        
        FormLayout layout = new FormLayout(
                "right:pref, 4dlu, 30dlu, 4dlu, right:pref, 4dlu, 30dlu",
                "pref, 2dlu, pref, 2dlu, pref");

        layout.setRowGroups(new int[][] {{1, 3}});
        
        final JDialog dialog = new JDialog(frame, "Create Splinter Robot", true);
        dialog.setLayout(layout);
        final JTextField name = new JTextField();
        final JTextField x = new JTextField(Double.toString((pose.getX())));
        final JTextField y = new JTextField(Double.toString((pose.getY())));
        final JButton cancel = new JButton("Cancel");
        final JButton ok = new JButton("OK");

        CellConstraints cc = new CellConstraints();
        dialog.add(new JLabel("Name"), cc.xy(1, 1));
        dialog.add(name, cc.xyw(3, 1, 5));
        dialog.add(new JLabel("x"), cc.xy(1, 3));
        dialog.add(x, cc.xy(3, 3));
        dialog.add(new JLabel("y"), cc.xy(5, 3));
        dialog.add(y, cc.xy(7, 3));
        dialog.add(cancel, cc.xyw(1, 5, 3));
        dialog.add(ok, cc.xyw(5, 5, 3));

        x.addFocusListener(new FocusAdapter()
        {
            @Override
            public void focusLost(FocusEvent e)
            {
                try
                {
                    pose.setX(Double.parseDouble(x.getText()));
                }
                catch (NumberFormatException ex)
                {
                    x.setText(Double.toString(pose.getX()));
                }
            }
        });
        
        y.addFocusListener(new FocusAdapter()
        {
            @Override
            public void focusLost(FocusEvent e)
            {
                try
                {
                    pose.setY(Double.parseDouble(y.getText()));
                }
                catch (NumberFormatException ex)
                {
                    y.setText(Double.toString(pose.getX()));
                }
            }
        });
        
        final ActionListener okListener = new ActionListener()
        {
            public void actionPerformed(ActionEvent e)
            {
                String robotName = name.getText().trim();
                if (robotName.isEmpty())
                {
                    logger.error("Create splinter: robot name empty");
                    return;
                }
                for (char c : robotName.toCharArray())
                    if (!Character.isDigit(c) && !Character.isLetter(c))
                    {
                        logger.error("Create splinter: illegal robot name");
                        return;
                    }

                controller.createSplinterRobot(robotName, pose, true);
                dialog.dispose();
            }
        };
        name.addActionListener(okListener);
        x.addActionListener(okListener);
        y.addActionListener(okListener);
        ok.addActionListener(okListener);

        ActionListener cancelAction = new ActionListener()
        {
            public void actionPerformed(ActionEvent e)
            {
                dialog.dispose();
            }
        };
        cancel.addActionListener(cancelAction);
        dialog.getRootPane().registerKeyboardAction(cancelAction, 
                KeyStroke.getKeyStroke(KeyEvent.VK_ESCAPE, 0), 
                JComponent.WHEN_IN_FOCUSED_WINDOW);
        
        dialog.setLocationRelativeTo(frame);
        dialog.pack();
        dialog.setVisible(true);
    }

    private final RobotEventListener listener = new RobotEventListener()
    {
        public void onEvent(RobotEvent event)
        {
            if (event instanceof RobotAddedEvent)
            {
                RobotAddedEvent e = (RobotAddedEvent) event;

                ViewRobot vr = addViewRobot(e.getRobot().getName());
                addViewLidars(e.getRobot().getName());
                addViewWaypoints(e.getRobot().getName());
                ViewTrajectory vt = addViewTrajectory(e.getRobot().getName());
                
                robotData.put(e.getRobot().getName(), new RobotData(vr, vt));
            }
            else if (event instanceof AfterResetEvent)
            {
                for (RobotData rd : robotData.values())
                    rd.vt.clear();
            }
            
            // there are a few unhandled actions

            actionManager.updateActions();
        }
    };

    private void setupViewerConfig(Config config)
    {
        config.setStrings("viewer.viewobjects", new String[] { "obstacles", "walls", "areas" });
        
        config.setString("viewer.obstacles.class", "april.viewer.ViewObstaclesReadOnly");
        
        config.setString("viewer.walls.class", "april.viewer.ViewWalls");
        Application.addImageData(config, "viewer.walls.obstacles.");
        
//        config.setString("viewer.floor.class", "april.viewer.ViewFloor"); // need to add floor to viewobjects list
//        addImageData(config, "viewer.floor.obstacles.");

        config.setString("viewer.areas.class", "april.viewer.ViewAreaDescriptions");

//        config.setString("viewer.skybox.class", "april.viewer.ViewSkybox"); // need to add skybox to viewobjects list
//        config.setString("viewer.skybox.north_image", "../common/north.jpg");
//        config.setString("viewer.skybox.south_image", "../common/south.jpg");
//        config.setString("viewer.skybox.east_image", "../common/east.jpg");
//        config.setString("viewer.skybox.west_image", "../common/west.jpg");
//        config.setString("viewer.skybox.up_image", "../common/top.jpg");
//        config.setString("viewer.skybox.down_image", "../common/floor.jpg");
    }
    
    private void addPositionInfo(Config config, String prefix,
            double[] position, double[] rpy, int[] color)
    {
        config.setDoubles(prefix + "position", position);
        config.setDoubles(prefix + "rollpitchyaw_degrees", rpy);
        if (color != null)
            config.setInts(prefix + "color", color);
    }

    private ViewRobot addViewRobot(String name)
    {
        Config config = new Config();
        config.setString("class", "april.viewer.ViewRobot");
        addPositionInfo(config, "avatar.", new double[] { 0, 0, 0 },
                new double[] { 0, 0, 0 }, null);

        Configs.toLog(logger, config);
        return (ViewRobot)viewer.addObject(name, config);
    }

    private void addViewLidars(String name)
    {
        Config config = new Config();
        config.setString("class", "april.viewer.ViewLaser");
        config.setString("pose", name);
        config.setStrings("channels", new String[] { "SIM_LIDAR_FRONT",
                "SICK_LIDAR_FRONT", "LIDAR_LOWRES", });
        addPositionInfo(config, "SIM_LIDAR_FRONT_" + name + ".", new double[] {
                0, 0, 0.4 }, new double[] { 0, 0, 0 }, new int[] { 1, 0, 0 });
        addPositionInfo(config, "SICK_LIDAR_FRONT_" + name + ".", new double[] {
                0, 0, 0.4 }, new double[] { 0, 0, 0 }, new int[] { 0, 1, 0 });
        addPositionInfo(config, "LIDAR_LOWRES_" + name + ".", new double[] { 0,
                0, 0.4 }, new double[] { 0, 0, 0 }, new int[] { 0, 0, 1 });

        Configs.toLog(logger, config);
        viewer.addObject(name + "lidars", config);
    }

    private void addViewWaypoints(String name)
    {
        Config config = new Config();
        config.setString("class", "april.viewer.ViewWaypoints");
        config.setString("channel", "WAYPOINTS_" + name);

        Configs.toLog(logger, config);
        viewer.addObject(name + "waypoints", config);
    }

    private ViewTrajectory addViewTrajectory(String name)
    {
        Config config = new Config();
        config.setString("class", "april.viewer.ViewTrajectory");
        config.setString("pose", name);

        Configs.toLog(logger, config);
        return (ViewTrajectory)viewer.addObject(name + "trajectory", config);
    }

    public void addObjectOnNextClick(String name)
    {
        synchronized (this)
        {
            objectToAdd = name;
        }
        status.setMessage("Click on map to place object.");
    }
    
    public void setStatusBarMessage(String message)
    {
        status.setMessage(message);
    }

    public Component getTopLevelAncestor()
    {
        return frame;
    }

    public ActionManager getActionManager()
    {
        return actionManager;
    }

    public void setResetPreferencesAtExit(boolean b)
    {
        resetPreferencesAtExit = b;
    }

    public Preferences getPreferences()
    {
        return PREFERENCES;
    }

}
