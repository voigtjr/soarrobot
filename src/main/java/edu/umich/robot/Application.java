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

import javax.swing.SwingUtilities;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import april.config.Config;
import april.config.ConfigUtil;
import edu.umich.robot.util.swing.SwingTools;

/**
 * @author voigtjr@gmail.com
 */
public class Application
{
    private static final Log logger = LogFactory.getLog(Application.class);
    
    private String[] toArgs(String config)
    {
        return new String[] { "--config", config };
    }
    
    public static void main(final String[] args)
    {
        new Application(args);
    }

    public Application(final String[] args)
    {
        Config config = ConfigUtil.getDefaultConfig(args);
        if (config == null || !config.hasKey("multiple-runs.configs"))
            runGui(args);
        else
            multipleRuns(config.getChild("multiple-runs"));
    }

    private void multipleRuns(Config config)
    {
        String[] configs = config.requireStrings("configs");
        int cycles = config.requireInt("cycles");
        int seconds = config.requireInt("seconds");
        
        for (int i = 0; i < configs.length; ++i)
        {
            logger.info("Running " + configs[i] + " for " + cycles + " cycles, timeout " + seconds + " seconds.");
            HeadlessApplication h = new HeadlessApplication(toArgs(configs[i]), cycles, seconds, this);
            if (h.go())
                break;
            logger.info("Done, garbage collect and 5 second sleep.");
            System.gc();
            try
            {
                Thread.sleep(5000);
            }
            catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }
        logger.info("Done.");
    }
    
    private void runGui(final String[] args)
    {
        SwingTools.initializeLookAndFeel();
        SwingUtilities.invokeLater(new Runnable()
        {
            public void run()
            {
                new GuiApplication(args);
            }
        });
    }
    
    public static void setupSimulatorConfig(Config config)
    {
        addImageData(config, "simulator.obstacles.");
    }
    
    public static void addImageData(Config config, String prefix)
    {
        config.setStrings(prefix + "image_path", config.requireStrings("image_path"));
        config.setStrings(prefix + "image_origin", config.requireStrings("image_origin"));
        config.setStrings(prefix + "meters_per_pixel", config.requireStrings("meters_per_pixel"));
        config.setString(prefix + "floor_texture_path", "../common/floor.jpg");
        config.setString(prefix + "wall_texture_path", "../common/wall.jpg");
    }
    
}
