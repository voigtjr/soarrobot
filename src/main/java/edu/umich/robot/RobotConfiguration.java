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

import java.util.List;

import com.google.common.collect.ImmutableList;

/**
 * @author voigtjr@gmail.com
 */
public class RobotConfiguration
{
    public static class Builder
    {
        private double limitVelocityLinearMax = 0;

        private double limitVelocityLinearMin = 0;

        private double limitVelocityAngularMax = 0;

        private double limitVelocityAngularMin = 0;

        private double geometryLength = 0;
        
        private double geometryWidth = 0;
        
        private double geometryHeight = 0;
        
        private double geometryWheelbase = 0;
        
        private List<Double> gainsHeading = newPidList(new double[] {0, 0, 0});
        
        private List<Double> gainsAngular = newPidList(new double[] {0, 0, 0});
        
        private List<Double> gainsLinear = newPidList(new double[] {0, 0, 0});
        
        private double objectFieldOfView = 0;
        
        private double objectManipulationDistanceMin = 0;

        private double objectManipulationDistanceMax = 0;

        public Builder limitVelocityLinearMax(double limitVelocityLinearMax)
        {
            this.limitVelocityLinearMax = limitVelocityLinearMax;
            return this;
        }

        public Builder limitVelocityLinearMin(double limitVelocityLinearMin)
        {
            this.limitVelocityLinearMin = limitVelocityLinearMin;
            return this;
        }

        public Builder limitVelocityAngularMax(double limitVelocityAngularMax)
        {
            this.limitVelocityAngularMax = limitVelocityAngularMax;
            return this;
        }

        public Builder limitVelocityAngularMin(double limitVelocityAngularMin)
        {
            this.limitVelocityAngularMin = limitVelocityAngularMin;
            return this;
        }

        public Builder geometryLength(double geometryLength)
        {
            this.geometryLength = geometryLength;
            return this;
        }
        
        public Builder geometryWidth(double geometryWidth)
        {
            this.geometryWidth = geometryWidth;
            return this;
        }

        public Builder geometryHeight(double geometryHeight)
        {
            this.geometryHeight = geometryHeight;
            return this;
        }

        public Builder geometryWheelbase(double geometryWheelbase)
        {
            this.geometryWheelbase = geometryWheelbase;
            return this;
        }
        
        private static List<Double> newPidList(double[] pid)
        {
            return new ImmutableList.Builder<Double>()
            .add(pid[0])
            .add(pid[1])
            .add(pid[2])
            .build();
        }

        public Builder gainsHeading(double[] pid)
        {
            this.gainsHeading = newPidList(pid);
            return this;
        }

        public Builder gainsAngular(double[] pid)
        {
            this.gainsAngular = newPidList(pid);
            return this;
        }

        public Builder gainsLinear(double[] pid)
        {
            this.gainsLinear = newPidList(pid);
            return this;
        }

        public Builder objectFieldOfView(double objectFieldOfView)
        {
            this.objectFieldOfView = objectFieldOfView;
            return this;
        }

        public Builder objectManipulationDistanceMin( double objectManipulationDistanceMin)
        {
            this.objectManipulationDistanceMin = objectManipulationDistanceMin;
            return this;
        }

        public Builder objectManipulationDistanceMax( double objectManipulationDistanceMax)
        {
            this.objectManipulationDistanceMax = objectManipulationDistanceMax;
            return this;
        }

        public RobotConfiguration build()
        {
            return new RobotConfiguration(this);
        }
    }

    private final double limitVelocityLinearMax;

    private final double limitVelocityLinearMin;

    private final double limitVelocityAngularMax;

    private final double limitVelocityAngularMin;

    private final double geometryLength;
    
    private final double geometryWidth;
    
    private final double geometryHeight;
    
    private final double geometryWheelbase;
    
    private final List<Double> gainsHeading;
    
    private final List<Double> gainsAngular;
    
    private final List<Double> gainsLinear;
    
    private final double objectFieldOfView;
    
    private double objectManipulationDistanceMin;

    private double objectManipulationDistanceMax;

    private RobotConfiguration(Builder builder)
    {
        this.limitVelocityLinearMax = builder.limitVelocityLinearMax;
        this.limitVelocityLinearMin = builder.limitVelocityLinearMin;
        this.limitVelocityAngularMax = builder.limitVelocityAngularMax;
        this.limitVelocityAngularMin = builder.limitVelocityAngularMin;
        this.geometryLength = builder.geometryLength;
        this.geometryWidth = builder.geometryWidth;
        this.geometryHeight = builder.geometryHeight;
        this.geometryWheelbase = builder.geometryWheelbase;
        this.gainsHeading = builder.gainsHeading;
        this.gainsAngular = builder.gainsAngular;
        this.gainsLinear = builder.gainsLinear;
        this.objectFieldOfView = builder.objectFieldOfView;
        this.objectManipulationDistanceMin = builder.objectManipulationDistanceMin;
        this.objectManipulationDistanceMax = builder.objectManipulationDistanceMax;
    }

    public double getLimitVelocityLinearMax()
    {
        return limitVelocityLinearMax;
    }

    public double getLimitVelocityLinearMin()
    {
        return limitVelocityLinearMin;
    }

    public double getLimitVelocityAngularMax()
    {
        return limitVelocityAngularMax;
    }

    public double getLimitVelocityAngularMin()
    {
        return limitVelocityAngularMin;
    }

    public double getGeometryLength()
    {
        return geometryLength;
    }

    public double getGeometryWidth()
    {
        return geometryWidth;
    }

    public double getGeometryHeight()
    {
        return geometryHeight;
    }

    public List<Double> getGainsHeading()
    {
        return gainsHeading;
    }

    public List<Double> getGainsAngular()
    {
        return gainsAngular;
    }

    public List<Double> getGainsLinear()
    {
        return gainsLinear;
    }

    public double getObjectFieldOfView()
    {
        return objectFieldOfView;
    }

    public double getGeometryWheelbase()
    {
        return geometryWheelbase;
    }

    public double getObjectManipulationDistanceMin()
    {
        return objectManipulationDistanceMin;
    }

    public double getObjectManipulationDistanceMax()
    {
        return objectManipulationDistanceMax;
    }

}
