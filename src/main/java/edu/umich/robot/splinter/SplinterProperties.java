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
package edu.umich.robot.splinter;

import edu.umich.robot.util.properties.PropertyKey;

/**
 * @author voigtjr@gmail.com
 */
public class SplinterProperties
{
    private static final String SPLINTER_PREFIX = "splinter.";

    private static <T> PropertyKey.Builder<T> key(String name, Class<T> type)
    {
        return PropertyKey.builder(SPLINTER_PREFIX + name, type);
    }

    public static PropertyKey<String> NAME = 
        key("name", String.class)
        .build();

    public static PropertyKey<String> ANGULAR_PID_NAME = 
        key("angular-pid-name", String.class)
        .defaultValue("angular")
        .build();

    public static PropertyKey<double[]> ANGULAR_PID_GAINS = 
        key("angular-pid-gains", double[].class)
        //.defaultValue( new double[] { 0.0238, 0, 0.0025 })
        .defaultValue( new double[] { 0.15, 0, 0.01 })
        .build();

    public static PropertyKey<String> LINEAR_PID_NAME = 
        key("linear-pid-name", String.class)
        .defaultValue("linear")
        .build();

    public static PropertyKey<double[]> LINEAR_PID_GAINS = 
        key("linear-pid-gains", double[].class)
        .defaultValue( new double[] { 0.12, 0, 0.025 })
        .build();

    public static PropertyKey<String> HEADING_PID_NAME = 
        key("heading-pid-name", String.class)
        .defaultValue("heading")
        .build();

    public static PropertyKey<double[]> HEADING_PID_GAINS = 
        key("heading-pid-gains", double[].class)
        //.defaultValue( new double[] { 0.6, 0, 0.125 })
        .defaultValue( new double[] { 1.8, 0, 0.125 })
        .build();

}
