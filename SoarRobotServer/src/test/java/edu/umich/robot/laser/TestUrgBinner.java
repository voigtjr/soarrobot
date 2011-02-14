package edu.umich.robot.laser;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertArrayEquals;

import org.junit.Test;

import april.lcmtypes.urg_range_t;

public class TestUrgBinner
{
    @Test(expected=IllegalArgumentException.class)
    public void testCtor1() throws Exception
    {
        new UrgBinner(-1, 0);
    }
    
    @Test(expected=IllegalArgumentException.class)
    public void testCtor2() throws Exception
    {
        new UrgBinner(0, 0);
    }
    
    @Test(expected=IllegalArgumentException.class)
    public void testCtor3() throws Exception
    {
        new UrgBinner(0, -1);
    }
    
    @Test
    public void testBinnedSize()
    {
        for (int i = 1; i < 10; ++i)
        {
            UrgBinner ub1 = new UrgBinner(i, Math.PI);
            assertEquals(ub1.getBinned().size(), i);
        }
    }

    @Test
    public void testCtor5()
    {
        new UrgBinner(1, Math.PI);
    }
    
    @Test
    public void testCtor6()
    {
        new UrgBinner(1, Math.PI * 2);
    }
    
    @Test(expected=IllegalArgumentException.class)
    public void testCtor7() throws Exception
    {
        new UrgBinner(1, Math.PI * 2.01);
    }
    
    @Test
    public void testSimple()
    {
        UrgBinner ub = new UrgBinner(5, Math.PI);
        urg_range_t urg = makeUrg(Math.PI / -2, Math.PI, 1, 1, 1, 1, 1);
        ub.update(urg);
        assertEquals(ub.getBinned().size(), 5);
        check(ub, 1, 1, 1, 1, 1);
    }
    
    private void check(UrgBinner ub, float ... expected)
    {
        for (int i = 0; i < expected.length; ++i)
            assertEquals(Float.valueOf(expected[i]), ub.getBinned().get(i));
    }
    
    private urg_range_t makeUrg(double rad0, double fov, double ... ranges)
    {
        urg_range_t urg = new urg_range_t();
        if (ranges.length == 0)
            return urg;
        
        urg.utime = System.nanoTime() / 1000;
        urg.num_ranges = ranges.length;
        urg.ranges = new float[ranges.length];
        for (int i = 0; i < ranges.length; ++i)
            urg.ranges[i] = (float)ranges[i];
        
        urg.rad0 = (float)rad0;
        urg.radstep = (float)(fov / ranges.length);
        
        return urg;
    }
    
    
}
