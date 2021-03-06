/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package edu.umich.robot.lcmtypes;
 
import java.io.*;
import java.nio.*;
import java.util.*;
import lcm.lcm.*;
 
public final class waypoint_t implements lcm.lcm.LCMEncodable
{
    public long utime;
    public double xLocal;
    public double yLocal;
    public double tLocal;
    public double xyThreshold;
    public double tThreshold;
 
    public waypoint_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xf58b77e3c343048dL;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class>());
    }
 
    public static long _hashRecursive(ArrayList<Class> classes)
    {
        if (classes.contains(waypoint_t.class))
            return 0L;
 
        classes.add(waypoint_t.class);
        long hash = LCM_FINGERPRINT_BASE
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        outs.writeLong(this.utime); 
 
        outs.writeDouble(this.xLocal); 
 
        outs.writeDouble(this.yLocal); 
 
        outs.writeDouble(this.tLocal); 
 
        outs.writeDouble(this.xyThreshold); 
 
        outs.writeDouble(this.tThreshold); 
 
    }
 
    public waypoint_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public waypoint_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static waypoint_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        waypoint_t o = new waypoint_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.utime = ins.readLong();
 
        this.xLocal = ins.readDouble();
 
        this.yLocal = ins.readDouble();
 
        this.tLocal = ins.readDouble();
 
        this.xyThreshold = ins.readDouble();
 
        this.tThreshold = ins.readDouble();
 
    }
 
    public waypoint_t copy()
    {
        waypoint_t outobj = new waypoint_t();
        outobj.utime = this.utime;
 
        outobj.xLocal = this.xLocal;
 
        outobj.yLocal = this.yLocal;
 
        outobj.tLocal = this.tLocal;
 
        outobj.xyThreshold = this.xyThreshold;
 
        outobj.tThreshold = this.tThreshold;
 
        return outobj;
    }
 
}

