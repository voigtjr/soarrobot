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

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import sml.Identifier;

/**
 * @author voigtjr@gmail.com
 */
abstract class OLCommand
{
    private static final Log logger = LogFactory.getLog(OLCommand.class);
    
    private final Identifier id;

    private final String name;

    private final long tt;

    private CommandStatus status = CommandStatus.ACCEPTED;

    private String statusMessage;

    protected OLCommand(Identifier id)
    {
        this.id = id;

        // cache these so they are accessible after the wme is invalid
        name = id.GetAttribute();
        tt = id.GetTimeTag();
    }
    
    protected String toStringPrefix()
    {
        StringBuilder sb = new StringBuilder("(");
        sb.append(name);
        sb.append(": ");
        sb.append(tt);
        return sb.toString();
    }

    @Override
    public String toString()
    {
        return toStringPrefix() + ")";
    }

    public Long getTimeTag()
    {
        return tt;
    }

    protected Identifier getId()
    {
        return id;
    }

    protected void setStatus(CommandStatus status)
    {
        assert status != null;
        logger.debug(tt + ": " + status);
        this.status = status;
    }

    protected void setStatus(CommandStatus status, String message)
    {
        setStatus(status);
        logger.debug(tt + ": " + message);
        this.statusMessage = message;
    }

    @Override
    public boolean equals(Object obj)
    {
        if (obj instanceof OLCommand)
        {
            OLCommand olc = (OLCommand) obj;
            return tt == olc.tt;
        }
        return super.equals(obj);
    }

    @Override
    public int hashCode()
    {
        return Long.valueOf(tt).hashCode();
    }

    abstract void update() throws SoarCommandError;
    
    void stopEvent()
    {
    }
    
    void startEvent()
    {
    }

    void dispose()
    {
    }

    public CommandStatus getStatus()
    {
        return status;
    }

    public String getStatusMessage()
    {
        return statusMessage;
    }
}
