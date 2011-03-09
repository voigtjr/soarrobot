package edu.umich.robot.events;

import edu.umich.robot.metamap.VirtualObject;

public class ObjectRemovedEvent implements AbstractProgramEvent
{
    private final VirtualObject obj;
    public ObjectRemovedEvent(VirtualObject obj) {
        this.obj = obj;
    }
    
    public VirtualObject getObject() {
        return obj;
    }
    
}
