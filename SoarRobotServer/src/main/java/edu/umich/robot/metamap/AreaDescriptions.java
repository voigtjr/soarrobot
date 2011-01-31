package edu.umich.robot.metamap;

import java.util.List;

public class AreaDescriptions
{
    public static StringBuilder render(StringBuilder builder, List<AreaDescription> areas)
    {
        if (builder == null)
            builder = new StringBuilder();
        
        for (AreaDescription ad : areas)
            render(builder, ad).append(" ; ");
        
        return builder;
    }
    
    /**
     * "r id x y w h [g id x y]*" 
     * 
     * r: literal (room)
     * id: int
     * x y w h: double
     * type: string (usually "door" or "room")
     * []: aren't printed
     * g: literal (gateway)
     * *: previous construct repeated 0 or more times
     * 
     * Example:
     * 
     * r 0 3.2 4.525 5.2 4.8 g 1 6 8.2 3.3 3.3
     * 
     * @param builder
     * @param area
     * @return
     */
    public static StringBuilder render(StringBuilder builder, AreaDescription area)
    {
        if (builder == null)
            builder = new StringBuilder();

        builder.append("r ").append(area.getId());
        
        builder.append(" ").append(area.getPose().getX());
        builder.append(" ").append(area.getPose().getY());
        builder.append(" ").append(area.getPose().getVel().get(0));
        builder.append(" ").append(area.getPose().getVel().get(1));
        
        for (Gateway gw : area.getGateways())
        {
            builder.append(" g ").append(gw.getId());
            builder.append(" ").append(gw.getPose().getX());
            builder.append(" ").append(gw.getPose().getY());
        }
        
        return builder;
    }
    
}
