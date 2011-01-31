package edu.umich.robot.metamap;

import java.util.List;
import java.util.Scanner;

import com.google.common.collect.ImmutableList;
import com.google.common.collect.Lists;

public class AbridgedAreaDescriptions
{
    public static List<AbridgedAreaDescription> parseAreas(String string)
    {
        Scanner scanner = new Scanner(string);
        scanner.useDelimiter(";");
        
        List<AbridgedAreaDescription> areas = Lists.newArrayList();
        
        while (scanner.hasNext())
            areas.add(parseArea(scanner.next()));
        return areas;
    }
    
    public static AbridgedAreaDescription parseArea(String string)
    {
        Scanner scanner = new Scanner(string);
        scanner.useDelimiter(" ");
        if (!scanner.hasNext("r"))
            return null;
        scanner.next("r");

        int id = Integer.valueOf(scanner.next()).intValue();
        ImmutableList.Builder<Double> xywh = new ImmutableList.Builder<Double>();
        xywh.add(Double.valueOf(scanner.next()));
        xywh.add(Double.valueOf(scanner.next()));
        xywh.add(Double.valueOf(scanner.next()));
        xywh.add(Double.valueOf(scanner.next()));

        ImmutableList.Builder<AbridgedGateway> gateways = new ImmutableList.Builder<AbridgedGateway>();

        while (scanner.hasNext())
        {
            if (!scanner.hasNext("g"))
                return null;
            scanner.next("g");
            
            int gid = Integer.valueOf(scanner.next()).intValue();
            ImmutableList.Builder<Double> xy = new ImmutableList.Builder<Double>();
            xy.add(Double.valueOf(scanner.next()));
            xy.add(Double.valueOf(scanner.next()));
            
            gateways.add(new AbridgedGateway(gid, xy.build()));
        }
        
        return new AbridgedAreaDescription(id, xywh.build(), gateways.build());
    }
}
