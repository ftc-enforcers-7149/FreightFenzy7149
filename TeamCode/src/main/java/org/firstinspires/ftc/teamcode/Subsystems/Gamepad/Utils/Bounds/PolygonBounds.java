package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils.Bounds;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils.Bounds.Bounds;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils.Point;

import java.util.ArrayList;

public class PolygonBounds extends Bounds {

    /*public static final PolygonBounds TOP_LEFT = new PolygonBounds(-100, 0, 0, 100);
    public static final PolygonBounds BOTTOM_LEFT = new PolygonBounds(-100, 0, -100, 0);
    public static final PolygonBounds TOP_RIGHT = new PolygonBounds(0, 100, 0, 100);
    public static final PolygonBounds BOTTOM_RIGHT = new PolygonBounds(0, 100, -100, 0);
    public static final PolygonBounds CENTER = new PolygonBounds(-50, 50, -50, 50);*/

    public PolygonBounds(ArrayList<Point> pointList) {

        points = pointList;
        minX = points.get(0).getX();
        maxX = minX;
        minY = points.get(0).getY();
        maxY = minY;

        for(int i = 1; i < points.size(); i++) {

            double tempX = points.get(i).getX();
            double tempY = points.get(i).getY();

            if(tempX > maxX) maxX = tempX;
            if(tempX < minX) minX = tempX;
            if(tempY > maxY) maxY = tempY;
            if(tempY < maxY) minY = tempY;

        }


    }

    @Override
    public boolean contains(Point p)
    {

        if(!super.contains(p)) return false;

        // https://wrf.ecse.rpi.edu/Research/Short_Notes/pnpoly.html
        boolean inside = false;
        for (int i = 0, j = points.size() - 1; i < points.size() ;j = i++)
        {
            if ((points.get(i).getY() > p.getY()) != (points.get(j).getY() > p.getY()) &&
                    p.getX() < (points.get(j).getX() - points.get(i).getX()) * ( p.getY() - points.get(i).getY()) / ( points.get(j).getY() - points.get(i).getY()) + points.get(i).getX())
            {
                inside = !inside;
            }
        }

        return inside;
    }

}
