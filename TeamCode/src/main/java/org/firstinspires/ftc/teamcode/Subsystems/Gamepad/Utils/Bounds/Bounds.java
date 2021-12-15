package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils.Bounds;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils.Point;

import java.util.ArrayList;

public class Bounds {

    public static final Bounds TOP_LEFT = new Bounds(-100, 0, 0, 100);
    public static final Bounds BOTTOM_LEFT = new Bounds(-100, 0, -100, 0);
    public static final Bounds TOP_RIGHT = new Bounds(0, 100, 0, 100);
    public static final Bounds BOTTOM_RIGHT = new Bounds(0, 100, -100, 0);
    public static final Bounds CENTER = new Bounds(-50, 50, -50, 50);

    ArrayList<Point> points;
    // Bounding box variables
    double minX, minY, maxX, maxY;

    public Bounds(double minX, double maxX, double minY, double maxY) {
        this.minX = minX;
        this.maxX = maxX;
        this.minY = minY;
        this.maxY = maxY;
    }

    public Bounds(Point p1, Point p2) {
        this.minX = Math.min(p1.getX(), p2.getX());
        this.maxX = Math.max(p1.getX(), p2.getX());
        this.minY = Math.min(p1.getY(), p2.getY());
        this.maxY = Math.max(p1.getY(), p2.getY());
    }

    public Bounds() {
        this(-100, 0, -100, 0);
    }

    public boolean contains(Point p) {

        if ( p.getX() < minX || p.getX() > maxX || p.getY() < minY || p.getY() > maxY )
        {
            return false;
        }

        return true;

    }

}
