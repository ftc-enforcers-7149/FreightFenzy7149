package org.firstinspires.ftc.teamcode.Mattu.ballsnblocks;

import org.opencv.core.Point;

public class Circle {
    public Point center;
    public double radius;

    public Circle() {
        this.center = new Point();
        this.radius = 0.0D;
    }

    public Circle(Point center, double radius) {
        this.center = center.clone();
        this.radius = radius;
    }

    public Circle(double[] vals) {
        this();
        this.set(vals);
    }

    public void set(double[] vals) {
        if (vals != null) {
            this.center.x = vals.length > 0 ? vals[0] : 0.0D;
            this.center.y = vals.length > 1 ? vals[1] : 0.0D;
            this.radius = vals.length > 2 ? vals[2] : 0.0D;
        } else {
            this.center.x = 0.0D;
            this.center.y = 0.0D;
            this.radius = 0.0D;
        }
    }

    public String toString() {
        return "{ " + this.center + " " + this.radius + " }";
    }
}
