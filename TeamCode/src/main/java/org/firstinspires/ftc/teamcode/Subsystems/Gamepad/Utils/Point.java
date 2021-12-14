package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils;

public class Point {

    private double x, y;

    public Point(double x, double y) {
        this.x = x; this.y = y;
    }

    public Point() {
        this(0, 0);
    }

    public void setPoint(double x, double y) {
        this.x = x; this.y = y;
    }

    public void setPoint(Point p) {
        this.x = p.getX(); this.y = p.getY();
    }

    public void setX(double x) { this.x = x; }
    public void setY(double y) { this.y = y; }

    public double getX() { return x; }
    public double getY() { return y; }
}
