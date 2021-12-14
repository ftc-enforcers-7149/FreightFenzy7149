package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils;

public class Bounds {

    private double leftX, rightX, bottomY, topY;

    public Bounds(double leftX, double rightX, double bottomY, double topY) {
        this.leftX = leftX;
        this.rightX = rightX;
        this.bottomY = bottomY;
        this.topY = topY;
    }

    public Bounds(Point p1, Point p2) {
        this.leftX = Math.min(p1.getX(), p2.getX());
        this.rightX = Math.max(p1.getX(), p2.getX());
        this.bottomY = Math.min(p1.getY(), p2.getY());
        this.topY = Math.max(p1.getY(), p2.getY());
    }

    public Bounds() {
        this(0, 0, 0, 0);
    }

    public boolean contains(Point p) {
        return (p.getX() >= leftX) && (p.getX() <= rightX) &&
                (p.getY() >= bottomY) && (p.getY() <= topY);
    }

    public void setBounds(double leftX, double rightX, double bottomY, double topY) {
        this.leftX = leftX;
        this.rightX = rightX;
        this.bottomY = bottomY;
        this.topY = topY;
    }

    public void setBounds(Point p1, Point p2) {
        this.leftX = Math.min(p1.getX(), p2.getX());
        this.rightX = Math.max(p1.getX(), p2.getX());
        this.bottomY = Math.min(p1.getY(), p2.getY());
        this.topY = Math.max(p1.getY(), p2.getY());
    }

    public void setBounds(Bounds b) {
        this.leftX = b.getLeftX();
        this.rightX = b.getRightX();
        this.bottomY = b.getBottomY();
        this.topY = b.getTopY();
    }

    public double getLeftX() { return leftX; }
    public double getRightX() { return rightX; }
    public double getBottomY() { return bottomY; }
    public double getTopY() { return topY; }
}
