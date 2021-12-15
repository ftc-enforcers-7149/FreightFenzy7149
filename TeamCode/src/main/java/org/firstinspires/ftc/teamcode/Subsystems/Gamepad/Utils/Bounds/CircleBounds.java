package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils.Bounds;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils.Point;

public class CircleBounds extends Bounds {

    double r;
    Point center;

    public CircleBounds(Point center, double r) {

        super(center.getX() - r, center.getX() + r, center.getY() - r, center.getY() + r);
        this.r = r;
        this.center = center;

    }

    @Override
    public boolean contains(Point p) {

        if(super.contains(p)) return false;

        return p.distanceTo(center) > r;

    }

}
