package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Touchpad;

// Vector packet handling class. Sure it may be redundant, but it's fun!
public class VectorPacket {

    // Important storage variables
    private final Point pos, lastPos;
    private double time, lastTime;

    // deadzone var; if less than, v = 0;
    private final double deadzone = .005;

    // Standard constructor when there's a starting point

    public VectorPacket(Point pos, Point lastPos, double time, double lastTime) {
        this.pos = pos; this.lastPos = lastPos; this.time = time; this.lastTime = lastTime;
    }

    // Zeroed constructor

    public VectorPacket() {
        pos = new Point(); lastPos = new Point(); time = 0; lastTime = 0;
    }

    public void setVectorPacket(VectorPacket vP) {
        pos.setPoint(vP.pos); lastPos.setPoint(vP.pos);
        time = vP.time; lastTime = vP.lastTime;
    }

    public double getDistance() {
        return pos.distanceTo(lastPos);
    }

    public double getVelocity() {
        double velocity = Math.sqrt(Math.pow(getXVel(), 2) + Math.pow(getYVel(), 2));
        return velocity > deadzone ? velocity : 0;
    }

    public double getAngle(AngleUnit angleUnit) {
        return lastPos.angleTo(pos, angleUnit);
    }

    public double getXVel() {
        return (pos.getX() - lastPos.getX()) / (time - lastTime);
    }

    public double getYVel() {
        return (pos.getY() - lastPos.getY()) / (time - lastTime);
    }

    public double getTime() {
        return time;
    }

    public void setTime(double time) {
        this.time = time;
    }

}
