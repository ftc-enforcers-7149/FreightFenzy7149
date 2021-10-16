package org.firstinspires.ftc.teamcode.NovaLmao;

import android.widget.Button;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.HashMap;

public class TouchButton {

    GamepadFun gFun;

    double standardMult;
    double leftX, rightX, bottomY, topY;
    double time, lastTime = 0;
    double pollingTime;

    public TouchButton(GamepadFun gFun, double leftX, double rightX, double bottomY, double topY, double pollingTime) {

        this.leftX = leftX;
        this.rightX = rightX;
        this.bottomY = bottomY;
        this.topY = topY;
        this.pollingTime = pollingTime;

    }

    public TouchButton(GamepadFun gFun, double leftX, double rightX, double bottomY, double topY, double pollingTime, double standardMult) {

        this.leftX = leftX;
        this.rightX = rightX;
        this.bottomY = bottomY;
        this.topY = topY;
        this.pollingTime = pollingTime;
        this.standardMult = standardMult;

    }

    public void update() {

        if(lastTime == time + pollingTime) lastTime = time;
        time = NanoClock.system().seconds();

    }

    public VectorPacket swipeVector() {

        VectorPacket v = new VectorPacket();
        boolean fingerOneInRange, fingerTwoInRange;

        switch(gFun.getNumFingers()) {

            case 1:

                fingerOneInRange = (gFun.getFingerOneX() <= rightX && gFun.getFingerOneX() >= leftX)
                        && (gFun.getFingerOneY() <= topY && gFun.getFingerOneY() >= bottomY);

                if(fingerOneInRange) {

                    v.setTime(time);
                    v.setLastTime(lastTime);
                    v.setX(gFun.getFingerOneX());
                    v.setY(gFun.getFingerOneY());
                    v.setLastX(gFun.getLastFingerOneX());
                    v.setLastY(gFun.getLastFingerOneY());

                }
                break;

            case 2:

                //check for both first
                fingerOneInRange = (gFun.getFingerOneX() <= rightX && gFun.getFingerOneX() >= leftX)
                        && (gFun.getFingerOneY() <= topY && gFun.getFingerOneY() >= bottomY);
                fingerTwoInRange = ((gFun.getFingerTwoX() <= rightX && gFun.getFingerTwoX() >= leftX)
                        && (gFun.getFingerTwoY() <= topY && gFun.getFingerTwoY() >= bottomY));

                if(fingerOneInRange && fingerTwoInRange) {

                    v.setTime(time);
                    v.setLastTime(lastTime);
                    v.setX(gFun.getFingerOneX());
                    v.setY(gFun.getFingerOneY());
                    v.setLastX(gFun.getLastFingerOneX());
                    v.setLastY(gFun.getLastFingerOneY());

                }
                else if (fingerTwoInRange) {

                    v.setTime(time);
                    v.setLastTime(lastTime);
                    v.setX(gFun.getFingerTwoX());
                    v.setY(gFun.getFingerTwoY());
                    v.setLastX(gFun.getLastFingerTwoX());
                    v.setLastY(gFun.getLastFingerTwoY());

                }
                else if(fingerOneInRange) {

                    v.setTime(time);
                    v.setLastTime(lastTime);
                    v.setX((Math.abs(gFun.getFingerTwoX() - gFun.getFingerOneX())) / 2);
                    v.setY((Math.abs(gFun.getFingerTwoY() - gFun.getFingerOneY())) / 2);
                    v.setLastX((Math.abs(gFun.getLastFingerTwoX() - gFun.getLastFingerOneX())) / 2);
                    v.setLastY((Math.abs(gFun.getLastFingerTwoY() - gFun.getLastFingerOneY())) / 2);

                }

                break;

            default:
                break;

        }

        return v;

    }

    public boolean isRange() {

        switch(gFun.getNumFingers()) {
            case 1:
                if((gFun.getFingerOneX() <= rightX && gFun.getFingerOneX() >= leftX)
                        && (gFun.getFingerOneY() <= topY && gFun.getFingerOneY() >= bottomY))
                    return true;
                break;

            case 2:
                if(((gFun.getFingerOneX() <= rightX && gFun.getFingerOneX() >= leftX)
                        && (gFun.getFingerOneY() <= topY && gFun.getFingerOneY() >= bottomY))
                        ||
                        ((gFun.getFingerTwoX() <= rightX && gFun.getFingerTwoX() >= leftX)
                        && (gFun.getFingerTwoY() <= topY && gFun.getFingerTwoY() >= bottomY)))
                    return true;
                break;
            default:
                return false;
        }

        return false;

    }

    public class VectorPacket {

        private double x, y, lastX, lastY, time, lastTime;

        public VectorPacket(double x, double y, double lastX, double lastY, double time, double lastTime) {

            this.x = x; this.y = y; this.lastX = lastX; this.lastY = lastY; this.time = time; this.lastTime = lastTime;

        }

        public VectorPacket() {
            x = 0; y = 0; lastX = 0; lastY = 0; time = 0; lastTime = 0;
        }

        public double getDistance() {
            return Math.sqrt((Math.pow((x - lastX), 2) + Math.pow((y - lastY), 2)));
        }

        public double getVelocity() {
            return Math.sqrt(Math.pow(getXVel(), 2) + Math.pow(getYVel(), 2));
        }

        public double getAngle() {
            return Math.toDegrees(Math.atan((y - lastY) /(x - lastX)));
        }

        public void setX(double x) {
            this.x = x;
        }

        public void setLastX(double lastX) {
            this.lastX  = x;
        }

        public void setY(double y) {
            this.y = y;
        }

        public void setLastY(double lastY) {
            this.lastY  = y;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public double getXVel() {
            return (x - lastX) / (time - lastTime);
        }

        public double getYVel() {
            return (y - lastY) / (time - lastTime);
        }

        public double getTime() {
            return time;
        }

        public void setTime(double time) {
            this.time = time;
        }

        public double getLastTime() {
            return lastTime;
        }

        public void setLastTime(double lastTime) {
            this.lastTime = lastTime;
        }

    }


}
