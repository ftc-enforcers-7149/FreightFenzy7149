package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Touchpad;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils.Point;
import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Utils.VectorPacket;

public class Swipe extends TouchObject<Boolean> {

    public enum SwipeType {
        ANY_SWIPE,
        LEFT_SWIPE,
        RIGHT_SWIPE,
        UP_SWIPE,
        DOWN_SWIPE,
    }

    private final SwipeType s;

    public Swipe(Touchpad touchpad, Boolean defaultValue, SwipeType s) {
        super(touchpad, defaultValue);
        this.s = s;
    }

    @Override
    public void updateInput() {
        Point finger, lastFinger;
        VectorPacket vel;
        if (touchpad.getNumFingers() == 2) {
            finger = touchpad.getFingerTwo();
            lastFinger = touchpad.getLastFingerTwo();
            vel = touchpad.getV2();
        }
        else if (touchpad.getNumFingers() == 1) {
            finger = touchpad.getFingerOne();
            lastFinger = touchpad.getLastFingerOne();
            vel = touchpad.getV1();
        }
        else {
            value = false;
            return;
        }

        double angle = vel.getAngle(AngleUnit.DEGREES);
        boolean angleHorz = (angle >= 345 && angle <= 15) || (angle >= 165 && angle <= 195);
        boolean angleVert = (angle >= 75 && angle <= 105) || (angle >= 255 && angle <= 285);

        double swipeVel = vel.getVelocity();
        boolean anySwipe = swipeVel > 0;
        boolean leftSwipe = vel.getXVel() < 0 && angleHorz;
        boolean rightSwipe = vel.getXVel() > 0 && angleHorz;
        boolean downSwipe = vel.getYVel() < 0 && angleVert;
        boolean upSwipe = vel.getYVel() > 0 && angleVert;

        boolean hold = finger.distanceTo(lastFinger) < 1;
        boolean left = finger.getX() < 0;
        boolean right = finger.getX() > 0;
        boolean down = finger.getX() < 0;
        boolean up = finger.getY() > 0;

        switch(s) {
            case ANY_SWIPE:
                value = anySwipe;
                return;
            case LEFT_SWIPE:
                value = leftSwipe || (!anySwipe && hold && left);
                return;
            case RIGHT_SWIPE:
                value = rightSwipe || (!anySwipe && hold && right);
                return;
            case UP_SWIPE:
                value = upSwipe || (!anySwipe && hold && up);
                return;
            case DOWN_SWIPE:
                value = downSwipe || (!anySwipe && hold && down);
                return;
            default:
                value = false;
        }
    }
}
