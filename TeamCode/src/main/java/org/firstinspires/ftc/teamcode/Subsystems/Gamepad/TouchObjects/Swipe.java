package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Touchpad;

public class Swipe extends TouchObject<Boolean> {

    private final int finger;
    private final Type s;

    public Swipe(Touchpad touchpad, Boolean defaultValue, int finger, Type s) {
        super(touchpad, defaultValue);
        this.finger = finger;
        this.s = s;
    }

    @Override
    public void updateInput() {
        boolean angleHoriz = finger == 1 ? touchpad.getV1().getAngle() >= -15 || touchpad.getV1().getAngle() <= 15
                : touchpad.getV2().getAngle() >= -15 || touchpad.getV2().getAngle() <= 15;
        boolean angleVert = finger == 1 ? touchpad.getV1().getAngle() >= 75 || touchpad.getV1().getAngle() <= 105
                : touchpad.getV2().getAngle() >= 75 || touchpad.getV2().getAngle() <= 105;
        boolean hold = finger == 1 ? touchpad.getLastFingerOneX() == touchpad.getFingerOneX() && touchpad.getLastFingerOneY() == touchpad.getFingerOneY()
                : touchpad.getLastFingerTwoX() == touchpad.getFingerTwoX() && touchpad.getLastFingerTwoY() == touchpad.getFingerTwoY();
        boolean left = finger == 1 ? touchpad.getFingerOneX() < 0 : touchpad.getFingerTwoX() < 0;
        boolean leftSwipe = finger == 1 ? touchpad.getV1().getXVel() < 0 : touchpad.getV2().getXVel() < 0;
        boolean right = finger == 1 ? touchpad.getFingerOneX() >= 0 : touchpad.getFingerTwoX() >= 0;
        boolean rightSwipe = finger == 1 ? touchpad.getV1().getXVel() > 0 : touchpad.getV2().getXVel() > 0;
        boolean up = finger == 1 ? touchpad.getFingerOneY() >= 0 : touchpad.getFingerTwoY() >= 0;
        boolean upSwipe = finger == 1 ? touchpad.getV1().getYVel() > 0 : touchpad.getV2().getYVel() > 0;
        boolean down = finger == 1 ? touchpad.getFingerOneY() < 0 : touchpad.getFingerTwoY() < 0;
        boolean downSwipe = finger == 1 ? touchpad.getV1().getYVel() < 0 : touchpad.getV2().getYVel() < 0;

        if(!touchpad.getFingerOn()) value = false;

        switch(s) {
            case BOOLEAN:
                value = leftSwipe || rightSwipe;
                return;
            case LEFT_SWIPE:
                value = (leftSwipe && angleHoriz) || ((!leftSwipe || !rightSwipe) && hold && left);
                return;
            case RIGHT_SWIPE:
                value = (rightSwipe && angleHoriz) || ((!leftSwipe || !rightSwipe) && hold && right);
                return;
            case UP_SWIPE:
                value = (upSwipe && angleVert) || ((!upSwipe || !downSwipe) && hold && up);
                return;
            case DOWN_SWIPE:
                value = (downSwipe && angleVert) || ((!upSwipe || !downSwipe) && hold && down);
                return;
            default:
                value = false;
        }
    }
}
