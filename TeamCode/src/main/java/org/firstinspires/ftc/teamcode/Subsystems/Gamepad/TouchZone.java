package org.firstinspires.ftc.teamcode.Subsystems.Gamepad;


public class TouchZone {

    Touchpad touchpad;
    String name;
    double leftX, rightX, bottomY, topY;

    public TouchZone(String name, double leftX, double rightX, double bottomY, double topY) {

        this.name = name;
        this.leftX = leftX;
        this.rightX = rightX;
        this.bottomY = bottomY;
        this.topY = topY;

    }

    public void setTouchButton(Touchpad touch) {
        touchpad = touch;
    }

    public boolean isRange() {

        switch(touchpad.getNumFingers()) {
            case 1:
                if((touchpad.getFingerOneX() <= rightX && touchpad.getFingerOneX() >= leftX)
                        && (touchpad.getFingerOneY() <= topY && touchpad.getFingerOneY() >= bottomY))
                    return true;
                break;

            case 2:
                if(((touchpad.getFingerOneX() <= rightX && touchpad.getFingerOneX() >= leftX)
                        && (touchpad.getFingerOneY() <= topY && touchpad.getFingerOneY() >= bottomY))
                        ||
                        ((touchpad.getFingerTwoX() <= rightX && touchpad.getFingerTwoX() >= leftX)
                                && (touchpad.getFingerTwoY() <= topY && touchpad.getFingerTwoY() >= bottomY)))
                    return true;
                break;
            default:
                return false;
        }

        return false;

    }

    public String getName() {
        return name;
    }

}