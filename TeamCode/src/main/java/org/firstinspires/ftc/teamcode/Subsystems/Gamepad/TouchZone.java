package org.firstinspires.ftc.teamcode.Subsystems.Gamepad;


public class TouchZone extends TouchObject<Boolean> {

    private double leftX, rightX, bottomY, topY;

    public TouchZone(String name, Touchpad touchpad, double leftX, double rightX, double bottomY, double topY) {

        super(name, touchpad);
        this.leftX = leftX;
        this.rightX = rightX;
        this.bottomY = bottomY;
        this.topY = topY;

    }


    public Boolean update() {

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

}