package org.firstinspires.ftc.teamcode.Subsystems.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Utils.Scale;

import java.lang.reflect.Type;

public abstract class TouchObject<Object> {

    Touchpad touchpad;
    String name;

    public enum SwipeType {

        BOOLEAN,
        LEFT_SWIPE,
        RIGHT_SWIPE,
        UP_SWIPE,
        DOWN_SWIPE,
        HORIZ_AXIS,
        VERT_AXIS

    }

    public TouchObject(String name, Touchpad touchpad) {

        this.name = name;
        this.touchpad = touchpad;

    }

    public String getName() {

        return name;

    }

    public abstract Object update();

}
