package org.firstinspires.ftc.teamcode.Subsystems.Gamepad.TouchObjects;

import org.firstinspires.ftc.teamcode.Subsystems.Gamepad.Touchpad;
import org.firstinspires.ftc.teamcode.Subsystems.Utils.Scale;

import java.lang.reflect.Type;

public abstract class TouchObject<T> {

    Touchpad touchpad;
    String name;

    public enum Type {

        BOOLEAN,
        LEFT_SWIPE,
        RIGHT_SWIPE,
        UP_SWIPE,
        DOWN_SWIPE,
        X_AXIS,
        Y_AXIS

    }

    public TouchObject(String name, Touchpad touchpad) {

        this.name = name;
        this.touchpad = touchpad;

    }

    public String getName() {

        return name;

    }

    public abstract T update();

}
