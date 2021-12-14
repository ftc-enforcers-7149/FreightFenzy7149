package org.firstinspires.ftc.teamcode.Subsystems.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Utils.Scale;

import java.lang.reflect.Type;

public abstract class TouchObject<Object> {

    Touchpad touchpad;
    String name;

    public TouchObject(String name, Touchpad touchpad) {

        this.name = name;
        this.touchpad = touchpad;

    }

    public String getName() {

        return name;

    }

    public abstract Object update();

}
