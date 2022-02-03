package org.firstinspires.ftc.teamcode.Subsystems;

import com.enforcers7149.touchpadplusplus.src.Touchpad;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;

public class EnforcersTouchpad implements Input {

    public Touchpad t;

    public EnforcersTouchpad(Gamepad g) {

        t = new Touchpad(g);

    }

    @Override
    public void updateInput() {
        t.updateInput();
    }

}
