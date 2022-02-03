package org.firstinspires.ftc.teamcode.Subsystems;

import com.enforcers7149.touchpadplusplus.src.TouchObjects.Button;
import com.enforcers7149.touchpadplusplus.src.Touchpad;
import com.enforcers7149.touchpadplusplus.src.Utils.Bounds.Bounds;

import org.firstinspires.ftc.teamcode.Subsystems.Utils.Input;

public class EnforcersButton implements Input {

    public Button b;

    public EnforcersButton(Touchpad t, boolean defaultValue, Bounds bounds, boolean requireClick) {

        b = new Button(t, defaultValue, bounds, requireClick);

    }

    @Override
    public void updateInput() {

        b.updateInput();

    }

}
