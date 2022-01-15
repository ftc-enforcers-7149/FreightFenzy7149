package org.firstinspires.ftc.teamcode.Subsystems.Utils;

import com.enforcers7149.touchpadplusplus.src.Utils.TouchpadHandler;
import com.enforcers7149.touchpadplusplus.src.Utils.Updateable;

import java.util.ArrayList;

public class EnforcersTouchpadHandler extends TouchpadHandler implements Input {

    public EnforcersTouchpadHandler(Updateable... updateables) {
        super(updateables);
    }

    @Override
    public void addInputs(Updateable... updateables) {
        super.addInputs(updateables);
    }

    @Override
    public void addInputs(ArrayList<Updateable> objects) {
        super.addInputs(objects);
    }

    @Override
    public void updateInput() {
        super.updateInputs();
    }

    @Override
    public void startInput() {
        super.startInputs();
    }

    @Override
    public void stopInput() {
            super.stopInputs();
    }
}
