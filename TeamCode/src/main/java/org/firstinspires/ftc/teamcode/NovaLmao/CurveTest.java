package org.firstinspires.ftc.teamcode.NovaLmao;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Base;

public class CurveTest extends OpMode {

    public enum AccelState {

        TURNING,
        ACCEL_TURNING,
        ACCELERATING,
        POST_TURN_RAMP,
        POST_ACCELERATING,
        RESTING

    }

    private AccelState aStateL = AccelState.RESTING, aStateR = AccelState.RESTING, lastAStateL, lastAStateR;

    public void init() {

    }


    public void loop() {



    }

}
