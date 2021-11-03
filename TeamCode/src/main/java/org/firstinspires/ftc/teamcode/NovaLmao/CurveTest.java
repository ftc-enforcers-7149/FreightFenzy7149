package org.firstinspires.ftc.teamcode.NovaLmao;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Base;

// Tank drive class demonstrating how the curve handling accel class works :)

public class CurveTest extends TeleOp_Base {

    protected double cTime, lTime, sTimeL, sTimeR;

    @Override
    protected void getInput() {

    }

    @Override
    protected void updateStateMachine() {

    }

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
