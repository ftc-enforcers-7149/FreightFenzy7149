package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Alliance;
import org.firstinspires.ftc.teamcode.Autonomous.Auto_V2_5;

@Autonomous(name = "Distance Correction Test")
//@Disabled
public class DistanceCorrectionTest extends Auto_V2_5 {

    @Override
    public void auto() {
        distCorrect.startRunning();

        while(opModeIsActive()) {
            updateInputs();

            drive.setPoseEstimate(distCorrect.correctPoseWithDist(drive.getPoseEstimate().getHeading()));
            telemetry.addData("Pose x:", drive.getPoseEstimate().getX());
            telemetry.addData("Pose y:", drive.getPoseEstimate().getY());
            telemetry.addData("Distance F Health:", distCorrect.getSensorF().getDeviceClient().getHealthStatus());

            updateOutputs();
        }
    }

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }
}
