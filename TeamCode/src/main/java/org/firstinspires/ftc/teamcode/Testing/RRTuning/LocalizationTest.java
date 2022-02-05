package org.firstinspires.ftc.teamcode.Testing.RRTuning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Odometry.DriveWheels.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.BulkRead;

import java.util.List;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
//@Disabled
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BulkRead bReadCH = new BulkRead(hardwareMap, "Control Hub");
        BulkRead bReadEH = new BulkRead(hardwareMap, "Expansion Hub");
        MecanumDrive drive = new MecanumDrive(hardwareMap, bReadCH, bReadEH);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        drive.startInput();
        drive.startOutput();

        while (!isStopRequested()) {
            bReadCH.updateInput();
            bReadEH.updateInput();
            drive.updateInput();

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));

            List<Double> wheelPositions = drive.getWheelPositions();

            telemetry.addLine("Wheel Positions:");
            telemetry.addLine(wheelPositions.get(0).toString());
            telemetry.addLine(wheelPositions.get(1).toString());
            telemetry.addLine(wheelPositions.get(2).toString());
            telemetry.addLine(wheelPositions.get(3).toString());
            telemetry.update();

            drive.updateOutput();
        }

        drive.startInput();
        drive.stopOutput();
    }
}
