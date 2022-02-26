package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Webcam.OpenCV;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.ballsnblocks.BallsNBlocksPipeline;

@Autonomous(name = "Test Balls N Blocks")
//@Disabled
public class TestBallsNBlocks extends OpMode {

    private OpenCV detector;
    private BallsNBlocksPipeline pipeline;

    private int index = 2;

    private boolean lastA;

    @Override
    public void init() {
        detector = new OpenCV(hardwareMap, FtcDashboard.getInstance());

        pipeline = new BallsNBlocksPipeline();
        detector.start(pipeline);
    }

    @Override
    public void init_loop() {
        detector.update();
    }

    @Override
    public void loop() {
        detector.update();

        if (gamepad1.a && !lastA) {
            index++; if (index > 2) index = 0;
            switch (index) {
                case 0:
                    detector.setPipeline(pipeline.ballsPipeline);
                    break;
                case 1:
                    detector.setPipeline(pipeline.blocksPipeline);
                    break;
                case 2:
                    detector.setPipeline(pipeline);
                    break;
            }
        }

        switch (index) {
            case 0:
                telemetry.addLine("Balls Pipeline");
                break;
            case 1:
                telemetry.addLine("Blocks Pipeline");
                break;
            case 2:
                telemetry.addLine("Balls N Blocks Pipeline");
                break;
        }

        lastA = gamepad1.a;
    }
}
