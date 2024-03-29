package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Webcam.OpenCV;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam.TSEPipeline;

@Autonomous(name = "Test Webcam")
@Disabled
public class TestWebcam extends OpMode {

    private OpenCV tseDetector;
    private TSEPipeline pipeline;

    private int index = 0;

    private boolean lastA;

    @Override
    public void init() {
        tseDetector = new OpenCV(hardwareMap, FtcDashboard.getInstance());

        pipeline = new TSEPipeline(0, 320, 360, 320);

        tseDetector.start(pipeline);
    }

    @Override
    public void init_loop() {
        tseDetector.update();
    }

    @Override
    public void loop() {
        tseDetector.update();

        if (gamepad1.a && !lastA) {
            index++; if (index > 3) index = 0;
            switch (index) {
                case 0:
                    tseDetector.setPipeline(new TSEPipeline(180, 320, 180, 320));
                    break;
                case 1:
                    tseDetector.setPipeline(new TSEPipeline(0, 180, 320, 180));
                    break;
                case 2:
                    tseDetector.setPipeline(new TSEPipeline(180, 320, 180, 320));
                    break;
                case 3:
                    tseDetector.setPipeline(new TSEPipeline(0, 180, 320, 180));
                    break;
            }
        }

        switch (index) {
            case 0:
                telemetry.addLine("Red Left");
                break;
            case 1:
                telemetry.addLine("Red Right");
                break;
            case 2:
                telemetry.addLine("Blue Left");
                break;
            case 3:
                telemetry.addLine("Blue Right");
                break;
        }

        lastA = gamepad1.a;
    }
}
