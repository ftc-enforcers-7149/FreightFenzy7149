package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Base;

/*
                 wF
   -------------------------------
   |  Field                      |
   |             sF              |
   |      -----------------      |
   |      |               |      |
   |      |               |      |
wL |   sL |     Robot     | sR   | wR
   |      |               |      |
   |      |               |      |
   |      -----------------      |
   |             sB              |
   |                      Field  |
   -------------------------------
                 wB
*/

@TeleOp(name = "Test Distance Sensors")
@Disabled
public class TestDistanceSensors extends TeleOp_Base {

    private static final double RADIUS = 9;

    //Front and Right sensors
    private ModernRoboticsI2cRangeSensor sF, sR;
    private Gyroscope gyro;

    private enum Wall {
        FRONT, RIGHT, BACK, LEFT
    }

    public void init() {
        initializeDrive();
        initializeVars();

        gyro = new Gyroscope(hardwareMap);
        gyro.setOffset(90);
        sF = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distF");
        sR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distR");
    }

    public void loop() {
        getInput();

        driveArcade();

        double angleDeg = gyro.getNewYaw();
        double rawSF = sF.getDistance(DistanceUnit.INCH);
        double rawSR = sR.getDistance(DistanceUnit.INCH);

        if (rawSF > 72) rawSF = 0;
        else rawSF += RADIUS;
        if (rawSR > 72) rawSR = 0;
        else rawSR += RADIUS;

        Wall frontSees, rightSees;

        //Determine which sensor sees which wall. With all four sensors we'd just pick
        //whichever two adjacent sensors have the smallest values
        if (adjSidesSeeSame(angleDeg, rawSF, rawSR)) {
            if (angleDeg < 90) {
                frontSees = Wall.RIGHT;
                rightSees = Wall.RIGHT;
            }
            else if (angleDeg < 180) {
                frontSees = Wall.FRONT;
                rightSees = Wall.FRONT;
            }
            else if (angleDeg < 270) {
                frontSees = Wall.LEFT;
                rightSees = Wall.LEFT;
            }
            else {
                frontSees = Wall.BACK;
                rightSees = Wall.BACK;
            }
        }
        //This is botched estimates. In reality we'd want 4 sensors, so the other two can
        //be swapped in for these two
        else {
            if (angleDeg >= 315 || angleDeg < 45) {
                frontSees = Wall.RIGHT;
                rightSees = Wall.BACK;
            }
            else if (angleDeg < 135) {
                frontSees = Wall.FRONT;
                rightSees = Wall.RIGHT;
            }
            else if (angleDeg < 225) {
                frontSees = Wall.LEFT;
                rightSees = Wall.FRONT;
            }
            else {
                frontSees = Wall.BACK;
                rightSees = Wall.LEFT;
            }
        }

        //Assuming sF sees wF, sR sees wR, etc
        double baseFTF = Math.cos(Math.abs(Math.toRadians(90 - angleDeg)));

        //Assuming sF sees wR, sR sees wB, etc
        double baseFTR = Math.cos(Math.toRadians(angleDeg));

        //Assuming sF sees wB, sR sees sL, etc
        double baseFTB = Math.cos(Math.abs(Math.toRadians(270 - angleDeg)));

        //Assuming sF sees wL, sR sees sF, etc
        double baseFTL = Math.cos(Math.abs(Math.toRadians(180 - angleDeg)));

        //Calculate each sensor's distance to their closest wall
        double distSF, distSR;
        double robotX=0, robotY=0;

        switch(frontSees) {
            default:
            case FRONT:
                distSF = rawSF * baseFTF;
                if (distSF != 0) robotY = 144-distSF;
                telemetry.addData("Front to Front is ", distSF);
                break;
            case RIGHT:
                distSF = rawSF * baseFTR;
                if (distSF != 0) robotX = 96-distSF;
                telemetry.addData("Front to Right is ", distSF);
                break;
            case BACK:
                distSF = rawSF * baseFTB;
                if (distSF != 0) robotY = distSF;
                telemetry.addData("Front to Back is ", distSF);
                break;
            case LEFT:
                distSF = rawSF * baseFTL;
                if (distSF != 0) robotX = distSF;
                telemetry.addData("Front to Left is ", distSF);
                break;
        }
        switch(rightSees) {
            case FRONT:
                distSR = rawSR * baseFTL;
                if (distSR != 0) robotY = 144-distSR;
                telemetry.addData("Right to Front is ", distSR);
                break;
            default:
            case RIGHT:
                distSR = rawSR * baseFTF;
                if (distSR != 0) robotX = 96-distSR;
                telemetry.addData("Right to Right is ", distSR);
                break;
            case BACK:
                distSR = rawSR * baseFTR;
                if (distSR != 0) robotY = distSR;
                telemetry.addData("Right to Back is ", distSR);
                break;
            case LEFT:
                distSR = rawSR * baseFTB;
                if (distSR != 0) robotX = distSR;
                telemetry.addData("Right to Left is ", distSR);
                break;
        }

        telemetry.addData("Robot X: ", robotX);
        telemetry.addData("Robot Y: ", robotY);
        telemetry.addLine();

        double simplifiedAngle = angleDeg % 90;

        if (90 - simplifiedAngle < 7 || simplifiedAngle < 7) {
            telemetry.addData("Expected Ratio: ", "None");
            telemetry.addData("Actual Ratio: ", "None");
        }
        else {
            telemetry.addData("Expected Ratio: ", Math.abs(Math.tan(Math.toRadians(angleDeg))));

            if (rawSR == 0) telemetry.addData("Actual Ratio: ", "None");
            else telemetry.addData("Actual Ratio: ", Math.abs(rawSF / rawSR));
        }
        telemetry.addLine();
        telemetry.addLine("Raw Readings");
        telemetry.addData("Raw Gyro", angleDeg);
        telemetry.addData("Raw sF: ", rawSF);
        telemetry.addData("Raw sR: ", rawSR);

        updateStateMachine();
    }

    /**
     * Calculates a predicted ratio for when the two sensors would see the same side, and
     * uses it to determine if they actually do or not
     * @param angleDeg Robot heading in degrees
     * @param left Sensor leftmost around the robot
     * @param right Sensor rightmost around the robot
     * @return Whether or not the two sensors predictably see the same side
     */
    private boolean adjSidesSeeSame(double angleDeg, double left, double right) {
        double simplifiedAngle = angleDeg % 90;

        //
        if (90 - simplifiedAngle < 7 || simplifiedAngle < 7) return false;

        double sameSideRatio = Math.abs(Math.tan(Math.toRadians(angleDeg)));
        double actualRatio = Math.abs(left/right);

        //Tolerance of +- 0.15 in the ratio
        return  sameSideRatio >= actualRatio - 0.3 &&
                sameSideRatio <= actualRatio + 0.3;
    }

    public void stop() {
        setMotorPowers(0, 0, 0, 0);
    }

    public void getInput() {
        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y;
        rightX = -Math.signum(gamepad1.right_stick_x) * Math.abs(Math.pow(gamepad1.right_stick_x, 3));
    }

    public void updateStateMachine() {
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;
    }
}
