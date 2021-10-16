package org.firstinspires.ftc.teamcode.Mattu.recordnreplay;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_Base;

import java.util.ArrayList;

/**
 * Recording is the first step in our Record 'N Replay Autonomous. At the face, it is simply
 * a TeleOp program that allows the driver to drive as normal. However, any time the driver does
 * a specific action (or even simply sits still), the program records key points with data as to
 * where the robot is and what the robot is doing. Then, when the program stops, the list of points,
 * which we're calling our path, is serialized and saved to the robot.
 */
@TeleOp(name = "Record")
@Disabled
public class Record extends TeleOp_Base {

    //Generated path
    private ArrayList<KeyPoint> path;
    private ArrayList<String> pathCode;

    //Timing
    private double lastPositionTime;

    private double lastX, lastY, lastH;

    //Functional inputs
    private boolean currA, lastA;
    private boolean currB, lastB;
    private double currRightTrig, lastRightTrig;

    @Override
    public void init() {
        //All of these methods initialize hardware or class variables (with default values)
        initializeDrive();
        initializeBulkRead();
        initializeGyro();
        try {
            initializeOdometry();
        } catch (Exception e) {
            e.printStackTrace();
        }
        initializeVars();

        //Create the path with its starting position
        path = new ArrayList<KeyPoint>();
        path.add(new KeyPoint(drive.getPoseEstimate().getX(),
                drive.getPoseEstimate().getY(),
                drive.getPoseEstimate().getHeading(),
                Actions.DO_NOTHING, false, false));

        //The pathCode will be used to give a physical (and usable) representation of the points in
        //the generated path. That way, we'd be able to actually edit the points and save them
        //as a literal code file.
        pathCode = new ArrayList<String>();
        pathCode.add("path.add(new KeyPoint(" +
                drive.getPoseEstimate().getX() +
                drive.getPoseEstimate().getY() +
                drive.getPoseEstimate().getHeading() +
                "Actions.DO_NOTHING, false, false));");

        lastPositionTime = 0;
        lastX = 0; lastY = 0; lastH = 0;
    }

    @Override
    public void start() {
        lastPositionTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        //Update sensors and inputs from gamepad
        updateBulkRead();
        gyro.update();
        drive.update();
        getInput();

        //Drive the robot
        driveHeadless(gyro.getRawYaw(), gamepad1.y);

        //Get current position
        double x = drive.getPoseEstimate().getX();
        double y = drive.getPoseEstimate().getY();
        double h = drive.getPoseEstimate().getHeading();

        //If the robot has moved at all since the last recorded KeyPoint
        boolean moved = Math.abs(x-lastX) > 1 || Math.abs(y-lastY) > 1;
        //If the robot has turned at all since the last recorded KeyPoint
        boolean turned;
        if (h > lastH) {
            turned = h - lastH > Math.toRadians(1) && h - lastH < (2 * Math.PI) - Math.toRadians(1);
        }
        else {
            turned = lastH - h > Math.toRadians(1) && lastH - h < (2 * Math.PI) - Math.toRadians(1);
        }

        //The current time (in milliseconds)
        double currentTime = System.currentTimeMillis();

        //On press of A
        if (currA && !lastA) {
            path.add(new KeyPoint(x, y, h, Actions.DO_NOTHING, moved, turned));

            pathCode.add("//Pressed A\n" +
                    "path.add(new KeyPoint(" +
                    x + ", " +
                    y + ", " +
                    h +
                    ", Actions.DO_NOTHING, " + moved + ", " + turned + "));");

            lastX = x; lastY = y; lastH = h;
            lastPositionTime = currentTime;
        }

        //On press of B
        if (currB && !lastB) {
            path.add(new KeyPoint(x, y, h, Actions.DO_NOTHING, moved, turned));

            pathCode.add("//Pressed B\n" +
                    "path.add(new KeyPoint(" +
                    x + ", " +
                    y + ", " +
                    h +
                    ", Actions.DO_NOTHING, " + moved + ", " + turned + "));");

            lastX = x; lastY = y; lastH = h;
            lastPositionTime = currentTime;
        }

        //On press of Right Trigger
        if ((currRightTrig > 0.1) && !(lastRightTrig > 0.1)) {
            path.add(new KeyPoint(x, y, h, Actions.DO_NOTHING, moved, turned));

            pathCode.add("//Pressed Right Trigger\n" +
                    "path.add(new KeyPoint(" +
                    x + ", " +
                    y + ", " +
                    h +
                    ", Actions.DO_NOTHING, " + moved + ", " + turned + "));");

            lastX = x; lastY = y; lastH = h;
            lastPositionTime = currentTime;
        }
        //On release of Right Trigger
        else if (!(currRightTrig > 0.1) && (lastRightTrig > 0.1)) {
            path.add(new KeyPoint(x, y, h, Actions.DO_NOTHING, moved, turned));

            pathCode.add("//Released Right Trigger\n" +
                    "path.add(new KeyPoint(" +
                    x + ", " +
                    y + ", " +
                    h +
                    ", Actions.DO_NOTHING, " + moved + ", " + turned + "));");

            lastX = x; lastY = y; lastH = h;
            lastPositionTime = currentTime;
        }

        //Get new position every 250 milliseconds
        //The time resets after any other KeyPoint is added
        if (currentTime - lastPositionTime >= 250) {
            //If the position has changed, add it as a new one
            if (moved || turned) {
                path.add(new KeyPoint(x, y, h, Actions.DO_NOTHING, moved, turned));

                pathCode.add("//Standard movement\n" +
                        "path.add(new KeyPoint(" +
                        x + ", " +
                        y + ", " +
                        h +
                        ", Actions.DO_NOTHING, " + moved + ", " + turned + "));");
            }
            //If the position hasn't changed for the whole 250 ms, add a "delay action"
            else {
                path.add(new KeyPoint(x, y, h, Actions.DELAY, false, false));

                pathCode.add("//Delay\n" +
                        "path.add(new KeyPoint(" +
                        x + ", " +
                        y + ", " +
                        h +
                        ", Actions.DELAY, false, false));");
            }

            lastX = x; lastY = y; lastH = h;
            //Here 250 milliseconds are added to lastPositionTime instead of setting it to current
            //time in case the current update of the loop does not fall on a time that is divisible
            //by 250. It basically keeps exact 250 millisecond delays no matter how long a loop
            //lasts, or how long the delay between is.
            lastPositionTime += 250;
        }

        //Basic telemetry of the robot's position and how the path is going
        telemetry.addData("Position: ", drive.getPoseEstimate());
        telemetry.addData("Total point: ", path.size());

        updateStateMachine();
    }

    @Override
    public void stop() {
        //Stop the robot
        setMotorPowers(0, 0, 0, 0);

        //If any path was created (aka the robot wasn't just started and immediately stopped),
        //save both the serialized path and the literal code
        if (path.size() > 1) {
            Path.savePathCode(pathCode, "/storage/self/primary/pathCode.txt");
            Path.savePath(path, "/storage/self/primary/path.ser");
        }
    }

    @Override
    protected void getInput() {
        leftX = curveInput(gamepad1.left_stick_x, 7)*lim;
        leftY = curveInput(gamepad1.left_stick_y, 7)*lim;
        rightX = curveInput(gamepad1.right_stick_x, 7)*lim;
        currA = gamepad1.a;
        currB = gamepad1.b;
        currRightTrig = gamepad1.right_trigger;
    }

    @Override
    protected void updateStateMachine() {
        lastLeftX = leftX; lastLeftY = leftY; lastRightX = rightX;
        lastA = currA; lastB = currB; lastRightTrig = currRightTrig;
    }
}
