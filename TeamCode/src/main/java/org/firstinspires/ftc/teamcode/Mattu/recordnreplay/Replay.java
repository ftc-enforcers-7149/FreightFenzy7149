package org.firstinspires.ftc.teamcode.Mattu.recordnreplay;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Autonomous_Base;

import java.util.ArrayList;

/**
 * Replaying is the second - and more difficult - step in our Record 'N Replay Autonomous. Its
 * intention is to take the path recorded in Record and follow it, doing any actions needed on the
 * way. First it deserializes the saved path into an array of key points. Then, it iterates through
 * the points, either following a spline path or making simple turns to get to where the robot was
 * in the Record step. At any point with an action that needs attention, it stops following to
 * complete the action.
 */
@Autonomous(name = "Replay")
@Disabled
public class Replay extends Autonomous_Base {

    private ArrayList<KeyPoint> path;

    @Override
    public void runOpMode() throws InterruptedException {
        /// Init ///
        path = Path.loadPath("/storage/self/primary/path.ser");

        //Initializes robot hardware
        initializeDrive();
        initializeBulkRead();
        try {
            initializeOdometry();
        } catch (Exception e) {
            e.printStackTrace();
            throw new InterruptedException();
        }

        /// Init Loop ///
        while (!isStarted() && !isStopRequested()) {
            updateBulkRead();
        }
        if (isStopRequested()) return;

        /// Start ///
        KeyPoint startPoint = path.remove(0); //Initial position at (0, 0, 0), not for driving
        drive.setPoseEstimate(new Pose2d(startPoint.x, startPoint.y, startPoint.heading));

        /// Main Code ///

        while (opModeIsActive() && path.size() > 0) {
            //Update all sensors and odometry
            updateBulkRead();
            drive.update();

            //Follow all drives up till the next significant point (usually contains an action)
            Actions action = driveToNextSignificantPoint();

            //Handle current actions
            switch (action) {
                //Delays are created in the Record program when the robot stays relatively still
                //for 250 ms
                case DELAY:
                    double startTime = System.currentTimeMillis();
                    while (opModeIsActive() && System.currentTimeMillis() < startTime + 250) {
                        System.out.println("DELAYING");
                        updateBulkRead();
                        drive.update();
                        updateSubsystems();
                        updateTelemetry();
                    }
                    break;
                case START_INTAKE:
                    //Start intaking
                    break;
                case STOP_INTAKE:
                    //Stop intaking
                    break;
                default:
                    break;
            }

            //Basic telemetry of where the robot is in the path
            telemetry.addData("Action: ", action);
            telemetry.addData("Steps left: ", path.size());

            //Update outputs
            updateSubsystems();
            updateTelemetry();
        }

        /// Stop ///
        setMotorPowers(0, 0, 0, 0);
    }

    /**
     * Follows a spline consisting of all points up to and including the next important action.
     * Depending on the action, it will use more accurate moveTo and rotateTo functions.
     */
    private Actions driveToNextSignificantPoint() {
        ArrayList<Pose2d> points = new ArrayList<Pose2d>();

        //Get all points in path up to (not including) the next significant action
        //If there are no more actions to do, it stops one before the end of the path
        while (path.size() > 1 && path.get(0).action == Actions.DO_NOTHING && path.get(0).moved) {
            points.add(new Pose2d(path.get(0).x, path.get(0).y, path.get(0).heading));
            path.remove(0);
        }
        //Either the next significant action or the last point in the path
        KeyPoint actionPoint = path.remove(0);

        if (points.size() == 0 && actionPoint == null) return null;

        customFollower(points, actionPoint);

        /*TrajectoryBuilder trajBuilder = splineTrajectory(points, actionPoint);

        //Follows the full path
        if (points.size() > 0 || actionPoint.moved) {
            System.out.println("BUILDING TRAJECTORY");
            drive.followTrajectoryAsync(trajBuilder.build());
        }

        //Waits for drive to complete
        while (opModeIsActive() && drive.isBusy()) {
            updateBulkRead();
            drive.update();
            updateSubsystems();
            updateTelemetry();
        }*/

        //A simple turn if necessary
        if (actionPoint.turned && !actionPoint.moved) {
            makeTurn(actionPoint);
        }

        //Depending on which action is left, this determines if there should be an added accuracy
        //step for x, y, and or heading.
        if (actionPoint.action.move) moveTo(-actionPoint.y, actionPoint.x);
        if (actionPoint.action.rotate) rotateTo(Math.toDegrees(actionPoint.heading));

        return actionPoint.action;
    }

    /**
     * Builds a trajectory of splines to follow all given points (plus actionPoint)
     * @param points An array of points to follow
     * @param actionPoint An optional last point in this trajectory
     * @return The trajectory builder with all splines, meaning it still needs to be built
     */
    private TrajectoryBuilder splineTrajectory(ArrayList<Pose2d> points, KeyPoint actionPoint) {
        TrajectoryBuilder trajBuilder = drive.trajectoryBuilder(drive.getPoseEstimate());

        double angleBetweenPoints;
        if (points.size() > 0) {
            Pose2d lastPoint = points.get(0);

            //The angleBetweenPoints is used for the angle of tangency for a particular point
            //in the spline. It is calculated as the angle of a line in a unit circle
            angleBetweenPoints = Math.atan2(lastPoint.getY() - drive.getPoseEstimate().getY(),
                    lastPoint.getX() - drive.getPoseEstimate().getX());

            //Add the first point's spline to the trajectory
            trajBuilder = trajBuilder.splineToSplineHeading(lastPoint, angleBetweenPoints);

            //Repeat this process for all other necessary points
            for (int i = 1; i < points.size(); i++) {
                angleBetweenPoints = Math.atan2(points.get(i).getY() - lastPoint.getY(),
                        points.get(i).getX() - lastPoint.getX());

                trajBuilder = trajBuilder.splineToSplineHeading(points.get(i), angleBetweenPoints);

                lastPoint = points.get(i); //Update last point for calculating angleBetweenPoints
            }

            //Update the angleBetweenPoints in case it is needed for the actionPoint
            angleBetweenPoints = Math.atan2(actionPoint.y - lastPoint.getY(),
                    actionPoint.x - lastPoint.getX());
        }
        else {
            //If there were no other points in this specific spline, the actionPoint is driven
            //from the robot's position, since there is no last point
            angleBetweenPoints = Math.atan2(actionPoint.y - drive.getPoseEstimate().getY(),
                    actionPoint.x - drive.getPoseEstimate().getX());
        }
        //Only add the actionPoint to the spline if it moved
        if (actionPoint.moved) {
            trajBuilder = trajBuilder.splineToSplineHeading(
                    new Pose2d(actionPoint.x, actionPoint.y, actionPoint.heading),
                    angleBetweenPoints);
        }

        return trajBuilder;
    }

    /**
     * Follows all points in the path (including actionPoint)
     * @param points An array of points to follow
     * @param actionPoint An optional last point in this trajectory
     */
    private void customFollower(ArrayList<Pose2d> points, KeyPoint actionPoint) {
        PIDFController hControl = new PIDFController(H_PID);
        hControl.setOutputBounds(0, 1);
        hControl.setTargetPosition(0);

        //How accurate each attribute should be at each point
        double xAcc = 3, yAcc = 3, hAcc = Math.toRadians(5);

        if (actionPoint.moved) {
            points.add(new Pose2d(actionPoint.x, actionPoint.y, actionPoint.heading));
        }

        for (int i = 0; i < points.size(); i++) {
            if (i == points.size() - 1) {
                xAcc = 0.5; yAcc = 0.5; hAcc = Math.toRadians(0.5);
            }

            //Current robot position
            double robotX = drive.getPoseEstimate().getX();
            double robotY = drive.getPoseEstimate().getY();
            double robotH = drive.getPoseEstimate().getHeading();

            //Current destination  point
            double destX = points.get(i).getX();
            double destY = points.get(i).getY();
            double destH = points.get(i).getHeading();

            //Calculate relatives
            double relX = destX - robotX;
            double relY = destY - robotY;
            double relH = deltaHeading(robotH, destH);

            double hWeight;

            //While robot is not at the current destination point
            while (opModeIsActive() &&
                    (Math.abs(relX) > xAcc ||
                            Math.abs(relY) > yAcc ||
                            Math.abs(relH) > hAcc)) {

                updateBulkRead();
                drive.update();

                //Update robot position
                robotX = drive.getPoseEstimate().getX();
                robotY = drive.getPoseEstimate().getY();
                robotH = drive.getPoseEstimate().getHeading();

                //Calculate relatives
                relX = destX - robotX;
                relY = destY - robotY;
                relH = deltaHeading(robotH, destH);

                hWeight = hControl.update(Math.abs(relH));

                if (Math.abs(relH) < hAcc) hWeight = 0;

                double driveAngle = deltaHeading(robotH, Math.atan2(relY, relX));

                double xPower = Math.cos(driveAngle);
                double yPower = Math.sin(driveAngle);
                double hPower = Math.copySign(hWeight, relH);

                if (Math.sqrt((relX*relX) + (relY*relY)) < 5) {
                    xPower *= 0.25;
                    yPower *= 0.25;
                }
                else {
                    if (Math.abs(xPower) < 0.25 && Math.abs(xPower) > 0)
                        xPower = Math.copySign(0.25, xPower);
                    if (Math.abs(yPower) < 0.25 && Math.abs(yPower) > 0)
                        yPower = Math.copySign(0.25, yPower);
                    if (Math.abs(hPower) < 0.25 && Math.abs(hPower) > 0)
                        hPower = Math.copySign(0.25, hPower);
                }

                drive.setWeightedDrivePower(new Pose2d(xPower, yPower, hPower));

                updateSubsystems();
                updateTelemetry();
            }
        }

        setMotorPowers(0, 0, 0, 0);

        //Remove action point to leave array unchanged
        if (actionPoint.moved) points.remove(points.size() - 1);
    }

    /**
     * Combines all consecutive turns into a single smooth turn
     * @param actionPoint The starting (and possible ending) consecutive turning point
     */
    private void makeTurn(KeyPoint actionPoint) {

        //Finds the last turn (that is just a turn, no movement or action)
        while (!path.get(0).moved && path.get(0).turned && path.get(0).action == Actions.DO_NOTHING) {
            actionPoint = path.remove(0);
        }

        //Gets the angle needed to turn by
        double angleToTurn = actionPoint.heading - drive.getPoseEstimate().getHeading();

        //Corrects the angle for over- or underflow, as in going from 15 to 345 degrees or the other
        //way around
        if (angleToTurn < -Math.PI) angleToTurn += Math.PI * 2;
        if (angleToTurn > Math.PI) angleToTurn -= Math.PI * 2;

        System.out.println("TURN BY: " + angleToTurn);
        drive.turnAsync(angleToTurn);

        //Waits for the turn to complete
        while (opModeIsActive() && drive.isBusy()) {
            updateBulkRead();
            drive.update();
            updateSubsystems();
            updateTelemetry();
        }
    }

    @Override
    protected void updateSubsystems() {
        if (USE_SUBS) {

        }
    }

    @Override
    protected void updateTelemetry() {
        telemetry.addData("Position", drive.getPoseEstimate());
        telemetry.update();
    }
}
