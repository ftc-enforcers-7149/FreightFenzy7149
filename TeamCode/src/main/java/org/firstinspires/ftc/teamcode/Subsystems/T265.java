package org.firstinspires.ftc.teamcode.Subsystems;

import android.content.Context;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import static org.firstinspires.ftc.robotcontroller.t265.T265Holder.CAMERA_OFFSET;
import static org.firstinspires.ftc.robotcontroller.t265.T265Holder.ENCODER_MEASUREMENT_COVARIANCE;
import static org.firstinspires.ftc.robotcontroller.t265.T265Holder.SLAMRA;
import static org.firstinspires.ftc.robotcontroller.t265.T265Holder.err;
import static org.firstinspires.ftc.robotcontroller.t265.T265Holder.init;
import static org.firstinspires.ftc.robotcontroller.t265.T265Holder.isStarted;

public class T265 {

    private Context appContext;

    //These are the main position variables from the camera
    //Translation is the x and y position in meters
    //Rotation is the heading of the robot in radians
    //Velocity is the x, y, and angular velocity of the robot in m/s and rad/s
    //Confidence is the camera's confidence in its estimated position
    private Translation2d translation = new Translation2d();
    private Rotation2d rotation = new Rotation2d();
    private ChassisSpeeds velocity = new ChassisSpeeds();
    private T265Camera.PoseConfidence confidence = T265Camera.PoseConfidence.Failed;

    //Failed is used to make sure that if the camera wasn't initialized properly, it won't cause errors
    //UpdateIsNull is used to keep track of if a camera update didn't return values
    //The update is usually only null in the first update, but it's good to know if anything's gone wrong
    private boolean failed = false;
    private boolean updateIsNull = false;

    public double X_OFFSET = 0;
    private double Y_OFFSET = 0;
    private double ANG_OFFSET = 0;

    private double START_X = 0;
    private double START_Y = 0;
    private double START_ANG = 0;
    private boolean waitingToStart = true;

    /**
     * This constructor initializes the T265 camera at a starting position of (0, 0) and an angle of 0.
     * Use this in an OpMode's init method
     *
     * @param hardwareMap                   The hardware map is used to get the context, which is needed to initialize the camera
     */
    public T265(HardwareMap hardwareMap) {
        //Initialize the starting position (0, 0, 0)
        appContext = hardwareMap.appContext;

        //Initialize the actual camera object. If there was an error, it failed
        //NOTE: If this was an error, make sure to remove 'arm64-v8a' from each of two places in the project's build.common.gradle
        try {
            if (SLAMRA == null) {
                init(appContext);
                SLAMRA.setPose(new Pose2d());
            }
        }
        catch (Exception e) {
            SLAMRA = null;
            failed = true;
            err = e;
        }
    }

    /**
     * This constructor initializes the T265 camera at a starting position of (0, 0) and an angle of 0.
     * Use this in an OpMode's init method
     *
     * @param hardwareMap                   The hardware map is used to get the context, which is needed to initialize the camera
     * @param encoderMeasurementCovariance  The amount of weight odometry is given in the position estimation. 1 is least, 0 is most.
     * @param offsetX                       The x offset of the camera in reference to the robot's center. Give inches
     * @param offsetY                       The y offset of the camera in reference to the robot's center. Give inches
     * @param offsetAng                     The angular offset of the camera in reference to the robot's front. Give degrees
     */
    public T265(HardwareMap hardwareMap, double encoderMeasurementCovariance, double offsetX, double offsetY, double offsetAng) {
        //This converts the inputs from inches and degrees to usable meters and radians and accounts for wacky camera positioning.
        double[] convertedOffset = fromConventionalToT265(offsetX, offsetY, offsetAng);

        //Initialize the starting position (0, 0, 0)
        //Initialize the camera's offset from the center of the robot
        Pose2d startingPose = new Pose2d(0, 0, new Rotation2d());
        CAMERA_OFFSET = new Transform2d(new Translation2d(convertedOffset[0], convertedOffset[1]),
                                                    new Rotation2d(convertedOffset[2]));
        ENCODER_MEASUREMENT_COVARIANCE = encoderMeasurementCovariance;
        appContext = hardwareMap.appContext;

        //Initialize the actual camera object. If there was an error, it failed
        //NOTE: If this was an error, make sure to remove 'arm64-v8a' from each of two places in the project's build.common.gradle
        try {
            if (SLAMRA == null) {
                init(appContext);
            }
            SLAMRA.setPose(startingPose);
        }
        catch (Exception e) {
            SLAMRA = null;
            failed = true;
            err = e;
        }
    }

    /**
     * Resets the position of the robot (not the offset of the camera).
     * Use this anywhere in an OpMode
     *
     * @param x     The x position, in inches, of the robot
     * @param y     The y position, in inches, of the robot
     * @param ang   The heading, in degrees, of the robot
     */
    public void setPose(double x, double y, double ang) {
        double[] convertedPose = fromConventionalToT265(x, y, ang);

        //Create offsets
        X_OFFSET = X_OFFSET + (convertedPose[0] - translation.getX());
        Y_OFFSET = Y_OFFSET + (convertedPose[1] - translation.getY());
        ANG_OFFSET = ANG_OFFSET + (convertedPose[2] - rotation.getRadians());

        translation = new Translation2d(translation.getX() + X_OFFSET,
                translation.getY() + Y_OFFSET);
        rotation = new Rotation2d(rotation.getRadians() + ANG_OFFSET);
    }

    /**
     * Starts the camera. Use this in an OpMode's start method
     */
    public void start(double x, double y, double ang) {
        START_X = x;
        START_Y = y;
        START_ANG = ang;

        //If the camera didn't fail, try to start it.
        //If starting it caused an error, it failed. NOTE: If this is an error, make sure to use version 2.0.1+ of the ftc265 library
        if (!failed) {
            try {
                if (!isStarted)
                    SLAMRA.start();
                update();
                if (confidence != T265Camera.PoseConfidence.Failed) {
                    setPose(x, y, ang);
                    waitingToStart = false;
                }
                isStarted = true;
            }
            catch (Exception e) {
                SLAMRA = null;
                failed = true;
                err = e;
                isStarted = false;
            }
        }
    }

    /**
     * Stops the camera. Use this in an OpMode's stop method
     */
    public void stop() {
        if (!failed && isStarted) {
            try {
                SLAMRA.stop();
                isStarted = false;
            } catch (Exception e) {
                SLAMRA = null;
                failed = true;
                err = e;
            }
        }
    }

    /**
     * Gets the most recent update from the camera and saves its variables to the class.
     * The camera updates in a separate (inaccessible) thread, which is constantly running.
     * You can use this anywhere, but it's recommended to use this in an OpMode's loop method
     */
    public void update() {
        //If the camera didn't fail to start, get the most recent update
        if (!failed && isStarted) {
            T265Camera.CameraUpdate update = SLAMRA.getLastReceivedCameraUpdate();

            //Try to use the update to get the position, heading, velocity, and confidence
            //If there was an error, the update is null. NOTE: This almost never happens, and doesn't cause an issue to the estimation
            try {
                confidence = update.confidence;

                if (confidence != T265Camera.PoseConfidence.Failed) {
                    Translation2d rawTranslation = update.pose.getTranslation();
                    translation = new Translation2d(rawTranslation.getX() + X_OFFSET,
                                                    rawTranslation.getY() + Y_OFFSET);
                    Rotation2d rawRotation = update.pose.getRotation();
                    rotation = new Rotation2d(rawRotation.getRadians() + ANG_OFFSET);
                    velocity = update.velocity;

                    if (waitingToStart) {
                        setPose(START_X, START_Y, START_ANG);
                        waitingToStart = false;
                    }
                }

                updateIsNull = false;
            }
            catch (Exception e) {
                updateIsNull = true;
                err = e;
            }
        }
    }

    /**
     * Send odometry data to the T265 camera for more accurate estimation
     * @param velX  The x velocity of the robot in inches / sec
     * @param velY  The y velocity of the robot in inches / sec
     */
    public void sendOdometry(double velX, double velY) {
        double[] convertedVel = fromConventionalToT265(velX, velY, 0);
        SLAMRA.sendOdometry(convertedVel[0], convertedVel[1]);
    }

    /**
     * Gets the translation of the robot
     * @return A Translation2d object, with x and y positions in meters
     */
    public Translation2d getTranslation() {
        return translation;
    }

    /**
     * Gets the rotation of the robot
     * @return A Rotation2d object, with heading in radians and degrees (-180 to 180)
     */
    public Rotation2d getRotation() {
        return rotation;
    }

    /**
     * Gets the velocity of the robot
     * @return A ChassisSpeeds object, with vx and vy in meters / sec and vang in radians / sec
     */
    public ChassisSpeeds getVelocity() {
        return velocity;
    }

    /**
     * Gets the confidence of the estimated position
     * @return A PoseConfidence enum object
     */
    public T265Camera.PoseConfidence getConfidence() {
        return confidence;
    }

    /**
     * Gets the failed variable
     * @return A boolean denoting whether or not the camera failed to start properly
     */
    public boolean isFailed() {
        return failed;
    }

    /**
     * gets the updateIsNull variable
     * @return A boolean denoting whether or not the most recent update was null
     */
    public boolean isUpdateIsNull() {
        return updateIsNull;
    }

    /**
     * Gets the x position of the robot
     * @return A double of the x position, in inches
     */
    public double getX() {
        //Convert from meters to inches and account for weird camera position
        return translation.getX() * 39.3701;
    }

    /**
     * Gets the y position of the robot
     * @return A double of the y position, in inches
     */
    public double getY() {
        //Convert from meters to inches and account for weird camera position
        return translation.getY() * 39.3701;
    }

    /**
     * Gets the heading of the robot
     * @return A double of the heading, in degrees (0-360)
     */
    public double getAng() {
        double angle = rotation.getDegrees();

        if (angle < 0) {
            return angle + 360;
        }

        return angle;
    }

    /**
     * Gets the x velocity of the robot
     * @return A double of the x velocity, in inches / sec
     */
    public double getVelX() {
        //Convert from meters / sec to inches / sec
        return velocity.vxMetersPerSecond * 39.3701;
    }

    /**
     * Gets the y velocity of the robot
     * @return A double of the y velocity, in inches / sec
     */
    public double getVelY() {
        //Convert from meters / sec to inches / sec
        return velocity.vyMetersPerSecond * 39.3701;
    }

    /**
     * Gets the angular velocity of the robot
     * @return A double of the angular velocity, in degrees / sec
     */
    public double getVelAng() {
        //Convert from radians / sec to degrees / sec
        double angle = Math.toDegrees(velocity.omegaRadiansPerSecond);

        if (angle < 0) {
            return angle + 360;
        }

        return angle;
    }

    /**
     * Convert conventional axes to T265 orientation
     * @param x Left and right in inches
     * @param y Forward and backward in inches
     * @param heading Rotation in degrees
     * @return
     */
    private double[] fromConventionalToT265(double x, double y, double heading) {
        return new double[] {y / 39.3701, -x / 39.3701, Math.toRadians(heading)};
    }

    /**
     * @return The error, if any, that caused the camera to fail
     */
    public Exception getError() {
        return err;
    }

    public boolean isStarted() {
        return isStarted;
    }
}
