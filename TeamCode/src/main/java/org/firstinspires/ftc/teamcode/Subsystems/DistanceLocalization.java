package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceLocalization {

    private static final int FRONT_WALL_DISTANCE = 144;
    private static final int RIGHT_WALL_DISTANCE = 72;

    //Distance sensors and related variables
    public final ModernRoboticsI2cRangeSensor sF;
    public final ModernRoboticsI2cRangeSensor sR;
    private double rawF, rawR, lastRawF, lastRawR;

    //Gyroscope
    public final Gyroscope gyro;
    private double theta, lastTheta;

    //Radius: distance from center of robot to sensor
    private double RADIUS;

    //Wall enumerator
    private enum Wall {
        FRONT, RIGHT, BACK, LEFT
    }

    //Robot position
    private double x, y;

    //Robot theoretical and actual side ratios
    private double theoreticalRatio, actualRatio;

    /**
     * @param hardwareMap OpMode's HardwareMap
     * @param fName Configuration name for front sensor
     * @param rName Configuration name for right sensor
     * @param radius Distance that each sensor is from the center of the robot (in inches)
     */
    public DistanceLocalization(HardwareMap hardwareMap, String fName, String rName, double radius) {
        sF = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, fName);
        sR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, rName);

        gyro = new Gyroscope(hardwareMap);
        gyro.setOffset(90);

        RADIUS = radius;

        rawF = sF.getDistance(DistanceUnit.INCH);
        rawR = sR.getDistance(DistanceUnit.INCH);
        lastRawF = rawF; lastRawR = rawR;

        x = 0; y = 0;
        theta = 90; lastTheta = 90;

        theoreticalRatio = 0; actualRatio = 0;
    }

    /**
     * Updates the position of the robot
     */
    public void update() {
        rawF = sF.getDistance(DistanceUnit.INCH); rawR = sR.getDistance(DistanceUnit.INCH);
        theta = gyro.getYaw();

        if (theta != lastTheta || rawF != lastRawF || rawR != lastRawR)
            calculatePosition(theta, rawF, rawR);

        //Update last variables
        lastRawF = rawF; lastRawR = rawR;
        lastTheta = theta;
    }

    /**
     * Calculates the robot's position based on distances and theta
     * @param angle Robot's current heading (in degrees)
     * @param rawSF Front sensor's distance
     * @param rawSR Right sensor's distance
     */
    private void calculatePosition(double angle, double rawSF, double rawSR) {
        //Add in robot radius
        if (rawSF > 72) rawSF = 0;
        else rawSF += RADIUS;
        if (rawSR > 72) rawSR = 0;
        else rawSR += RADIUS;

        Wall frontSees, rightSees;

        //Determine which sensor sees which wall. With all four sensors we'd just pick
        //whichever two adjacent sensors have the smallest values
        if (adjSidesSeeSame(angle, rawSF, rawSR)) {
            if (angle < 90) {
                frontSees = Wall.RIGHT;
                rightSees = Wall.RIGHT;
            }
            else if (angle < 180) {
                frontSees = Wall.FRONT;
                rightSees = Wall.FRONT;
            }
            else if (angle < 270) {
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
            if (angle >= 315 || angle < 45) {
                frontSees = Wall.RIGHT;
                rightSees = Wall.BACK;
            }
            else if (angle < 135) {
                frontSees = Wall.FRONT;
                rightSees = Wall.RIGHT;
            }
            else if (angle < 225) {
                frontSees = Wall.LEFT;
                rightSees = Wall.FRONT;
            }
            else {
                frontSees = Wall.BACK;
                rightSees = Wall.LEFT;
            }
        }

        //Calculate each sensor's distance to their closest wall
        double distSF = getFrontDistance(theta, rawSF, frontSees);
        double distSR = getRightDistance(theta, rawSR, rightSees);

        //If both sensors see the same wall, average their distances to get a better estimate
        if (frontSees == rightSees) {
            if (distSF == 0) setPositionFromWall(distSR, rightSees);
            else if (distSR == 0) setPositionFromWall(distSF, frontSees);
            else setPositionFromWall((distSF + distSR) / 2, frontSees);
        }
        else {
            setPositionFromWall(distSF, frontSees);
            setPositionFromWall(distSR, rightSees);
        }
    }

    /**
     * Compares two distances with the angle of the robot to determine if they see the same wall
     * @param angleDeg Robot's heading in degrees
     * @param left The distance sensor "leftmost" around the robot
     * @param right The distance sensor "rightmost" around the robot
     * @return Whether or not the sensors see the same wall
     */
    private boolean adjSidesSeeSame(double angleDeg, double left, double right) {
        double simplifiedAngle = angleDeg % 90;

        //If the robot is almost "straight" in any direction, there's no way two adjacent
        //sensors see the same side
        if (90 - simplifiedAngle < 7 || simplifiedAngle < 7) return false;

        //If one of the distances is 0, it isn't a correct reading and has no correlation
        if (left == 0 || right == 0) return false;

        double sameSideRatio = Math.abs(Math.tan(Math.toRadians(angleDeg)));
        double actualRatio = Math.abs(left/right);

        //TODO: Take this out when it works
        theoreticalRatio = sameSideRatio;
        this.actualRatio = actualRatio;

        //Tolerance of +- 0.3 in the ratio
        return  sameSideRatio >= actualRatio - 0.3 &&
                sameSideRatio <= actualRatio + 0.3;
    }

    /**
     * Converts a sensor distance to the robot's distance from a specific wall
     * @param theta The robot's heading in degrees
     * @param rawDist The raw sensor distance
     * @param frontSees What wall the sensor sees
     * @return The robot's distance from specified wall
     */
    private double getFrontDistance(double theta, double rawDist, Wall frontSees) {
        if (rawDist == 0) return 0;

        switch(frontSees) {
            case FRONT:
                return rawDist * getBaseFTF(theta);
            case RIGHT:
                return rawDist * getBaseFTR(theta);
            case BACK:
                return rawDist * getBaseFTB(theta);
            case LEFT:
                return rawDist * getBaseFTL(theta);
        }

        return 0;
    }

    /**
     * Converts a sensor distance to the robot's distance from a specific wall
     * @param theta The robot's heading in degrees
     * @param rawDist The raw sensor distance
     * @param rightSees What wall the sensor sees
     * @return The robot's distance from specified wall
     */
    private double getRightDistance(double theta, double rawDist, Wall rightSees) {
        if (rawDist == 0) return 0;

        switch(rightSees) {
            case FRONT:
                return rawDist * getBaseFTL(theta);
            case RIGHT:
                return rawDist * getBaseFTF(theta);
            case BACK:
                return rawDist * getBaseFTR(theta);
            case LEFT:
                return rawDist * getBaseFTB(theta);
        }

        return 0;
    }

    /**
     * Converts a sensor distance to the robot's distance from a specific wall
     * @param theta The robot's heading in degrees
     * @param rawDist The raw sensor distance
     * @param backSees What wall the sensor sees
     * @return The robot's distance from specified wall
     */
    private double getBackDistance(double theta, double rawDist, Wall backSees) {
        if (rawDist == 0) return 0;

        switch(backSees) {
            case FRONT:
                return rawDist * getBaseFTB(theta);
            case RIGHT:
                return rawDist * getBaseFTL(theta);
            case BACK:
                return rawDist * getBaseFTF(theta);
            case LEFT:
                return rawDist * getBaseFTR(theta);
        }

        return 0;
    }

    /**
     * Converts a sensor distance to the robot's distance from a specific wall
     * @param theta The robot's heading in degrees
     * @param rawDist The raw sensor distance
     * @param leftSees What wall the sensor sees
     * @return The robot's distance from specified wall
     */
    private double getLeftDistance(double theta, double rawDist, Wall leftSees) {
        if (rawDist == 0) return 0;

        switch(leftSees) {
            case FRONT:
                return rawDist * getBaseFTR(theta);
            case RIGHT:
                return rawDist * getBaseFTB(theta);
            case BACK:
                return rawDist * getBaseFTL(theta);
            case LEFT:
                return rawDist * getBaseFTF(theta);
        }

        return 0;
    }

    /**
     * Set robot position based on robot distance to a specific wall
     * @param dist The robot's distance to a wall
     * @param sees The wall specified
     */
    private void setPositionFromWall(double dist, Wall sees) {
        if (dist == 0) return;

        switch (sees) {
            case FRONT:
                y =  FRONT_WALL_DISTANCE - dist; break;
            case RIGHT:
                x = RIGHT_WALL_DISTANCE - dist; break;
            case BACK:
                y = dist; break;
            case LEFT:
                x = dist; break;
        }
    }

    /**
     * Assuming sF sees wF, sR sees wR, etc
     * @param angle Robot heading in degrees
     * @return The "base" of the equation to convert the distance. Multiple by raw distance
     */
    private double getBaseFTF(double angle) {
        return Math.cos(Math.abs(Math.toRadians(90 - angle)));
    }

    /**
     * Assuming sF sees wR, sR sees wB, etc
     * @param angle Robot heading in degrees
     * @return The "base" of the equation to convert the distance. Multiple by raw distance
     */
    private double getBaseFTR(double angle) {
        return Math.cos(Math.toRadians(angle));
    }

    /**
     * Assuming sF sees wB, sR sees sL, etc
     * @param angle Robot heading in degrees
     * @return The "base" of the equation to convert the distance. Multiple by raw distance
     */
    private double getBaseFTB(double angle) {
        return Math.cos(Math.abs(Math.toRadians(270 - angle)));
    }

    /**
     * Assuming sF sees wL, sR sees sF, etc
     * @param angle Robot heading in degrees
     * @return The "base" of the equation to convert the distance. Multiple by raw distance
     */
    private double getBaseFTL(double angle) {
        return Math.cos(Math.abs(Math.toRadians(180 - angle)));
    }

    public void setHeading(double heading) {
        gyro.setOffset(heading - gyro.getRawYaw());
    }

    //TODO: Take this out when it works
    public double getTheoreticalRatio() {
        return theoreticalRatio;
    }

    public double getActualRatio() {
        return actualRatio;
    }

    public double getRawF() {
        return rawF;
    }

    public double getRawR() {
        return rawR;
    }

    public double getRADIUS() {
        return RADIUS;
    }

    public void setRADIUS(double radius) {
        RADIUS = radius;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }
}
