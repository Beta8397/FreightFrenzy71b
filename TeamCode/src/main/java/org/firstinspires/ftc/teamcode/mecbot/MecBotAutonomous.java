package org.firstinspires.ftc.teamcode.mecbot;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.logging.BetaLog;
import org.firstinspires.ftc.teamcode.logging.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.util.AngleUtils;
import org.firstinspires.ftc.teamcode.util.CubicSpline2D;
import org.firstinspires.ftc.teamcode.util.MotionProfile;
import org.firstinspires.ftc.teamcode.util.ParametricFunction2D;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Updatable;

/**
 * An abstract class that extends LinearOpMode and provides navigation methods that can be called by autonomous op modes
 * that utilize a MechBot object.
 */
public abstract class MecBotAutonomous extends LoggingLinearOpMode {

    public static final float STD_TURN_COEFF = 2.0f;        //Proportionate coefficient for turning
    public final float HEADING_CORRECTION_FACTOR = 2.0f;
    private final float DISTANCE_CORRECTION_FACTOR = 2.0f;
    public static final float STD_MAX_TURN_SPEED = 1.0f;    //Radians per sec

    protected boolean loggingEnabled = false;

    /*
     * The MechBot object that will be used by the navigation methods in this class. This must be assigned a value
     * by calling the setBot method.
     */
    MecBot bot;

    public interface Predicate {
        boolean isTrue();
    }

    /**
     * Assign a MechBot object to bot. This object should already be initialized before being provided.
     *
     * @param b
     */
    public void setBot(MecBot b) {
        bot = b;
    }

    /**
     * Drive the robot straight in the specified direction, at the specified speed, while maintaining a specified
     * orientation, until it has travelled the specified number of inches.
     *
     * @param speed                Speed, in inches per second.
     * @param directionDegrees     Direction of travel (world coordinates), in degrees
     * @param targetHeadingDegrees Target orientation (world coordinates), in degrees
     * @param finished             Predicate which tells the bot when to stop
     */
    public void driveStraight(float speed, float directionDegrees,
                              float targetHeadingDegrees, Predicate finished) {

        float directionRadians = (float) Math.toRadians(directionDegrees);
        float targetHeadingRadians = (float) Math.toRadians(targetHeadingDegrees);

        /*
         * Control loop for this operation. Break from the loop after the specified distance has been travelled.
         */
        while (opModeIsActive()) {
            bot.updateOdometry();                                               //Determine current bot position             //Distance travelled (inches)
            if (finished.isTrue()) {
                break;                                                //Break from loop if we've travelled far enouch
            }

            float vx = -speed * (float) Math.sin(directionRadians - bot.getPose().theta);
            float vy = speed * (float) Math.cos(directionRadians - bot.getPose().theta);

            float angleOffset = (float) AngleUtils.normalizeRadians(targetHeadingRadians - bot.getPose().theta);
            float va = STD_TURN_COEFF * angleOffset;

            bot.setDriveSpeed(vx, vy, va);
        }
        // We have travelled far enough and broken from loop, so stop the robot.
        bot.setDrivePower(0, 0, 0);
    }

    /**
     * Turn to the specified heading using proportionate control
     *
     * @param targetHeadingDegrees
     * @param toleranceDegrees
     * @param propCoeff
     */
    public void turnToHeading(float targetHeadingDegrees, float toleranceDegrees,
                              float propCoeff, float maxDegreesPerSec) {
        float targetHeadingRadians = targetHeadingDegrees * (float) Math.PI / 180;
        float toleranceRadians = toleranceDegrees * (float) Math.PI / 180;
        float maxRadiansPerSec = maxDegreesPerSec * (float)Math.PI/180;
        float priorHeading = bot.getHeadingRadians();
        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive()) {
            bot.updateOdometry();
            float currentHeading = bot.getPose().theta;
            /*
             * Normalized difference between target heading and current heading
             */
            float angleDiff = (float) AngleUtils.normalizeRadians(targetHeadingRadians - currentHeading);
            /*
             * Only check for completion if we believe we have a new reading from gyro. Assume a new reading
             * if: current reading is different from old one OR more than 50 ms has elapsed since the last
             * (assumed) new reading. After the test for completion, reset the timer and update priorHeading.
             */
//            if (currentHeading != priorHeading || et.milliseconds() > 50) {
//                float headingChange = (float) AngleUtils.normalizeRadians(currentHeading - priorHeading);
//                if (Math.abs(angleDiff) < toleranceRadians && Math.abs(headingChange) < toleranceRadians / 5) {
//                    break;
//                } else {
//                    et.reset();
//                    priorHeading = currentHeading;
//                }
//            }
            if (Math.abs(angleDiff) < toleranceRadians) break;

            float va = propCoeff * angleDiff;
            if (Math.abs(va) > maxRadiansPerSec){
                va = (float)Math.signum(va) * maxRadiansPerSec;
            }
            bot.setDriveSpeed(0, 0, va);
        }
        bot.setDrivePower(0, 0, 0);
    }

    /**
     * Turn to the specified heading using proportionate control
     *
     * @param targetHeadingDegrees
     * @param toleranceDegrees
     * @param propCoeff
     */
    public void turnToHeadingPD(float targetHeadingDegrees, float toleranceDegrees,
                              float propCoeff, float derivCoeff, float maxDegreesPerSec) {
        float targetHeadingRadians = targetHeadingDegrees * (float) Math.PI / 180;
        float toleranceRadians = toleranceDegrees * (float) Math.PI / 180;
        float maxRadiansPerSec = maxDegreesPerSec * (float)Math.PI/180;
        float priorHeading = bot.getHeadingRadians();
        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive()) {
            bot.updateOdometry();
            float currentHeading = bot.getPose().theta;
            /*
             * Normalized difference between target heading and current heading
             */
            float angleDiff = (float) AngleUtils.normalizeRadians(targetHeadingRadians - currentHeading);
            float turnSpeed = 0;
            /*
             * Only check for completion if we believe we have a new reading from gyro. Assume a new reading
             * if: current reading is different from old one OR more than 50 ms has elapsed since the last
             * (assumed) new reading. After the test for completion, reset the timer and update priorHeading.
             */
            if (currentHeading != priorHeading || et.milliseconds() > 50) {
                float headingChange = (float) AngleUtils.normalizeRadians(currentHeading - priorHeading);
                if (Math.abs(angleDiff) < toleranceRadians && Math.abs(headingChange) < toleranceRadians / 5) {
                    break;
                } else {
                    turnSpeed = headingChange/ (float) et.milliseconds();
                    et.reset();
                    priorHeading = currentHeading;
                }
            }
            float va = propCoeff * angleDiff - derivCoeff * turnSpeed;
            if (Math.abs(va) > maxRadiansPerSec){
                va = (float)Math.signum(va) * maxRadiansPerSec;
            }
            bot.setDriveSpeed(0, 0, va);
        }
        bot.setDrivePower(0, 0, 0);
    }

    public void turnToHeading(float targetHeadingDegrees, float toleranceDegrees,
                              float propCoeff, float maxDegreesPerSec, Updatable action) {
        float targetHeadingRadians = targetHeadingDegrees * (float) Math.PI / 180;
        float toleranceRadians = toleranceDegrees * (float) Math.PI / 180;
        float maxRadiansPerSec = maxDegreesPerSec * (float)Math.PI/180;
        float priorHeading = bot.getHeadingRadians();
        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive()) {
            bot.updateOdometry();
            action.update();
            float currentHeading = bot.getPose().theta;
            /*
             * Normalized difference between target heading and current heading
             */
            float angleDiff = (float) AngleUtils.normalizeRadians(targetHeadingRadians - currentHeading);
            /*
             * Only check for completion if we believe we have a new reading from gyro. Assume a new reading
             * if: current reading is different from old one OR more than 50 ms has elapsed since the last
             * (assumed) new reading. After the test for completion, reset the timer and update priorHeading.
             */
            if (currentHeading != priorHeading || et.milliseconds() > 50) {
                float headingChange = (float) AngleUtils.normalizeRadians(currentHeading - priorHeading);
                if (Math.abs(angleDiff) < toleranceRadians && Math.abs(headingChange) < toleranceRadians / 5) {
                    break;
                } else {
                    et.reset();
                    priorHeading = currentHeading;
                }
            }
            float va = propCoeff * angleDiff;
            if (Math.abs(va) > maxRadiansPerSec){
                va = (float)Math.signum(va) * maxRadiansPerSec;
            }
            bot.setDriveSpeed(0, 0, va);
        }
        bot.setDrivePower(0, 0, 0);
    }


    /**
     * Drive the specified spline, from beginning to end.
     *
     * Note that the "reverse" parameter refers to the
     * orientation of the robot while traveling, not the direction of travel.
     * @param motionProfile     // The (vMin, vMax, and accel) for this operation
     * @param reverse           // If true, robot travels the spline in reverse orientation
     * @param tol               // When estimated remaining travel distance is <tol, stop
     * @param spline            // The CubicSpline2D to travel
     */
    protected void driveSpline(MotionProfile motionProfile, boolean reverse, float tol, CubicSpline2D spline) {

        // Set index to 0 to start at beginning of spline
        spline.setIndex(0);
        float s0 = 0;

        //Determine the total path length of the spline
        float totalPathLength = spline.getTotalPathLength();
        float vMinSquared = motionProfile.vMin * motionProfile.vMin;

        while (opModeIsActive()) {
            bot.updateOdometry();

            // Find s-value for closest point on the spline, incrementing the index if needed.
            s0 = spline.nextClosestPt(bot.getPose().x, bot.getPose().y, s0, this);

            // Find current pathlength along spline, and remaining pathlength until the end
            float currentPathLength = spline.getPathLength(s0);
            float remainingPathLength = totalPathLength - currentPathLength;

            // Get current target speed, based on motion profile and distances from begining and end
            float speed = (float)Math.min(Math.min(motionProfile.vMax, Math.sqrt(vMinSquared + 2.0*motionProfile.accel*currentPathLength)),
                    Math.sqrt(vMinSquared + 2.0*motionProfile.accel*(remainingPathLength)));

            // Look for a finished condition
            if (spline.getIndex() == (spline.getNumSegments() - 1)){
                if (s0 >= 1) break;
                float remainingS = 1.0f - s0;
                float remainingDist = remainingS * spline.d1(s0).dotProduct(spline.d1(s0));
                if (remainingDist < tol) break;
            }

            VectorF targetPos = spline.p(s0);       // Location of closest point on the spline

            // Vector from current position to closest point on the spline
            VectorF posOffset = targetPos.subtracted(new VectorF(bot.getPose().x, bot.getPose().y));

            // First and second derivatives of the spline function with respect to parameter s0
            VectorF d1 = spline.d1(s0);
            VectorF d2 = spline.d2(s0);

            /*
             * Calculate desired robot velocity. fwdVel is along the direction of travel along the spline. corrVel is
             * a proportionate velocity correction toward the closest point on the spline. These are in field
             * coordinates.
             *
             * totalVBot is in robot coordinates.
             */
            VectorF fwdVel = d1.multiplied(speed / d1.magnitude());
            VectorF corrVel = posOffset.multiplied(2 * DISTANCE_CORRECTION_FACTOR);
            VectorF totalV = fwdVel.added(corrVel);
            VectorF totalVBot = fieldToBot(totalV, bot.getPose().theta);

            /*
             * Determine the target heading (just the direction of travel along the spline), and the
             * nominal rate of change of heading for travel along the spline at the current speed
             * (targetHeadingChangeRate).
             */
            float targetHeading = reverse ? (float) Math.atan2(-d1.get(1), -d1.get(0)) : (float) Math.atan2(d1.get(1), d1.get(0));
            float targetHeadingChangeRate = (d2.get(1) * d1.get(0) - d2.get(0) * d1.get(1)) / (float) Math.pow(d1.dotProduct(d1), 1.5f) * speed;

            /*
             * headingOffset (i.e., error in current heading) is the difference between target heading and current heading.
             * Set the robot's angular speed to the targetHeadingChangeRate PLUS a proportionate correction
             * to account for the current heading offset.
             */
            float headingOffset = (float) AngleUtils.normalizeRadians(targetHeading - bot.getPose().theta);
            float va = targetHeadingChangeRate + headingOffset * HEADING_CORRECTION_FACTOR;

            bot.setDriveSpeed(totalVBot.get(0), totalVBot.get(1), va);
        }

        bot.setDriveSpeed(0, 0, 0);
    }

    /**
     * Drive spline while turning from initial heading to requested final heading.
     * The turn happens during the middle "turnFraction" of the total path length of the spline.
     * At the end there is a brief adjustment of position to the final point in the spline.
     *
     * @param motionProfile     vMin, vMax, and accel for the operation
     * @param turnFraction      The fraction of the total path length over which the turn occurs
     * @param finalHeadingDegrees   Requested final heading
     * @param adjDist    When remaining path length is <adjDist, stop driving spline and drive directly toward end point
     * @param tol                   Tolerance for end point
     * @param spline                The 2D cubic spline
     */
    protected void driveSpline(MotionProfile motionProfile, float turnFraction, float finalHeadingDegrees, float adjDist, float tol, CubicSpline2D spline) {

        // Set the index of the spline to 0, so that we are starting with the first segment
        spline.setIndex(0);
        float s0 = 0;   // Not really necessary, as we're calculating s0 at the beginning of each iteration
        boolean adjustMode = false;     // When this becomes true, do proportionate adjustment to final position

        float totalPathLength = spline.getTotalPathLength();    // Total path length of spline
        float turnPathLength = turnFraction * totalPathLength;  // Travel distance over which the turning will happen
        float preTurnPathLength = (totalPathLength - turnPathLength) / 2.0f;    // Non-turning distance at beginning
        float finalHeadingRadians = (float)Math.toRadians(finalHeadingDegrees);
        float initHeadingRadians = bot.updateOdometry().theta;
        float turnRadians = (float)AngleUtils.normalizeRadians(finalHeadingRadians - initHeadingRadians);

        // Radians to turn per inch pathlength along the turning portion of the path
        float radiansPerInch = turnRadians / turnPathLength;
        float vMinSquared = motionProfile.vMin * motionProfile.vMin;
        float finalX = spline.getPoints()[spline.getNumSegments()].get(0);
        float finalY = spline.getPoints()[spline.getNumSegments()].get(1);

        VectorF totalV;
        VectorF totalVBot;
        float va;

        // Drive the spline, then the final adjustment
        while (opModeIsActive()) {
            bot.updateOdometry();

            // dOffset is distance from final position; if within tolerance, quit
            float xOffset = finalX - bot.getPose().x;
            float yOffset = finalY - bot.getPose().y;
            float dOffset = (float)Math.hypot(xOffset, yOffset);

            if (dOffset < tol) break;

            // Find the parameter s0 for closest point on spline, incrementing the index if necessary.
            s0 = spline.nextClosestPt(bot.getPose().x, bot.getPose().y, s0, this);
            float currentPathLength = spline.getPathLength(s0);     // Distance traveled along path so far
            float remainingPathLength = totalPathLength - currentPathLength;    // Remaining path distance

            /*
             * If we are within adjDist of end of path, or if we have gone beyond end of spline,
             * enter adjust mode.
             */
            if (!adjustMode &&
                    (remainingPathLength < adjDist ||  (s0 >= 1 && spline.getIndex() == (spline.getNumSegments() - 1)))){
                adjustMode = true;
            }

            if (adjustMode) {

                float speed = (float)Math.min(Math.sqrt(vMinSquared + 2.0*motionProfile.accel*dOffset),
                        motionProfile.vMax);
                totalV = new VectorF(speed * xOffset/dOffset, speed * yOffset/dOffset);
                totalVBot = fieldToBot(totalV, bot.getPose().theta);
                float headingOffset = (float)AngleUtils.normalizeRadians(finalHeadingRadians - bot.getPose().theta);
                va = headingOffset * HEADING_CORRECTION_FACTOR;

            } else {

                /*
                 * targetPos is closest point on the spline. posOffset is vector from current bot pose to targetPos.
                 * d1 and d2 are first and second derivatives of spline function relative to the parameter s0.
                 */
                VectorF targetPos = spline.p(s0);
                VectorF posOffset = targetPos.subtracted(new VectorF(bot.getPose().x, bot.getPose().y));
                VectorF d1 = spline.d1(s0);
                VectorF d2 = spline.d2(s0);

                /*
                 * Get current target speed, based on the distance from the beginning and end points, and the
                 * motion profile.
                 */
                float speed = (float) Math.min(
                        Math.min(motionProfile.vMax, Math.sqrt(vMinSquared + 2.0 * motionProfile.accel * currentPathLength)),
                        Math.sqrt(vMinSquared + 2.0 * motionProfile.accel * remainingPathLength));

                /*
                 * fwdVel is nominal forward velocity along the spline. corrVel is a velocity correction to account
                 * for the position offset (i.e., error). These are in field coordinates.
                 *
                 * totalVBot is the desired bot velocity in robot coordinates.
                 */
                VectorF fwdVel = d1.multiplied(speed / d1.magnitude());
                VectorF corrVel = posOffset.multiplied(2 * DISTANCE_CORRECTION_FACTOR);
                totalV = fwdVel.added(corrVel);
                totalVBot = fieldToBot(totalV, bot.getPose().theta);

                /*
                 * nominalAngularSpeed will be 0 for the pre-turn and post-turn parts of the path. For the turning
                 * segment of the path, it will be the turn radians per inch multiplied by current robot speed.
                 * The target heading is equal to the initial heading during the pre-turn segment and is equal
                 * to the final heading during the post-turn segment. During the turning segment, the target heading
                 * changes continuously as a linear function of distance traveled.
                 */
                float nominalAngularSpeed;
                float targetHeadingRadians;
                if (currentPathLength < preTurnPathLength) {
                    nominalAngularSpeed = 0;
                    targetHeadingRadians = initHeadingRadians;
                } else if (currentPathLength < (preTurnPathLength + turnPathLength)) {
                    nominalAngularSpeed = radiansPerInch * speed;
                    targetHeadingRadians = (float) AngleUtils.normalizeRadians(((currentPathLength - preTurnPathLength) / turnPathLength) * turnRadians + initHeadingRadians);
                } else {
                    nominalAngularSpeed = 0;
                    targetHeadingRadians = finalHeadingRadians;
                }

                /*
                 * The requested robot angular speed is the nominal angular speed, PLUS a proportionate correction
                 * to account for the current heading error (headingOffset)
                 */
                float headingOffset = (float) AngleUtils.normalizeRadians(targetHeadingRadians - bot.getPose().theta);
                va = nominalAngularSpeed + headingOffset * HEADING_CORRECTION_FACTOR;
            }

            bot.setDriveSpeed(totalVBot.get(0), totalVBot.get(1), va);
        }


        bot.setDriveSpeed(0, 0, 0);
    }

    protected void driveSpline(MotionProfile motionProfile, float preTurnFraction, float turnFraction, float finalHeadingDegrees, float adjDist, float tol, CubicSpline2D spline) {

        // Set the index of the spline to 0, so that we are starting with the first segment
        spline.setIndex(0);
        float s0 = 0;   // Not really necessary, as we're calculating s0 at the beginning of each iteration
        boolean adjustMode = false;     // When this becomes true, do proportionate adjustment to final position

        float totalPathLength = spline.getTotalPathLength();    // Total path length of spline
        float turnPathLength = turnFraction * totalPathLength;  // Travel distance over which the turning will happen
        float preTurnPathLength = totalPathLength * preTurnFraction;    // Non-turning distance at beginning
        float finalHeadingRadians = (float)Math.toRadians(finalHeadingDegrees);
        float initHeadingRadians = bot.updateOdometry().theta;
        float turnRadians = (float)AngleUtils.normalizeRadians(finalHeadingRadians - initHeadingRadians);

        // Radians to turn per inch pathlength along the turning portion of the path
        float radiansPerInch = turnRadians / turnPathLength;
        float vMinSquared = motionProfile.vMin * motionProfile.vMin;
        float finalX = spline.getPoints()[spline.getNumSegments()].get(0);
        float finalY = spline.getPoints()[spline.getNumSegments()].get(1);

        VectorF totalV;
        VectorF totalVBot;
        float va;

        // Drive the spline, then the final adjustment
        while (opModeIsActive()) {
            bot.updateOdometry();

            // dOffset is distance from final position; if within tolerance, quit
            float xOffset = finalX - bot.getPose().x;
            float yOffset = finalY - bot.getPose().y;
            float dOffset = (float)Math.hypot(xOffset, yOffset);

            if (dOffset < tol) break;

            // Find the parameter s0 for closest point on spline, incrementing the index if necessary.
            s0 = spline.nextClosestPt(bot.getPose().x, bot.getPose().y, s0, this);
            float currentPathLength = spline.getPathLength(s0);     // Distance traveled along path so far
            float remainingPathLength = totalPathLength - currentPathLength;    // Remaining path distance

            /*
             * If we are within adjDist of end of path, or if we have gone beyond end of spline,
             * enter adjust mode.
             */
            if (!adjustMode &&
                    (remainingPathLength < adjDist ||  (s0 >= 1 && spline.getIndex() == (spline.getNumSegments() - 1)))){
                adjustMode = true;
            }

            if (adjustMode) {

                float speed = (float)Math.min(Math.sqrt(vMinSquared + 2.0*motionProfile.accel*dOffset),
                        motionProfile.vMax);
                totalV = new VectorF(speed * xOffset/dOffset, speed * yOffset/dOffset);
                totalVBot = fieldToBot(totalV, bot.getPose().theta);
                float headingOffset = (float)AngleUtils.normalizeRadians(finalHeadingRadians - bot.getPose().theta);
                va = headingOffset * HEADING_CORRECTION_FACTOR;

            } else {

                /*
                 * targetPos is closest point on the spline. posOffset is vector from current bot pose to targetPos.
                 * d1 and d2 are first and second derivatives of spline function relative to the parameter s0.
                 */
                VectorF targetPos = spline.p(s0);
                VectorF posOffset = targetPos.subtracted(new VectorF(bot.getPose().x, bot.getPose().y));
                VectorF d1 = spline.d1(s0);
                VectorF d2 = spline.d2(s0);

                /*
                 * Get current target speed, based on the distance from the beginning and end points, and the
                 * motion profile.
                 */
                float speed = (float) Math.min(
                        Math.min(motionProfile.vMax, Math.sqrt(vMinSquared + 2.0 * motionProfile.accel * currentPathLength)),
                        Math.sqrt(vMinSquared + 2.0 * motionProfile.accel * remainingPathLength));

                /*
                 * fwdVel is nominal forward velocity along the spline. corrVel is a velocity correction to account
                 * for the position offset (i.e., error). These are in field coordinates.
                 *
                 * totalVBot is the desired bot velocity in robot coordinates.
                 */
                VectorF fwdVel = d1.multiplied(speed / d1.magnitude());
                VectorF corrVel = posOffset.multiplied(2 * DISTANCE_CORRECTION_FACTOR);
                totalV = fwdVel.added(corrVel);
                totalVBot = fieldToBot(totalV, bot.getPose().theta);

                /*
                 * nominalAngularSpeed will be 0 for the pre-turn and post-turn parts of the path. For the turning
                 * segment of the path, it will be the turn radians per inch multiplied by current robot speed.
                 * The target heading is equal to the initial heading during the pre-turn segment and is equal
                 * to the final heading during the post-turn segment. During the turning segment, the target heading
                 * changes continuously as a linear function of distance traveled.
                 */
                float nominalAngularSpeed;
                float targetHeadingRadians;
                if (currentPathLength < preTurnPathLength) {
                    nominalAngularSpeed = 0;
                    targetHeadingRadians = initHeadingRadians;
                } else if (currentPathLength < (preTurnPathLength + turnPathLength)) {
                    nominalAngularSpeed = radiansPerInch * speed;
                    targetHeadingRadians = (float) AngleUtils.normalizeRadians(((currentPathLength - preTurnPathLength) / turnPathLength) * turnRadians + initHeadingRadians);
                } else {
                    nominalAngularSpeed = 0;
                    targetHeadingRadians = finalHeadingRadians;
                }

                /*
                 * The requested robot angular speed is the nominal angular speed, PLUS a proportionate correction
                 * to account for the current heading error (headingOffset)
                 */
                float headingOffset = (float) AngleUtils.normalizeRadians(targetHeadingRadians - bot.getPose().theta);
                va = nominalAngularSpeed + headingOffset * HEADING_CORRECTION_FACTOR;
            }

            bot.setDriveSpeed(totalVBot.get(0), totalVBot.get(1), va);
        }


        bot.setDriveSpeed(0, 0, 0);
    }

    protected void driveFunction(float speed, float s0, ParametricFunction2D pf, Predicate finish) {
        while (opModeIsActive()) {
            bot.updateOdometry();
            if (finish.isTrue()) break;
            s0 = findClosestPt(bot.getPose().x, bot.getPose().y, s0, pf);
            VectorF targetPos = pf.p(s0);
            VectorF posOffset = targetPos.subtracted(new VectorF(bot.getPose().x, bot.getPose().y));
            VectorF d1 = pf.d1(s0);
            VectorF d2 = pf.d2(s0);

            VectorF totalV = posOffset.multiplied(2 * DISTANCE_CORRECTION_FACTOR).added(d1.multiplied(speed / d1.magnitude()));
            VectorF totalVBot = fieldToBot(totalV, bot.getPose().theta);

            float targetHeading = (float) Math.atan2(d1.get(1), d1.get(0));
            float targetHeadingChangeRate = (d2.get(1) * d1.get(0) - d2.get(0) * d1.get(1)) / (float) Math.pow(d1.dotProduct(d1), 1.5f) * speed;
            float headingOffset = (float) AngleUtils.normalizeRadians(targetHeading - bot.getPose().theta);
            float va = targetHeadingChangeRate + headingOffset * HEADING_CORRECTION_FACTOR;

            bot.setDriveSpeed(totalVBot.get(0), totalVBot.get(1), va);
        }
    }

    protected VectorF fieldToBot(VectorF vField, float heading) {
        float sinTheta = (float) Math.sin(heading);
        float cosTheta = (float) Math.cos(heading);
        return new VectorF(vField.get(0) * sinTheta - vField.get(1) * cosTheta, vField.get(0) * cosTheta + vField.get(1) * sinTheta);
    }

    protected float findClosestPt(float x0, float y0, float s0, ParametricFunction2D pf) {
        float epsilon = .0001f;
        float delta = 100;
        while(delta > epsilon && opModeIsActive()) {
            VectorF p = pf.p(s0);
            VectorF d1 = pf.d1(s0);
            VectorF d2 = pf.d2(s0);
            float f = (p.get(0) - x0) * d1.get(0) + (p.get(1) - y0) * d1.get(1);
            float fDeriv = (p.get(0) - x0) * d2.get(0) + d1.get(0) * d1.get(0) + (p.get(1) - y0) * d2.get(1) + d1.get(1) * d1.get(1);
            float s = s0 - f / fDeriv;
            delta = Math.abs(s0 - s);
            s0 = s;
        }
        return s0;
    }

    protected void driveToPosition(float vMax, float vMin, float targetX, float targetY, float targetThetaDegrees, float cp, float tolerance) {
        float headingTargetRadians = targetThetaDegrees * (float)Math.PI / 180;

        while (opModeIsActive()) {
            bot.updateOdometry();

            float xError = targetX - bot.getPose().x;
            float yError = targetY - bot.getPose().y;
            float thetaError = (float)AngleUtils.normalizeRadians(headingTargetRadians - bot.getPose().theta);

            if(Math.hypot(xError, yError) < tolerance) break;

            float sinTheta = (float)Math.sin(bot.getPose().theta);
            float cosTheta = (float)Math.cos(bot.getPose().theta);

            float xErrorRobot = xError * sinTheta - yError * cosTheta;
            float yErrorRobot = xError * cosTheta + yError * sinTheta;

            float vx = xErrorRobot * cp;
            float vy = yErrorRobot * cp;
            float v = (float)Math.hypot(vx, vy);
            if(v > vMax) {
                vx *= vMax / v;
                vy *= vMax / v;
            } else if(v < vMin) {
                vx *= vMin / v;
                vy *= vMin / v;
            }
            float va = HEADING_CORRECTION_FACTOR * thetaError;
            bot.setDriveSpeed(vx, vy, va);
        }
        bot.setDriveSpeed(0, 0,0);
        bot.updateOdometry();
    }

    protected void driveToPosition(float vMax, float vMin, float targetX, float targetY, float targetThetaDegrees, float cp, float tolerance, Updatable action) {
        float headingTargetRadians = targetThetaDegrees * (float)Math.PI / 180;

        while (opModeIsActive()) {
            bot.updateOdometry();
            action.update();

            float xError = targetX - bot.getPose().x;
            float yError = targetY - bot.getPose().y;
            float thetaError = (float)AngleUtils.normalizeRadians(headingTargetRadians - bot.getPose().theta);

            if(Math.hypot(xError, yError) < tolerance) break;

            float sinTheta = (float)Math.sin(bot.getPose().theta);
            float cosTheta = (float)Math.cos(bot.getPose().theta);

            float xErrorRobot = xError * sinTheta - yError * cosTheta;
            float yErrorRobot = xError * cosTheta + yError * sinTheta;

            float vx = xErrorRobot * cp;
            float vy = yErrorRobot * cp;
            float v = (float)Math.hypot(vx, vy);
            if(v > vMax) {
                vx *= vMax / v;
                vy *= vMax / v;
            } else if(v < vMin) {
                vx *= vMin / v;
                vy *= vMin / v;
            }
            float va = HEADING_CORRECTION_FACTOR * thetaError;
            bot.setDriveSpeed(vx, vy, va);
        }
        bot.setDriveSpeed(0, 0,0);
        bot.updateOdometry();
    }

    protected void driveToPosition(MotionProfile mProf, float targetX, float targetY, float targetThetaDegrees,
                                   float tolerance) {
        float headingTargetRadians = targetThetaDegrees * (float)Math.PI / 180;

        bot.updateOdometry();
        float x0 = bot.getPose().x;
        float y0 = bot.getPose().y;

        if (loggingEnabled) {
            BetaLog.dd("\n\nDriveToPosition Init", "x0 = %.2f  y0 = %.2f  hd0 = %.1f  xTarg = %.2f  yTarg = %.2f  hdTarg = %.1f",
                    x0, y0, Math.toDegrees(bot.getPose().theta), targetX, targetY, targetThetaDegrees);
        }

        while (opModeIsActive()) {
            bot.updateOdometry();
            float d0 = (float)Math.hypot(bot.getPose().y - y0, bot.getPose().x - x0);

            float xError = targetX - bot.getPose().x;
            float yError = targetY - bot.getPose().y;
            float d1 = (float)Math.hypot(xError, yError);
            float thetaError = (float)AngleUtils.normalizeRadians(headingTargetRadians - bot.getPose().theta);

            if(d1 < tolerance) break;

            float sinTheta = (float)Math.sin(bot.getPose().theta);
            float cosTheta = (float)Math.cos(bot.getPose().theta);

            float xErrorRobot = xError * sinTheta - yError * cosTheta;
            float yErrorRobot = xError * cosTheta + yError * sinTheta;

            float v0 = (float)Math.sqrt(mProf.vMin * mProf.vMin + 2 * mProf.accel * d0);
            float v1 = (float)Math.sqrt(mProf.vMin * mProf.vMin + 2 * mProf.accel * d1);
            float v = Math.min(v0, v1);
            v = Math.min(v, mProf.vMax);

            float vx = xErrorRobot * v / d1;
            float vy = yErrorRobot * v / d1;

            float va = HEADING_CORRECTION_FACTOR * thetaError;

            if (loggingEnabled){
                BetaLog.dd("DriveToPosition","x = %.2f  y = %.2f  th = %.2f  xErr = %.2f  yErr = %.2f  thErr = %.2f",
                        bot.getPose().x, bot.getPose().y, Math.toDegrees(bot.getPose().theta),
                        xError, yError, Math.toDegrees(thetaError));
                BetaLog.dd("DriveToPosition", "xErrRobot = %.2f  yErrRobot = %.2f  vx = %.3f  vy = %.3f  va = %.2f",
                        xErrorRobot, yErrorRobot, vx, vy, Math.toDegrees(va));
            }
//            BetaLog.dd("pos", "x = %.1f  y = %.1f  th = %.1f  vx = %.1f  vy = %.1f  va = %.1f",
//                    bot.getPose().x, bot.getPose().y, Math.toDegrees(bot.getPose().theta),
//                    vx, vy, va);
            bot.setDriveSpeed(vx, vy, va);
        }
        bot.setDriveSpeed(0, 0,0);
        bot.updateOdometry();
    }


    protected void driveToPosition(float speed, float targetX, float targetY, float targetThetaDegrees,
                                   float tolerance) {
        float headingTargetRadians = targetThetaDegrees * (float)Math.PI / 180;

        bot.updateOdometry();
        float x0 = bot.getPose().x;
        float y0 = bot.getPose().y;

        if (loggingEnabled) {
            BetaLog.dd("\n\nDriveToPosition Init", "x0 = %.2f  y0 = %.2f  hd0 = %.1f  xTarg = %.2f  yTarg = %.2f  hdTarg = %.1f",
                    x0, y0, Math.toDegrees(bot.getPose().theta), targetX, targetY, targetThetaDegrees);
        }

        while (opModeIsActive()) {
            bot.updateOdometry();

            float xError = targetX - bot.getPose().x;
            float yError = targetY - bot.getPose().y;
            float d1 = (float)Math.hypot(xError, yError);
            float thetaError = (float)AngleUtils.normalizeRadians(headingTargetRadians - bot.getPose().theta);

            if(d1 < tolerance) break;

            float sinTheta = (float)Math.sin(bot.getPose().theta);
            float cosTheta = (float)Math.cos(bot.getPose().theta);

            float xErrorRobot = xError * sinTheta - yError * cosTheta;
            float yErrorRobot = xError * cosTheta + yError * sinTheta;



            float vx = xErrorRobot * speed / d1;
            float vy = yErrorRobot * speed / d1;

            float va = HEADING_CORRECTION_FACTOR * thetaError;

            if (loggingEnabled){
                BetaLog.dd("DriveToPosition","x = %.2f  y = %.2f  th = %.2f  xErr = %.2f  yErr = %.2f  thErr = %.2f",
                        bot.getPose().x, bot.getPose().y, Math.toDegrees(bot.getPose().theta),
                        xError, yError, Math.toDegrees(thetaError));
                BetaLog.dd("DriveToPosition", "xErrRobot = %.2f  yErrRobot = %.2f  vx = %.3f  vy = %.3f  va = %.2f",
                        xErrorRobot, yErrorRobot, vx, vy, Math.toDegrees(va));
            }
//            BetaLog.dd("pos", "x = %.1f  y = %.1f  th = %.1f  vx = %.1f  vy = %.1f  va = %.1f",
//                    bot.getPose().x, bot.getPose().y, Math.toDegrees(bot.getPose().theta),
//                    vx, vy, va);
            bot.setDriveSpeed(vx, vy, va);
        }
        bot.setDriveSpeed(0, 0,0);
        bot.updateOdometry();
    }

    protected void driveToPosition(MotionProfile mProf, float targetX, float targetY, float targetThetaDegrees,
                                   float tolerance, Updatable action) {
        float headingTargetRadians = targetThetaDegrees * (float)Math.PI / 180;

        bot.updateOdometry();
        float x0 = bot.getPose().x;
        float y0 = bot.getPose().y;
        action.update();

        while (opModeIsActive()) {
            bot.updateOdometry();
            float d0 = (float)Math.hypot(bot.getPose().y - y0, bot.getPose().x - x0);

            float xError = targetX - bot.getPose().x;
            float yError = targetY - bot.getPose().y;
            float d1 = (float)Math.hypot(xError, yError);
            float thetaError = (float)AngleUtils.normalizeRadians(headingTargetRadians - bot.getPose().theta);

            if(d1 < tolerance) break;
            action.update();

            float sinTheta = (float)Math.sin(bot.getPose().theta);
            float cosTheta = (float)Math.cos(bot.getPose().theta);

            float xErrorRobot = xError * sinTheta - yError * cosTheta;
            float yErrorRobot = xError * cosTheta + yError * sinTheta;

            float v0 = (float)Math.sqrt(mProf.vMin * mProf.vMin + 2 * mProf.accel * d0);
            float v1 = (float)Math.sqrt(mProf.vMin * mProf.vMin + 2 * mProf.accel * d1);
            float v = Math.min(v0, v1);
            v = Math.min(v, mProf.vMax);

            float vx = xErrorRobot * v / d1;
            float vy = yErrorRobot * v / d1;

            float va = HEADING_CORRECTION_FACTOR * thetaError;
            bot.setDriveSpeed(vx, vy, va);
        }
        bot.setDriveSpeed(0, 0,0);
        bot.updateOdometry();
    }

    protected void driveToPosition(MotionProfile mProf, float targetX, float targetY, float targetThetaDegrees,
                                   float tolerance, Predicate isFinished) {
        float headingTargetRadians = targetThetaDegrees * (float)Math.PI / 180;

        bot.updateOdometry();
        float x0 = bot.getPose().x;
        float y0 = bot.getPose().y;

        while (opModeIsActive()) {
            bot.updateOdometry();
            if (isFinished.isTrue())
                break;
            float d0 = (float)Math.hypot(bot.getPose().y - y0, bot.getPose().x - x0);

            float xError = targetX - bot.getPose().x;
            float yError = targetY - bot.getPose().y;
            float d1 = (float)Math.hypot(xError, yError);
            float thetaError = (float)AngleUtils.normalizeRadians(headingTargetRadians - bot.getPose().theta);

            if(d1 < tolerance) break;

            float sinTheta = (float)Math.sin(bot.getPose().theta);
            float cosTheta = (float)Math.cos(bot.getPose().theta);

            float xErrorRobot = xError * sinTheta - yError * cosTheta;
            float yErrorRobot = xError * cosTheta + yError * sinTheta;

            float v0 = (float)Math.sqrt(mProf.vMin * mProf.vMin + 2 * mProf.accel * d0);
            float v1 = (float)Math.sqrt(mProf.vMin * mProf.vMin + 2 * mProf.accel * d1);
            float v = Math.min(v0, v1);
            v = Math.min(v, mProf.vMax);

            float vx = xErrorRobot * v / d1;
            float vy = yErrorRobot * v / d1;

            float va = HEADING_CORRECTION_FACTOR * thetaError;
            bot.setDriveSpeed(vx, vy, va);
        }
        bot.setDriveSpeed(0, 0,0);
        bot.updateOdometry();
    }

    protected void driveToPositionGyro(float vMax, float vMin, float targetX, float targetY,
                                       float targetThetaDegrees, float cp, float tolerance) {
        float headingTargetRadians = targetThetaDegrees * (float)Math.PI / 180;

        while (opModeIsActive()) {
            bot.updateOdometry();

            float xError = targetX - bot.getPose().x;
            float yError = targetY - bot.getPose().y;
            float thetaError = (float)AngleUtils.normalizeRadians(headingTargetRadians - bot.getPose().theta);

            if(Math.hypot(xError, yError) < tolerance) {
                break;
            }

            float sinTheta = (float)Math.sin(bot.getPose().theta);
            float cosTheta = (float)Math.cos(bot.getPose().theta);

            float xErrorRobot = xError * sinTheta - yError * cosTheta;
            float yErrorRobot = xError * cosTheta + yError * sinTheta;

            float vx = xErrorRobot * cp;
            float vy = yErrorRobot * cp;
            float v = (float)Math.hypot(vx, vy);
            if(v > vMax) {
                vx *= vMax / v;
                vy *= vMax / v;
            } else if(v < vMin) {
                vx *= vMin / v;
                vy *= vMin / v;
            }
            float va = HEADING_CORRECTION_FACTOR * thetaError;

            bot.setDriveSpeed(vx, vy, va);

        }
        bot.setDriveSpeed(0, 0,0);
    }

    protected void setLoggingEnabled(boolean enabled){
        loggingEnabled = enabled;
    }

    protected boolean getLoggingEnabled(){
        return loggingEnabled;
    }


}