package org.firstinspires.ftc.teamcode.mecbot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.RowMajorMatrixF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.logging.BetaLog;
import org.firstinspires.ftc.teamcode.util.KalmanMeasurementUpdater;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.AngleUtils;

import java.util.List;

/**
 * The MechBot class represents a mecanum-wheeled robot with a BNO055IMU and a color sensor. It has methods
 * that provide the basic functionality of such a robot
 */
public class MecBot {

    /*
     * Constants
     */

    public final MotorType MOTOR_TYPE;
    public final float GEAR_RATIO;
    public final float ABS_GEAR_RATIO;
    public final float TICKS_PER_ROTATION;

    public static final float MAX_TICKS_PER_SECOND = 2500;

    public final float TICKS_PER_INCH_FWD;
    public final float TICKS_PER_INCH_STRAFE;
    public final float TICKS_PER_RADIAN;
    public final float STRAFE_VARIANCE_COEFF = 0.16f;
    public final float FWD_VARIANCE_COEFF = 0.16f;
    public final float HEADING_VARIANCE = 0.00122f;

    protected boolean loggingEnabled = false;

    /*
     * Drive Motors
     */
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    /*
     * The BNO055Enhanced (gyro)
     */
    public BNO055IMU imu;

    /*
     * The heading offset (this gives us flexibility in specifying the world coordinate system).
     */
    public float headingOffsetRadians = 0;

    public float[] rawIMUAngles = new float[3];

    /*
     * The current pose of the robot
     */
    protected Pose pose = new Pose(0, 0, 0);

    /*
     * The most recent previous readings of the drive motor ticks
     */
    int ticksBL, ticksFL, ticksFR, ticksBR;

    protected KalmanMeasurementUpdater kalmanMeasurementUpdater = null;
    protected MatrixF covariance = new GeneralMatrixF(2,2);

    public void setKalmanMeasurementUpdater(KalmanMeasurementUpdater kMU) {
        kalmanMeasurementUpdater = kMU;
    }

    public void setCovariance(float covXX,float covYY,float covXY) {
        covariance.put(0,0,covXX);
        covariance.put(0,1,covXY);
        covariance.put(1,0,covXY);
        covariance.put(1,1,covYY);

    }

    public MatrixF getCovariance() {
        return covariance;
    }

    /**
     * Enum MotorType represents types of motors that can be used to power the drive.
     */
    public enum MotorType {
        Neverest40(1120, false),
        Neverest20(560, false),
        NeverestOrbital20(560, true),
        Neverest60(1680, false);

        MotorType(double ticksPerRotation, boolean reversed){
            this.ticksPerRotation = ticksPerRotation;
            this.reversed = reversed;
        }

        private double ticksPerRotation;
        private boolean reversed;
    }

    /**
     * Constructor: USE THIS ONE FOR A REAL ROBOT!
     * @param mType     Motor Type
     * @param w         Wheel Base Width (inches)
     * @param l         Wheel Base Length (inches)
     * @param wheelDiam     Wheel Diameter (inches)
     * @param rollerAngle   Wheel Roller Angle (degrees)
     * @param gearRatio     Gear Ratio (motor output shaft rotations per wheel rotation)
     * @param axesMap       Axes Map for the BNO055Enhanced
     * @param axesSign      Axes Sign for the BNO055Enhanced
     */
    public MecBot(MotorType mType, float w, float l, float wheelDiam, float rollerAngle, float gearRatio, BNO055Enhanced.AxesMap axesMap, BNO055Enhanced.AxesSign axesSign){
        MOTOR_TYPE = mType;
          float tan_alpha = (float)Math.tan(Math.toRadians(rollerAngle));
          float wl_avg = 0.5f*w + 0.5f*l/tan_alpha;
          float wheel_circumference = (float)Math.PI * wheelDiam;
        GEAR_RATIO = gearRatio;
        ABS_GEAR_RATIO = Math.abs(gearRatio);
//        AXES_MAP = axesMap;
//        AXES_SIGN = axesSign;
        TICKS_PER_ROTATION = (float)mType.ticksPerRotation;
        TICKS_PER_INCH_FWD = TICKS_PER_ROTATION * ABS_GEAR_RATIO / wheel_circumference;
        TICKS_PER_INCH_STRAFE = TICKS_PER_ROTATION * ABS_GEAR_RATIO /
                (wheel_circumference * tan_alpha);
        TICKS_PER_RADIAN = wl_avg * TICKS_PER_ROTATION * ABS_GEAR_RATIO / wheel_circumference;
    }

    public MecBot(MotorType mType, float ticksPerInchFwd, float ticksPerInchStrafe,
                  float ticksPerRadian, float gearRatio) {
        MOTOR_TYPE = mType;
        TICKS_PER_INCH_FWD = ticksPerInchFwd;
        TICKS_PER_INCH_STRAFE = ticksPerInchStrafe;
        TICKS_PER_RADIAN = ticksPerRadian;
        TICKS_PER_ROTATION = (float)mType.ticksPerRotation;
        GEAR_RATIO = gearRatio;
        ABS_GEAR_RATIO = Math.abs(gearRatio);
    }

    /**
     *  No-argument constructor:  USE THIS ONE FOR THE SIMULATOR!!
     */
    public MecBot(){
        MOTOR_TYPE = MotorType.NeverestOrbital20;
//        TAN_ALPHA = 1;
//        WL_AVG = 15;
//        WHEEL_CIRCUMFERENCE = 6*(float)Math.PI;
        GEAR_RATIO = 1;
        ABS_GEAR_RATIO = 1;
//        AXES_MAP = BNO055Enhanced.AxesMap.XYZ;
//        AXES_SIGN = BNO055Enhanced.AxesSign.PPP;
        TICKS_PER_ROTATION = (float)MOTOR_TYPE.ticksPerRotation;
        TICKS_PER_INCH_FWD = 29.82f;
        TICKS_PER_INCH_STRAFE = 29.82f; //was 24.86
        TICKS_PER_RADIAN = 354.75f;

    }

    /**
     * Obtain instances of the robot hardware using the hardware map, and initialize the BNO055IMU
     * @param hwMap
     */
    public boolean init(HardwareMap hwMap) {
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        frontLeft = hwMap.get(DcMotorEx.class, "front_left_motor");
        frontRight = hwMap.get(DcMotorEx.class, "front_right_motor");
        backLeft = hwMap.get(DcMotorEx.class, "back_left_motor");
        backRight = hwMap.get(DcMotorEx.class, "back_right_motor");

        //setDriveVelocityPIDF(15, 0.3, 0, 20);

        /*
         * Either the right or the left motors need to have their directions reversed, depending upon
         * the MotorType
         */
        if (MOTOR_TYPE.reversed && GEAR_RATIO > 0
                || !MOTOR_TYPE.reversed && GEAR_RATIO < 0) {
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
         * Set Mode of all four drive motors to RUN_USING_ENCODER. That way, calls to setPower result in PIDF
         * control to keep the motor at the specified speed (as a fraction of Max), rather than raw power.
         */
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hwMap.get(BNO055IMU.class, "imu");

//        BNO055Enhanced.Parameters parameters = new BNO055Enhanced.Parameters();
        BNO055IMU.Parameters parameters = new BNO055Enhanced.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BN055Cali.json";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
//        parameters.axesMap = AXES_MAP;
//        parameters.axesSign = AXES_SIGN;

        return imu.initialize(parameters);
    }

    public void setDriveVelocityPIDF(double p, double i, double d, double f) {
        backLeft.setVelocityPIDFCoefficients(p, i, d, f);
        backRight.setVelocityPIDFCoefficients(p, i, d, f);
        frontLeft.setVelocityPIDFCoefficients(p, i, d, f);
        frontRight.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public PIDFCoefficients getDriveVelocityPIDF(){
        return backLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Return current robot position and orientation as a Pose object
     * @return robot pose
     */
    public Pose getPose() {
        return pose;
    }

    /*
     * Methods that provide access to the individual motors, mainly needed for diagnostics
     */
    public DcMotorEx getBackLeft() {return backLeft;}
    public DcMotorEx getFrontLeft() {return frontLeft;}
    public DcMotorEx getFrontRight() {return frontRight;}
    public DcMotorEx getBackRight() {return backRight;}

    /**
     * Obtain the current robot heading using the IMU (note use of the heading offset)
     * @return heading in radians
     */
    public float getHeadingRadians(){
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        rawIMUAngles[0] = (float)orientation.firstAngle;
        rawIMUAngles[1] = (float)orientation.secondAngle;
        rawIMUAngles[2] = (float)orientation.thirdAngle;
        return headingOffsetRadians + orientation.firstAngle;
    }

    /**
     * Set the robot's heading to the specified value, in degrees. Store this heading in the Pose object, adjust the
     * headingOffsetRadians value as necessary, and update the tick readings of the drive motors.
     * @param headingDegrees
     */
    public void setHeadingDegrees(float headingDegrees){
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        headingOffsetRadians = (float)Math.toRadians(headingDegrees) - orientation.firstAngle;
        pose = new Pose(pose.x, pose.y, (float)Math.toRadians(headingDegrees));
        updateTicks();
    }

    public void setPose(float x, float y, float headingDegrees){
        pose = new Pose(x, y, pose.theta);
        setHeadingDegrees(headingDegrees);
    }

    public void setPose(float x, float y){
        pose = new Pose(x, y, pose.theta);
        updateTicks();
    }

    /**
     * Update the tick readings of the drive motors.
     */
    public void updateTicks(){
        ticksBL = backLeft.getCurrentPosition();
        ticksFL = frontLeft.getCurrentPosition();
        ticksFR = frontRight.getCurrentPosition();
        ticksBR = backRight.getCurrentPosition();
    }

    /**
     * Set the drive powers for the forward, rotation, and strafe directions. Note that these powers are in
     * the range of -1 to +1, and represent the FRACTION of maximal possible drive speed in each direction.
     * @param px    rightward strafe (robot-X-axis) power
     * @param py    forward power (Y-direction)
     * @param pa    rotation power (counter-clockwise if positive)
     */
    public void setDrivePower(float px, float py, float pa) {
        float frontLeftPower = py - pa + px;
        float frontRightPower = py + pa - px;
        float backLeftPower = py - pa - px;
        float backRightPower = py + pa + px;

        float largest = 1;
        largest = Math.max(largest, Math.abs(frontLeftPower));
        largest = Math.max(largest, Math.abs(frontRightPower));
        largest = Math.max(largest, Math.abs(backLeftPower));
        largest = Math.max(largest, Math.abs(backRightPower));

        frontLeftPower /= largest;
        frontRightPower /= largest;
        backLeftPower /= largest;
        backRightPower /= largest;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

    }

    /**
     * Set robot drive speeds in INCHES PER SECOND (for vx and vy) or RADIANS PER SECOND (for va)
     * @param vx    Drive speed along robot-X-axis (rightward if positive)
     * @param vy    Drive speed along robot-Y-axis (forward if positive)
     * @param va    Rotation speed in radians/sec (counter-clockwise if positive)
     */
    public void setDriveSpeed(float vx, float vy, float va){
        float px = vx * TICKS_PER_INCH_STRAFE / MAX_TICKS_PER_SECOND;
        float py = vy * TICKS_PER_INCH_FWD / MAX_TICKS_PER_SECOND;
        float pa = va * TICKS_PER_RADIAN / MAX_TICKS_PER_SECOND;
        setDrivePower(px,py,pa);
    }

    /**
     * Update robot pose (position on the field, and heading) for a single iteration of a control loop. This is a
     * gyro-assisted odometry algorithm.
     * @return
     */
    public Pose updateOdometry(){
        /*
         * Obtain new heading using the IMU. Then calculate the (small) interval change in the heading, as well as
         * the average heading during the (small) interval of time since the previous iteration. Note the use of the
         * AngleUtils.normalizeRadians method to keep angles in the -PI to +PI range.
         */
        float heading = getHeadingRadians();
        float headingChange = (float)AngleUtils.normalizeRadians(heading - pose.theta);
        float avgHeading = (float)AngleUtils.normalizeRadians(pose.theta + 0.5 * headingChange);

        /*
         * Get the current drive motor ticks
         */
        int currBL = backLeft.getCurrentPosition();
        int currFL = frontLeft.getCurrentPosition();
        int currFR = frontRight.getCurrentPosition();
        int currBR = backRight.getCurrentPosition();

        /*
         * Calculate the NEW ticks that have occurred since the previous update (new = current - previous)
         */
        int newBL = currBL - ticksBL;
        int newFL = currFL - ticksFL;
        int newFR = currFR - ticksFR;
        int newBR = currBR - ticksBR;

        /*
         * Update the fields that store the tick values, so they will be ready for the next iteration.
         */
        ticksBL = currBL;
        ticksFL = currFL;
        ticksFR = currFR;
        ticksBR = currBR;

        /*
         * Determine the distance that the surface of each wheel has rolled.
         *
         * NOTE: Analysis of DIMENSIONS can be helpful in getting this right. We want to get from "new ticks" to
         * "inches of travel".
         *
         * newBL has dimensions of ticks, and TICKS_PER_ROTATION has dimensions of ticks/rotation.
         * So (newBL / TICKS_PER_ROTATION) has dimensions of Rotations.
         * WHEEL_CIRCUMFERENCE has dimensions of inches/rotation.
         * So (newBL / TICKS_PER_ROTATION) * WHEEL_CIRCUMFERENCE has dimensions of  inches.
         *
         */

        /*
         * Determine small increment of robot motion in ROBOT COORDINATE SYSTEM
         */

        float dXR = 0.25f * (-newBL + newFL - newFR + newBR) / TICKS_PER_INCH_STRAFE;
        float dYR = 0.25f * (newBL + newFL + newFR + newBR) / TICKS_PER_INCH_FWD;

        /*
         * Convert this small increment of robot motion into WORLD COORDINATES
         */
        float sin = (float) Math.sin(avgHeading);
        float cos = (float) Math.cos(avgHeading);
        float dX = dXR * sin + dYR * cos;
        float dY = -dXR * cos + dYR * sin;

        /*
         * Update the Pose object with the new values for X, Y, and Heading
         */
        Pose poseMinus = new Pose(pose.x + dX, pose.y + dY, heading);

        if (kalmanMeasurementUpdater == null) {
            pose = poseMinus;
        } else {
            float varXR = Math.abs(dXR*STRAFE_VARIANCE_COEFF);
            float varYR = Math.abs(dYR*FWD_VARIANCE_COEFF);
            float varTheta = HEADING_VARIANCE;

            /*
             * Determine the uncertainty (in the form of a covariance matrix Q) that the odometry
             * iteration adds to the previously-existing uncertainty of our pose estimate.
             */
            MatrixF Q = new GeneralMatrixF(2,2);
            Q.put(0, 0, varXR * sin * sin + varYR * cos * cos +
                            varTheta*(dXR*cos - dYR*sin)*(dXR*cos - dYR*sin));
            Q.put(1,1,varXR*cos*cos + varYR*sin*sin +
                    varTheta*(dXR*sin + dYR*cos)*(dXR*sin + dYR*cos));
            float covxy = (varYR - varXR)*sin*cos +
                    varTheta*((dXR*dXR - dYR*dYR)*sin*cos + dXR*dYR*(cos*cos - sin*sin));
            Q.put(0,1,covxy);
            Q.put(1,0,covxy);
            covariance.add(Q);

            /*
             * Apply measurements using Kalman filter algorithm. In addition to returning the pose,
             * this method call will update (in place) the covariance matrix.
             */
            pose = kalmanMeasurementUpdater.kalmanMeasurementUpdate(poseMinus, covariance);
        }

        /*
         * Return the updated Pose object
         */
        return pose;
    }

    public Pose getVelocityRobot() {
        float vBL = (float)backLeft.getVelocity();
        float vBR = (float)backRight.getVelocity();
        float vFL = (float)frontLeft.getVelocity();
        float vFR = (float)frontRight.getVelocity();

        float vX = .25f * (-vBL + vFL - vFR + vBR)/TICKS_PER_INCH_STRAFE;
        float vY = .25f * (vBL + vFL + vFR + vBR)/TICKS_PER_INCH_FWD;
        float vA = .25f * (-vBL - vFL + vFR + vBR)/TICKS_PER_RADIAN;

        return new Pose(vX, vY, vA);
    }

    public Pose getVelocity() {
        Pose robotVelocity = getVelocityRobot();

        float vX = robotVelocity.x * (float)Math.sin(getPose().theta) + robotVelocity.y * (float)Math.cos(getPose().theta);
        float vY = -robotVelocity.x * (float)Math.cos(getPose().theta) + robotVelocity.y * (float)Math.sin(getPose().theta);

        return new Pose(vX, vY, robotVelocity.theta);
    }

    public void setLoggingEnabled(boolean enabled){
        loggingEnabled = enabled;
    }

    public boolean getLoggingEnabled(){
        return loggingEnabled;
    }
}
