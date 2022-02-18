package org.firstinspires.ftc.teamcode.odombot;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.logging.BetaLog;
import org.firstinspires.ftc.teamcode.mecbot.MecBot;
import org.firstinspires.ftc.teamcode.util.AngleUtils;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Updatable;
import org.firstinspires.ftc.teamcode.util.odometry.Encoder;

import java.util.List;

public class OdometryBot extends MecBot {
    public Encoder rightEncoder = null;
    public Encoder leftEncoder = null;
    public Encoder horizEncoder = null;

    DcMotorEx rightEncoderMotor;
    DcMotorEx leftEncoderMotor;
    DcMotorEx horizontalEncoderMotor;

    public static final float TICKS_PER_INCH = 1869f;
    public static final float HORIZ_TICKS_PER_RAD = -13.3f;
    public static final float ROTATION_COEFF = 22176f;
    public static final float FRAC_LEFT = 0.4896f;
    public static final float FRAC_RIGHT = 0.5104f;
    public int rightTicks, leftTicks, horizTicks;
    public float gyroHeading;

    public OdometryBot() {
        super();
    }

    @Override
    public boolean init(HardwareMap hwMap) {
        boolean result = super.init(hwMap);
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        rightEncoderMotor = hwMap.get(DcMotorEx.class, "odom_right");
        rightEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftEncoderMotor = hwMap.get(DcMotorEx.class, "odom_left");
        leftEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalEncoderMotor = hwMap.get(DcMotorEx.class, "odom_horizontal");
        horizontalEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalEncoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder = new Encoder(rightEncoderMotor, false);
        leftEncoder = new Encoder(leftEncoderMotor, false);
        horizEncoder = new Encoder(horizontalEncoderMotor, false);

        //imu = hwMap.get(BNO055Enhanced.class, "imu");
        return result;
    }

    @Override
    public Pose updateOdometry(){
        int rightCurrentTicks = rightEncoder.getCurrentPosition();
        int leftCurrentTicks = leftEncoder.getCurrentPosition();
        int horizCurrentTicks = horizEncoder.getCurrentPosition();
        int rightNewTicks = rightCurrentTicks - rightTicks;
        int leftNewTicks = leftCurrentTicks - leftTicks;
        int horizNewTicks = horizCurrentTicks - horizTicks;
        rightTicks = rightCurrentTicks;
        leftTicks = leftCurrentTicks;
        horizTicks = horizCurrentTicks;

//        BetaLog.dd("ODOM", "New ticks   Left = %d   Right = %d   Horiz = %d",
//                leftNewTicks, rightNewTicks, horizNewTicks);

        float dyR = (FRAC_LEFT * rightNewTicks + FRAC_RIGHT * leftNewTicks) / TICKS_PER_INCH;
        float newGyroHeading = getHeadingRadians();
        float dT;
        float heading;

        if (newGyroHeading != gyroHeading) {
            dT = (float)AngleUtils.normalizeRadians(newGyroHeading - pose.theta);
            gyroHeading = newGyroHeading;
            heading = newGyroHeading;
        } else {
            dT = (rightNewTicks - leftNewTicks) / (ROTATION_COEFF);
            heading = (float)AngleUtils.normalizeRadians(pose.theta + dT);
        }
//        dT = (rightNewTicks - leftNewTicks) / (ROTATION_COEFF);
//        heading = (float)AngleUtils.normalizeRadians(pose.theta + dT);

        float horizEncoderAngleTicks = HORIZ_TICKS_PER_RAD * dT;
        float dxR = (horizNewTicks - horizEncoderAngleTicks) / TICKS_PER_INCH;
//        BetaLog.dd("ODOM", "dT = %.1f   horizEncoderAngleTicks = %.1f", dT, horizEncoderAngleTicks);
        float avgHeading = (float) AngleUtils.normalizeRadians(pose.theta + 0.5 * dT);
//        BetaLog.dd("ODOM", "dxR = %.3f    dyR = %.3f", dxR, dyR);
        float dX = dxR * (float)Math.sin(avgHeading) + dyR * (float)Math.cos(avgHeading);
        float dY = -dxR * (float)Math.cos(avgHeading) + dyR * (float)Math.sin(avgHeading);
//        BetaLog.dd("ODOM", "dX = %.3f   dY = %.3f", dX, dY);
        /*
         * Update the Pose object with the new values for X, Y, and Heading
         */
        pose = new Pose(pose.x + dX, pose.y + dY, heading);
//        BetaLog.dd("ODOM", "NewPose: x = %.3f   y = %.3f    theta = %.1f", pose.x, pose.y, Math.toDegrees(pose.theta));
        /*
         * Return the updated Pose object
         */
        BetaLog.dd("ODOM", "%d  %d  %d  %d  %d  %d  %.4f  %.6f  %.6f  %.3f  %.4f  %.6f  %.4f  %.4f  %.4f  %.4f",
                rightCurrentTicks, leftCurrentTicks, horizCurrentTicks, rightNewTicks, leftNewTicks, horizNewTicks,
                dyR, dT, heading, horizEncoderAngleTicks, dxR, avgHeading, dX, dY, pose.x, pose.y);
        return pose;

    }
    @Override
    public void updateTicks(){
        leftTicks = leftEncoder.getCurrentPosition();
        rightTicks = rightEncoder.getCurrentPosition();
        horizTicks = horizEncoder.getCurrentPosition();

    }

    @Override
    public void setHeadingDegrees(float headingDegrees){
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        headingOffsetRadians = (float)Math.toRadians(headingDegrees) - orientation.firstAngle;
        pose = new Pose(pose.x, pose.y, (float)Math.toRadians(headingDegrees));
        updateTicks();
        gyroHeading = (float)Math.toRadians(headingDegrees);
    }

}
