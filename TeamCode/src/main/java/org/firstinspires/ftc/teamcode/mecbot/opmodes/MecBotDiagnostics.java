/**
 * Diagnostic OpMode for use with our 2nd MechBot (with Neverest 40s, gear ratio 0.5, and Rev Hub mounted on its side
 */

package org.firstinspires.ftc.teamcode.mecbot.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.apache.commons.math3.special.Beta;
import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.logging.BetaLog;
import org.firstinspires.ftc.teamcode.logging.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.mecbot.MecBot;
import org.firstinspires.ftc.teamcode.util.AngleUtils;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@TeleOp(name = "MecBotDiagnostics", group = "Test")
//@Disabled
public class MecBotDiagnostics extends LoggingLinearOpMode {

    private MecBot bot = new MecBot();

    boolean drivingStraight = false;
    float targetHeading = 0;

    ButtonToggle toggleLeftBmp1 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.left_bumper;
        }
    };



    @Override
    public void runLoggingOpMode() {


        bot.init(hardwareMap);

//        bot.getFrontLeft().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bot.getFrontRight().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bot.getBackLeft().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bot.getBackRight().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bot.setHeadingDegrees(0);

        PIDFCoefficients pidfBL = bot.getBackLeft().getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfBR = bot.getBackRight().getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfFL = bot.getFrontLeft().getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfFR = bot.getFrontRight().getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("DIAGNOSTICS", "BL  P = %.3f  I = %.3f  D = %.3f  F = %.3f",
                pidfBL.p, pidfBL.i, pidfBL.d, pidfBL.f);
        telemetry.addData("DIAGNOSTICS", "BR  P = %.3f  I = %.3f  D = %.3f  F = %.3f",
                pidfBR.p, pidfBR.i, pidfBR.d, pidfBR.f);
        telemetry.addData("DIAGNOSTICS", "FL  P = %.3f  I = %.3f  D = %.3f  F = %.3f",
                pidfFL.p, pidfFL.i, pidfFL.d, pidfFL.f);
        telemetry.addData("DIAGNOSTICS", "FR  P = %.3f  I = %.3f  D = %.3f  F = %.3f",
                pidfFR.p, pidfFR.i, pidfFR.d, pidfFR.f);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            doOneIteration();
            telemetry.update();
        }



    }

    protected void doOneIteration(){

        bot.updateOdometry();

        boolean ltBmp1Toggled = toggleLeftBmp1.update();
        if (ltBmp1Toggled){
            drivingStraight = !drivingStraight;
            if (drivingStraight){
                targetHeading = bot.getPose().theta;
            }
        }

        if (drivingStraight){
            telemetry.addData("DRIVINGFORWARD","");
            float thetaError =
                    (float)AngleUtils.normalizeRadians(bot.getPose().theta - targetHeading);
            float pFwd = -gamepad1.left_stick_y;
            float pTheta = -1.0f * thetaError;
            handleBotPowers(0, pFwd, pTheta);
        }
        else if (gamepad1.a) handleMotorPowers(0.2f,0,0,0);
        else if (gamepad1.x) handleMotorPowers(0,0.2f,0,0);
        else if (gamepad1.y) handleMotorPowers(0,0,0.2f,0);
        else if (gamepad1.b) handleMotorPowers(0,0,0,0.2f);
        else if (gamepad1.dpad_right) handleBotPowers(0.2f, 0, 0);
        else if (gamepad1.dpad_up) handleBotPowers(0, 0.2f, 0);
        else if (gamepad1.dpad_down) handleBotPowers(0, 0, 0.2f);
        else {
            float px = gamepad1.left_stick_x;
            float py = -gamepad1.left_stick_y;
            float pa = -gamepad1.right_stick_x;
            if (Math.abs(px) < 0.05) px = 0;
            if (Math.abs(py) < 0.05) py = 0;
            if (Math.abs(pa) < 0.05) pa = 0;
            handleBotPowers(px, py, pa);
        }

        telemetry.addData("Angles", "Hd %.1f  2nd %.1f  3rd %.1f",
                bot.getPose().theta * 180.0 / Math.PI,
                Math.toDegrees(bot.rawIMUAngles[1]),
                Math.toDegrees(bot.rawIMUAngles[2]));
        telemetry.addData("Odometry", "X = %.1f  Y = %.1f  Theta = %.1f",
                bot.getPose().x, bot.getPose().y, bot.getPose().theta *180.0 / Math.PI);
        telemetry.addData("Encoders", "BL %d  FL %d  FR %d  BR %d",
                bot.getBackLeft().getCurrentPosition(), bot.getFrontLeft().getCurrentPosition(),
                bot.getFrontRight().getCurrentPosition(), bot.getBackRight().getCurrentPosition());
        telemetry.addData("Speeds", "BL %.0f  FL %.0f,  FR %.0f  BR %.0f",
                bot.getBackLeft().getVelocity(), bot.getFrontLeft().getVelocity(),
                bot.getFrontRight().getVelocity(), bot.getBackRight().getVelocity());

//        BetaLog.dd("DIAGNOSTICS", "ticks  BL = %d  FL = %d  FR = %d  BR = %d",
//                bot.getBackLeft().getCurrentPosition(), bot.getFrontLeft().getCurrentPosition(),
//                bot.getFrontRight().getCurrentPosition(), bot.getBackRight().getCurrentPosition());
//        BetaLog.dd("DIAGNOSTICS", "velocities  BL = %.0f  FL = %.0f  FR = %.0f  BR = %.0f",
//                bot.getBackLeft().getVelocity(), bot.getFrontLeft().getVelocity(),
//                bot.getFrontRight().getVelocity(), bot.getBackRight().getVelocity());
//        BetaLog.d("DIAGNOSTICS");

    }

    private void handleBotPowers(float px, float py, float pa){
        bot.setDrivePower(px, py, pa);
        telemetry.addData("Robot Power", "PX = %.2f  PY = %.2f  PA = %.2f", px, py, pa);
        telemetry.addData("Motor Powers", "BL: %.2f  FL: %.2f  FR: %.2f  BR: %.2f",
                bot.getBackLeft().getPower(), bot.getFrontLeft().getPower(),
                bot.getFrontRight().getPower(), bot.getBackRight().getPower());
    }

    private void handleMotorPowers(float p1, float p2, float p3, float p4){
        bot.getBackLeft().setPower(p1);
        bot.getFrontLeft().setPower(p2);
        bot.getFrontRight().setPower(p3);
        bot.getBackRight().setPower(p4);
        telemetry.addData("Motor Powers", "BL: %.2f  FL: %.2f  FR: %.2f  BR: %.2f", p1, p2, p3, p4);
    }

}
