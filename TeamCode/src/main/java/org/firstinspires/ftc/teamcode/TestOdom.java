package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.odombot.OdometryBot;
import org.firstinspires.ftc.teamcode.logging.BetaLog;
import org.firstinspires.ftc.teamcode.logging.LoggingLinearOpMode;

@TeleOp (name ="Test Odom", group = "test")

public class TestOdom extends LoggingLinearOpMode {

    OdometryBot bot = new OdometryBot();

    @Override
    public void runLoggingOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();

        bot.setPose(0,0,90);

        float armTarget = 0;

        while (opModeIsActive()){

            bot.updateOdometry();
            telemetry.addData("ticks", "L %d R %d H %d", bot.leftTicks, bot.rightTicks, bot.horizTicks);
            telemetry.addData("pose", "x %.1f y %.1f t %.1f", bot.getPose().x, bot.getPose().y, Math.toDegrees(bot.getPose().theta));
            telemetry.addData("gyroheading", Math.toDegrees(bot.gyroHeading));


            float px = gamepad1.left_stick_x * 0.5f;
            float py = -gamepad1.left_stick_y * 0.5f;
            float pa = (gamepad1.left_trigger - gamepad1.right_trigger) * 0.5f;
            telemetry.update();


            bot.setDrivePower(px, py, pa);

        }
    }
}
