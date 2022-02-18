package org.firstinspires.ftc.teamcode.freightbot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.freightbot.FreightBot;
import org.firstinspires.ftc.teamcode.freightbot.FreightBotAutonomous;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@Autonomous(name = "RedLeftAutoStorageDelay", group = "redAuto")
public class RedLeftAutoStorageDelay extends FreightBotAutonomous {

    FreightBot bot = new FreightBot();
    WebcamName webcam = null;
    ButtonToggle toggleDPUp = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.dpad_up;
        }
    };
    ButtonToggle toggleDPDown = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.dpad_down;
        }
    };

    double delay = 0;

    @Override
    public void runLoggingOpMode() {
        bot.init(hardwareMap, true);
        super.setBot(bot);
        webcam = hardwareMap.get(WebcamName.class, "webcam");
        VuforiaNavigator.activate(null, webcam);
        bot.setPose(8, 114, 180);
        bot.closeArmCapServo();
        telemetry.addData("press start when ready", "");
        telemetry.update();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("delay (seconds)", delay);
            telemetry.addData("press start when ready", "");
            telemetry.update();
            if (toggleDPUp.update()) delay += 0.5;
            if (toggleDPDown.update()) delay -= 0.5;
            delay = Range.clip(delay,0,7);
        }

        MarkerPos markerPos = getMarkerPos(CameraStartPos.RIGHT);

        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive() && et.seconds() < delay) {
            telemetry.addData("countdown",(int)(delay-et.seconds()));
            telemetry.update();
        }

        int armAngleTicks;
        float x1;
        float y1;
        if (markerPos == MarkerPos.LEFT) {
            armAngleTicks = 610;
            x1 = 19.0f; //was 17.25
            y1 = 107.5f; //was 109.25
        } else if (markerPos == MarkerPos.CENTER) {
            armAngleTicks = 390; // was 420
            x1 = 20f; //was 19
            y1 = 109.5f; //was 108.5
        } else {
            armAngleTicks = 200;
            x1 = 22; //was 21
            y1 = 105.8f; //was 103
        }

        bot.setArmExtensionTicks(600);

        if (markerPos == MarkerPos.LEFT) {
            driveToPosition(8, x1, y1, 180, 1);
        } else {
            driveToPosition(SLOW, x1, y1, 180, 1);
        }


        bot.setIntakeState(FreightBot.IntakeState.CENTER_MID);
        rotateTapeAndAngleArm(armAngleTicks, markerPos);
        turnToHeading(135, 3, 8, 60);

//        bot.setArmExtensionTicks(950);
//        sleep(250);
//        bot.setArmServoPosition(FreightBot.DUMPER_EXTENDED);
//        sleep(1750);
//        bot.setArmExtensionTicks(0);
//        sleep(300);
//        bot.setArmServoPosition(FreightBot.DUMPER_RETRACTED);
//        bot.setArmAngleTicks(0);

        deliverShippingHub();
        driveToPosition(SLOW, bot.getPose().x-2, bot.getPose().y+2,
                (float)Math.toDegrees(bot.getHeadingRadians()), 1);

        turnToHeading(-90, 3, 8, 60);
        driveToPosition(SLOW, 20, 136, -90, 1);

        carouselDrive(Alliance.RED);
        bot.setSpeedSpinnerMotor(0);

        driveToParkStorage(Alliance.RED);

    }
}
