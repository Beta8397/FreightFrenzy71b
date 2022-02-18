package org.firstinspires.ftc.teamcode.freightbot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.freightbot.FreightBot;
import org.firstinspires.ftc.teamcode.freightbot.FreightBotAutonomous;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@Autonomous(name = "RedRightAutoNoCarousel", group = "redAuto")
public class RedRightAutoNoCarousel extends FreightBotAutonomous {

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
        bot.setPose(8, 66, 180);
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

        telemetry.addData("marker pos", markerPos);
        telemetry.update();
        // TODO everything else
        telemetry.addData("first drive done", "");
        telemetry.update();
        int armAngleTicks;
        float x1;
        float y1;
        int barrierDriveX;
        if (markerPos == MarkerPos.LEFT) {
            armAngleTicks = 610;
            x1 = 21.5f;
            y1 = 63.75f;
            barrierDriveX = 25;
        } else if (markerPos == MarkerPos.CENTER) {
            armAngleTicks = 390; //was 410
            x1 = 17; // was 19
            y1 = 65; // was 63
            barrierDriveX = 25;
        } else {
            armAngleTicks = 200;
            x1 = 21;
            y1 = 65;
            barrierDriveX = 21;
        }

        bot.setArmExtensionTicks(600);

        if (markerPos == MarkerPos.LEFT) {
            driveToPosition(8, x1, y1, 180, 1);
        } else {
            driveToPosition(SLOW, x1, y1, 180, 1);
        }


        bot.setIntakeState(FreightBot.IntakeState.CENTER_MID);
        rotateTapeAndAngleArm(armAngleTicks, markerPos);
        turnToHeading(-135, 3, 8, 60);

//        bot.setArmExtensionTicks(950);
//        sleep(250);
//        bot.setArmServoPosition(FreightBot.DUMPER_EXTENDED);
//        sleep(1750);
//        bot.setArmExtensionTicks(0);
//        sleep(100);
//        bot.setArmServoPosition(FreightBot.DUMPER_RETRACTED);
//        bot.setArmAngleTicks(0);

        deliverShippingHub();

        turnToHeading(-90, 3, 8, 60);
        driveToPosition(SLOW, 26, 65, -90, 1);
        driveToPosition(SLOW, barrierDriveX, 75, -90, 1);
        driveToPosition(FAST, barrierDriveX, 9, -90, 1);


    }
}
