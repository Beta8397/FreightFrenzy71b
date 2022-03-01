package org.firstinspires.ftc.teamcode.freightbot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.freightbot.FreightBot;
import org.firstinspires.ftc.teamcode.freightbot.FreightBotAutonomous;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@Autonomous(name = "RedRightAutoTSE", group = "redAuto")
public class RedRightAutoTSE extends FreightBotAutonomous {

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
        bot.init(hardwareMap);
        super.setBot(bot);
        webcam = hardwareMap.get(WebcamName.class, "webcam");
        VuforiaNavigator.activate(null, webcam);
        bot.setPose(8, 66, 0);
        bot.openCapHolderServo();


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
        float flipperPosition;
        if (markerPos == MarkerPos.LEFT) {
            armAngleTicks = 70;
            flipperPosition = FLIPPER_BOTTOM;
            x1 = 30.5f;
            y1 = 74f;
        } else if (markerPos == MarkerPos.CENTER) {
            armAngleTicks = 320;  //was 410
            flipperPosition = FLIPPER_MID;
            x1 = 30f;
            y1 = 73.5f;
        } else {
            armAngleTicks = 615;
            flipperPosition = FLIPPER_TOP;
            x1 = 30.5f;
            y1 = 74f;
        }

//        bot.setArmAngleTicks(1000);
//        sleep(1000);
        int rotationTicks = markerPos == MarkerPos.LEFT? -2855 : markerPos == MarkerPos.CENTER? -2380 : -2030;
        int extensionTicks = markerPos == MarkerPos.LEFT? 23453 : markerPos == MarkerPos.CENTER? 27700 : 33382;
        float tapeElevation = markerPos == MarkerPos.LEFT? 0.614f : markerPos == MarkerPos.CENTER? 0.599f : 0.590f;

        pickUpTSE(rotationTicks,extensionTicks,tapeElevation);
        bot.closeCapHolderServo();


        bot.setArmAngleTicks(armAngleTicks);
        bot.setIntakeFlipper(flipperPosition);

        driveToPosition(12,x1,y1,0,1);

        turnToHeading(45, 3, 8, 60);


        bot.setIntakePower(-0.2);
        sleep(1000);
        driveToPosition(SLOW, 28, bot.getPose().y, 45, 1);
        turnToHeading(-90, 3, 8, 60);
        driveToPosition(SLOW, 28f, 75, -90, 1);
        bot.setIntakePower(0);
        bot.setArmAngleTicks(200);
        bot.setIntakeFlipper(0.3f);
        sleep(500);
        bot.setSpeedSpinnerMotor(0);


        driveToPosition(FAST, 28f, 9, -90, 1);


    }
}
