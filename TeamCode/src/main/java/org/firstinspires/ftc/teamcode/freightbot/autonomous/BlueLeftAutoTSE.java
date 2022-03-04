package org.firstinspires.ftc.teamcode.freightbot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.freightbot.FreightBot;
import org.firstinspires.ftc.teamcode.freightbot.FreightBotAutonomous;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@Autonomous(name = "BlueLeftAutoTSE", group = "blueAuto")
public class BlueLeftAutoTSE extends FreightBotAutonomous {

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
    float headingAngleDegrees = 135;  //was 150

    @Override
    public void runLoggingOpMode() {
        bot.init(hardwareMap);
        super.setBot(bot);
        webcam = hardwareMap.get(WebcamName.class, "webcam");
        VuforiaNavigator.activate(null, webcam);
        bot.setPose(-8, 59, 180);
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
            armAngleTicks = ARM_BOTTOM;
            flipperPosition = FLIPPER_BOTTOM;
            x1 = -31.5f;//  was -28.8
            y1 = 69.5f; //  was 71.5
        } else if (markerPos == MarkerPos.CENTER) {
            armAngleTicks = ARM_MID;
            flipperPosition = FLIPPER_MID;
            x1 = -30.5f; //   was -31
            y1 = 68.5f; //  was 69
        } else {
            armAngleTicks = ARM_TOP;
            flipperPosition = FLIPPER_TOP;
            x1 = -31f; //   was -28.8
            y1 = 69f; //   was 71.5
        }

        // Rotation 200 = 10 degrees
        // Extension 1000 = 1 inch, center extension was 26216
        int rotationTicks = markerPos == MarkerPos.LEFT? -3099 : markerPos == MarkerPos.CENTER? -2550 : -2175; // was -3099, -2580, -2175
        int extensionTicks = markerPos == MarkerPos.LEFT? 23707 : markerPos == MarkerPos.CENTER? 25500 : 31200; // was 23707, 27016, 31688
        float tapeElevation = markerPos == MarkerPos.LEFT? 0.64f : markerPos == MarkerPos.CENTER? 0.6f : 0.594f; // was 0.64f, 0.6f, 0.594f

        pickUpTSEMulti(rotationTicks,extensionTicks,tapeElevation);
        bot.closeCapHolderServo();


        bot.setArmAngleTicks(armAngleTicks);
        bot.setIntakeFlipper(flipperPosition);

        driveToPosition(12,x1,y1,180,1);

        turnToHeading(headingAngleDegrees, 3, 8, 60);

        bot.setIntakePower(-0.2);
        sleep(1000);
        driveToPosition(SLOW, -24, bot.getPose().y, headingAngleDegrees, 1);
        turnToHeading(-90, 3, 8, 60);
        driveToPosition(SLOW, -24f, 75, -90, 1);
        bot.setArmAngleTicks(200);
        bot.setIntakeFlipper(FLIPPER_BARRIER);
        sleep(750);
        bot.setIntakePower(0);

        driveToPosition(FAST, -24f, 9, -90, 1);

    }
}
