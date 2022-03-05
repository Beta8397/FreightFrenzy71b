package org.firstinspires.ftc.teamcode.freightbot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.freightbot.FreightBot;
import org.firstinspires.ftc.teamcode.freightbot.FreightBotAutonomous;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@Disabled
@Autonomous(name = "RedRightAutoWarehouse", group = "redAuto")
public class RedRightAutoWarehouse extends FreightBotAutonomous {

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
            armAngleTicks = ARM_BOTTOM;
            flipperPosition = FLIPPER_BOTTOM;
            x1 = 30f; //was -18
            y1 = 73.5f; //was 107.3
        } else if (markerPos == MarkerPos.CENTER) {
            armAngleTicks = ARM_MID;  //was 410
            flipperPosition = FLIPPER_MID;
            x1 = 29f;
            y1 = 72.5f;
        } else {
            armAngleTicks = ARM_TOP;
            flipperPosition = FLIPPER_TOP;
            x1 = 30.5f; //was -23
            y1 = 74f; // was 105
        }

        bot.setArmAngleTicks(armAngleTicks);
        bot.setIntakeFlipper(flipperPosition);

        driveToPosition(12,x1,y1,0,1);

        turnToHeading(45, 3, 8, 60);


        bot.setIntakePower(-0.2);
        sleep(1000);
        driveToPosition(SLOW, 24, bot.getPose().y, 45, 1);
        turnToHeading(-90, 3, 8, 60);
        driveToPosition(SLOW, 24f, 75, -90, 1);
        bot.setIntakePower(0);
        bot.setArmAngleTicks(200);
        bot.setIntakeFlipper(FLIPPER_BARRIER);
        sleep(500);
        bot.setSpeedSpinnerMotor(0);


        driveToPosition(FAST, 24f, 9, -90, 1);


    }
}
