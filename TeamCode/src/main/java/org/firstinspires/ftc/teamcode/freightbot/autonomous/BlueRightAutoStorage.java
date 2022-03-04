package org.firstinspires.ftc.teamcode.freightbot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.freightbot.FreightBot;
import org.firstinspires.ftc.teamcode.freightbot.FreightBotAutonomous;
import org.firstinspires.ftc.teamcode.freightbot_old.FreightBotAutonomous_Old;
import org.firstinspires.ftc.teamcode.freightbot_old.FreightBot_Old;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@Autonomous(name = "BlueRightAutoStorage", group = "blueAuto")
public class BlueRightAutoStorage extends FreightBotAutonomous {

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
        bot.setPose(-8, 103, 180);
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
            x1 = -32f; //was -18
            y1 = 96f; //was 107.3
        } else if (markerPos == MarkerPos.CENTER) {
            armAngleTicks = ARM_MID;  //was 410
            flipperPosition = FLIPPER_MID;
            x1 = -31.5f; // was -32
            y1 = 98.5f;  // was 98
        } else {
            armAngleTicks = ARM_TOP;
            flipperPosition = FLIPPER_TOP;
            x1 = -32f; //was -23
            y1 = 98f; // was 105
        }

        bot.setArmAngleTicks(armAngleTicks);
        bot.setIntakeFlipper(flipperPosition);

        driveToPosition(12,-27,103,180,1);

        turnToHeading(-135, 3, 8, 60);

        driveToPosition(12, x1, y1, -135, 0.5f);


        bot.setIntakePower(-0.2);
        sleep(1000);
        driveToPosition(12,-20f,111,-135,1);
        bot.setIntakePower(0);
        bot.setArmAngleTicks(0);
        bot.setIntakeFlipper(FLIPPER_BARRIER);

        turnToHeading(180, 3, 8, 90);

        driveToPosition(SLOW, bot.getPose().x,136,180,1);


        carouselDrive(Alliance.BLUE);
        bot.setSpeedSpinnerMotor(0);

        driveToParkStorage(Alliance.BLUE);

    }
}
