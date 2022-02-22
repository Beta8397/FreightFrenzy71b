package org.firstinspires.ftc.teamcode.freightbot_old.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.freightbot_old.FreightBot_Old;
import org.firstinspires.ftc.teamcode.freightbot_old.FreightBotAutonomous_Old;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@Autonomous(name = "BlueRightAutoStorageDelay", group = "blueAuto")
public class BlueRightAutoStorageDelay extends FreightBotAutonomous_Old {

    FreightBot_Old bot = new FreightBot_Old();
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
        bot.setPose(-8, 104, 0);

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
            x1 = -20.5f; //was -18
            y1 = 105f; //was 107.3
        } else if (markerPos == MarkerPos.CENTER) {
            armAngleTicks = 390;  //was 410
            x1 = -18;
            y1 = 105.8f;
        } else {
            armAngleTicks = 200;
            x1 = -22; //was -23
            y1 = 104f; // was 105
        }
        bot.setArmExtensionTicks(600);

        //extendAndAngleArm(armAngleTicks, 450, markerPos);
        if (markerPos == MarkerPos.LEFT) {
            driveToPosition(8, x1, y1, 0, 0.5f);
        } else {
            driveToPosition(SLOW, x1, y1, 0, 1);
        }

        bot.setIntakeState(FreightBot_Old.IntakeState.CENTER_MID);
        rotateTapeAndAngleArm(armAngleTicks, markerPos);
        turnToHeading(45, 3, 8, 60);


        deliverShippingHub();

        driveToPosition(SLOW, bot.getPose().x+2, bot.getPose().y+2,
                (float)Math.toDegrees(bot.getHeadingRadians()), 1);        turnToHeading(45, 3, 8, 60);

        turnToHeading(180, 3, 8, 90);

        driveToPosition(SLOW, -20,bot.getPose().y,180,1);

        driveToPosition(SLOW, bot.getPose().x,136,180,1);


        carouselDrive(Alliance.BLUE);
        bot.setSpeedSpinnerMotor(0);

        driveToParkStorage(Alliance.BLUE);

    }
}
