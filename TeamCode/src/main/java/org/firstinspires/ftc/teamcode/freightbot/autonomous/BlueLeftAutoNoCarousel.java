package org.firstinspires.ftc.teamcode.freightbot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.freightbot.FreightBot;
import org.firstinspires.ftc.teamcode.freightbot.FreightBotAutonomous;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@Autonomous(name = "BlueLeftAutoNoCarousel", group = "blueAuto")
public class BlueLeftAutoNoCarousel extends FreightBotAutonomous {

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
        bot.setPose(-8, 59, 0);
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
        if (markerPos == MarkerPos.LEFT) {
            armAngleTicks = 610;
            x1 = -15.75f; //was -19
            y1 = 68.75f; //was 61.25f
        } else if (markerPos == MarkerPos.CENTER) {
            armAngleTicks = 390;  //was 400
            x1 = -17.5f; //was -19
            y1 = 61.5f; // was 63
        } else {
            armAngleTicks = 200;
            x1 = -21;
            y1 = 65;
        }

        bot.setArmExtensionTicks(600);

        if (markerPos == MarkerPos.LEFT) {
            driveToPosition(8, x1, y1, 0, 1);
        } else {
            driveToPosition(SLOW, x1, y1, 0, 1);
        }

        bot.setIntakeState(FreightBot.IntakeState.CENTER_MID);
        rotateTapeAndAngleArm(armAngleTicks, markerPos);
        turnToHeading(markerPos == MarkerPos.LEFT? -30 : -45, 3, 8, 60);


        deliverShippingHub();

        driveToPosition(SLOW, -21, bot.getPose().y, -45, 1);
        turnToHeading(-90, 3, 8, 60);
        driveToPosition(SLOW,-24,bot.getPose().y,-90,1);
//        driveToPosition(SLOW,-24f,bot.getPose().y,-90,1);
        driveToPosition(SLOW, -24f, 75, -90, 1);
        driveToPosition(FAST, -24f, 9, -90, 1);

    }
}
