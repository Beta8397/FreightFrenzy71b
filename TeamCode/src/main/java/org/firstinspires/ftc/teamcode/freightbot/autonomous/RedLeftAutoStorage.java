package org.firstinspires.ftc.teamcode.freightbot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.freightbot.FreightBot;
import org.firstinspires.ftc.teamcode.freightbot.FreightBotAutonomous;

@Disabled
@Autonomous(name = "RedLeftAutoStorage", group = "redAuto")
public class RedLeftAutoStorage extends FreightBotAutonomous {

    FreightBot bot = new FreightBot();
    WebcamName webcam = null;

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
        waitForStart();
        MarkerPos markerPos = getMarkerPos(CameraStartPos.RIGHT);

        int armAngleTicks;
        float x1;
        float y1;
        if (markerPos == MarkerPos.LEFT) {
            armAngleTicks = 610;
            x1 = 18.25f; //was 17.25
            y1 = 108.25f; //was 109.25
        } else if (markerPos == MarkerPos.CENTER) {
            armAngleTicks = 420;
            x1 = 20f; //was 19
            y1 = 109.5f; //was 108.5
        } else {
            armAngleTicks = 200;
            x1 = 22; //was 21
            y1 = 105.8f; //was 103
        }

        bot.setArmExtensionTicks(600);

        //extendAndAngleArm(armAngleTicks, 450, markerPos);

        driveToPosition(SLOW, x1, y1, 180, 1);
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
