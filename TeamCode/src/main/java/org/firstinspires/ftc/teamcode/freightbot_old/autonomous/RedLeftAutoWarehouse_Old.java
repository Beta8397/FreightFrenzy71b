package org.firstinspires.ftc.teamcode.freightbot_old.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.freightbot_old.FreightBot_Old;
import org.firstinspires.ftc.teamcode.freightbot_old.FreightBotAutonomous_Old;


public class RedLeftAutoWarehouse_Old extends FreightBotAutonomous_Old {

    FreightBot_Old bot = new FreightBot_Old();
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
            x1 = 18; //was 19
            y1 = 108.95f; //was 106.75
        } else if (markerPos == MarkerPos.CENTER) {
            armAngleTicks = 400;
            x1 = 17.5f; //was 19
            y1 = 108f; //was 105
        } else {
            armAngleTicks = 200;
            x1 = 21; //was 21
            y1 = 104.8f; //was 103
        }

        bot.setArmExtensionTicks(450);

//        extendAndAngleArm(armAngleTicks, 150, markerPos);

        driveToPosition(SLOW, x1, y1, 180, 1);
        rotateTapeAndAngleArm(armAngleTicks, markerPos);
        turnToHeading(135, 3, 8, 60);

//        bot.setArmExtensionTicks(950);
//        sleep(250);
//        bot.setArmServoPosition(FreightBot.DUMPER_EXTENDED);
//        sleep(1750);
//        bot.setArmExtensionTicks(0);
//        sleep(100);
//        bot.setArmServoPosition(FreightBot.DUMPER_RETRACTED);
//        sleep(400);
//        bot.setArmAngleTicks(0);

        deliverShippingHub();

        turnToHeading(-90, 3, 8, 60);
        driveToPosition(SLOW, 20, 136, -90, 1);

        carouselDrive(Alliance.RED);
        bot.setSpeedSpinnerMotor(0);

        driveToParkFromCarousel(Alliance.RED);

    }
}
