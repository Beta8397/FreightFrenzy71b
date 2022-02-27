package org.firstinspires.ftc.teamcode.freightbot_old.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.freightbot_old.FreightBot_Old;
import org.firstinspires.ftc.teamcode.freightbot_old.FreightBotAutonomous_Old;

@Disabled
@Autonomous(name = "RedRightAutoCarousel", group = "redAuto")
public class RedRightAutoCarousel_Old extends FreightBotAutonomous_Old {

    FreightBot_Old bot = new FreightBot_Old();
    WebcamName webcam = null;

    @Override
    public void runLoggingOpMode() {
        bot.init(hardwareMap, true);
        super.setBot(bot);
        webcam = hardwareMap.get(WebcamName.class, "webcam");
        VuforiaNavigator.activate(null, webcam);
        bot.setPose(8, 66, 180);
        telemetry.addData("press start when ready", "");
        telemetry.update();
        waitForStart();
        MarkerPos markerPos = getMarkerPos(CameraStartPos.RIGHT);
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
            x1 = 19;
            y1 = 61.25f;
        } else if (markerPos == MarkerPos.CENTER) {
            armAngleTicks = 390;
            x1 = 19;
            y1 = 63;
        } else {
            armAngleTicks = 200;
            x1 = 21;
            y1 = 65;
        }

        extendAndAngleArm(armAngleTicks, 150, markerPos);

        driveToPosition(SLOW, x1, y1, 180, 1);
        turnToHeading(-135, 3, 8, 60);

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
