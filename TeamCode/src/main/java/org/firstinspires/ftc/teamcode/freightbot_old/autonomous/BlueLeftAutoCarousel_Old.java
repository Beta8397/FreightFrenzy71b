package org.firstinspires.ftc.teamcode.freightbot_old.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.freightbot_old.FreightBot_Old;
import org.firstinspires.ftc.teamcode.freightbot_old.FreightBotAutonomous_Old;

@Disabled
@Autonomous(name = "BlueLeftAutoCarousel", group = "blueAuto")
public class BlueLeftAutoCarousel_Old extends FreightBotAutonomous_Old {

    FreightBot_Old bot = new FreightBot_Old();
    WebcamName webcam = null;

    @Override
    public void runLoggingOpMode() {
        bot.init(hardwareMap, true);
        super.setBot(bot);
        webcam = hardwareMap.get(WebcamName.class, "webcam");
        VuforiaNavigator.activate(null, webcam);
        bot.setPose(-8, 59, 0);
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
            x1 = -19;
            y1 = 61.25f;
        } else if (markerPos == MarkerPos.CENTER) {
            armAngleTicks = 400;
            x1 = -19;
            y1 = 63;
        } else {
            armAngleTicks = 200;
            x1 = -21;
            y1 = 65;
        }

        extendAndAngleArm(armAngleTicks, 150, markerPos);

        driveToPosition(SLOW, x1, y1, 0, 1);
        turnToHeading(-45, 3, 8, 60);

        deliverShippingHub();

        turnToHeading(180, 3, 8, 60);
        driveToPosition(SLOW, -20, 136, 180, 1);

        carouselDrive(Alliance.BLUE);
        bot.setSpeedSpinnerMotor(0);

        driveToParkFromCarousel(Alliance.BLUE);

    }
}
