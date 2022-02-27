package org.firstinspires.ftc.teamcode.freightbot_old.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.freightbot_old.FreightBot_Old;
import org.firstinspires.ftc.teamcode.freightbot_old.FreightBotAutonomous_Old;


public class BlueRightAutoStorage_Old extends FreightBotAutonomous_Old {

    FreightBot_Old bot = new FreightBot_Old();
    WebcamName webcam = null;

    @Override
    public void runLoggingOpMode() {
        bot.init(hardwareMap, true);
        super.setBot(bot);
        webcam = hardwareMap.get(WebcamName.class, "webcam");
        VuforiaNavigator.activate(null, webcam);
        bot.setPose(-8, 104, 0);
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
            x1 = -20.5f; //was -18
            y1 = 105f; //was 107.3
        } else if (markerPos == MarkerPos.CENTER) {
            armAngleTicks = 410;
            x1 = -18;
            y1 = 105.8f;
        } else {
            armAngleTicks = 200;
            x1 = -23;
            y1 = 105f;
        }
        bot.setArmExtensionTicks(600);

        //extendAndAngleArm(armAngleTicks, 450, markerPos);
        if (markerPos == MarkerPos.LEFT) {
            driveToPosition(8, x1, y1, 0, 0.5f);
        } else {
            driveToPosition(SLOW, x1, y1, 0, 1);
        }
        rotateTapeAndAngleArm(armAngleTicks, markerPos);
        turnToHeading(45, 3, 8, 60);

//        bot.setArmExtensionTicks(950);
//        sleep(250);
//        bot.setArmServoPosition(FreightBot.DUMPER_EXTENDED);
//        sleep(1750);
//        bot.setArmExtensionTicks(0);
//        sleep(300);
//        bot.setArmServoPosition(FreightBot.DUMPER_RETRACTED);
//        bot.setArmAngleTicks(0);

        deliverShippingHub();

        driveToPosition(SLOW, bot.getPose().x+2, bot.getPose().y+2,
                (float)Math.toDegrees(bot.getHeadingRadians()), 1);        turnToHeading(45, 3, 8, 60);

        turnToHeading(180, 3, 8, 60);

        driveToPosition(SLOW, -20,bot.getPose().y,180,1);
//        driveToPosition(SLOW, -20, 136, 180, 1);
//        int bl0 = bot.getBackLeft().getCurrentPosition();
//        int fl0 = bot.getFrontLeft().getCurrentPosition();
//        int fr0 = bot.getFrontRight().getCurrentPosition();
//        int br0 = bot.getBackRight().getCurrentPosition();
//        telemetry.addData("ticks 0", "bl %d  fl %d  fr %d  br %d", bl0,fl0,fr0,br0);

//        bot.setLoggingEnabled(true);
//        setLoggingEnabled(true);

        driveToPosition(SLOW, bot.getPose().x,136,180,1);
//        int bl1 = bot.getBackLeft().getCurrentPosition();
//        int fl1 = bot.getFrontLeft().getCurrentPosition();
//        int fr1 = bot.getFrontRight().getCurrentPosition();
//        int br1 = bot.getBackRight().getCurrentPosition();

//        telemetry.addData("ticks 1", "bl %d  fl %d  fr %d  br %d", bl1,fl1,fr1,br1);
//        telemetry.addData("dif", "bl %d  fl %d  fr %d  br %d", bl1 - bl0,
//                fl1 - fl0,fr1 - fr0,br1 - br0);
//        telemetry.update();

//        setLoggingEnabled(false);
//        bot.setLoggingEnabled(false);

//        while (opModeIsActive()) {
//            continue;
//        }


        carouselDrive(Alliance.BLUE);
        bot.setSpeedSpinnerMotor(0);

        driveToParkStorage(Alliance.BLUE);

    }
}
