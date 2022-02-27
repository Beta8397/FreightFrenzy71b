package org.firstinspires.ftc.teamcode.freightbot_old.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cv.VuforiaNavigator;
import org.firstinspires.ftc.teamcode.freightbot_old.FreightBot_Old;
import org.firstinspires.ftc.teamcode.freightbot_old.FreightBotAutonomous_Old;
import org.firstinspires.ftc.teamcode.freightbot_old.FreightBotTeleOp_Old;

@Disabled
@Autonomous(name = "RedLeftAutoWarehouseRemote", group = "redAuto")
public class RedLeftAutoWarehouseRemote_Old extends FreightBotAutonomous_Old {

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
            x1 = 17.25f; //was 17.25
            y1 = 109.25f; //was 109.25
        } else if (markerPos == MarkerPos.CENTER) {
            armAngleTicks = 420;
            x1 = 20f; //was 17.5
            y1 = 109.5f; //was 108
        } else {
            armAngleTicks = 200;
            x1 = 22; //was 21
            y1 = 105.8f; //was 103
        }

        bot.setArmExtensionTicks(450);

//        extendAndAngleArm(armAngleTicks, 150, markerPos);

        driveToPosition(SLOW, x1, y1, 180, 1);
        rotateTapeAndAngleArm(armAngleTicks, markerPos);
        turnToHeading(135, 3, 8, 60);

        deliverShippingHub();

        if (markerPos != MarkerPos.RIGHT) {
            driveToPosition(SLOW, bot.getPose().x-4, bot.getPose().y, 135, 1);
            turnToHeading(0, 3, 8, 80);
            driveToPosition(FAST, bot.getPose().x, 54, 0, 1);
            bot.setPose(bot.getPose().x-2, 57);
        } else {
            turnToHeading(-90, 3, 8, 80);
            driveToPosition(FAST, bot.getPose().x, 71, -90, 1);

        }


        float x2;
        float y2;
        if (markerPos == MarkerPos.LEFT) {
            x2 = bot.getPose().x - 5;
            y2 = 68.5f;
        } else if (markerPos == MarkerPos.CENTER) {
            x2 = bot.getPose().x - 5;
            y2 = 58.5f;
        } else {
            x2 = bot.getPose().x;
            y2 = 68.5f;
        }

        bot.setArmExtensionTicks(150);
        bot.setIntakeExtensionTicks(FreightBot_Old.INTAKE_EXTENSION_MIN_TICKS);
        bot.setIntakeFlipper(FreightBot_Old.INTAKE_FLIPPER_OUT);
        bot.setIntakeWheelState(FreightBot_Old.IntakeWheelState.FORWARD);
        bot.armCapServo.setPosition(.25f);

        if (markerPos != MarkerPos.RIGHT)
            driveToPosition(SLOW, x2, y2, 0, 1);

        float x3;
        float y3;
        if (markerPos == MarkerPos.CENTER) {
            x3 = 25;
            y3 = bot.getPose().y;
        } else if (markerPos == MarkerPos.LEFT) {
            x3 = 22;
            y3 = bot.getPose().y;
        } else {
            x3 = 33f; //33.75
            y3 = bot.getPose().y;
        }

        if (markerPos == MarkerPos.LEFT || markerPos == MarkerPos.CENTER) {
            driveToPosition(SUPER_SLOW, x3, y3, 0, 1);
        } else {
            driveToPosition(SLOW, x3, y3, -90, 1);
//            driveToPosition(SUPER_SLOW, 33, 51, -90,1);
            bot.setDriveSpeed(0,15,0);
            sleep(750);
            bot.setDriveSpeed(0,0,0);
            sleep(300);
            bot.setPose(32,64,(float)Math.toDegrees(bot.getHeadingRadians()));
            driveToPosition(FAST, bot.getPose().x, bot.getPose().y+1, (float)Math.toDegrees(bot.getHeadingRadians()), 1);
            sleep(200);
        }

        bot.setIntakeWheelState(FreightBot_Old.IntakeWheelState.STOPPED);
        bot.setIntakeState(FreightBot_Old.IntakeState.IN_MID);
        bot.setIntakeExtensionTicks(FreightBotTeleOp_Old.MIN_ARM_EXTENSION_TICKS);
        sleep(250);
        bot.setArmExtensionTicks(0);
        sleep(250);
        bot.setIntakePower(-.4f);
        sleep(1250);

        //second deliver to shipping hub drive
        if (markerPos == MarkerPos.LEFT) {
            driveToPosition(SLOW, 22.5f, 61f, (float)Math.toDegrees(bot.getPose().theta), 1);
        } else {
            driveToPosition(SLOW, 24.5f, 66f, (float)Math.toDegrees(bot.getPose().theta), 1);
        }

        bot.setIntakeState(FreightBot_Old.IntakeState.CENTER_MID);
        bot.closeArmCapServo();

        if (markerPos == MarkerPos.LEFT) {
            bot.setArmExtensionTicks(450);
        } else {
            bot.setArmExtensionTicks(450);
        }

        bot.setArmAngleTicks(200, 0.7f);
        while (opModeIsActive() && bot.getActualArmExtensionTicks() < 215) {
            continue;
        }
        bot.setArmServoPosition(FreightBot_Old.DUMPER_EXTENDED);

        float duckDeliveryHeadingDegrees = markerPos != MarkerPos.RIGHT  ? -125 : -133;

        turnToHeading(duckDeliveryHeadingDegrees, 1, 10, 80);

        deliverShippingHub();

        turnToHeading(-90, 3, 8, 60);

        if (markerPos != MarkerPos.RIGHT) {
            driveToPosition(FAST, 11, 100, -90, 1);
            driveToPosition(FAST, 14, 132, -90, 1);
        } else {
            driveToPosition(FAST, 16, 90, -90, 1); // was 18, 100
            driveToPosition(FAST, 20, 133, -90, 1);
        }


        carouselDrive(Alliance.RED);
        bot.setSpeedSpinnerMotor(0);

        //bot.setIntakeState(FreightBot.IntakeState.IN_MID);

//        if (markerPos == MarkerPos.RIGHT){
//            bot.setPose(17, 135, (float)Math.toDegrees(bot.getHeadingRadians()));
//            CubicSpline2D spline = new CubicSpline2D(new float[] {17,135,25,90,28,12}, -90, -90);
//
//
//            driveSpline(SLOW, 0.6f, -90, 6, 6, spline);
//        } else {
//            driveToParkFromCarousel(Alliance.RED);
//        }

        float x4;
        float y4;
        float x5;
        float y5;
        if (markerPos == MarkerPos.RIGHT) {
            x4 = 23;
            y4 = 132;
            x5 = 25;
            y5 = 12;
        } else {
            x4 = 22;
            y4 = 132;
            x5 = 32;
            y5 = 12;
        }

        float driveToParkAngleDegrees = (float)Math.toDegrees(Math.atan2(y5 - y4,x5 - x4));
        bot.setPose(17,135, (float) Math.toDegrees(bot.getHeadingRadians()));
        driveToPosition(SLOW,x4,y4,(float)Math.toDegrees(bot.getPose().theta),1);
//        turnToHeading(-80.9f,3, 8, 80);
//        driveToPosition(FAST,28,82,-80.9f,1);
//        turnToHeading(-90,3, 8, 80);
//        driveToPosition(FAST,28,12,-90,1);
        turnToHeading(driveToParkAngleDegrees,3,8,80);
        driveToPosition(FAST,x5,y5,driveToParkAngleDegrees,1);
    }
}
