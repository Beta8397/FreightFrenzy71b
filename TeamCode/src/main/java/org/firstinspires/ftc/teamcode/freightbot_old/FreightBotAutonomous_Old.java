package org.firstinspires.ftc.teamcode.freightbot_old;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.cv.Blob;
import org.firstinspires.ftc.teamcode.cv.BlobHelper;
import org.firstinspires.ftc.teamcode.cv.HSV_Range;
import org.firstinspires.ftc.teamcode.mecbot.MecBotAutonomous;
import org.firstinspires.ftc.teamcode.util.AngleUtils;
import org.firstinspires.ftc.teamcode.util.CubicSpline2D;
import org.firstinspires.ftc.teamcode.util.MotionProfile;

import java.util.List;

public abstract class FreightBotAutonomous_Old extends MecBotAutonomous {

    public static final MotionProfile SUPER_SLOW = new MotionProfile(15, 20, 20);

    public static final MotionProfile SLOW = new MotionProfile(15, 45, 20);

    public static final MotionProfile FAST = new MotionProfile(15, 45, 40);

    private FreightBot_Old bot = null;

    public enum CameraStartPos {LEFT,RIGHT}
    public enum MarkerPos {LEFT,CENTER,RIGHT}
    public enum Alliance {RED,BLUE}

    public final static HSV_Range HSV_RANGE = new HSV_Range(60, 100, 0.5f, 1,
            0.25f, 1);

    public void setBot(FreightBot_Old freightBotOld) {
        bot = freightBotOld;
        super.setBot(freightBotOld);
    }

    public MarkerPos getMarkerPos(CameraStartPos cameraStartPos) {
        BlobHelper blobHelper = new BlobHelper(640, 480, 0, 0, 640,
                480, 2);

        boolean success = false;

        while (!success && opModeIsActive()) {
            success = blobHelper.updateImage();
        }

        if (!success) return MarkerPos.CENTER;

        List<Blob> blobs = blobHelper.getBlobs(HSV_RANGE,
                new org.firstinspires.ftc.robotcore.external.Predicate<Blob>() {
                    @Override
                    public boolean test(Blob blob) {
                        if (blob.getNumPts() > 500) return true;
                        else return false;
                    }
                });

        if (blobs == null) return MarkerPos.CENTER;

        if (blobs.size() == 0) {
            if (cameraStartPos == CameraStartPos.LEFT) {
                return MarkerPos.RIGHT;
            } else {
                return MarkerPos.LEFT;
            }
        }

        Blob biggestBlob = blobs.get(0);

        while (blobs.size() > 1) {
            if (blobs.get(1).getNumPts() > biggestBlob.getNumPts()) {
                biggestBlob = blobs.get(1);
                blobs.remove(0);
            } else {
                blobs.remove(1);
            }
        }
        if (cameraStartPos == CameraStartPos.LEFT) {
            if (biggestBlob.getAvgX() < 160) {
                return MarkerPos.LEFT;
            } else {
                return MarkerPos.CENTER;
            }
        } else {
            if (biggestBlob.getAvgX() < 160) {
                return MarkerPos.CENTER;
            } else {
                return MarkerPos.RIGHT;
            }
        }
    }

    public void carouselDrive(Alliance alliance) {
        bot.setSpeedSpinnerMotor(alliance == Alliance.RED? -0.3f : .3f);

        float x0 = bot.getPose().x;
        float y0 = bot.getPose().y;
        float headingRadians = alliance == Alliance.RED? -(float) Math.PI/2.0f: (float) Math.PI;

        while (opModeIsActive()) {
            bot.updateOdometry();
            if ((y0 - bot.getPose().y) > 1) break;
            VectorF velocity = new VectorF(0, -20);
            VectorF vRobot = fieldToBot(velocity, bot.getPose().theta);
            float angleOffSet = (float)AngleUtils.normalizeRadians(headingRadians - bot.getPose().theta);
            float va = 2 * HEADING_CORRECTION_FACTOR * angleOffSet;
            bot.setDriveSpeed(vRobot.get(0), vRobot.get(1), va);
        }
        ElapsedTime et = new ElapsedTime();
        x0 = bot.getPose().x;
        boolean againstCarousel = false;
        while (opModeIsActive()) {
            bot.updateOdometry();

            if (et.seconds() > 4) {
                againstCarousel = true;
            }

            if (et.seconds() > 6) break;

            VectorF vRobot;
            float va;
            if (!againstCarousel) {
                float vx = alliance == Alliance.RED? -10 : 10; // WAS -15 : 15
                float vy = 20f * (y0 - bot.getPose().y - 1.5f);
                VectorF velocity = new VectorF(vx, vy);
                vRobot = fieldToBot(velocity, bot.getPose().theta);
                float angleOffSet = (float)AngleUtils.normalizeRadians(headingRadians - bot.getPose().theta);
                va = 2 * HEADING_CORRECTION_FACTOR * angleOffSet;
            } else {
                float vx = alliance == Alliance.RED? -1 : 1;    // Was -2 and 2
                VectorF velocity = new VectorF(vx, 0);
                vRobot = fieldToBot(velocity, bot.getPose().theta);
                va = 0;
            }

            bot.setDriveSpeed(vRobot.get(0), vRobot.get(1), va);
        }
        bot.setDriveSpeed(0,0,0);
    }

    public void driveToParkFromCarousel(Alliance alliance) {
        if (alliance == Alliance.RED) {
            bot.setPose(17, 135, (float)Math.toDegrees(bot.getHeadingRadians()));
            CubicSpline2D spline = new CubicSpline2D(new float[] {17,135,25,90,39,12}, -90, -90);
            driveSpline(SLOW, 0.6f, -90, 6, 6, spline);
        } else {
            bot.setPose(-17, 135);
            driveToPosition(SLOW, -21, 129, (float)Math.toDegrees(bot.getPose().theta), 1);
            turnToHeading(-90, 3, 8, 60);
            CubicSpline2D spline = new CubicSpline2D(new float[] {bot.getPose().x,bot.getPose().y,-34,8}, -90, -90);
           // driveSpline(SLOW, .1f, 0.5f, -90, 6, 6, spline);
            driveSpline(SLOW, false, 1, spline);

        }
    }

    MotionProfile slowPark = new MotionProfile(15, 20, 5);

    public void driveToParkStorage (Alliance alliance) {
        if (alliance == Alliance.RED) {
            bot.setPose(17, 135);
            driveToPosition(slowPark, 36.8f, 130,
                    (float)Math.toDegrees(bot.getPose().theta), 1);
            turnToHeading(-90, 3, 8, 45);
            driveToPosition(slowPark, 36.8f, 136, -90, 1);

        } else {
            bot.setPose(-17, 135);
            driveToPosition(slowPark, -32f, 130,
                    (float)Math.toDegrees(bot.getPose().theta), 1);
            turnToHeading(-90, 3, 8, 90);
            driveToPosition(slowPark, -32f, 136, -90, 1);
        }
    }

    public void cycleShippingHub() {

    }

    public void deliverShippingHub() {
            bot.setArmExtensionTicks(950, 0.6f);
            sleep(350);
            bot.openArmCapServo();
            sleep(500);
            bot.setArmExtensionTicks(450, 0.6f);
            sleep(300);
            bot.setArmServoPosition(FreightBot_Old.DUMPER_RETRACTED);
            sleep(500);
            bot.setArmAngleTicks(0);
            sleep(600);
            bot.setArmExtensionTicks(0, 0.6f);
            bot.setIntakeState(FreightBot_Old.IntakeState.CENTER_MIN);
            bot.setIntakeWheelState(FreightBot_Old.IntakeWheelState.STOPPED);
    }

    public void extendAndAngleArm(int angleTicks, int extensionTicks, MarkerPos pos) {
        bot.setArmExtensionTicks(extensionTicks, 0.7f);
        sleep(100);
        bot.setArmAngleTicks(angleTicks, 0.7f);
        while (opModeIsActive() && bot.getActualArmExtensionTicks() < (extensionTicks - 50)) {
            continue;
        }
        bot.setArmServoPosition(pos == MarkerPos.LEFT? FreightBot_Old.DUMPER_MID: FreightBot_Old.DUMPER_EXTENDED);
    }

    public void rotateTapeAndAngleArm (int angleTicks, MarkerPos pos) {
        bot.setTapeRotationPower(1f);
        sleep(1000);
        bot.setTapeRotationPower(0);
        bot.setArmAngleTicks(angleTicks, 0.7f);
        while (opModeIsActive() && bot.getActualArmExtensionTicks() < 550) {
            continue;
        }
        bot.setArmServoPosition(pos == MarkerPos.LEFT? FreightBot_Old.DUMPER_MID: FreightBot_Old.DUMPER_EXTENDED);
    }
}
