package org.firstinspires.ftc.teamcode.freightbot;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.cv.Blob;
import org.firstinspires.ftc.teamcode.cv.BlobHelper;
import org.firstinspires.ftc.teamcode.cv.HSV_Range;
import org.firstinspires.ftc.teamcode.freightbot_old.FreightBot_Old;
import org.firstinspires.ftc.teamcode.mecbot.MecBotAutonomous;
import org.firstinspires.ftc.teamcode.util.AngleUtils;
import org.firstinspires.ftc.teamcode.util.CubicSpline2D;
import org.firstinspires.ftc.teamcode.util.MotionProfile;

import java.util.List;

public abstract class FreightBotAutonomous extends MecBotAutonomous {

    public static final MotionProfile SUPER_SLOW = new MotionProfile(15, 20, 20);

    public static final MotionProfile SLOW = new MotionProfile(15, 45, 20);

    public static final MotionProfile FAST = new MotionProfile(15, 45, 40);

    public static final float FLIPPER_TOP = 0.89f;
    public static final float FLIPPER_MID = 0.82f;
    public static final float FLIPPER_BOTTOM = 0.61f;
    public static final float FLIPPER_BARRIER = 0.62f;
    public static final int ARM_TOP = 510;
    public static final int ARM_MID = 270;
    public static final int ARM_BOTTOM = 70;


    private FreightBot bot = null;

    public enum CameraStartPos {LEFT,RIGHT}
    public enum MarkerPos {LEFT,CENTER,RIGHT}
    public enum Alliance {RED,BLUE}

    public final static HSV_Range HSV_RANGE = new HSV_Range(60, 100, 0.5f, 1,
            0.25f, 1);

    public void setBot(FreightBot freightBot) {
        bot = freightBot;
        super.setBot(freightBot);
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
        bot.setSpeedSpinnerMotor(alliance == Alliance.RED? -0.2f : .2f);

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

            if (et.seconds() > 8) break;

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
            driveToPosition(slowPark, -34f, 130,
                    (float)Math.toDegrees(bot.getPose().theta), 1);
            turnToHeading(-90, 3, 8, 90);
            driveToPosition(slowPark, -34f, 136, -90, 1);
        }
    }

    public void driveToParkWarehouse (Alliance alliance) {
        if (alliance == Alliance.RED) {
            bot.setPose(17,135);
            driveToPosition(SLOW,30,90,(float)Math.toDegrees(bot.getPose().theta),1);
            turnToHeading(-90,3,8,90);
            driveToPosition(FAST,34,9,-90,1);
        } else {
            bot.setPose(-17,135);
            driveToPosition(SLOW,-21,132,(float)Math.toDegrees(bot.getPose().theta),1);
            turnToHeading(-90,3,8,90);
            driveToPosition(FAST,-28,9,-90,1);
        }
    }

    public void pickUpTSE (int rotation, int extension, float elevation) {
        positionTape(rotation,bot.tapeExtensionEncoder.getCurrentPosition(),FreightBot.TAPE_ELEVATION_MAX, 1,1000);
        positionTape(rotation,extension,0.560f,1,2750); // was 2000
        positionTape(rotation,extension,elevation,1,1000);
        positionTape(rotation,extension,0.185f,1,1000);
        positionTape(-400,14000,0.185f,0.3f,3000);
        positionTape(-400,14000,0.497f,1,1000);
        positionTape(-400,0,0.497f,1,1000);
        positionTape(-400,0,FreightBot.TAPE_ELEVATION_MAX,1,1000);
        positionTape(0,0,FreightBot.TAPE_ELEVATION_MAX,0.5f,1000);

    }

    public void pickUpTSEMulti (int rotation, int extension, float elevation) {
        int dR = 70;
        float dEL = .05f;
        positionTape(rotation+dR,bot.tapeExtensionEncoder.getCurrentPosition(),FreightBot.TAPE_ELEVATION_MAX, 1,1000);
        positionTape(rotation+dR,extension,0.560f,1,2750); // was 2000

        positionTape(rotation + dR,extension,elevation,1,1000);
        positionTape(rotation + dR,extension,elevation - dEL,1,200);
        positionTape(rotation,extension,elevation - dEL,1,500);
        positionTape(rotation,extension,elevation,1,200);
        positionTape(rotation,extension,elevation - dEL,1,200);
        positionTape(rotation - dR,extension,elevation - dEL,1,500);
        positionTape(rotation - dR,extension,elevation,1,300);

        positionTape(rotation - dR,extension,0.185f,1,1000);
        positionTape(-450,13000,0.185f,0.3f,3000);
        positionTape(-450,13000,0.497f,1,1000);
        positionTape(-450,0,0.497f,1,1000);
        positionTape(-450,0,FreightBot.TAPE_ELEVATION_MAX,1,1000);
        positionTape(0,0,FreightBot.TAPE_ELEVATION_MAX,0.5f,1000);

    }

    public void positionTape (int rotation, int extension, float elevation, float maxRotationPower, double delay) {
        bot.setTapeElevation(elevation);
        ElapsedTime et = new ElapsedTime();

        while (opModeIsActive() && et.milliseconds() < delay) {
            bot.updateTapeRotation(rotation,maxRotationPower);
            bot.updateTapeExtension(extension);
        }
        bot.setTapeRotationPower(0);
        bot.setTapeExtensionPower(0);
    }


}
