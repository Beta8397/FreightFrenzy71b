package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class KalmanDistanceUpdater implements KalmanMeasurementUpdater{

    public final float FIELD_LENGTH = 144;
    public final float FIELD_WIDTH = 144;
    public final float Y_BACK = 0;
    public final float Y_AUDIENCE = FIELD_LENGTH;
    public final float X_RED;
    public final float X_BLUE;
    public final float MAX_HEADING_OFFSET = (float)Math.toRadians(7.0f);
    public final float MAX_DIST_ERROR_FRACTION = 0.2f;
    public final float STD_DEV_FRAC = 0.03f;
    public final MatrixF IDENTITY = new GeneralMatrixF(2,2,
            new float[]{1,0,0,1});

    DistanceSensor[] sensors = null;
    float[] offSets = null;
    AllianceColor alliance;

    public KalmanDistanceUpdater(DistanceSensor[] sensors, float[] offSets, AllianceColor alliance) {
        this.sensors = sensors;
        this.offSets = offSets;
        X_RED = alliance == AllianceColor.RED? 0 : -FIELD_WIDTH;
        X_BLUE = alliance == AllianceColor.BLUE? 0 : FIELD_WIDTH;
        this.alliance = alliance;
    }

    public Pose kalmanMeasurementUpdate(Pose pose, MatrixF covMinus) {
        int headingIndex = (int) (pose.theta/(Math.PI/2));
        float headingOffSet = (float) Math.abs(AngleUtils.normalizeRadians(
                pose.theta - Math.PI*headingIndex/2));
        if (headingOffSet > MAX_HEADING_OFFSET) {
            return pose;
        }

        int blueIndex = (headingIndex + 4)%4;
        int backIndex = (headingIndex + 5)%4;
        int redIndex = (headingIndex + 2)%4;
        int audIndex = (headingIndex + 3)%4;
        float blueDist = (float) sensors[blueIndex].getDistance(DistanceUnit.INCH);
        float backDist = (float) sensors[backIndex].getDistance(DistanceUnit.INCH);
        float redDist = (float) sensors[redIndex].getDistance(DistanceUnit.INCH);
        float audDist = (float) sensors[audIndex].getDistance(DistanceUnit.INCH);

        boolean blueValid = true, backValid = true, redValid = true, audValid = true;
        boolean xValid = false;
        boolean yValid = false;
        float xMeas = 0;
        float yMeas = 0;
        float xVar = 0;
        float yVar = 0;

        if (blueDist < 2.5 || blueDist > 72) blueValid = false;
        if (backDist < 2.5 || backDist > 72) backValid = false;
        if (redDist < 2.5 || redDist > 72) redValid = false;
        if (audDist < 2.5 || audDist > 72) audValid = false;

        float blueEXP = X_BLUE - pose.x - offSets[blueIndex];
        float backEXP = pose.y - Y_BACK - offSets[backIndex];
        float redEXP = pose.x - X_RED - offSets[redIndex];
        float audEXP = Y_AUDIENCE - pose.y - offSets[audIndex];
        float blueError = (blueDist - blueEXP)/blueEXP;
        float backError = (backDist - backEXP)/backEXP;
        float redError = (redDist - redEXP)/redEXP;
        float audError = (audDist - audEXP)/audEXP;

        if (Math.abs(blueError) > MAX_DIST_ERROR_FRACTION) blueValid = false;
        if (Math.abs(backError) > MAX_DIST_ERROR_FRACTION) backValid = false;
        if (Math.abs(redError) > MAX_DIST_ERROR_FRACTION) redValid = false;
        if (Math.abs(audError) > MAX_DIST_ERROR_FRACTION) audValid = false;

        if (blueValid || redValid) {
            xValid = true;
            if (blueValid) {
                xMeas = X_BLUE - blueDist - offSets[blueIndex];
                xVar = STD_DEV_FRAC*STD_DEV_FRAC*blueDist*blueDist;
            }
            if (redValid && (!blueValid || redDist < blueDist)) {
                xMeas = redDist - X_RED - offSets[redIndex];
                xVar = STD_DEV_FRAC*STD_DEV_FRAC*redDist*redDist;
            }
        }

        if (backValid || audValid) {
            yValid = true;
            if (backValid) {
                yMeas = backDist - Y_BACK - offSets[backIndex];
                yVar = STD_DEV_FRAC*STD_DEV_FRAC*backDist*backDist;
            }
            if (audValid && (!backValid || audDist < backDist)) {
                yMeas = Y_AUDIENCE - audDist - offSets[audIndex];
                yVar = STD_DEV_FRAC*STD_DEV_FRAC*audDist*audDist;
            }
        }

    // TODO: do the Kalman math and supply a return value and modify covariance matrix.
        if (!xValid && !yValid) {
            return pose;
        }
        MatrixF K;
        MatrixF R;

        if (xValid && yValid) {
            R = new GeneralMatrixF(2,2,
                    new float[]{xVar,0,0,yVar});
            K = covMinus.multiplied(covMinus.added(R).inverted());
        } else if (xValid) {
            K = new GeneralMatrixF(2,2,
                    new float[]{
                            covMinus.get(0,0)/(covMinus.get(0,0) + xVar),0,
                            covMinus.get(1,0)/(covMinus.get(0,0) + xVar),0});
        } else {
            K = new GeneralMatrixF(2,2,
                    new float[]{
                            0,covMinus.get(0,1)/(covMinus.get(1,1) + yVar),
                            0,covMinus.get(1,1)/(covMinus.get(1,1) + yVar)});
        }

        VectorF xMinus = new VectorF(pose.x,pose.y);
        VectorF z = new VectorF(xMeas,yMeas);

        VectorF x = xMinus.added(K.multiplied(z.subtracted(xMinus)));

        MatrixF cov = IDENTITY.subtracted(K).multiplied(covMinus);

        covMinus.put(0,0,cov.get(0,0));
        covMinus.put(0,1,cov.get(0,1));
        covMinus.put(1,0,cov.get(1,0));
        covMinus.put(1,1,cov.get(1,1));

        return new Pose(x.get(0),x.get(1),pose.theta);
    }

}
