package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;

public class KalmanReturnData {
    public final float x;
    public final float y;
    public final MatrixF covariance;

    public KalmanReturnData(float xx, float yy, MatrixF cov) {
        x = xx;
        y = yy;
        covariance = cov;
    }
}
