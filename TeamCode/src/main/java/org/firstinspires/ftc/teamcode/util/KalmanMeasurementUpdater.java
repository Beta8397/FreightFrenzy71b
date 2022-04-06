package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;

public interface KalmanMeasurementUpdater {
    KalmanReturnData kalmanMeasurementUpdate(float x, float y, MatrixF pMinus);
}

