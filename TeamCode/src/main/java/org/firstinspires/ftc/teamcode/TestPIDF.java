package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

/**
 * Class to test adjustment of PIDF parameters on a DcMotorEx.
 *
 * After pressing INIT, do the following:
 *
 * dpad_left to toggle motor on/off
 * dpad_up and dpad_down to set target motor speed (whether motor currently running or not)
 * dpad_right to switch between setting of P, I, D, or F coefficients
 * x, y, b: to change the most, mid, and least significant digits of P, I, D, or F
 * left_bumper and right_bumper to decrease/increase the 10's multiplier for P,I,D, or F
 * a: to save the new P, I, D, or F value
 *
 * START: to start recording motor speed versus time and saving data to file.
 */
@Disabled
@Autonomous(name = "TestPIDF", group = "Test")
public class TestPIDF extends LinearOpMode {

    private DcMotorEx motor = null;

    enum SetMode { P, I, D, F};

    SetMode setMode = SetMode.P;

    String path;

    BufferedWriter bufferedWriter = null;


    ButtonToggle toggleDpadRight1 = new ButtonToggle(ButtonToggle.Mode.PRESSED) { protected boolean getButtonState() {
            return gamepad1.dpad_right;
        }};
    ButtonToggle toggleX1 = new ButtonToggle(ButtonToggle.Mode.PRESSED) { protected boolean getButtonState() {
        return gamepad1.x;
    }};
    ButtonToggle toggleY1 = new ButtonToggle(ButtonToggle.Mode.PRESSED) { protected boolean getButtonState() {
        return gamepad1.y;
    }};
    ButtonToggle toggleB1 = new ButtonToggle(ButtonToggle.Mode.PRESSED) { protected boolean getButtonState() {
        return gamepad1.b;
    }};
    ButtonToggle toggleRightBumper1 = new ButtonToggle(ButtonToggle.Mode.PRESSED) { protected boolean getButtonState() {
        return gamepad1.right_bumper;
    }};
    ButtonToggle toggleLeftBumper1 = new ButtonToggle(ButtonToggle.Mode.PRESSED) { protected boolean getButtonState() {
        return gamepad1.left_bumper;
    }};
    ButtonToggle toggleA1 = new ButtonToggle(ButtonToggle.Mode.PRESSED) { protected boolean getButtonState() {
        return gamepad1.a;
    }};
    ButtonToggle toggleDpadLeft1 = new ButtonToggle(ButtonToggle.Mode.PRESSED) { protected boolean getButtonState() {
        return gamepad1.dpad_left;
    }};

    private boolean running = false;


    public void runOpMode(){

        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorConfigurationType motorConfigurationType = motor.getMotorType();
        double maxTicksPerSec = motorConfigurationType.getAchieveableMaxTicksPerSecond();

        PIDFCoefficients initialPIDF = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(initialPIDF.p, initialPIDF.i, initialPIDF.d, initialPIDF.f);

        int p, i, d, f;
        double pm, im, dm, fm;

        pm = pidf.p > 0.0000001? Math.pow(10, Math.floor(Math.log10(pidf.p))) : 0.1;
        im = pidf.i > 0.0000001? Math.pow(10, Math.floor(Math.log10(pidf.i))) : 0.1;
        dm = pidf.d > 0.0000001? Math.pow(10, Math.floor(Math.log10(pidf.d))) : 0.1;
        fm = pidf.f > 0.0000001? Math.pow(10, Math.floor(Math.log10(pidf.f))) : 0.1;

        p = (int)Math.floor(100.0 * pidf.p / pm);
        i = (int)Math.floor(100.0 * pidf.i / im);
        d = (int)Math.floor(100.0 * pidf.d / dm);
        f = (int)Math.floor(100.0 * pidf.f / fm);

        path = String.format("/sdcard/P%d_%dI%d_%dD%d_%dF%d_%d.csv",
                p, pidf.p > 0.0000001? (int)Math.floor(Math.log10(pidf.p)) : 0,
                i, pidf.i > 0.0000001? (int)Math.floor(Math.log10(pidf.i)) : 0,
                d, pidf.d > 0.0000001? (int)Math.floor(Math.log10(pidf.d)) : 0,
                f, pidf.f > 0.0000001? (int)Math.floor(Math.log10(pidf.f)) : 0);

        double targetSpeedFraction = 0;
        ElapsedTime setSpeedTimer = new ElapsedTime();

        while (!opModeIsActive() && !isStarted()){

            if (toggleDpadLeft1.update()){
                running = !running;
                if (!running) motor.setPower(0);
                else motor.setVelocity(targetSpeedFraction * maxTicksPerSec);
            }

            if (setSpeedTimer.milliseconds() > 50){
                setSpeedTimer.reset();
                if (gamepad1.dpad_up) {
                    targetSpeedFraction = Math.min(1.0, targetSpeedFraction + 0.01);
                    if (running) motor.setVelocity(targetSpeedFraction * maxTicksPerSec);
                } else if (gamepad1.dpad_down) {
                    targetSpeedFraction = Math.max(-1.0, targetSpeedFraction - 0.01);
                    if (running) motor.setVelocity(targetSpeedFraction * maxTicksPerSec);
                }
            }

            telemetry.addData("Target Speed", "Frac = %.4f  Abs = %.2f",
                    targetSpeedFraction, targetSpeedFraction * maxTicksPerSec);

            double actualSpeed = motor.getVelocity();
            telemetry.addData("Actual Speed", " %.3f", actualSpeed);

            if (toggleDpadRight1.update()) setMode = SetMode.values()[(setMode.ordinal()+1)%4];
            telemetry.addData("Set Mode ", setMode);

            int value = setMode == SetMode.P? p : setMode == SetMode.I? i : setMode == SetMode.D? d : f;
            double multiplier = setMode == SetMode.P? pm : setMode == SetMode.I? im : setMode == SetMode.D? dm : fm;
            int ones = value % 10;
            int tens = (value / 10) % 10;
            int hundreds = (value / 100) % 10;

            if (toggleX1.update()) hundreds = (hundreds + 1) % 10;
            if (toggleY1.update()) tens = (tens + 1) % 10;
            if (toggleB1.update()) ones = (ones + 1) % 10;
            if (toggleRightBumper1.update()) multiplier *= 10;
            if (toggleLeftBumper1.update()) multiplier /= 10;

            value = ones + 10 * tens + 100 * hundreds;

            if (setMode == SetMode.P){
                p = value;
                pm = multiplier;
            } else if (setMode == SetMode.I){
                i = value;
                im = multiplier;
            } else if (setMode == SetMode.D){
                d = value;
                dm = multiplier;
            } else {
                f = value;
                fm = multiplier;
            }

            PIDFCoefficients tempPIDFCoefficients = new PIDFCoefficients(p*pm/100.0, i*im/100.0, d*dm/100.0, f*fm/100.0);

            if (toggleA1.update()){
                pidf = tempPIDFCoefficients;
                motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
                path = String.format("/sdcard/P%d_%dI%d_%dD%d_%dF%d_%d.csv",
                        p, pidf.p > 0.0000001? (int)Math.floor(Math.log10(pidf.p)) : 0,
                        i, pidf.i > 0.0000001? (int)Math.floor(Math.log10(pidf.i)) : 0,
                        d, pidf.d > 0.0000001? (int)Math.floor(Math.log10(pidf.d)) : 0,
                        f, pidf.f > 0.0000001? (int)Math.floor(Math.log10(pidf.f)) : 0);
            }
            telemetry.addData("Path ", path);
            telemetry.addData("Temp Coeffs","'A' to set.");
            telemetry.addData("P value ", tempPIDFCoefficients.p);
            telemetry.addData("I value ", tempPIDFCoefficients.i);
            telemetry.addData("D value ", tempPIDFCoefficients.d);
            telemetry.addData("F value ", tempPIDFCoefficients.f);
            telemetry.update();

        }


        try {
            bufferedWriter = new BufferedWriter(new FileWriter(path, false));
        }
        catch(IOException e){
            closeWriter();
        }

        if (bufferedWriter == null) return;

        ElapsedTime writeTimer = new ElapsedTime();

        while (opModeIsActive()){
            if (toggleDpadLeft1.update()){
                running = !running;
                if (!running) motor.setPower(0);
                else motor.setVelocity(targetSpeedFraction * maxTicksPerSec);
            }

            try {
                bufferedWriter.write(String.format("%f,%f", writeTimer.milliseconds(), motor.getVelocity()));
                bufferedWriter.newLine();
            } catch (IOException e){
                break;
            }
        }

        closeWriter();

    }

    private void closeWriter(){
        if (bufferedWriter != null){
            try{
                bufferedWriter.close();
            } catch (IOException e) {
                return;
            } finally {
                bufferedWriter = null;
            }
        }
    }

}
