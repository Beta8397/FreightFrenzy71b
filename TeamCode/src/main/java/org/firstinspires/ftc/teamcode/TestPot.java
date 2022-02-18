package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@Disabled
@Autonomous (name = "Test Pot", group = "Test")
public class TestPot extends OpMode {

    double power = 0;
    double targetVoltage = 1.5;
    DcMotor motor;
    AnalogInput pot;
    final double COEFF = -0.6;

    public void init() {
        pot = hardwareMap.get(AnalogInput.class, "pot");
        motor = hardwareMap.get(DcMotor.class, "potMotor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("max voltage", pot.getMaxVoltage());
        telemetry.update();
    }
     public void loop() {
        if (gamepad1.y) {
            targetVoltage = targetVoltage + 0.01;
        }else if (gamepad1.a){
            targetVoltage = targetVoltage - 0.01;
        }
        targetVoltage = Range.clip(targetVoltage, 0.05, 3.25);
        double volts;
        volts = pot.getVoltage();
        double error = targetVoltage - volts;
        if (Math.abs(error) < 0.05) {
            power = 0;
        }else {
            power = COEFF * error;
            if (Math.abs(power) < 0.05) {
                power = 0.05 * Math.signum(power);
            }
        }
        motor.setPower(power);
        telemetry.addData("power", power);
        telemetry.addData("voltage", volts);
        telemetry.addData("targetVoltage", targetVoltage);
    }
}
