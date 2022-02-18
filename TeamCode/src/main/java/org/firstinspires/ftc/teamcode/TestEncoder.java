package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@TeleOp(name = "TestEncoder", group = "Test")
public class TestEncoder extends LinearOpMode {

    DcMotorEx motor;

    ButtonToggle toggleA1 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.a;
        }
    };

    public void runOpMode(){
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()){
            int ticks = motor.getCurrentPosition();
            telemetry.addData("Ticks ", ticks);
            telemetry.update();
            double power = -gamepad1.left_stick_y;
            motor.setPower(power);

        }
    }
}
