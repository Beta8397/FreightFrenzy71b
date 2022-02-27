package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@TeleOp(name = "Test Servo", group = "test")
public class TestServo extends LinearOpMode {

    Servo grabber;
    ButtonToggle toggleA = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.a;
        }
    };

    ButtonToggle toggleB = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.b;
        }
    };

    public void runOpMode() {
        grabber = hardwareMap.get(Servo.class, "cap_holder_servo");
        waitForStart();
        float pos = 0;
        while (opModeIsActive()) {
            if (gamepad1.a) {
                pos = pos + 0.002f;
            }
            if (gamepad1.b) {
                pos = pos - 0.002f;
            }

            if (pos < 0) {
                pos = 0;
            } else if (pos > 1) {
                pos = 1;
            }

            grabber.setPosition(pos);
            telemetry.addData("SErrrrrrrvo position", pos);
            telemetry.update();
        }
    }

}
