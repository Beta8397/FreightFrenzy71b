package org.firstinspires.ftc.teamcode.freightbot_old;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.mecbot.MecBot;
import org.firstinspires.ftc.teamcode.mecbot.MecBotTeleOp;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@TeleOp (name = "FreightBotTeleOp OLD", group = "FreightBot OLD")
public class FreightBotTeleOp_Old extends MecBotTeleOp {

    public static final int MAX_ARM_EXTENSION_TICKS = -1000;
    public static final int MIN_ARM_EXTENSION_TICKS = 50;
    static final float SPINNER_SPEED = 1f;
    private boolean reversed = false;
    private float tapeElevationPosition = 0.5f * (FreightBot_Old.TAPE_ELEVATION_MAX + FreightBot_Old.TAPE_ELEVATION_MIN);
    private float armServoPosition = 0.05f;
    private boolean dumperExtended = false;
    private boolean armCapOpen = true;



    FreightBot_Old bot = new FreightBot_Old(MecBot.MotorType.NeverestOrbital20, 13, 10, 6, 45, 1,
            BNO055Enhanced.AxesMap.XYZ, BNO055Enhanced.AxesSign.PPP);

    private ButtonToggle toggle2Y = new ButtonToggle(ButtonToggle.Mode.RELEASED) {
        protected boolean getButtonState() {
            return gamepad2.y;
        }
    };
    private ButtonToggle toggle2X = new ButtonToggle(ButtonToggle.Mode.RELEASED) {
        protected boolean getButtonState() {
            return gamepad2.x;
        }
    };
    private ButtonToggle toggle2B = new ButtonToggle(ButtonToggle.Mode.RELEASED) {
        protected boolean getButtonState() {
            return gamepad2.b;
        }
    };
    private ButtonToggle toggle2A = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        protected boolean getButtonState() { return gamepad2.a;}
    };
    private ButtonToggle toggle2RightBumper = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        protected boolean getButtonState() { return gamepad2.right_bumper;}
    };
    private ButtonToggle toggle2LeftBumper = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        protected boolean getButtonState() { return gamepad2.left_bumper;}
    };
    private ButtonToggle toggle1RightBumper = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        protected boolean getButtonState() { return gamepad1.right_bumper;}
    };
    private ButtonToggle toggle1LeftBumper = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        protected boolean getButtonState() { return gamepad1.left_bumper;}
    };
    private ButtonToggle toggle1A = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        protected boolean getButtonState() {
            return gamepad1.a;
        }
    };
    private ButtonToggle toggle1B = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        protected boolean getButtonState() {
            return gamepad1.b;
        }
    };
    private ButtonToggle toggle2Start = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        protected boolean getButtonState() {
            return gamepad2.start;
        }
    };
    private ButtonToggle toggle1Start = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        protected boolean getButtonState() {
            return gamepad1.start;
        }
    };
    private ButtonToggle toggle1Back = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        protected boolean getButtonState() {
            return gamepad1.back;
        }
    };

    @Override
    public void runLoggingOpMode() {
        bot.init(hardwareMap, true);
        super.setup(bot);
        bot.setArmServoPosition(FreightBot_Old.DUMPER_RETRACTED);
        bot.setIntakeFlipper(FreightBot_Old.INTAKE_FLIPPER_CENTER);
        bot.openArmCapServo();
        bot.setTapeElevation(tapeElevationPosition);

        // TODO: Get rid of this next statement!
        bot.rightArmAngleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.leftArmAngleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.armExtenderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()) {
            doDriveControl();
            int actualArmExtensionTicks = bot.getActualArmExtensionTicks();
            int rightArmAngleMotorPosition = bot.rightArmAngleMotor.getCurrentPosition();
            int leftArmAngleMotorPosition = bot.leftArmAngleMotor.getCurrentPosition();

            if (gamepad1.start) {
                bot.setSpeedSpinnerMotor(SPINNER_SPEED);
            } else if (gamepad1.back){
                bot.setSpeedSpinnerMotor(-SPINNER_SPEED);
            } else {
                bot.setSpeedSpinnerMotor(0);
            }

            boolean toggle2APressed = toggle2A.update();

            if (toggle2APressed) {
               dumperExtended = !dumperExtended;
               armServoPosition = dumperExtended ? 0.98f - 0.000395f * rightArmAngleMotorPosition :
                       FreightBot_Old.DUMPER_RETRACTED;
               bot.setArmServoPosition(armServoPosition);
            } else if (gamepad2.back) {
                dumperExtended = false;
                armServoPosition = FreightBot_Old.DUMPER_RETRACTED;
                bot.setArmServoPosition(armServoPosition);
            }

            if (toggle2Start.update()) {
                bot.adjustArmExtensionTickOffset(MIN_ARM_EXTENSION_TICKS);
            }

            // TODO: Fix arm extension control!!

            bot.armExtenderMotor.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

//            int armExtensionPower;
//            if (gamepad2.back) {
//                armExtensionPower = rightArmAngleMotorPosition < 100 ? -130 : 0;
//                if(!armCapOpen) {
//                    armCapOpen = true;
//                    bot.openArmCapServo();
//                }
//            } else {
//                armExtensionPower = (int)(140 * (gamepad2.right_trigger - gamepad2.left_trigger));
//            }
//
//
//            if (armExtensionPower != 0) {
//                int newArmPosition = armExtensionPower + actualArmExtensionTicks;
//
//                if (!gamepad2.dpad_left) {
//                    newArmPosition = Range.clip(newArmPosition, MIN_ARM_EXTENSION_TICKS, MAX_ARM_EXTENSION_TICKS);
//                }
//
//                bot.setArmExtensionTicks(newArmPosition);
//            }

            if (gamepad2.y) {
                bot.closeArmCapServo();
                armCapOpen = false;
                bot.setArmAngleTicks(0);
                bot.setArmExtensionTicks(800);
                if (actualArmExtensionTicks > 200) {
                    armServoPosition = 0.98f - 0.000395f * rightArmAngleMotorPosition;
                    bot.setArmServoPosition(armServoPosition);
                }
            }

            if (gamepad2.right_stick_y < -0.5) {
                bot.closeArmCapServo();
                armCapOpen = false;
                bot.setArmExtensionTicks(250);
                if (actualArmExtensionTicks > 100) {
                    bot.setArmAngleTicks(FreightBot_Old.MAX_ARM_ANGLE_TICKS);
                    armServoPosition = 0.98f - 0.000395f * rightArmAngleMotorPosition;
                    bot.setArmServoPosition(armServoPosition);
                }
            }

            if (toggle2X.update()){
                if (armCapOpen){
                    bot.closeArmCapServo();
                } else {
                    bot.openArmCapServo();
                }
                armCapOpen = !armCapOpen;
            }

            // TODO: Fix the arm angle control!!


//            int armAnglePower = 0;
//
//            if (gamepad2.left_stick_y < - 0.5f) {
//                armAnglePower = 60;
//            } else if (gamepad2.left_stick_y > 0.5f) {
//                armAnglePower = -60;
//            } else if (gamepad2.back) {
//                armAnglePower = -100;
//            }
//
//            if (armAnglePower != 0) {
//                int newArmAnglePosition = armAnglePower + rightArmAngleMotorPosition;
//                newArmAnglePosition = Range.clip(newArmAnglePosition, 0, FreightBot.MAX_ARM_ANGLE_TICKS);
//
//                bot.setArmAngleTicks(newArmAnglePosition);
//            }

            float gp2LeftY = gamepad2.left_stick_y;
            bot.rightArmAngleMotor.setPower(gp2LeftY);
            bot.leftArmAngleMotor.setPower(gp2LeftY);



            boolean toggle1BPressed = toggle1B.update();
            boolean toggle1APressed = toggle1A.update();
            boolean toggleRight1BumperPressed = toggle1RightBumper.update();
            boolean toggleLeft1BumperPressed = toggle1LeftBumper.update();
            boolean toggleLeft2BumperPressed = toggle2LeftBumper.update() && gamepad2.start;
            boolean toggleBothLeftBumper = toggleLeft1BumperPressed || (toggleLeft2BumperPressed);

            if (toggle1APressed) {
                bot.setIntakeState(FreightBot_Old.IntakeState.CENTER_MID);
            }

            switch (bot.getIntakeState()) {
                case IN_MID:
                case CENTER_MIN:
                    if (toggle1BPressed) {
                        bot.setIntakeState(FreightBot_Old.IntakeState.OUT_MAX);
                    } else if (toggle1APressed) {
                        bot.setIntakeState(FreightBot_Old.IntakeState.CENTER_MID);
                    }
                    break;
                case OUT_MAX:
                    if (toggle1BPressed) {
                        bot.setIntakeState(FreightBot_Old.IntakeState.IN_MID);
                    } else if (toggle1APressed) {
                        bot.setIntakeState(FreightBot_Old.IntakeState.CENTER_MID);
                    }
                    break;
                case CENTER_MID:
                    if (toggle1BPressed) {
                        bot.setIntakeState(FreightBot_Old.IntakeState.OUT_MAX);
                    } else if (toggle1APressed) {
                        bot.setIntakeState(FreightBot_Old.IntakeState.CENTER_MIN);
                    }
                    break;
            }

            switch (bot.getIntakeWheelState()) {
                case STOPPED:
                    if (toggleBothLeftBumper) {
                        bot.setIntakeWheelState(FreightBot_Old.IntakeWheelState.REVERSE);
                    } else if (toggleRight1BumperPressed) {
                        bot.setIntakeWheelState(FreightBot_Old.IntakeWheelState.FORWARD);
                    } else if (toggle1BPressed && bot.getIntakeState() == FreightBot_Old.IntakeState.OUT_MAX) {
                        bot.setIntakeWheelState(FreightBot_Old.IntakeWheelState.FORWARD);
                    }
                    break;
                case FORWARD:
                    if (toggleBothLeftBumper) {
                        bot.setIntakeWheelState(FreightBot_Old.IntakeWheelState.REVERSE);
                    } else if (toggleRight1BumperPressed) {
                        bot.setIntakeWheelState(FreightBot_Old.IntakeWheelState.STOPPED);
                    } else if (toggle1BPressed && bot.getIntakeState() == FreightBot_Old.IntakeState.IN_MID) {
                        bot.setIntakeWheelState(FreightBot_Old.IntakeWheelState.STOPPED);
                    }
                    break;
                case REVERSE:
                    if (toggleBothLeftBumper) {
                        bot.setIntakeWheelState(FreightBot_Old.IntakeWheelState.STOPPED);
                    } else if (toggleRight1BumperPressed) {
                        bot.setIntakeWheelState(FreightBot_Old.IntakeWheelState.FORWARD);
                    } else if (toggle1BPressed && bot.getIntakeState() == FreightBot_Old.IntakeState.OUT_MAX) {
                        bot.setIntakeWheelState(FreightBot_Old.IntakeWheelState.FORWARD);
                    } else if (toggle1BPressed && bot.getIntakeState() == FreightBot_Old.IntakeState.IN_MID) {
                        bot.setIntakeWheelState(FreightBot_Old.IntakeWheelState.STOPPED);
                    }
                    break;
            }

            if (gamepad2.left_bumper && !gamepad2.start) {
                bot.setTapeExtensionPower(-1);
            } else if (gamepad2.right_bumper && !gamepad2.start) {
                bot.setTapeExtensionPower(1);
            } else {
                bot.setTapeExtensionPower(0);
            }

//            bot.setTapeRotationPower(gamepad2.right_stick_x);

            float rightStick2Y = gamepad2.right_stick_y;

//            if (gamepad2.dpad_down) {
//                bot.setTapeElevationPower(.2f);
//            } else if (gamepad2.dpad_up) {
//                bot.setTapeElevationPower(-.2f);
//            } else {
//                bot.setTapeElevationPower(0);
//            }

            if (gamepad2.dpad_down){
                tapeElevationPosition += 0.003f;
                tapeElevationPosition = Range.clip(tapeElevationPosition, FreightBot_Old.TAPE_ELEVATION_MIN,
                        FreightBot_Old.TAPE_ELEVATION_MAX);
            } else if (gamepad2.dpad_up) {
                tapeElevationPosition -= 0.003f;
                tapeElevationPosition = Range.clip(tapeElevationPosition, FreightBot_Old.TAPE_ELEVATION_MIN,
                        FreightBot_Old.TAPE_ELEVATION_MAX);
            }
            bot.setTapeElevation(tapeElevationPosition);

            if (gamepad2.dpad_left) {
                bot.setTapeRotationPower(gamepad2.b? .4f: .15f);
            } else if (gamepad2.dpad_right) {
                bot.setTapeRotationPower(gamepad2.b? -.4f: -.15f);
            } else {
                bot.setTapeRotationPower(0f);
            }


            telemetry.addData("arm servo position", armServoPosition);
            telemetry.addData("arm extension", actualArmExtensionTicks);
            telemetry.addData("right arm angle", rightArmAngleMotorPosition);
            telemetry.addData("left arm angle", leftArmAngleMotorPosition);
            telemetry.addData("Intake State", bot.getIntakeState());
            telemetry.addData("Intake Wheel State", bot.getIntakeWheelState());
            telemetry.addData("Tape Elevation", tapeElevationPosition);
            telemetry.update();
        }
    }

    public void doDriveControl() {
        if (gamepad1.start) {
            bot.setDriveSpeed(0, -2, 0);
        } else {
            super.doDriveControl();
        }
    }

}
