package org.firstinspires.ftc.teamcode.freightbot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.freightbot_old.FreightBot_Old;
import org.firstinspires.ftc.teamcode.mecbot.MecBotTeleOp;
import org.firstinspires.ftc.teamcode.util.Updatable;
import org.firstinspires.ftc.teamcode.util.gamepad.ButtonToggle;

@TeleOp (name = "FreightBotTeleOp", group = "FreightBot")
public class FreightBotTeleOp extends MecBotTeleOp {

    private enum TapeOperationState {ROTATING, ELEVATING, EXTENDING, DESCENDING}

    static final float SPINNER_SPEED = 1f;
    static final float STD_FLIPPER_POS = 0.41f;
    private boolean reversed = false;
    private float intakeFlipperPosition = STD_FLIPPER_POS;
    private float tapeElevationPosition = FreightBot.TAPE_ELEVATION_MAX;
    private boolean altMode = false;
    private int tapeRotationTarget = 0;
    private int tapeExtensionTarget = 0;

    private Updatable autoTapeControl = null;


    FreightBot bot = new FreightBot();

    ButtonToggle toggleRbump1 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad1.right_bumper;
        }
    };
    ButtonToggle toggleStart2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad2.start;
        }
    };

    ButtonToggle toggleY2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() {
            return gamepad2.y;
        }
    };

    ButtonToggle toggleX2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() { return gamepad2.x; }
    };
    ButtonToggle toggleRightTrig2 = new ButtonToggle(ButtonToggle.Mode.PRESSED) {
        @Override
        protected boolean getButtonState() { return gamepad2.right_trigger > 0.25f; }
    };





    @Override
    public void runOpMode() {
        bot.init(hardwareMap);
        super.setup(bot);
        bot.setIntakeFlipper(STD_FLIPPER_POS);
        bot.setTapeElevation(tapeElevationPosition);
        bot.closeCapHolderServo();

        waitForStart();

        while(opModeIsActive()) {
            doDriveControl();
            int rightArmAngleMotorPosition = bot.rightArmAngleMotor.getCurrentPosition();
            int leftArmAngleMotorPosition = bot.leftArmAngleMotor.getCurrentPosition();

            if (gamepad1.start) {
                bot.setSpeedSpinnerMotor(SPINNER_SPEED);
            } else if (gamepad1.back){
                bot.setSpeedSpinnerMotor(-SPINNER_SPEED);
            } else {
                bot.setSpeedSpinnerMotor(0);
            }

            if (toggleRightTrig2.update()) {
                if (bot.capHolderServo.getPosition() < FreightBot.CAP_SERVO_MID) {
                    bot.closeCapHolderServo();
                } else {
                    bot.openCapHolderServo();
                }
            }

            if (toggleStart2.update()) {
                if (altMode) {
                    altMode = false;
                    bot.leftArmAngleMotor.setPower(0);
                    bot.rightArmAngleMotor.setPower(0);
                    bot.setArmAngleMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.setArmAngleMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else {
                    altMode = true;
                    bot.leftArmAngleMotor.setPower(0);
                    bot.rightArmAngleMotor.setPower(0);
                    bot.setArmAngleMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }

            if (altMode) {
                float pwr = -gamepad2.left_stick_y;
                bot.leftArmAngleMotor.setPower(pwr);
                bot.rightArmAngleMotor.setPower(pwr);
            } else {

                int armAnglePower = 0;

                armAnglePower = (int) (-120.0 * gamepad2.left_stick_y);


                if (armAnglePower != 0) {
                    int newArmAnglePosition = armAnglePower + rightArmAngleMotorPosition;
                    newArmAnglePosition = Range.clip(newArmAnglePosition, 0, FreightBot.MAX_ARM_ANGLE_TICKS);

                    bot.setArmAngleTicks(newArmAnglePosition);
                }
            }


            if (gamepad1.y){
                intakeFlipperPosition += 0.005;
                intakeFlipperPosition = Range.clip(intakeFlipperPosition, 0, 1);
            } else if (gamepad1.x){
                intakeFlipperPosition -= 0.005;
                intakeFlipperPosition = Range.clip(intakeFlipperPosition, 0, 1);
            } else if (gamepad1.a){
                intakeFlipperPosition = STD_FLIPPER_POS;
            }

            bot.intakeFlipper.setPosition(intakeFlipperPosition);

            boolean toggledBR1 = toggleRbump1.update();

            switch (bot.getIntakeWheelState()) {
                case REVERSE:
                    if (!gamepad1.left_bumper && !gamepad2.a) bot.setIntakeWheelState(FreightBot.IntakeWheelState.STOPPED);
                    break;
                case STOPPED:
                    if (gamepad1.left_bumper || gamepad2.a) bot.setIntakeWheelState(FreightBot.IntakeWheelState.REVERSE);
                    else if (toggledBR1) bot.setIntakeWheelState(FreightBot.IntakeWheelState.FORWARD);
                    break;
                case FORWARD:
                    if (gamepad1.left_bumper || gamepad2.a) bot.setIntakeWheelState(FreightBot.IntakeWheelState.REVERSE);
                    else if (toggledBR1) bot.setIntakeWheelState(FreightBot.IntakeWheelState.SLOW_FORWARD);
                    break;
                case SLOW_FORWARD:
                    if (gamepad1.b) bot.setIntakeWheelState(FreightBot.IntakeWheelState.STOPPED);
                    else if (toggledBR1) bot.setIntakeWheelState(FreightBot.IntakeWheelState.FORWARD);
                    else if (gamepad1.left_bumper || gamepad2.a) bot.setIntakeWheelState(FreightBot.IntakeWheelState.REVERSE);
            }


            if (bot.getIntakeWheelState() == FreightBot.IntakeWheelState.REVERSE){
                if (!gamepad1.left_bumper && !gamepad2.a) bot.setIntakeWheelState(FreightBot.IntakeWheelState.STOPPED);
            } else if (bot.getIntakeWheelState() == FreightBot.IntakeWheelState.STOPPED){
                if (gamepad1.left_bumper || gamepad2.a) bot.setIntakeWheelState(FreightBot.IntakeWheelState.REVERSE);
                else if (toggledBR1) bot.setIntakeWheelState(FreightBot.IntakeWheelState.FORWARD);
            } else {
                if (gamepad1.left_bumper || gamepad2.a) bot.setIntakeWheelState(FreightBot.IntakeWheelState.REVERSE);
                else if (toggledBR1) bot.setIntakeWheelState(FreightBot.IntakeWheelState.STOPPED);
            }

            boolean y2Toggled = toggleY2.update();
            boolean x2Toggled = toggleX2.update();

            if ((y2Toggled || x2Toggled) && autoTapeControl == null){
                if (y2Toggled){
                    autoTapeControl = new StoredTSEGrabber();
                } else {
                    autoTapeControl = new TSEStorer();
                }
            } else if (autoTapeControl != null && !gamepad2.y && !gamepad2.x){
                autoTapeControl = null;
                bot.setTapeRotationPower(0);
                bot.setTapeExtensionPower(0);
            }


            if (autoTapeControl != null) {
                autoTapeControl.update();
            } else {

                if (gamepad2.left_bumper) {
                    bot.setTapeExtensionPower(-1);
                } else if (gamepad2.right_bumper) {
                    bot.setTapeExtensionPower(1);
                } else {
                    bot.setTapeExtensionPower(0);
                }


                if (gamepad2.dpad_down) {
                    tapeElevationPosition += 0.002f;
                    tapeElevationPosition = Range.clip(tapeElevationPosition, FreightBot_Old.TAPE_ELEVATION_MIN,
                            FreightBot_Old.TAPE_ELEVATION_MAX);
                } else if (gamepad2.dpad_up) {
                    tapeElevationPosition -= 0.002f;
                    tapeElevationPosition = Range.clip(tapeElevationPosition, FreightBot_Old.TAPE_ELEVATION_MIN,
                            FreightBot_Old.TAPE_ELEVATION_MAX);
                }
                bot.setTapeElevation(tapeElevationPosition);


                if (gamepad2.dpad_left) {
                    bot.setTapeRotationPower(gamepad2.b ? .4f : .15f);
                } else if (gamepad2.dpad_right) {
                    bot.setTapeRotationPower(gamepad2.b ? -.4f : -.15f);
                } else {
                    bot.setTapeRotationPower(0f);
                }
            }

            telemetry.addData("right arm angle", rightArmAngleMotorPosition);
            telemetry.addData("left arm angle", leftArmAngleMotorPosition);
//            telemetry.addData("right target", bot.rightArmAngleMotor.getTargetPosition());
//            telemetry.addData("left target", bot.leftArmAngleMotor.getTargetPosition());
            telemetry.addData("arm mode", bot.getArmAngleMode());
//            telemetry.addData("Intake Wheel State", bot.getIntakeWheelState());
            telemetry.addData("Intake Flipper", intakeFlipperPosition);
            telemetry.addData("Tape Elevation", tapeElevationPosition);
            telemetry.addData("Tape Rotation", bot.tapeRotationEncoder.getCurrentPosition());
            telemetry.addData("Tape Extension", bot.tapeExtensionEncoder.getCurrentPosition());
            telemetry.addData("Tape Extension Power",bot.tapeExtensionCRServo.getPower());
            telemetry.addData("Tape Rotation Power", bot.tapeRotationCRServo.getPower());
            telemetry.update();
        }
    }

    public class StoredTSEGrabber implements Updatable {

        private TapeOperationState state = TapeOperationState.ROTATING;
        ElapsedTime et = new ElapsedTime();

        public void update() {
            switch (state) {
                case ROTATING:
                    bot.openCapHolderServo();
                    bot.updateTapeRotation(-660, 1);
                    if (et.milliseconds() > 500) state = TapeOperationState.ELEVATING;
                    break;
                case ELEVATING:
                    bot.updateTapeRotation(-660, 1);
                    tapeElevationPosition = 0.3f;
                    bot.setTapeElevation(tapeElevationPosition);
                    if (et.milliseconds() > 1000) state = TapeOperationState.EXTENDING;
                    break;
                case EXTENDING:
                    bot.updateTapeRotation(-660, 1);
                    bot.updateTapeExtension(12400);
                    if (et.milliseconds() > 2000) state = TapeOperationState.DESCENDING;
                    break;
                case DESCENDING:
                    bot.updateTapeRotation(-660, 1);
                    bot.updateTapeExtension(12400);
                    tapeElevationPosition = 0.497f;
                    bot.setTapeElevation(tapeElevationPosition);
            }

        }
    }

    public class TSEStorer implements Updatable {
        TapeOperationState state = TapeOperationState.ELEVATING;
        ElapsedTime et = new ElapsedTime();
        public void update() {
            switch (state) {
                case ELEVATING:
                    tapeElevationPosition = 0.185f;
                    bot.setTapeElevation(tapeElevationPosition);
                    if (et.milliseconds() > 500) state = TapeOperationState.EXTENDING;
                    break;
                case EXTENDING:
                    bot.updateTapeExtension(13500);
                    if (bot.tapeExtensionEncoder.getCurrentPosition() < 14000) state = TapeOperationState.ROTATING;
                    break;
                case ROTATING:

                case DESCENDING:
                    bot.updateTapeExtension(13500);
                    bot.updateTapeRotation(-400,0.5f);
            }
        }
    }


}
