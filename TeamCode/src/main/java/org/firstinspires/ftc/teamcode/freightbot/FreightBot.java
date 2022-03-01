package org.firstinspires.ftc.teamcode.freightbot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.mecbot.MecBot;

import java.util.Random;

public class FreightBot extends MecBot {
    public DcMotorEx rightArmAngleMotor;
    public DcMotorEx leftArmAngleMotor;
    public Servo intakeFlipper;
    public Servo tapeElevationServo;
    public Servo capHolderServo;
    public CRServo spinnerCRServo;
    public CRServo intakeRightSpinner;
    public CRServo intakeLeftSpinner;

    public CRServo tapeExtensionCRServo;
    public DcMotorEx tapeExtensionEncoder;
    public CRServo tapeRotationCRServo;
    public DcMotorEx tapeRotationEncoder;

    public static final int MAX_ARM_ANGLE_TICKS = 2200;
    public static final float TAPE_ELEVATION_MIN = 0.09f;
    public static final float TAPE_ELEVATION_MAX = 0.77f;
    public static final float TAPE_ROTATION_COEFF = 0.001f;
    public static final float TAPE_EXTENSION_COEFF = 0.0001f;
    public static final float CAP_SERVO_OPENED = 1f;
    public static final float CAP_SERVO_CLOSED = 0.27f;
    public static final float CAP_SERVO_MID = 0.5f * (CAP_SERVO_OPENED + CAP_SERVO_CLOSED);

    public enum IntakeWheelState {
        STOPPED, FORWARD, REVERSE, SLOW_FORWARD
    }

    private IntakeWheelState intakeWheelState = IntakeWheelState.STOPPED;

    /**
     * Constructor: USE THIS ONE FOR A REAL ROBOT!
     * @param mType     Motor Type
     * @param w         Wheel Base Width (inches)
     * @param l         Wheel Base Length (inches)
     * @param wheelDiam     Wheel Diameter (inches)
     * @param rollerAngle   Wheel Roller Angle (degrees)
     * @param gearRatio     Gear Ratio (motor output shaft rotations per wheel rotation)
     * @param axesMap       Axes Map for the BNO055Enhanced
     * @param axesSign      Axes Sign for the BNO055Enhanced
     */
    public FreightBot(MotorType mType, float w, float l, float wheelDiam, float rollerAngle, float gearRatio, BNO055Enhanced.AxesMap axesMap, BNO055Enhanced.AxesSign axesSign){
        super(mType, w, l, wheelDiam, rollerAngle, gearRatio, axesMap, axesSign);
    }

    public FreightBot() {
        super();
    }

    public boolean init(HardwareMap hwMap) {

        boolean result = super.init(hwMap);

        spinnerCRServo = hwMap.get(CRServo.class, "spinner_servo");
        rightArmAngleMotor = hwMap.get(DcMotorEx.class, "right_arm_angle_motor");
        leftArmAngleMotor = hwMap.get(DcMotorEx.class, "left_arm_angle_motor");
        leftArmAngleMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeFlipper = hwMap.get(Servo.class, "intake_flipper");
        intakeLeftSpinner = hwMap.get(CRServo.class, "intake_left_spinner");
        intakeRightSpinner = hwMap.get(CRServo.class, "intake_right_spinner");
        tapeElevationServo = hwMap.get(Servo.class, "tape_elevation_servo");
        tapeExtensionCRServo = hwMap.get(CRServo.class, "tape_extension_crservo");
        tapeExtensionEncoder = hwMap.get(DcMotorEx.class, "tape_extension_encoder");
        tapeRotationCRServo = hwMap.get(CRServo.class, "tape_rotation_crservo");
        tapeRotationEncoder = hwMap.get(DcMotorEx.class, "tape_rotation_encoder");
        capHolderServo = hwMap.get(Servo.class,"cap_holder_servo");

        rightArmAngleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmAngleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightArmAngleMotor.setTargetPosition(rightArmAngleMotor.getCurrentPosition());
        leftArmAngleMotor.setTargetPosition(leftArmAngleMotor.getCurrentPosition());
        rightArmAngleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArmAngleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        tapeRotationEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tapeRotationEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tapeExtensionEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tapeExtensionEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        return result;
    }


    public void setIntakePower(double pow) {
        intakeRightSpinner.setPower(pow);
        intakeLeftSpinner.setPower(-pow);
    }

    public void setIntakeWheelState(IntakeWheelState state) {
        intakeWheelState = state;
        if (intakeWheelState == IntakeWheelState.STOPPED) {
            setIntakePower(0);
        } else if (intakeWheelState == IntakeWheelState.FORWARD) {
            setIntakePower(1);
        } else if (intakeWheelState == IntakeWheelState.REVERSE){
            setIntakePower(-.3);
        } else {
            setIntakePower(.1);
        }
    }

    public IntakeWheelState getIntakeWheelState() {
        return intakeWheelState;
    }

    public void setIntakeFlipper(double pos) {
        intakeFlipper.setPosition(pos);
    }

    public void setArmAngleMode(DcMotor.RunMode mode) {
        if (rightArmAngleMotor.getMode() == mode && leftArmAngleMotor.getMode() == mode) return;
        if (mode == DcMotor.RunMode.RUN_TO_POSITION){
            rightArmAngleMotor.setTargetPosition(rightArmAngleMotor.getCurrentPosition());
            leftArmAngleMotor.setTargetPosition(leftArmAngleMotor.getCurrentPosition());
        }
        rightArmAngleMotor.setMode(mode);
        leftArmAngleMotor.setMode(mode);
    }

    public DcMotor.RunMode getArmAngleMode(){
        return rightArmAngleMotor.getMode();
    }

    public void setArmAngleTicks(int pos) {
        rightArmAngleMotor.setTargetPosition(pos);
        leftArmAngleMotor.setTargetPosition(pos);
        rightArmAngleMotor.setPower(1f);
        leftArmAngleMotor.setPower(1f);
    }

    public void setArmAngleTicks(int pos, float power) {
        rightArmAngleMotor.setTargetPosition(pos);
        leftArmAngleMotor.setTargetPosition(pos);
        rightArmAngleMotor.setPower(power);
        leftArmAngleMotor.setPower(power);
    }

    public void setSpeedSpinnerMotor(float speed) {
        spinnerCRServo.setPower(speed);
    }

    public float getSpeedSpinnerMotor() {
        return (float) spinnerCRServo.getPower();
    }

    public void setTapeExtensionPower(float power) {
        tapeExtensionCRServo.setPower(power);
    }

    public void setTapeElevation(float pos) {
        tapeElevationServo.setPosition(pos);
    }

    public void setTapeRotationPower (float power) {
        tapeRotationCRServo.setPower(power);
    }

    public void updateTapeRotation(int target, float maxPower) {
        float power = -TAPE_ROTATION_COEFF * (target - tapeRotationEncoder.getCurrentPosition());
        power = Range.clip(power,-maxPower,maxPower);
        tapeRotationCRServo.setPower(power);
    }

    public void updateTapeExtension(int target) {
        float power = TAPE_EXTENSION_COEFF * (target - tapeExtensionEncoder.getCurrentPosition());
        power = Range.clip(power,-1,1);
        tapeExtensionCRServo.setPower(power);
    }

    public void openCapHolderServo() {
        capHolderServo.setPosition(CAP_SERVO_OPENED);
    }

    public void closeCapHolderServo() {
        capHolderServo.setPosition(CAP_SERVO_CLOSED);
    }

}
