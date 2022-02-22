package org.firstinspires.ftc.teamcode.freightbot_old;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.mecbot.MecBot;

public class FreightBot_Old extends MecBot {
    public static final int INTAKE_EXTENSION_MID_TICKS = 435; //was 435
    public static final int INTAKE_EXTENSION_MIN_TICKS = 0;
    public static final int INTAKE_EXTENSION_MAX_TICKS = 1000; //was 1150
    public DcMotorEx rightArmAngleMotor;
    public DcMotorEx leftArmAngleMotor;
    public DcMotorEx armExtenderMotor;
    public DcMotorEx intakeExtensionMotor;
    public Servo armServo;
    public Servo armCapServo;
    public Servo intakeFlipper;
    public Servo tapeElevationServo;
    public CRServo spinnerCRServo;
    public CRServo intakeRightSpinner;
    public CRServo intakeLeftSpinner;
    public CRServo tapeExtensionCRServo;
    public CRServo tapeRotationCRServo;
    public int armExtensionTickOffset = 0;
    public static final int MAX_ARM_ANGLE_TICKS = -1800; //was 850
    public static final float DUMPER_RETRACTED = 0.04f; //was 0.04
    public static final float DUMPER_EXTENDED = 0.7f; //was 0.7
    public static final float DUMPER_MID = 0.3f; // was 0.3
    public static final float ARM_CAP_CLOSED = 0.5f; //was 0.5
    public static final float ARM_CAP_OPEN = 0.15f; //was .15
    public static final float INTAKE_FLIPPER_OUT = .661f; //was 0.66
    public static final float INTAKE_FLIPPER_CENTER = 0.27f; //was 0.34
    public static final float INTAKE_FLIPPER_IN = 0.17f; //was .17
    public static final float TAPE_ELEVATION_MIN = 0.09f;
    public static final float TAPE_ELEVATION_MAX = 0.77f;



    public enum IntakeWheelState {
        STOPPED, FORWARD, REVERSE
    }
    public enum IntakeState {
        OUT_MAX, CENTER_MIN, CENTER_MID, IN_MID
    }
    private IntakeWheelState intakeWheelState = IntakeWheelState.STOPPED;
    private IntakeState intakeState = IntakeState.CENTER_MIN;

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
    public FreightBot_Old(MotorType mType, float w, float l, float wheelDiam, float rollerAngle, float gearRatio, BNO055Enhanced.AxesMap axesMap, BNO055Enhanced.AxesSign axesSign){
        super(mType, w, l, wheelDiam, rollerAngle, gearRatio, axesMap, axesSign);
    }

    public FreightBot_Old() {
        super();
    }

    public boolean init(HardwareMap hwMap, boolean resetArmAndIntake) {

        boolean result = super.init(hwMap);

        spinnerCRServo = hwMap.get(CRServo.class, "spinner_servo");
        rightArmAngleMotor = hwMap.get(DcMotorEx.class, "right_arm_angle_motor");
        leftArmAngleMotor = hwMap.get(DcMotorEx.class, "left_arm_angle_motor");
        leftArmAngleMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armExtenderMotor = hwMap.get(DcMotorEx.class, "arm_extender_motor");
        //intakeExtensionMotor = hwMap.get(DcMotorEx.class, "intake_extension_motor");
        armServo = hwMap.get(Servo.class, "arm_servo");
        armCapServo = hwMap.get(Servo.class, "arm_cap_servo");
        intakeFlipper = hwMap.get(Servo.class, "intake_flipper");
        intakeLeftSpinner = hwMap.get(CRServo.class, "intake_left_spinner");
        intakeRightSpinner = hwMap.get(CRServo.class, "intake_right_spinner");
        tapeElevationServo = hwMap.get(Servo.class, "tape_elevation_servo");
        tapeExtensionCRServo = hwMap.get(CRServo.class, "tape_extension_crservo");
        tapeRotationCRServo = hwMap.get(CRServo.class, "tape_rotation_crservo");



        if (resetArmAndIntake) {
            rightArmAngleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftArmAngleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armExtenderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //intakeExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        rightArmAngleMotor.setTargetPosition(rightArmAngleMotor.getCurrentPosition());
        leftArmAngleMotor.setTargetPosition(leftArmAngleMotor.getCurrentPosition());
        armExtenderMotor.setTargetPosition(armExtenderMotor.getCurrentPosition());
        //intakeExtensionMotor.setTargetPosition(intakeExtensionMotor.getCurrentPosition());
        rightArmAngleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArmAngleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtenderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //intakeExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        return result;
    }

    public void openArmCapServo(){armCapServo.setPosition(ARM_CAP_OPEN);}
    public void closeArmCapServo(){armCapServo.setPosition(ARM_CAP_CLOSED);}

    public void setArmServoPosition(float pos) {
        armServo.setPosition(pos);
    }

    public void setArmExtensionTicks(int pos) {
        armExtenderMotor.setTargetPosition(pos - armExtensionTickOffset);
        armExtenderMotor.setPower(1.0f);
    }

    public void setArmExtensionTicks(int pos, float power) {
        armExtenderMotor.setTargetPosition(pos - armExtensionTickOffset);
        armExtenderMotor.setPower(power);
    }

    public int getActualArmExtensionTicks() {
        return armExtensionTickOffset + armExtenderMotor.getCurrentPosition();
    }

    public void adjustArmExtensionTickOffset(int actualArmExtensionTicks) {
       armExtensionTickOffset = actualArmExtensionTicks - armExtenderMotor.getCurrentPosition();
    }

    public void setIntakeExtensionTicks(int pos) {
        //intakeExtensionMotor.setTargetPosition(pos);
        //intakeExtensionMotor.setPower(0.5f);
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
        } else {
            setIntakePower(-.4);
        }
    }

    public IntakeWheelState getIntakeWheelState() {
        return intakeWheelState;
    }

    public void setIntakeState(IntakeState state) {
        switch(state){
            case OUT_MAX:
                intakeState = IntakeState.OUT_MAX;
                setIntakeExtensionTicks(INTAKE_EXTENSION_MAX_TICKS);
                setIntakeFlipper(INTAKE_FLIPPER_OUT);
                break;
            case CENTER_MIN:
                intakeState = IntakeState.CENTER_MIN;
                setIntakeExtensionTicks(INTAKE_EXTENSION_MIN_TICKS);
                setIntakeFlipper(INTAKE_FLIPPER_CENTER);
                break;
            case CENTER_MID:
                intakeState = IntakeState.CENTER_MID;
                setIntakeExtensionTicks(INTAKE_EXTENSION_MID_TICKS);
                setIntakeFlipper(INTAKE_FLIPPER_CENTER);
                break;
            case IN_MID:
                intakeState = IntakeState.IN_MID;

                setIntakeFlipper(INTAKE_FLIPPER_IN);
                try {
                    Thread.sleep(100);
                } catch (InterruptedException ex){
                    return;
                }

                setIntakeExtensionTicks(INTAKE_EXTENSION_MID_TICKS);


                break;
        }
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }

    public void setIntakeFlipper(double pos) {
        intakeFlipper.setPosition(pos);
    }

    public void setArmAngleTicks(int pos) {
        rightArmAngleMotor.setTargetPosition(pos);
        rightArmAngleMotor.setPower(1f);
        leftArmAngleMotor.setTargetPosition(pos);
        leftArmAngleMotor.setPower(1f);
    }

    public void setArmAngleTicks(int pos, float power) {
        rightArmAngleMotor.setTargetPosition(pos);
        rightArmAngleMotor.setPower(power);
        leftArmAngleMotor.setTargetPosition(pos);
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


}
