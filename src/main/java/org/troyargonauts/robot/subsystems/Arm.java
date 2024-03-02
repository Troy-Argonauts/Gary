package org.troyargonauts.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.troyargonauts.robot.Constants;

import static org.troyargonauts.robot.Constants.Arm.*;
import static org.troyargonauts.robot.Constants.Arm.UP_I;

/**
 * Class representing the Arm subsystem
 *
 * @author Ashwin Shrivastav
 */
public class Arm extends SubsystemBase {
    private TalonFX leftArmMotor, rightArmMotor;

    private DigitalInput limitSwitch;

    private double leftArmEncoder, rightArmEncoder = 0;
    private double armTarget = 0;
    private double oldTarget;

    private DoubleLogEntry armLeftEncoderLog;
    private DoubleLogEntry armRightEncoderLog;
    private DoubleLogEntry armLeftOutputCurrentLog;
    private DoubleLogEntry armRightOutputCurrentLog;
    private DoubleLogEntry armLeftMotorVoltage;
    private DoubleLogEntry armRightMotorVoltage;
    private DoubleLogEntry armTargetLog;
    private DoubleLogEntry armAvgEncoderLog;
    private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
    public final Slot0Configs upConfig = new Slot0Configs();
    public final Slot1Configs downConfig = new Slot1Configs();
    public final Slot2Configs zeroConfig = new Slot2Configs();


    /**
     * Instantiates and configures motor controllers and sensors; creates Data Logs. Assigns PID constants.
     */
    public Arm() {
        upConfig.kP = UP_P;
        upConfig.kI = UP_I;
        upConfig.kD = UP_D;

        downConfig.kP = DOWN_P;
        downConfig.kI = DOWN_I;
        downConfig.kD = DOWN_D;

        zeroConfig.kP = ZERO;
        zeroConfig.kI = ZERO;
        zeroConfig.kD = ZERO;

        TalonFXConfiguration allConfigs = new TalonFXConfiguration().withSlot0(upConfig).withSlot1(downConfig).withSlot2(zeroConfig);


        leftArmMotor = new TalonFX(LEFT_MOTOR_ID, CANBUS_NAME);
        rightArmMotor = new TalonFX(RIGHT_MOTOR_ID, CANBUS_NAME);

        leftArmMotor.getConfigurator().apply(allConfigs);
        rightArmMotor.getConfigurator().apply(allConfigs);

        leftArmMotor.setInverted(false);
        rightArmMotor.setInverted(true);


        leftArmMotor.setNeutralMode(NeutralModeValue.Brake);
        rightArmMotor.setNeutralMode(NeutralModeValue.Brake);

        limitSwitch = new DigitalInput(LIMIT_SWITCH_SLOT);

        DataLog log = DataLogManager.getLog();

//        armLeftEncoderLog = new DoubleLogEntry(log, "Arm Left Encoder Values");
//        armRightEncoderLog = new DoubleLogEntry(log, "Arm Right Encoder Values");
//        armLeftOutputCurrentLog = new DoubleLogEntry(log, "Arm Motor Output Current ");
//        armRightOutputCurrentLog = new DoubleLogEntry(log, "Arm Motor Output Current ");
//        armLeftMotorVoltage = new DoubleLogEntry(log, "Arm Motor Bus Voltage");
//        armRightMotorVoltage = new DoubleLogEntry(log, "Arm Motor Bus Voltage");
//        armTargetLog = new DoubleLogEntry(log, "Arm Target Log");
        armAvgEncoderLog = new DoubleLogEntry(log, "Arm Average Encoder Values");
    }

    /**
     * Updates the encoders and outputs their positions to the SmartDashboard periodically. Append values to each data log periodically
     */
    @Override
    public void periodic() {
        leftArmEncoder = leftArmMotor.getPosition().getValueAsDouble();
        rightArmEncoder = rightArmMotor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("oldtarget",oldTarget);
        SmartDashboard.putNumber("current Target", armTarget);
        SmartDashboard.putNumber("Average Arm Encoder: ", (leftArmEncoder+rightArmEncoder)/2);
        SmartDashboard.putBoolean("Arm Limit: ", getLimitSwitch());
        if (getLimitSwitch()){
            leftArmMotor.getConfigurator().apply(new Slot0Configs().withKP(UP_P).withKI(UP_I).withKD(UP_D));
            rightArmMotor.getConfigurator().apply(new Slot0Configs().withKP(UP_P).withKI(UP_I).withKD(UP_D));

//            positionVoltage.Slot = 0;
        }

//        armLeftEncoderLog.append(leftArmEncoder);
//        armRightEncoderLog.append(rightArmEncoder);
//        armLeftOutputCurrentLog.append(leftArmMotor.getSupplyCurrent().getValue());
//        armRightOutputCurrentLog.append(rightArmMotor.getSupplyCurrent().getValue());
//        armLeftMotorVoltage.append(leftArmMotor.getMotorVoltage().getValue());
//        armRightMotorVoltage.append(rightArmMotor.getMotorVoltage().getValue());
        //armTargetLog.append(armTarget);
        //armAvgEncoderLog.append((leftArmEncoder+rightArmEncoder)/2);
    }

    /**
     * Resets the encoders of both the left and right Arm motors.
     * Sets both motor positions to 0.
     */
    public void resetEncoders() {
        leftArmMotor.setPosition(0);
        rightArmMotor.setPosition(0);
    }

    /**
     * Sets raw motor power for both left and right Arm motors
     *
     * @param power desired power (value between -1 and 1)
     */
    public void setPower(double power) {
        leftArmMotor.set(power);
        rightArmMotor.set(power);
    }

    /**
     * Sets the PID loops for the left and right Arm motors to their corresponding target positions
     */
    public void run() {

//        if (armTarget == 0 && leftArmEncoder <= 1 && rightArmEncoder <= 1){
//            leftArmMotor.getConfigurator().apply(new Slot0Configs().withKP(ZERO).withKI(ZERO).withKD(ZERO));
//            rightArmMotor.getConfigurator().apply(new Slot0Configs().withKP(ZERO).withKI(ZERO).withKD(ZERO));
//
//        }
//        if (oldTarget > armTarget){
//            leftArmMotor.getConfigurator().apply(new Slot0Configs().withKP(DOWN_P).withKI(DOWN_I).withKD(DOWN_D));
//            rightArmMotor.getConfigurator().apply(new Slot0Configs().withKP(DOWN_P).withKI(DOWN_I).withKD(DOWN_D));
//
//        } else{
//            leftArmMotor.getConfigurator().apply(new Slot0Configs().withKP(UP_P).withKI(UP_I).withKD(UP_D));
//            rightArmMotor.getConfigurator().apply(new Slot0Configs().withKP(UP_P).withKI(UP_I).withKD(UP_D));
//        }



        if (armTarget == 0 && leftArmEncoder <= 1 && rightArmEncoder <= 1){
            positionVoltage.Slot = 2; //zero
        }
        if (oldTarget > armTarget){
            positionVoltage.Slot = 1; //down
        } else{
            positionVoltage.Slot = 0; //up
        }



        leftArmMotor.setControl(positionVoltage.withPosition(armTarget));
        rightArmMotor.setControl(positionVoltage.withPosition(armTarget));

    }

    /**
     * Checks if the PID loops for Left and Right Arm motors are within the window for their setpoints
     *
     * @return Whether the PIDs are finished
     */
    public boolean isPIDFinished() {
        return (Math.abs((armTarget - ((leftArmMotor.getPosition().getValueAsDouble())) + rightArmMotor.getPosition().getValueAsDouble()) / 2) <= 5);

    }

    /**
     * Changes the setpoint based on joystick value
     * .
     * @param joystickValue joystick value between -1 and 1
     */
    public void adjustSetpoint(double joystickValue) {
        if (!limitSwitch.get() || joystickValue != 0) {
            armTarget += (joystickValue * 0.03);
        }
    }

    /**
     * Sets enumerators for encoder positions of various Arm States
     */
    public enum ArmStates{
        /**
         * Floor Intake Arm position
         */
        FLOOR_INTAKE(0),

        /**
         * Amp scoring Arm position
         */
        AMP(12),

        /**
         * Stage scoring Arm position
         */
        STAGE(5.33),

        /**
         * Subwoofer scoring Arm position
         */
        SUBWOOFER(0),

        /**
         * Climbing Arm position
         */
        CLIMBER(0);

        final double armPosition;

        ArmStates(double armPosition){
            this.armPosition = armPosition;
        }
    }

    /**
     * Returns the average of the Arm encoder values.
     *
     * @return Average arm encoder value.
     */
    public double getEncoderValue() {
        return (rightArmEncoder + leftArmEncoder) / 2;
    }

    /**
     * Returns value of current Arm target position
     *
     * @return current Arm target
     */
    public double getCurrentTarget() {
        return armTarget;
    }

    /**
     * Sets target of arm to desired target that is not included in ArmStates
     *
     * @param desiredTarget Desired arm target
     */
    public void setDesiredTarget(double desiredTarget) {
        armTarget = desiredTarget;
    }

    /**
     * Sets the arm target to the desired Arm State position
     * @param state Desired Arm state
     */
    public void setState(ArmStates state) {
        oldTarget = armTarget;
        armTarget = state.armPosition;
    }

    /**
     * Gets limit switch state
     *
     * @return Limit switch state
     */
    public boolean getLimitSwitch() {

        return !limitSwitch.get();
    }
}
