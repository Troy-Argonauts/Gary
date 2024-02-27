package org.troyargonauts.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.troyargonauts.robot.Constants.Arm.*;

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

    private DoubleLogEntry armLeftEncoderLog;
    private DoubleLogEntry armRightEncoderLog;
    private DoubleLogEntry armLeftOutputCurrentLog;
    private DoubleLogEntry armRightOutputCurrentLog;
    private DoubleLogEntry armLeftMotorVoltage;
    private DoubleLogEntry armRightMotorVoltage;
    private DoubleLogEntry armTargetLog;
    private DoubleLogEntry armAvgEncoderLog;
    private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);

    /**
     * Instantiates and configures motor controllers and sensors; creates Data Logs. Assigns PID constants.
     */
    public Arm() {
        leftArmMotor = new TalonFX(LEFT_MOTOR_ID, CANBUS_NAME);
        rightArmMotor = new TalonFX(RIGHT_MOTOR_ID, CANBUS_NAME);

        leftArmMotor.getConfigurator().apply(new Slot0Configs().withKP(P).withKI(I).withKD(D));
        rightArmMotor.getConfigurator().apply(new Slot0Configs().withKP(P).withKI(I).withKD(D));

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

        SmartDashboard.putNumber("Average Arm Encoder: ", (leftArmEncoder+rightArmEncoder)/2);
        SmartDashboard.putBoolean("Arm Limit: ", getLimitSwitch());

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
        if (!limitSwitch.get() || joystickValue < 0) {
            armTarget += (joystickValue * 20);
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
         * Podium scoring Arm position
         */
        PODIUM(5.33),

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
