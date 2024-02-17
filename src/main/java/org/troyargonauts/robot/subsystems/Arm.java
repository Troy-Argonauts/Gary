package org.troyargonauts.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.troyargonauts.robot.Constants.Arm.*;

/**
 * Class representing the Arm subsystem, including the Data Logging and PID
 * @author Ashwin Shrivastav
 */
public class Arm extends SubsystemBase {
    private TalonFX leftArmMotor, rightArmMotor;

    private double leftArmEncoder, rightArmEncoder = 0;
    private double armTarget = 0;

    private DoubleLogEntry armLeftEncoderLog;
    private DoubleLogEntry armRightEncoderLog;
    private DoubleLogEntry armLeftOutputCurrentLog;
    private DoubleLogEntry armRightOutputCurrentLog;
    private DoubleLogEntry armLeftMotorVoltage;
    private DoubleLogEntry armRightMotorVoltage;
    private DoubleLogEntry armTargetLog;
    
    private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);

    /**
     * Instantiated motor controllers, motors, data logging values and data log, target, and motor IDs.
     */
    public Arm() {
        leftArmMotor = new TalonFX(LEFT_MOTOR_ID, CANBUS_NAME);
        rightArmMotor = new TalonFX(RIGHT_MOTOR_ID, CANBUS_NAME);

        leftArmMotor.getConfigurator().apply(new Slot0Configs().withKP(P).withKI(I).withKD(D));
        rightArmMotor.getConfigurator().apply(new Slot0Configs().withKP(P).withKI(I).withKD(D));

        leftArmMotor.setInverted(true);
        rightArmMotor.setInverted(false);

        DataLog log = DataLogManager.getLog();

        armLeftEncoderLog = new DoubleLogEntry(log, "Arm Left Encoder Values");
        armRightEncoderLog = new DoubleLogEntry(log, "Arm Right Encoder Values");
        armLeftOutputCurrentLog = new DoubleLogEntry(log, "Arm Motor Output Current ");
        armRightOutputCurrentLog = new DoubleLogEntry(log, "Arm Motor Output Current ");
        armLeftMotorVoltage = new DoubleLogEntry(log, "Arm Motor Bus Voltage");
        armRightMotorVoltage = new DoubleLogEntry(log, "Arm Motor Bus Voltage");
        armTargetLog = new DoubleLogEntry(log, "Arm Target Log");
    }

    /**
     * Periodic will constantly update the encoder values and put things on the SmartDashboard
     * and will add all of the values to the data logging.
     */
    @Override
    public void periodic() {
        leftArmEncoder = leftArmMotor.getPosition().getValueAsDouble();
        rightArmEncoder = rightArmMotor.getPosition().getValueAsDouble();

        SmartDashboard.putNumber("Left Arm Encoder: ", leftArmEncoder);
        SmartDashboard.putNumber("Right Arm Encoder: ", rightArmEncoder);

        armLeftEncoderLog.append(leftArmEncoder);
        armRightEncoderLog.append(rightArmEncoder);
        armLeftOutputCurrentLog.append(leftArmMotor.getSupplyCurrent().getValue());
        armRightOutputCurrentLog.append(rightArmMotor.getSupplyCurrent().getValue());
        armLeftMotorVoltage.append(leftArmMotor.getMotorVoltage().getValue());
        armRightMotorVoltage.append(rightArmMotor.getMotorVoltage().getValue());
        armTargetLog.append(armTarget);
    }

    /**
     * Resets the encoders of the left and right motors.
     * Sets both motor's positions to 0.
     */
    public void resetEncoders() {
        leftArmMotor.setPosition(0);
        rightArmMotor.setPosition(0);
    }

    /**
     * Sets the left and right arm motors to their corresponding targets and
     * sets the motors power to that.
     *
     */
    public void run() {
        leftArmMotor.setControl(positionVoltage.withPosition(armTarget));
        rightArmMotor.setControl(positionVoltage.withPosition(armTarget));
    }

    /**
     * Checks if the PID for Left and Right arm motors are finished.
     * Checks if the distance in between the Target position and Current Position is less than or equal to 5
     *
     * @return Returns false if the motor ID is not 1 or 2.
     */
    public boolean isPidFinished() {
        return (Math.abs((armTarget - ((leftArmMotor.getVelocity().getValueAsDouble())) + rightArmMotor.getVelocity().getValueAsDouble()) / 2) <= 5);

    }

    /**
     * Changes setpoint based on joystick value parameter.
     * @param joystickValue joystick value being passed in to the function
     */
    public void adjustSetpoint(double joystickValue) {
        armTarget += (joystickValue * 20);
    }

    /**
     * Sets enumerators of ArmStates
     */
    public enum ArmStates{
        FLOOR_INTAKE(100),
        AMP(200),
        PODIUM(233234),
        SUBWOOFER(1234),

        CLIMBER(123);

        final double armPosition;

        ArmStates(double armPosition){
            this.armPosition = armPosition;
        }
    }

    /**
     * Returns average of two arm encoder values.
     * @return double -  average arm encoder value.
     */
    public double getEncoderValue() {
        return (rightArmEncoder + leftArmEncoder) / 2;
    }

    /**
     * Returns value of current arm target variable.
     * @return double - current arm target
     */
    public double getCurrentTarget() {
        return armTarget;
    }

    /**
     * Sets target of arm to desired target
     * @param desiredTarget desired arm target
     */
    public void setDesiredTarget(double desiredTarget) {
        armTarget = desiredTarget;
    }

    /**
     * Sets the arm target to an Arm State position
     * @param state ArmStates enumerator for arm position
     */
    public void setState(ArmStates state) {
        armTarget = state.armPosition;
    }
}
