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


public class Arm extends SubsystemBase {

    private TalonFX leftArmMotor, rightArmMotor;

    private double leftArmEncoder = 0, rightArmEncoder = 0;

    private double armTarget = 0;

    private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
    private DoubleLogEntry armLeftEncoderLog;
    private DoubleLogEntry armRightEncoderLog;
    private DoubleLogEntry armLeftOutputCurrentLog;
    private DoubleLogEntry armRightOutputCurrentLog;
    private DoubleLogEntry armLeftMotorVoltage;
    private DoubleLogEntry armRightMotorVoltage;
    private DoubleLogEntry armTargetLog;

    public Arm() {
        leftArmMotor = new TalonFX(LEFT_MOTOR_ID);
        rightArmMotor = new TalonFX(RIGHT_MOTOR_ID);

        leftArmMotor.getConfigurator().apply(new Slot0Configs().withKP(P).withKI(I).withKD(D));
        rightArmMotor.getConfigurator().apply(new Slot0Configs().withKP(P).withKI(I).withKD(D));

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
     * @param target target position of motors
     */
    public void run() {
        leftArmMotor.setControl(positionVoltage.withPosition(armTarget));
        rightArmMotor.setControl(positionVoltage.withPosition(armTarget));
    }

    /**
     * Checks if the PID for Left and Right arm motors are finished.
     * Checks if the distance in between the Target position and Current Position is less than or equal to 5
     *
     * @param motorID Motor ID of motor checked.
     * @return Returns false if the motor ID is not 1 or 2.
     */
    public boolean isPidFinished() {
        return (Math.abs((armTarget - ((leftArmMotor.getVelocity().getValueAsDouble())) + rightArmMotor.getVelocity().getValueAsDouble())/2) <= 5);

    }

    public void adjustSetpoint(double joystickValue){
        armTarget += joystickValue;
    }

    public enum ARM{
        FLOOR_INTAKE(0),
        AMP(1),
        SHOOTER(2),
        FAR_SHOOTER(3);
    }

    /**
     * Returns average of two arm encoder values.
     * @return double -  average arm encoder value.
     */
    public double getEncoderValue(){
        return (rightArmEncoder + leftArmEncoder)/2;
    }

    /**
     * Returns value of current arm target variable.
     * @return double - current arm target
     */
    public double getCurrentTarget(){
        return armTarget;
    }
}