package org.troyargonauts.robot.subsystems;


import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.Rev2mDistanceSensor;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.troyargonauts.robot.Constants;

import static org.troyargonauts.robot.Constants.Climber.*;

/**
 * Class representing climber subsystem.
 *
 * @author Nihaar57, SavageCabbage360, shaquilleinoatmeal
 */

public class Climber extends SubsystemBase {
    private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
    private TalonFX motor;
    private double motorEncoder = 0;
    private double motorTarget = 0;
    private Rev2mDistanceSensor distanceSensor;
    double distanceSensorOutput = distanceSensor.getRange();
    private DoubleLogEntry climberEncoderLog;
    private DoubleLogEntry climberOutputCurrentLog;
    private DoubleLogEntry climberMotorVoltage;
    private DoubleLogEntry climberTargetLog;
    private DoubleLogEntry distanceEncoder;

    /**
     * Instantiates motorController object and sets the neutral mode to break.
     */

    public Climber(){
        motor = new TalonFX(MOTOR_ID);
        motor.setNeutralMode(NeutralModeValue.Brake);

        DataLog log = DataLogManager.getLog();
        distanceEncoder = new DoubleLogEntry((log), "Distance Encoder Values");
        climberTargetLog = new DoubleLogEntry((log), "Climber Target Values");
        climberMotorVoltage = new DoubleLogEntry((log), "Climber Motor Voltage");
        climberOutputCurrentLog = new DoubleLogEntry((log), "Climber Current Output Values");
        climberEncoderLog = new DoubleLogEntry((log), "Climber Encoder Values");
        distanceSensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kOnboard);
        distanceSensor.setAutomaticMode(true);
    }

    public void resetEncoder() {
        motor.setPosition(0);
    }

    /**
     * Runs the motor to a specified target value expressed as a double..
     */

    public void run() {
        motor.setControl(positionVoltage.withPosition(motorTarget));
    }

    public enum MotorStates {
        TOP(100),
        BOTTOM(0);
        final double encoderPosition;

        MotorStates(double encoderPosition) {
            this.encoderPosition = encoderPosition;
        }

        public double getEncoderPosition() {
            return this.encoderPosition;
        }
    }
    public boolean isPidFinished() {
        return (Math.abs((motorTarget - motor.getPosition().getValueAsDouble())) <= 5);
    }

    @Override
    public void periodic() {
        climberEncoderLog.append(motorEncoder);
        climberOutputCurrentLog.append(motor.getSupplyCurrent().getValue());
        climberMotorVoltage.append(motor.getMotorVoltage().getValue());
        climberTargetLog.append(motorEncoder);
        distanceEncoder.append(distanceSensorOutput);

        motorEncoder = motor.getPosition().getValueAsDouble();

        SmartDashboard.putNumber("Climber Encoder: ", motorEncoder);
        SmartDashboard.putNumber("Range Onboard", distanceSensor.getRange());
        SmartDashboard.putBoolean("Timestamp Onboard", distanceSensor.isRangeValid());
        SmartDashboard.putNumber("Distance Sensor", distanceSensorOutput);
    }

    public boolean distanceWithinRange(double minDistance, double maxDistance){
        if (((minDistance < distanceSensorOutput) && (distanceSensorOutput < maxDistance))){
            return true;
        }
        else {
            return false;
        }
    }

    public double getDistance(){
        return distanceSensor.getRange();
    }
}

