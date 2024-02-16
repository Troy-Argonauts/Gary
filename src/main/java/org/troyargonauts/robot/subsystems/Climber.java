package org.troyargonauts.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.Rev2mDistanceSensor;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.troyargonauts.robot.Constants.Climber.*;

/**
 * Class representing climber subsystem.
 *
 * @author Nihaar57, SavageCabbage360, firearcher2012, shaquilleinoatmeal
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
    private DoubleLogEntry climberDistanceEncoder;

    /**
     * Instantiates motorController object and sets the neutral mode to break.
     * Outputs values of motor target, voltage, current, and encoder to the DataLog.
     */

    public Climber(){
        motor = new TalonFX(MOTOR_ID);
        motor.setNeutralMode(NeutralModeValue.Brake);

        motor.getConfigurator().apply(new Slot0Configs().withKP(P).withKI(I).withKD(D));

        DataLog log = DataLogManager.getLog();
        climberDistanceEncoder = new DoubleLogEntry((log), "Distance Encoder Values");
        climberTargetLog = new DoubleLogEntry((log), "Climber Target Values");
        climberMotorVoltage = new DoubleLogEntry((log), "Climber Motor Voltage");
        climberOutputCurrentLog = new DoubleLogEntry((log), "Climber Current Output Values");
        climberEncoderLog = new DoubleLogEntry((log), "Climber Encoder Values");
        distanceSensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kOnboard);
        distanceSensor.setAutomaticMode(true);
    }

    /**
     * Resets encoder value.
     */

    public void resetEncoder() {
        motor.setPosition(0);
    }

    /**
     * Runs the motor to a specified target value expressed as a double.
     */

    public void run() {
        motor.setControl(positionVoltage.withPosition(motorTarget));
    }

    public void setTarget() {
        motor.setPosition(100);
    }

    /**
     * Checks whether the PID is finished.
     * @return finished or not.
     */

    public boolean isPidFinished() {
        return (Math.abs((motorTarget - motor.getPosition().getValueAsDouble())) <= 5);
    }

    /**
     * Stores values of encoders and displays values on smartDashboard.
     */

    @Override
    public void periodic() {
        climberEncoderLog.append(motorEncoder);
        climberOutputCurrentLog.append(motor.getSupplyCurrent().getValue());
        climberMotorVoltage.append(motor.getMotorVoltage().getValue());
        climberTargetLog.append(motorEncoder);
        climberDistanceEncoder.append(distanceSensorOutput);

        motorEncoder = motor.getPosition().getValueAsDouble();

        SmartDashboard.putNumber("Climber Encoder: ", motorEncoder);
        SmartDashboard.putNumber("Range Onboard", distanceSensor.getRange());
        SmartDashboard.putBoolean("Timestamp Onboard", distanceSensor.isRangeValid());
        SmartDashboard.putNumber("Distance Sensor", distanceSensorOutput);
    }

    /**
     * Determines if robot is within range.
     * @param minDistance determines the minimum of the range.
     * @param maxDistance determines the maximum of the range.
     * @return whether the robot is in range.
     */

    public boolean distanceWithinRange(double minDistance, double maxDistance){
        if (((minDistance < distanceSensorOutput) && (distanceSensorOutput < maxDistance))){
            return true;
        }
        else {
            return false;
        }
    }

    /**
     * Determines the range between the robot and the nearest object.
     * @return distance as a double.
     */

    public double getDistance(){
        return distanceSensor.getRange();
    }
}

