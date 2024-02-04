package org.troyargonauts.robot.subsystems;


import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.Rev2mDistanceSensor;
import edu.wpi.first.util.datalog.DoubleLogEntry;
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

    /**
     * Creates motorController object.
     */
    private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
    private TalonFX climberMotor;

    private Rev2mDistanceSensor distanceSensor;
    double distanceSensorOutput = distanceSensor.getRange();

    private double climberEncoder = 0;

    private double climberTarget = 0;

    private DoubleLogEntry climberEncoderLog;
    private DoubleLogEntry climberOutputCurrentLog;
    private DoubleLogEntry climberBusVoltage;
    private DoubleLogEntry climberTargetLog;


    /**
     * Instantiates motorController object and sets the neutral mode to break.
     */

    public Climber(){
        climberMotor = new TalonFX(CLIMBER_MOTOR_PORT);
        climberMotor.setNeutralMode(NeutralModeValue.Brake);

        distanceSensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kOnboard);
        distanceSensor.setAutomaticMode(true);

        climberEncoder = climberMotor.getPosition().getValueAsDouble();

        SmartDashboard.putNumber("Climber Encoder: ", climberEncoder);
        SmartDashboard.putNumber("Range Onboard", distanceSensor.getRange());
        SmartDashboard.putBoolean("Timestamp Onboard", distanceSensor.isRangeValid());

        climberEncoderLog.append(climberEncoder);
        climberOutputCurrentLog.append(climberMotor.getSupplyCurrent().getValue());
        climberBusVoltage.append(climberMotor.getMotorVoltage().getValue());
        climberTargetLog.append(climberEncoder);
    }

    public void resetEncoders() {
        climberMotor.setPosition(0);
    }

    /**
     * Creates 3 different motor states (OFF, DOWN, and UP).
     */


    public void run(double target) {
        climberMotor.setControl(positionVoltage.withPosition(target));
    }

    public enum MotorStates {
        UP, OFF
    }

    public boolean isPidFinished(int motorID) {
        return (Math.abs((climberTarget - climberMotor.getVelocity().getValueAsDouble())) <= 5);

    }

    /**
     * Sets power to the motorController for each state (OFF has no power, DOWN and UP have a power value of 0.75 in their respective directions).
     * @param states: pulls the state from MotorStates.
     */

    public void setClimberMotor(MotorStates states){
        switch (states) {
            case OFF: climberMotor.set(CLIMBER_MOTOR_OFF_SPEED);
            break;
            case UP: climberMotor.set(CLIMBER_MOTOR_ON_SPEED);
            break;
        }
    }



    public boolean distanceWithinRange(){
        if (((Constants.Climber.DISTANCE_MIN < distanceSensorOutput) && (distanceSensorOutput < Constants.Climber.DISTANCE_MAX))){
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

