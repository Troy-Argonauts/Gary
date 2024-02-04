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
    private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
    private TalonFX Motor;
    private double Encoder = 0;
    private double Target = 0;
    private Rev2mDistanceSensor distanceSensor;
    double distanceSensorOutput = distanceSensor.getRange();
    private DoubleLogEntry climberEncoderLog;
    private DoubleLogEntry climberOutputCurrentLog;
    private DoubleLogEntry climberBusVoltage;
    private DoubleLogEntry climberTargetLog;

    /**
     * Instantiates motorController object and sets the neutral mode to break.
     */

    public Climber(){
        Motor = new TalonFX(MOTOR_ID);
        Motor.setNeutralMode(NeutralModeValue.Brake);

        Encoder = Motor.getPosition().getValueAsDouble();

        SmartDashboard.putNumber("Climber Encoder: ", Encoder);
        distanceSensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kOnboard);
        distanceSensor.setAutomaticMode(true);

        Encoder = Motor.getPosition().getValueAsDouble();

        SmartDashboard.putNumber("Climber Encoder: ", Encoder);
        SmartDashboard.putNumber("Range Onboard", distanceSensor.getRange);
        SmartDashboard.putBoolean("Timestamp Onboard", distanceSensor.isRangeValid());

        climberEncoderLog.append(Encoder);
        climberOutputCurrentLog.append(Motor.getSupplyCurrent().getValue());
        climberBusVoltage.append(Motor.getMotorVoltage().getValue());
        climberTargetLog.append(Encoder);
    }

    public void resetEncoder() {
        Motor.setPosition(0);
    }

    /**
     * Runs the motor to a specified target value expressed as a double.
     * @param target: the target you want the motor to run to.
     */

    public void run(double target) {
        Motor.setControl(positionVoltage.withPosition(target));
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
        return (Math.abs((Target - Motor.getPosition().getValueAsDouble())) <= 5);
    }

    @Override
    public void periodic() {
        run(Target);
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

