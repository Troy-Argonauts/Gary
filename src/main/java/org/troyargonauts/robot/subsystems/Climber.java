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
 * Class representing the Climber subsystem.
 *
 * @author Nihaar57, SavageCabbage360, firearcher2012, shaquilleinoatmeal
 */
public class Climber extends SubsystemBase {
    private TalonFX motor;

    private double motorEncoder = 0;
    private double motorTarget = 0;

    private Rev2mDistanceSensor distanceSensor;
    private double distanceSensorOutput;

    private DoubleLogEntry climberEncoderLog;
    private DoubleLogEntry climberOutputCurrentLog;
    private DoubleLogEntry climberMotorVoltage;
    private DoubleLogEntry climberTargetLog;
    private DoubleLogEntry climberDistanceEncoder;

    private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);

    /**
     * Instantiates motor controller and sets the neutral mode to brake. Creates data logs
     */
    public Climber() {
        motor = new TalonFX(MOTOR_ID, CANBUS_NAME);
        motor.setNeutralMode(NeutralModeValue.Brake);

        motor.getConfigurator().apply(new Slot0Configs().withKP(P).withKI(I).withKD(D));

//        DataLog log = DataLogManager.getLog();
//
//        climberDistanceEncoder = new DoubleLogEntry((log), "Distance Encoder Values");
//        climberTargetLog = new DoubleLogEntry((log), "Climber Target Values");
//        climberMotorVoltage = new DoubleLogEntry((log), "Climber Motor Voltage");
//        climberOutputCurrentLog = new DoubleLogEntry((log), "Climber Current Output Values");
//        climberEncoderLog = new DoubleLogEntry((log), "Climber Encoder Values");

//       // distanceSensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kOnboard);
//        distanceSensor.setAutomaticMode(true);

   //     distanceSensorOutput = distanceSensor.getRange();
    }

    /**
     * Sets motor encoder position to 0
     */
    public void resetEncoder() {
        motor.setPosition(0);
    }

    /**
     * Sets the PID loop for the motor to its corresponding target position
     */
    public void run() {
        motor.setControl(positionVoltage.withPosition(motorTarget));
    }

    /**
     * Sets setpoint to fully climbed position
     */
    public void setTarget() {
        motor.setPosition(100);
    }

    /**
     * Checks if the PID loop is within the window for the position setpoint
     *
     * @return Whether the PID is finished
     */
    public boolean isPIDFinished() {
        return (Math.abs((motorTarget - motor.getPosition().getValueAsDouble())) <= 5);
    }

    /**
     * Updates the encoder value and outputs its position to the SmartDashboard periodically. Append values to each data log periodically
     */
    @Override
    public void periodic() {
//        climberEncoderLog.append(motorEncoder);
//        climberOutputCurrentLog.append(motor.getSupplyCurrent().getValue());
//        climberMotorVoltage.append(motor.getMotorVoltage().getValue());
//        climberTargetLog.append(motorEncoder);
     //   climberDistanceEncoder.append(distanceSensorOutput);

        motorEncoder = motor.getPosition().getValueAsDouble();

        SmartDashboard.putNumber("Climber Encoder: ", motorEncoder);
//        SmartDashboard.putNumber("Range Onboard", distanceSensor.getRange());
//        SmartDashboard.putBoolean("Timestamp Onboard", distanceSensor.isRangeValid());
//        SmartDashboard.putNumber("Distance Sensor", distanceSensorOutput);
    }

//    /**
//     * Determines if the distance sensor is returning a value within a provided range
//     * .
//     * @param minDistance the minimum of the range
//     * @param maxDistance the maximum of the range
//     * @return whether the distance sensor is outputting within provided range
//     */
//    public void distanceWithinRange(double minDistance, double maxDistance) {
////        if (((minDistance < distanceSensorOutput) && (distanceSensorOutput < maxDistance))){
////            return true;
////        }
////        else {
////            return false;
////        }
//
//    }

//    /**
//     * Determines the distance between the distance sensor and the nearest object.
//     *
//     * @return distance as a double in meters.
//     */
////    public void getDistance() {
////        //return distanceSensor.getRange();
////    }
}

