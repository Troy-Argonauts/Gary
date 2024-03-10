package org.troyargonauts.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

import static org.troyargonauts.robot.Constants.Intake.*;

/**
 * Class representing Intake subsystem
 *
 * @author firearcher2012, SavageCabbage360, JJCgits, firelite2023
 */
public class Intake extends SubsystemBase {
    private TalonFX motorLeft;

    private TalonFX motorRight;

    private DigitalInput noteSensor;

    private DoubleLogEntry intakeMotorLeftVoltage;
    private DoubleLogEntry intakeOutputRightCurrentLog;
    private DoubleLogEntry intakeMotorRightVoltage;
    private DoubleLogEntry intakeOutputLeftCurrentLog;
    public BooleanSupplier noteReady;

    /**
     * Instantiates motor controllers and sensors; creates data logs
     */
    public Intake() {
        motorLeft = new TalonFX(LEFT_MOTOR_CAN_ID, CANBUS_NAME);
        motorRight = new TalonFX(RIGHT_MOTOR_CAN_ID, CANBUS_NAME);

        noteSensor = new DigitalInput(NOTE_SENSOR_SLOT);

        DataLog log = DataLogManager.getLog();
//
        intakeMotorLeftVoltage =  new DoubleLogEntry(log, "Intake Left Motor Bus Voltage log");
        intakeOutputRightCurrentLog =  new DoubleLogEntry(log, "Intake Right Motor Output Current log");
        intakeMotorRightVoltage =  new DoubleLogEntry(log, "Intake Right Motor Bus Voltage log");
        intakeOutputLeftCurrentLog =  new DoubleLogEntry(log, "Intake Left Motor Output Current log");
    }

    /**
     * Gets the state of the beam break sensor on the manipulator - indicates whether a note is ready to shoot
     *
     * @return Whether a note is ready to shoot
     */
    public boolean isNoteReady() {
        return !noteSensor.get();
    }


    /**
     * Append values to each data log periodically and output Note Sensor value to SmartDashboard
     */
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Note_Readiness",isNoteReady());
        intakeOutputRightCurrentLog.append(motorRight.getStatorCurrent().getValue());
        intakeMotorRightVoltage.append(motorRight.getMotorVoltage().getValue());
        intakeOutputLeftCurrentLog.append(motorLeft.getStatorCurrent().getValue());
        intakeMotorLeftVoltage.append(motorLeft.getMotorVoltage().getValue());
        noteReady = this::isNoteReady;
    }

    /**
     * Sets enumerators for various Intake States
     */
    public enum IntakeStates {
        /**
         * Intake rollers IN
         */
        IN,

        /**
         * Intake rollers OFF
         */
        OFF,

        /**
         * Intake rollers OUT
         */
        OUT
    }


    /**
     * Sets the power applied to the Intake motors depending on the provided IntakeState
     *
     * @param state State of the Intake
     */
    public void setState(IntakeStates state) {
        switch (state){
            case IN:
                motorRight.set(0.4);;
                motorLeft.set(0.4);;
                break;
            case OFF:
                motorRight.set(0);
                motorLeft.set(0);
                break;
            case OUT:
                motorRight.set(-0.4);
                motorLeft.set(-0.4);
                break;
        }
    }

}
