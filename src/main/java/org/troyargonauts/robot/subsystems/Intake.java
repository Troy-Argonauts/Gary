package org.troyargonauts.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
    private TalonFX motorTop;

    private TalonFX motorBottom;

    private DigitalInput noteSensor;

    private DoubleLogEntry intakeMotorTopVoltage;
    private DoubleLogEntry intakeOutputBottomCurrentLog;
    private DoubleLogEntry intakeMotorBottomVoltage;
    private DoubleLogEntry intakeOutputTopCurrentLog;
    public BooleanSupplier noteReady;
    public CurrentLimitsConfigs intakeConfigs;
    /**
     * Instantiates motor controllers and sensors; creates data logs
     */
    public Intake() {
        motorTop = new TalonFX(TOP_MOTOR_CAN_ID, CANBUS_NAME);
        motorBottom = new TalonFX(BOTTOM_MOTOR_CAN_ID, CANBUS_NAME);

        noteSensor = new DigitalInput(NOTE_SENSOR_SLOT);

        DataLog log = DataLogManager.getLog();

       intakeConfigs = new CurrentLimitsConfigs();
//       intakeConfigs.withStatorCurrentLimit(40);

//
        intakeMotorTopVoltage =  new DoubleLogEntry(log, "Intake Left Motor Bus Voltage log");
        intakeOutputBottomCurrentLog =  new DoubleLogEntry(log, "Intake Right Motor Output Current log");
        intakeMotorBottomVoltage =  new DoubleLogEntry(log, "Intake Right Motor Bus Voltage log");
        intakeOutputTopCurrentLog =  new DoubleLogEntry(log, "Intake Left Motor Output Current log");
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
        intakeOutputBottomCurrentLog.append(motorBottom.getTorqueCurrent().getValue());
  //      intakeOutputBottomCurrentLog.append(motorBottom.getStatorCurrent().getValue());
        intakeMotorBottomVoltage.append(motorBottom.getMotorVoltage().getValue());
        intakeOutputTopCurrentLog.append(motorTop.getTorqueCurrent().getValue());
 //       intakeOutputTopCurrentLog.append(motorTop.getStatorCurrent().getValue());
        intakeMotorTopVoltage.append(motorTop.getMotorVoltage().getValue());
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
                motorBottom.set(0.2);
                motorTop.set(-0.55);
                break;
            case OFF:
                motorBottom.set(0);
                motorTop.set(0);
                break;
            case OUT:
                motorBottom.set(-0.2);
                motorTop.set(0.55);
                break;
        }
    }

}
