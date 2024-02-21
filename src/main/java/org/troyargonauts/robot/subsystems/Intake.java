package org.troyargonauts.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.troyargonauts.robot.Constants.Intake.*;

/**
 * Class representing Intake system
 *
 * @author firearcher2012, SavageCabbage360, JJCgits, firelite2023
 */
public class Intake extends SubsystemBase {
    private TalonFX motor;

    private DigitalInput noteSensor;

    private DoubleLogEntry intakeMotorVoltage;
    private DoubleLogEntry intakeOutputCurrentLog;

    /**
     * Makes a new intake with a motor and a note sensor
     */
    public Intake() {
        motor = new TalonFX(MOTOR_CAN_ID, CANBUS_NAME);

        noteSensor = new DigitalInput(NOTE_SENSOR_SLOT);

//        DataLog log = DataLogManager.getLog();
//
//        intakeMotorVoltage =  new DoubleLogEntry(log, "Intake Bus Voltage log");
//        intakeOutputCurrentLog =  new DoubleLogEntry(log, "Intake Output Current log");
    }

    /**
     * @return a boolean (true if the note is ready and false if the note isn't)
     */
    public boolean isNoteReady() {
        return !noteSensor.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Note_Readiness",isNoteReady());
//        intakeOutputCurrentLog.append(motor.getSupplyCurrent().getValue());
      //  intakeMotorVoltage.append(motor.getMotorVoltage().getValue());
    }

    /**
     * Makes an enum for the 3 states the motor could be (In, Out, or Off)
     */
    public enum IntakeStates {
        IN,
        OFF,
        OUT
    }

    /**
     * Sets the state of the intake
     * @param State determines whether the Intake is going In, Out, or Off
     */
    public void setState(IntakeStates state) {
        switch (state){
            case IN:
                motor.set(-0.3);;
                break;
            case OFF:
                motor.set(0);
                break;
            case OUT:
                motor.set(0.3);
                break;
        }
    }
}
