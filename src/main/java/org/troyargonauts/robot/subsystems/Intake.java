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
    private TalonFX motorLeft;

    private TalonFX motorRight;

    private DigitalInput noteSensor;

    private DoubleLogEntry intakeMotorLeftVoltage;
    private DoubleLogEntry intakeOutputRightCurrentLog;
    private DoubleLogEntry intakeMotorRightVoltage;
    private DoubleLogEntry intakeOutputLeftCurrentLog;

    /**
     * Makes a new intake with a motor and a note sensor
     */
    public Intake() {
        motorLeft = new TalonFX(MOTOR_CAN_ID, CANBUS_NAME);
        motorRight = new TalonFX(MOTOR_CAN_ID2, CANBUS_NAME2);

        noteSensor = new DigitalInput(NOTE_SENSOR_SLOT);

        DataLog log = DataLogManager.getLog();

        intakeMotorLeftVoltage =  new DoubleLogEntry(log, "Intake Left Motor Bus Voltage log");
        intakeOutputRightCurrentLog =  new DoubleLogEntry(log, "Intake Right Motor Output Current log");
        intakeMotorRightVoltage =  new DoubleLogEntry(log, "Intake Right Motor Bus Voltage log");
        intakeOutputLeftCurrentLog =  new DoubleLogEntry(log, "Intake Left Motor Output Current log");
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
        intakeOutputRightCurrentLog.append(motorRight.getSupplyCurrent().getValue());
        intakeMotorRightVoltage.append(motorRight.getMotorVoltage().getValue());
        intakeOutputLeftCurrentLog.append(motorLeft.getSupplyCurrent().getValue());
        intakeMotorLeftVoltage.append(motorLeft.getMotorVoltage().getValue());
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
     * @param state determines whether the Intake is going In, Out, or Off
     */
    public void setState(IntakeStates state) {
        switch (state){
            case IN:
                motorRight.set(-0.3);;
                motorLeft.set(-0.3);;
                break;
            case OFF:
                motorRight.set(0);
                motorLeft.set(0);
                break;
            case OUT:
                motorRight.set(0.3);
                motorLeft.set(0.3);
                break;
        }
    }

}
