package org.troyargonauts.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.troyargonauts.robot.Constants;

/**
 * Class representing Intake system
 *
 * @author firearcher2012, SavageCabbage360
 */

public class Intake extends SubsystemBase {
    private TalonFX motor;
    private DigitalInput noteSensor;
    private DoubleLogEntry intakeMotorVoltage;
    private DoubleLogEntry intakeOutputCurrentLog;

    public Intake() {
        motor = new TalonFX(Constants.Intake.MOTOR_CAN_ID);
        noteSensor = new DigitalInput(Constants.Intake.NOTE_SENSOR_SLOT);

        SmartDashboard.putNumber("motor1SetPoint", 0);

        DataLog log = DataLogManager.getLog();
        intakeMotorVoltage =  new DoubleLogEntry(log, "Intake Bus Voltage log");
        intakeOutputCurrentLog =  new DoubleLogEntry(log, "Intake Output Current log");


    }




    public boolean isNoteReady() {
        return noteSensor.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Note_Readiness",isNoteReady());
        intakeOutputCurrentLog.append(motor.getMotorVoltage().getValue());
        intakeMotorVoltage.append(motor.getMotorVoltage().getValue());
    }




    /**
     * Sets power of intake
     * @param speed determines speed of motor.
     */
    public void setRawPower( double speed) {
        motor.set(speed);
    }

    public enum MotorState{
        IN,
        OFF,
        OUT
    }

    /**
     * Sets the state of the intake
     * @param State determines whether the Intake is going In, Out, or Off
     */
    public void setState(MotorState State){

        switch (State){
            case IN:
                setRawPower(-0.3);
                break;

            case OFF:
                setRawPower(0);
                break;

            case OUT:
                setRawPower(0.3);
                break;
        }

    }

}
