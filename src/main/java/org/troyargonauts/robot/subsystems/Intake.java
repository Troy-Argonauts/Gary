package org.troyargonauts.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Class representing Intake system
 *
 * @author firearcher2012, SavageCabbage360
 */

public class Intake extends SubsystemBase {
    private TalonFX motor;
    private DigitalInput noteSensor;

    public Intake() {
        motor = new TalonFX(1);
        noteSensor = new DigitalInput(2);

        SmartDashboard.putNumber("motor1SetPoint", 0);

    }

    public boolean isNoteReady() {
        if (noteSensor.get()) {
            return true;
        }
        else{
            return false;
        }

    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Note_Readiness",isNoteReady());
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
