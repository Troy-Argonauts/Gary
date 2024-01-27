package org.troyargonauts.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    public void setRawPower( double speed) {
        motor.set(speed);
    }

    enum MotorState{
        IN,
        OFF,
        OUT
    }


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
