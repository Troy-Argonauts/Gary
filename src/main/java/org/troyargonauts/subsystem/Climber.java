package org.troyargonauts.subsystem;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {

    private final TalonFX climberMotor;

    public Climber() {
        climberMotor = new TalonFX(13);
    }

    public void setRawPower()
    {
        climberMotor.set(0.5);
    }

    public enum MotorStates
    {
        UP, DOWN, INPLACE, OFF;
    }
    public void setState(MotorStates state)
    {
        switch(state) {
            case UP:
                climberMotor.set
            case DOWN:

            case INPLACE:

            case OFF

        }
    }
}
