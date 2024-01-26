package org.troyargonauts.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {

    private final TalonFX climberMotor;

    public Climber() {
        climberMotor = new TalonFX(13);

        climberMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setRawPower()
    {
        climberMotor.set(0.5);
    }

    public enum MotorStates
    {
        UP, DOWN, OFF
    }
    public void setState(MotorStates state)
    {
        switch(state) {
            case UP:
                climberMotor.set(0.5);
                break;
            case DOWN:
                climberMotor.set(-0.5);
                break;
            case OFF:
                climberMotor.set(0);
                break;
        }
    }
}
