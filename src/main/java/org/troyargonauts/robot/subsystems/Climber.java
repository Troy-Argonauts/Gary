package org.troyargonauts.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    private TalonFX climberMotor;
    public Climber(){
        climberMotor = new TalonFX(13);
        climberMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    public enum MotorStates {
        UP, DOWN, OFF;
    }
    public void setClimberMotor(MotorStates states){
        switch (states) {
            case OFF: climberMotor.set(0);
            break;
            case DOWN: climberMotor.set(-0.75);
            case UP: climberMotor.set(0.75);

        }
    }
}

