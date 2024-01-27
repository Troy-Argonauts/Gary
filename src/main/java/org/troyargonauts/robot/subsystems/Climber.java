package org.troyargonauts.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Class representing climber subsystem.
 *
 * @author Nihaar57, SavageCabbage360, shaquilleinoatmeal
 */

public class Climber extends SubsystemBase {

    /**
     * Creates motorController object.
     */

    private TalonFX climberMotor;

    /**
     * Instantiates motorController object and sets the neutral mode to break.
     */

    public Climber(){
        climberMotor = new TalonFX(13);
        climberMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Creates 3 different motor states (OFF, DOWN, and UP).
     */

    public enum MotorStates {
        UP, DOWN, OFF;
    }

    /**
     * Sets power to the motorController for each state (OFF has no power, DOWN and UP have a power value of 0.75 in their respective directions).
     */

    public void setClimberMotor(MotorStates states){
        switch (states) {
            case OFF: climberMotor.set(0);
            break;
            case DOWN: climberMotor.set(-0.75);
            break;
            case UP: climberMotor.set(0.75);
            break;

        }
    }
}

