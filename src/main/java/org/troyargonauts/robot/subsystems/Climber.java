package org.troyargonauts.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.troyargonauts.robot.Constants.Climber.*;

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
        climberMotor = new TalonFX(CLIMBER_MOTOR_PORT);
        climberMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Creates 3 different motor states (OFF, DOWN, and UP).
     */

    public enum MotorStates {
        UP, OFF
    }

    /**
     * Sets power to the motorController for each state (OFF has no power, DOWN and UP have a power value of 0.75 in their respective directions).
     * @param states: pulls the state from MotorStates.
     */

    public void setClimberMotor(MotorStates states){
        switch (states) {
            case OFF: climberMotor.set(CLIMBER_MOTOR_OFF_SPEED);
            break;
            case UP: climberMotor.set(CLIMBER_MOTOR_ON_SPEED);
            break;
        }
    }
}

