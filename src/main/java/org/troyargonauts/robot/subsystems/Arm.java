package org.troyargonauts.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.troyargonauts.common.motors.wrappers.LazyTalonFX;

public class Arm extends SubsystemBase {

    private LazyTalonFX rightArmMotor;
    private LazyTalonFX leftArmMotor;
    private PIDController armPIDController;
    public Arm(){
        rightArmMotor = new LazyTalonFX();
        leftArmMotor = new LazyTalonFX();
        armPIDController = new PIDController();
    }

    public void resetEncoders(){
        rightArmMotor.setEncoderCounts(0);
        leftArmMotor.setEncoderCounts(0);
    }

    
}
