package org.troyargonauts.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.troyargonauts.common.motors.wrappers.LazyTalonFX;

public class Shooter extends SubsystemBase {
    private LazyTalonFX motor1, motor2;

    double motor1P = 0.3;
    double motor1I = 0.1;
    double motor1D = 0;
    double motor1Target = 0;

    public Shooter(int talon){
        motor1 = new LazyTalonFX(9);
        motor2 = new LazyTalonFX(10);
    }
    public void setDesiredTarget(int motorID, double target){
    }

    public boolean isPidFinished(int motorID){
        
    }


}
