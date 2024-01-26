package org.troyargonauts.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.troyargonauts.common.motors.wrappers.LazyTalonFX;


public class Shooter extends SubsystemBase {
    private final TalonFX motor1, motor2;

    double motor1Target = 0.0;
    double motor2Target = 0.0;

    public Shooter(int talon){
        motor1 = new TalonFX(9);
        motor2 = new TalonFX(10);
    }

    public void toPower() {
        motor1.set(motor2Target);
        motor2.set(motor1Target);
    }

    public void setDesiredTarget(int MotorID, double target){
        if(MotorID == 9) {
            motor1Target = target;
        } else if(MotorID == 10) {
            motor2Target = target;
        }
    }

    public void setRawPower(int MotorID, double speed) {

        if(MotorID == 9) {
            motor1.set(speed);
        } else if (MotorID == 10) {
            motor2.set(speed);
        }
    }
}