package org.troyargonauts.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.troyargonauts.common.motors.wrappers.LazyTalonFX;

public class Shooter extends SubsystemBase {
    private TalonFX motor1, motor2;
    double motor1Target = 0.0;
    double motor2Target = 0.0;
    public double motor1Encoder;
    public double motor2Encoder;
    public double motor3Encoder;

    double motor1P = 0.3;
    double motor1I = 0.1;
    double motor1D = 0;
    double motor2P = 0.3;
    double motor2I = 0.1;
    double motor2D = 0;
    double motor3P;
    double motor3I;
    double motor3D;
    final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);

    public void run() {
        motor1.setControl(velocityVoltage.withVelocity(motor1Target));
        motor2.setControl(velocityVoltage.withVelocity(motor2Target));
    }


    public Shooter(int talon){
        motor1 = new TalonFX(9);
        motor2 = new TalonFX(10);
    }

    public void setDesiredTarget(int motorID, double target){
        if (motorID == 9){
            motor1Target = target;
        } else if (motorID == 10){
            motor2Target = target;
        }
    }


}
