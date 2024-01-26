package org.troyargonauts.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.troyargonauts.common.motors.MotorCreation;
import org.troyargonauts.common.motors.wrappers.LazyCANSparkMax;
import org.troyargonauts.robot.Constants;

public class Intake extends SubsystemBase {
    private TalonFX motor1;
    private DigitalInput noteSensor;
    public double motor1Encoder;
    public double motor2Encoder;
    public double motor3Encoder;



    double motor1Target = 0.0;
    final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);



    public Intake() {
        motor1 = new TalonFX(1);
        noteSensor = new DigitalInput(2);

        SmartDashboard.putNumber("motor1SetPoint", 0);
        SmartDashboard.putNumber("motor2SetPoint", 0);
        SmartDashboard.putNumber("motor3SetPoint", 0);
    }

    @Override
    public void periodic() {

        motor1Encoder = motor1.getVelocity().getValueAsDouble();
//        motor3Encoder = motor3.getVelocity().getValueAsDouble();

        motor1Target = SmartDashboard.getNumber("motor1SetPoint",0);

        SmartDashboard.putNumber("Motor1Error", (motor1Target - motor1.getVelocity().getValueAsDouble()) * 60);

        SmartDashboard.putNumber("Motor1RPM", (motor1.getVelocity().getValueAsDouble()) * 60);
    }




    public boolean isNoteReady() {
        if (noteSensor.get()) {
            return true;
        }
        else{
            return false;
        }


    }
    public void setRawPower(int motorID, double speed) {

        if (motorID == 1) {
            motor1.set(speed);
        }
    }
    enum state{
        IN,
        OFF,
        OUT
    }
   state noteState;

    public void setState(){

        switch (noteState){
            case IN:
                setRawPower(1,-0.3);
                break;

            case OFF:
                setRawPower(1,0);
                break;

            case OUT:
                setRawPower(1,0.3);
                break;
        }


    }

}
