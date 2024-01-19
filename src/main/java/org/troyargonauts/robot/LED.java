package org.troyargonauts.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.module.Configuration;
import com.ctre.phoenix.led.FireAnimation;

public class LED extends SubsystemBase {

    public static final int pink = 252;
    public static final int pink2 = 3;
    public static final int pink3 = 207;
    private final CANdle candle;
    public CANdleConfiguration config;
    public LED (int deviceId) {
        candle = new CANdle(deviceId);
        config = new CANdleConfiguration();
        config.stripType = CANdle.LEDStripType.RGB;
        candle.configAllSettings(config);
    }
    public void fire(int ledLength) {
        config.brightnessScalar = 0.5;
        candle.configAllSettings(config);
        FireAnimation fireAnimation = new FireAnimation(1, 1, ledLength, 0.5, 0.4);
        candle.animate(fireAnimation);
    }
    public void color() {
        candle.setLEDs(pink, pink2, pink3);
    }
    public void ledOff() {
        candle.setLEDs(0, 0, 0);
    }

}
