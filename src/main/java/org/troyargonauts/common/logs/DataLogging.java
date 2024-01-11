package org.troyargonauts.robot.subsystems;

import java.io.File;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.FileWriter;
import java.io.IOException;

public class DataLogging extends SubsystemBase {
    private File logFile;
    private FileWriter logWriter;
    private Pigeon2 pigeon;

    public DataLogging() {
        logFile = new File("/home/lvuser/data_log.csv"); // Replace the path with the desired file location
        try {
            logWriter = new FileWriter(logFile);
            logWriter.write("Time,Yaw,Pitch,Roll,OtherData1,OtherData2\n"); // Add more columns as needed
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void logData(double[] yawPitchRoll, double otherData1, double otherData2) {
        // Log data to SmartDashboard
        SmartDashboard.putNumber("Yaw", yawPitchRoll[0]);
        SmartDashboard.putNumber("Pitch", yawPitchRoll[1]);
        SmartDashboard.putNumber("Roll", yawPitchRoll[2]);

        // Log data to the file
        try {
            logWriter.write(
                    System.currentTimeMillis() + "," + // Log the timestamp
                            yawPitchRoll[0] + "," +
                            yawPitchRoll[1] + "," +
                            yawPitchRoll[2] + "," +
                            otherData1 + "," +
                            otherData2 + "\n"
            );
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void close() {
        // Close the FileWriter when done logging
        try {
            logWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public Pigeon2 getPigeon() {
        return pigeon;
    }
}
