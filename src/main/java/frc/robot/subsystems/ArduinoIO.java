package frc.robot.subsystems;

import java.io.IOException;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// TODO clean up code and prioritize readability.

public class ArduinoIO extends SubsystemBase {
    private final int NUM_BYTES_TO_RECEIVE = 2; // We know beforehand how many bytes we should receive each read cycle. Check color sensor documentation.
    private SerialPort m_arduino;
    private int numBytesLogged = 0;
    File logFile = new File("arduinolog.txt");
    FileWriter logFileWriter = new FileWriter();
    
    public ArduinoIO() {
        m_arduino = new SerialPort(9600, SerialPort.Port.kOnboard);
        if(logFile.exists() == false) {
            try {
                logFile.createNewFile();
            } catch(Exception e) {
                e.printStackTrace();
            }
        }
        
        try {
            logFile.
        }
        
        }
    }

    @Override
    public void periodic() {
       m_arduino.write(new byte[] {(byte) 255}, 1);
       
    }
}