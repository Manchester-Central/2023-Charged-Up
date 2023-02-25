package frc.robot.subsystems;

import java.util.concurrent.ConcurrentHashMap;

import com.fazecast.jSerialComm.SerialPort;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// TODO clean up code and prioritize readability.

public class ArduinoIO extends SubsystemBase {
    private final int NUM_BYTES_TO_RECEIVE = 12; // We know beforehand how many bytes we should receive each read cycle. Check color sensor documentation.

    private SerialPort m_arduino;
        private RGB m_rgbValues = new RGB(0, 0, 0);
        private Object mutexObject = new Object();

    public ArduinoIO() {
        m_arduino = SerialPort.getCommPort(Constants.CommConstants.arduinoPort);
    }

    @Override
    public void periodic() {
        m_arduino.openPort();
            if(m_arduino.bytesAvailable() >= 12) {
                

            }
            System.out.print("RED: ");
            System.out.println(getRGBValues().R);
            System.out.print("GREEN: ");
            System.out.println(getRGBValues().G);
            System.out.print("BLUE: ");
            System.out.println(getRGBValues().B);
    }

    public RGB getRGBValues() {
        synchronized(mutexObject) {
            return m_rgbValues;
        }
    }

    private int[] readAndInterpretColors() {
        byte[] incomingBytes = new byte[NUM_BYTES_TO_RECEIVE];
        m_arduino.readBytes(incomingBytes, NUM_BYTES_TO_RECEIVE);
        int green = (incomingBytes[3]&0xff)|((incomingBytes[4]&0xff)<<8);
        int blue = (incomingBytes[6]&0xff)|((incomingBytes[7]&0xff)<<8);
        int red = (incomingBytes[9]&0xff)|(((incomingBytes[10]&0xff)<<8));
        if(red > blue && red > green && red < 210) {
            red += 75;
        }
        synchronized(mutexObject) {
            m_rgbValues = new RGB(red, green, blue);
        }
        return new int[] {red, green, blue};
    }

    private RGB[] normalizeColors() {
        return null;
    }


    public class RGB {
        private int R = 0;
        private int G = 0;
        private int B = 0;
        public RGB(int iR, int iG, int iB) {
            R = iR;
            G = iG;
            B = iB;
        }
       
    }
}
