import frc.robot.subsystems.SwerveModule; // Set the closestTarget function to public when testing.

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class SwerveModuleTest {
    public static final double DELTA = 0; // acceptable deviation range
  
    
  
    public double clampAngle(double angle) {
        return Math.IEEEremainder(angle, 360);
    }

    public static double closestTarget(double currentModuleAngle, double targetAngle) {
        double angleOffset = Math.floor(currentModuleAngle / 360) * 360;
        targetAngle += angleOffset; // Produces an equivalent angle that is closer to current encoder value
        double d = (targetAngle >= currentModuleAngle) ? -1 : 1;
        double alternateTargetAngle = targetAngle + (d * 360);
    
        double currentToTarget = targetAngle - currentModuleAngle;
        double currentToAlternateTarget = alternateTargetAngle - currentModuleAngle;
    
        if(Math.abs(currentToTarget) < Math.abs(currentToAlternateTarget)) {
          return currentModuleAngle + currentToTarget;
        } else if(Math.abs(currentToTarget) <= Math.abs(currentToAlternateTarget)){
          return currentModuleAngle + currentToAlternateTarget;
        } else {
          return currentModuleAngle;
        }
    }
    @Test
    public void testAngle() {
        assertEquals(SwerveModule.closestTarget(0, 0), 0);
        assertEquals(SwerveModule.closestTarget(1800, 360), 1800);
        assertEquals(SwerveModule.closestTarget(90, 180), 180);
        assertEquals(SwerveModule.closestTarget(0, 180), 180);
        assertEquals(SwerveModule.closestTarget(0, -180), -180);
        assertEquals(-1800, closestTarget(-1800, -360));
        assertEquals(-3600, closestTarget(-3600, 360));
        assertEquals(3600, closestTarget(3600, -720));
        assertEquals(3600.5, closestTarget(3600.87, 0.5));
        
    }
  
    // "Inflation happens man" - JoshuaAllard2022
  } 
