import frc.robot.subsystems.SwerveModule; // Set the closestTarget function to public when testing.

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class SwerveModuleTest {
    @Test
    public void testAngle() {
        assertEquals(0, SwerveModule.closestTarget(0, 0));
        assertEquals(1800, SwerveModule.closestTarget(1800, 360));
        assertEquals(180, SwerveModule.closestTarget(90, 180));
        assertEquals(180, SwerveModule.closestTarget(0, 180));
        assertEquals(-180, SwerveModule.closestTarget(0, -180));
        assertEquals(-1800, SwerveModule.closestTarget(-1800, -360)); 
        assertEquals(-3600, SwerveModule.closestTarget(-3600, 360));
        assertEquals(3600, SwerveModule.closestTarget(3600, -720));
        assertEquals(3600.5, SwerveModule.closestTarget(3600.87, 0.5));
        assertEquals(390, SwerveModule.closestTarget(390, 30 ));
        assertEquals(230, SwerveModule.closestTarget(390, 590));
        
    }

    // "Inflation happens man" - JoshuaAllard2022 
  } 
