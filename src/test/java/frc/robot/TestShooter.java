package test.java.frc.robot;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import robot.src.main.java.org.frc10506.rebuilt2026.subsystems.Shooter;
import com.revrobotics.spark.SparkMaxSim;
import edu.wpi.first.hal.HAL;

public class TestShooter {
    Shooter shooter;
    SparkMaxSim fakeshootermotor;

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        shooter = new Shooter();
        fakeshootermotor =
            new SparkMaxSim();
  }
}


