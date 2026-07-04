package robot.src.main.java.org.frc10506.rebuilt2026.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Constants {

    public static class ShooterConstants {
        public static final double kP = 0.01;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.1;
        public static final double kV = 0.001;
        
        public static double farvelocity = 4500;
        public static double closevelocity = 3000;
        //public static double farvoltage = 10.5;
        //public static double closevoltage = 10.0;
    }

    public static class TestConstants {
        public static final Pose2d red = new Pose2d(13,4, new Rotation2d(90));
        public static final Pose2d blue = new Pose2d(3.58,4.02, new Rotation2d(0));
    }
}
