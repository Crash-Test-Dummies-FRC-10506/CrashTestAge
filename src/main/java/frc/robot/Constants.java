// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kShooterControllerPort = 1;
  }

  public static class DrivetrainConstants {
    public static final int kFrontLeftMotorID = 5;
    public static final int kRearLeftMotorID = 4;
    public static final int kFrontRightMotorID = 3;
    public static final int kRearRightMotorID = 2;
  }

  public static class IntakeConstants {
    public static final int kIntakeMotorID = 6;
  }

  public static class ShooterConstants {
    public static final int kShooterMotorID = 7;
    public static final double kFeedForward = 0.5; // needs tuning
    public static final double kPID_Proportional = 0.5; // needs tuning
  }

  public static class AutoConstants { // all trials and errors so don't rely on this too much
    public static final Pose2d redLeft = new Pose2d(13.086, 0.539, new Rotation2d(0));
    public static final Pose2d redMiddle = new Pose2d(13.086, 3.998, new Rotation2d(0));
    public static final Pose2d redRight = new Pose2d(13.086, 7.584, new Rotation2d(0));
    public static final Pose2d blueLeft = new Pose2d(3.457, 0.539, new Rotation2d(0));
    public static final Pose2d blueMiddle = new Pose2d(3.457, 3.998, new Rotation2d(0));
    public static final Pose2d blueRight = new Pose2d(3.457, 7.584, new Rotation2d(0));
  }

  public static class CameraConstants {
    public static final String kLimelightName = "limelight-crash";
  }
}
