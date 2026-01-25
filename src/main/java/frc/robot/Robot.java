// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    if(DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) { // I'm sorry if this format is downgrade but for specific side this is kinda needed
        if (DriverStation.getRawAllianceStation() == AllianceStationID.Red1) {
          m_robotContainer.m_drive.setPose(Constants.AutoConstants.redLeft);
        } else if (DriverStation.getRawAllianceStation() == AllianceStationID.Red2) {
          m_robotContainer.m_drive.setPose(Constants.AutoConstants.redMiddle);
        } else if (DriverStation.getRawAllianceStation() == AllianceStationID.Red3) {
          m_robotContainer.m_drive.setPose(Constants.AutoConstants.redRight);
        } else {
          m_robotContainer.m_drive.setPose(Constants.AutoConstants.redMiddle);
        }
      } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
        if (DriverStation.getRawAllianceStation() == AllianceStationID.Blue1) {
          m_robotContainer.m_drive.setPose(Constants.AutoConstants.blueLeft);
        } else if (DriverStation.getRawAllianceStation() == AllianceStationID.Blue2) {
          m_robotContainer.m_drive.setPose(Constants.AutoConstants.blueMiddle);
        } else if (DriverStation.getRawAllianceStation() == AllianceStationID.Blue3) {
          m_robotContainer.m_drive.setPose(Constants.AutoConstants.blueRight);
        } else {
          m_robotContainer.m_drive.setPose(Constants.AutoConstants.blueMiddle);
        }
      } else { // usually returns invalid
        m_robotContainer.m_drive.setPose(Constants.AutoConstants.redMiddle);
      }
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Calculate drivetrain commands from Joystick values
    double pwr = Constants.Drivetrain.kMechanumPower;
    double forward = -controller.getLeftY()  * pwr;
    double strafe  = -controller.getLeftX()  * pwr;
    double turn    = -controller.getRightX() * pwr;

    // Read in relevant data from the Camera
    boolean targetVisible = false;
    double targetYaw = 0.0;
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        // At least one AprilTag was seen by the camera
        for (var target : result.getTargets()) {
          if (target.getFiducialId() == 7) {
            // Found Tag 7, record its information
            targetYaw = target.getYaw();
            targetVisible = true;
          }
        }
      }
    }

    // Auto-align when requested
    if (controller.getAButton() && targetVisible) {
      // Driver wants auto-alignment to tag 7
      // And, tag 7 is in sight, so we can turn toward it.
      // Override the driver's turn command with an automatic one that turns toward the tag.
      turn = -1.0 * targetYaw * VISION_TURN_kP * pwr;
    }

    // Command drivetrain motors based on target speeds
    drivetrain.drive(forward, strafe, turn);

    // Put debug information to the dashboard
    SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
