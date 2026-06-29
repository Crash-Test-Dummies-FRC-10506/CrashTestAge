package robot.src.main.java.org.frc10506.rebuilt2026.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import robot.src.main.java.org.frc10506.rebuilt2026.commands.DriveCommands.DriveAuto;
import robot.src.main.java.org.frc10506.rebuilt2026.commands.DriveCommands.DriveLooped;
import robot.src.main.java.org.frc10506.rebuilt2026.commands.ShooterCommands.IntaketoShooterCommand;
import robot.src.main.java.org.frc10506.rebuilt2026.commands.ShooterCommands.LowShootRawCommand;
import robot.src.main.java.org.frc10506.rebuilt2026.subsystems.Shooter;
import robot.src.main.java.org.frc10506.rebuilt2026.subsystems.Drivetrain;


public class DriveOnly extends SequentialCommandGroup {
  public DriveOnly(Drivetrain drivetrain) {
    addCommands(
       new DriveAuto(drivetrain, 0, -0.5, 0).withTimeout(2.5) // drives back for 2.5 seconds
    );
  }
}