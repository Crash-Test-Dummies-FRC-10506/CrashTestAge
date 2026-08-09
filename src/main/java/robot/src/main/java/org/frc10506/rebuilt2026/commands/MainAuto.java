package robot.src.main.java.org.frc10506.viridian.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import robot.src.main.java.org.frc10506.viridian.commands.DriveAuto;
import robot.src.main.java.org.frc10506.viridian.commands.DriveLooped;
import robot.src.main.java.org.frc10506.viridian.commands.ShooterButVoltageCommand;
import robot.src.main.java.org.frc10506.viridian.commands.ShooterFeederCommand;
import robot.src.main.java.org.frc10506.viridian.commands.ShooterCommand;
import robot.src.main.java.org.frc10506.viridian.subsystems.Shooter;
import robot.src.main.java.org.frc10506.viridian.subsystems.MechanumDrive;

// TODO: refactor these so we don't have to use custom power to shut off shooter anymore
public class MainAuto extends SequentialCommandGroup {
  public MainAuto(MechanumDrive drivetrain, Shooter shooter) {
    addCommands(
        new DriveAuto(drivetrain, 0, -0.6, 0).withTimeout(0.31), // goes back for 0.31 seconds
        new ShooterCommand(shooter, 0.6), // power up flywheel to shoot
        new WaitCommand(3), // wait 3 secs
        new ShooterFeederCommand(shooter, 1).withTimeout(10), // shoots the balls with 10 seconds
        new ShooterCommand(shooter, 0) // stops the shooter
    );
  }
}