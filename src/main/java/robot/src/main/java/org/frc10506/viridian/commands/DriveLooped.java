package robot.src.main.java.org.frc10506.viridian.commands;

import edu.wpi.first.wpilibj2.command.Command;
import framework.src.main.java.org.frc10506.framework.control.Axis;
import robot.src.main.java.org.frc10506.viridian.subsystems.MechanumDrive;

public class DriveLooped extends Command{

    private final MechanumDrive drive;

    private Axis driveForward;
    private Axis driveSideways;
    private Axis driveRotation;

    public DriveLooped(MechanumDrive drive, Axis driveForward, Axis driveSideways, Axis driveRotation) {
        this.drive = drive;
        this.driveForward = driveForward;
        this.driveSideways = driveSideways;
        this.driveRotation = driveRotation;
    }

   // public DriveLooped(MechanumDrive drive2, double d, int i, int j) {
        //TODO Auto-generated constructor stub
    //}

    @Override
    public void execute() {
        this.drive.driveMechanum(this.driveForward.get(), this.driveSideways.get(), this.driveRotation.get());
    }

}
