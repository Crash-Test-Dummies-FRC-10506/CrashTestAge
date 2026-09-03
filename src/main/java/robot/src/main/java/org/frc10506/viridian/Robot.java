package robot.src.main.java.org.frc10506.viridian;
// I devoured shake shack's cheese sauce while working on this midnight

import framework.src.main.java.org.frc10506.framework.AutoSelector;
import framework.src.main.java.org.frc10506.framework.PhaseDrivenRobot;
import framework.src.main.java.org.frc10506.framework.control.Controller;
import framework.src.main.java.org.frc10506.framework.scheduler.task.Task;
import framework.src.main.java.org.frc10506.framework.scheduler.task.TaskPersistence;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import robot.src.main.java.org.frc10506.viridian.commands.DriveLooped;
import robot.src.main.java.org.frc10506.viridian.commands.MainAuto;
import robot.src.main.java.org.frc10506.viridian.commands.ShooterButVoltageCommand;
import robot.src.main.java.org.frc10506.viridian.commands.ShooterCommand;
import robot.src.main.java.org.frc10506.viridian.commands.ShooterFeederCommand;
import robot.src.main.java.org.frc10506.viridian.commands.ShooterOnly;
import robot.src.main.java.org.frc10506.viridian.subsystems.MechanumDrive;
import robot.src.main.java.org.frc10506.viridian.subsystems.Shooter;
import robot.src.main.java.org.frc10506.viridian.util.NetworkTables;

import static robot.src.main.java.org.frc10506.viridian.util.IDs.*;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public final class Robot extends PhaseDrivenRobot {


    private final Controller driverController = new Controller(this.scheduler, DRIVER_CONTROLLER, 0.1);
    private final Controller operatorController = new Controller(this.scheduler, OPERATOR_CONTROLLER, 0.1);

    private final MechanumDrive drive = new MechanumDrive();
    private final Shooter shooter = new Shooter();

    private final ShooterCommand shooterCommand = new ShooterCommand(shooter, 0.6);
    private final ShooterCommand shooterFullCommand = new ShooterCommand(shooter, 0.75);
    private final ShooterCommand unjamShooterCommand = new ShooterCommand(shooter, -1);
    private final ShooterFeederCommand unjamFeederCommand = new ShooterFeederCommand(shooter, 1);
    private final ShooterFeederCommand shooterFeederCommand = new ShooterFeederCommand(shooter, -1);
    private final ShooterButVoltageCommand shooterButVoltageCommand = new ShooterButVoltageCommand(shooter, 6.25); // I think this is a good voltage lololol
    private final ShooterButVoltageCommand highshooterButVoltageCommand = new ShooterButVoltageCommand(shooter, 7.4);
    private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    private void configureAutos() {
        m_autoChooser.addOption("Main Auto", new MainAuto(drive, shooter));
        SmartDashboard.putData("Auto Chooser", m_autoChooser);
    }

    public Robot() {
        configureAutos();
    }

    /*private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
    private final NetworkTable table = this.nt.getTable("Auto");

    private final AutoSelector autoSelector = new AutoSelector()
        .add("Main", () -> new MainAuto(drive, shooter))
        .add("Shoot", () -> new ShooterOnly(shooter));

    {
        {
            var profiles = new String[this.autoSelector.getProfiles().size()];
            for (var i = 0; i < profiles.length; i++) {
                profiles[i] = this.autoSelector.getProfiles().get(i).name();
            }

            var autoChoicesPub = NetworkTables.PublisherFactory(this.table, "Choices", profiles);
            autoChoicesPub.accept(profiles);
        }
    }

    private final StringPublisher autoPublisher = NetworkTables.PublisherFactory(
        this.table,
        "Profile",
        this.autoSelector.getProfiles().isEmpty() ? "" : this.autoSelector.getProfiles().
        get(0)
        .name()
    );

    private final StringSubscriber autoSubscriber = NetworkTables.SubscriberFactory(this.table, this.autoPublisher.getTopic());
    */

    @Override
    public void autonomousSequence() {
        /*NetworkTables.SetPersistence(this.autoPublisher.getTopic(), true);
        String autoProfile = this.autoSubscriber.get();

        if (autoProfile == null || autoProfile.isEmpty()) {
            if (!this.autoSelector.getProfiles().isEmpty()) {
                autoProfile = this.autoSelector.getProfiles().get(0).name();
            }
        }

        var autoCommand = this.autoSelector.select(autoProfile);

        this.scheduler.scheduleAutoCommand(autoCommand);

        System.out.println(autoProfile);
        */
        this.scheduler.scheduleAutoCommand(m_autoChooser.getSelected());
        System.out.println(m_autoChooser.getSelected());
    }

    @Override
    public void teleopSequence() {
        this.scheduler.scheduleDefaultCommand(
            new DriveLooped(
                drive, 
                this.driverController.LEFT_Y_AXIS.negated(), 
                this.driverController.LEFT_X_AXIS, 
                this.driverController.RIGHT_X_AXIS
            ), TaskPersistence.GAMEPLAY
        );

        this.operatorController.RIGHT_BUMPER.whileHeld(highshooterButVoltageCommand, TaskPersistence.GAMEPLAY);
        this.driverController.RIGHT_BUMPER.whileHeld(shooterFeederCommand, TaskPersistence.GAMEPLAY);
        this.operatorController.LEFT_BUMPER.whileHeld(shooterButVoltageCommand, TaskPersistence.GAMEPLAY);
        this.operatorController.A.whileHeld(unjamFeederCommand, TaskPersistence.GAMEPLAY);
        this.operatorController.B.whileHeld(unjamShooterCommand, TaskPersistence.GAMEPLAY);
    }

    @Override
    public void testSequence() {
    }

    @Override
    protected void disabledSequence() {

    }
}
