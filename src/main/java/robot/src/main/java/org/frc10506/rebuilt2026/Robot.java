package robot.src.main.java.org.frc10506.rebuilt2026;

import static robot.src.main.java.org.frc10506.rebuilt2026.util.IDs.DRIVER_CONTROLLER;
import static robot.src.main.java.org.frc10506.rebuilt2026.util.IDs.OPERATOR_CONTROLLER;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import framework.src.main.java.org.frc10506.framework.AutoSelector;
import framework.src.main.java.org.frc10506.framework.PhaseDrivenRobot;
import framework.src.main.java.org.frc10506.framework.control.Controller;
import framework.src.main.java.org.frc10506.framework.scheduler.subsystem.TickedSubsystem;
import framework.src.main.java.org.frc10506.framework.scheduler.task.TaskPersistence;
import framework.src.main.java.org.frc10506.framework.scheduler.task.lock.LockPriority;
import robot.src.main.java.org.frc10506.rebuilt2026.commands.AutoCommands.DriveOnly;
import robot.src.main.java.org.frc10506.rebuilt2026.commands.AutoCommands.MainAuto;
import robot.src.main.java.org.frc10506.rebuilt2026.commands.DriveCommands.DriveLooped;
import robot.src.main.java.org.frc10506.rebuilt2026.commands.ShooterCommands.HighShootRawCommand;
import robot.src.main.java.org.frc10506.rebuilt2026.commands.ShooterCommands.IntaketoShooterCommand;
import robot.src.main.java.org.frc10506.rebuilt2026.commands.ShooterCommands.LowShootRawCommand;
import robot.src.main.java.org.frc10506.rebuilt2026.commands.ShooterCommands.SetShooterModeCommand;
import robot.src.main.java.org.frc10506.rebuilt2026.subsystems.Drivetrain;
import robot.src.main.java.org.frc10506.rebuilt2026.subsystems.Shooter;
import robot.src.main.java.org.frc10506.rebuilt2026.util.Constants;
import robot.src.main.java.org.frc10506.rebuilt2026.util.Constants.ShooterConstants;
import robot.src.main.java.org.frc10506.rebuilt2026.util.NetworkTables;

public final class Robot extends PhaseDrivenRobot {

    private final Controller driverController = new Controller(this.scheduler, DRIVER_CONTROLLER, 0.1);
    private final Controller operatorController = new Controller(this.scheduler, OPERATOR_CONTROLLER, 0.1);
    private final Drivetrain drivetrain = new Drivetrain();
    private final Shooter shooter = new Shooter();

    //private final ResetCommand resetCommand = new ResetCommand(drivetrain, intake, shooter, storage);
    // private final ReadyToRumbleCommand readyToRumbleCommand = new ReadyToRumbleCommand(driverController);

    private final IntaketoShooterCommand shooterIntake = new IntaketoShooterCommand(shooter);
    private final LowShootRawCommand lowShootRawCommand = new LowShootRawCommand(shooter);
    private final HighShootRawCommand highShootRawCommand = new HighShootRawCommand(shooter);
    private final SetShooterModeCommand lowShootCommand = new SetShooterModeCommand(shooter, 1);
    private final SetShooterModeCommand highShootCommand = new SetShooterModeCommand(shooter, 2); 

    private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
    private final NetworkTable table = this.nt.getTable("Auto");

    private final AutoSelector autoSelector = new AutoSelector()
        .add("Main", () -> new MainAuto(drivetrain, shooter))
        .add("Drive", () -> new DriveOnly(drivetrain));

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

    public Robot() {
        SmartDashboard.putNumber("Close Velocity (Near Hub)", ShooterConstants.closevelocity);
        SmartDashboard.putNumber("Far Velocity (Tower)", ShooterConstants.farvelocity);
     }

    private final StringPublisher autoPublisher = NetworkTables.PublisherFactory(
        this.table,
        "Profile",
        this.autoSelector.getProfiles().isEmpty() ? "" : this.autoSelector.getProfiles().
        get(0)
        .name()
    );

    private final StringSubscriber autoSubscriber = NetworkTables.SubscriberFactory(this.table, this.autoPublisher.getTopic());

    @Override
    public void autonomousSequence() {
        NetworkTables.SetPersistence(this.autoPublisher.getTopic(), true);
        String autoProfile = this.autoSubscriber.get();

        if (autoProfile == null || autoProfile.isEmpty()) {
            if (!this.autoSelector.getProfiles().isEmpty()) {
                autoProfile = this.autoSelector.getProfiles().get(0).name();
            }
        }

        var autoCommand = this.autoSelector.select(autoProfile);

        this.scheduler.scheduleAutoCommand(autoCommand);

        System.out.println(autoProfile);

    }

    @Override
    public void teleopSequence() {
        this.scheduler.scheduleDefaultCommand(
                new DriveLooped(
                    this.drivetrain, 
                    this.driverController.LEFT_Y_AXIS.negated(), 
                    this.driverController.LEFT_X_AXIS, 
                    this.driverController.RIGHT_X_AXIS
                ),
                TaskPersistence.GAMEPLAY
        );

        this.driverController.RIGHT_BUMPER.whileHeld(shooterIntake, TaskPersistence.GAMEPLAY);
        this.driverController.LEFT_BUMPER.whileHeld(highShootRawCommand, TaskPersistence.GAMEPLAY);
        this.driverController.RIGHT_BUMPER.whileHeld(lowShootRawCommand, TaskPersistence.GAMEPLAY);
        this.driverController.RIGHT_TRIGGER.button().whileHeld(lowShootCommand, TaskPersistence.GAMEPLAY);
        this.driverController.LEFT_TRIGGER.button().whileHeld(highShootCommand, TaskPersistence.GAMEPLAY);
    }

    @Override
    public void testSequence() {
        if(DriverStation.getAlliance().isPresent())
            this.drivetrain.setPose(DriverStation.getAlliance().get() == Alliance.Red ? 
                Constants.TestConstants.red :
                Constants.TestConstants.blue
        );
    }

    @Override
    protected void disabledSequence() {
        
    }
}
