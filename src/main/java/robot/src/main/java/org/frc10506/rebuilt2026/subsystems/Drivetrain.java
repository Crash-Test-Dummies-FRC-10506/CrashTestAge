package robot.src.main.java.org.frc10506.rebuilt2026.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static robot.src.main.java.org.frc10506.rebuilt2026.util.IDs.*;

public class Drivetrain extends SubsystemBase {

    private final SparkMax FrontLeft;
    private final SparkMax RearLeft;
    private final SparkMax FrontRight;
    private final SparkMax RearRight;

    private final MecanumDrive m_drive;

    public Drivetrain() {

        FrontLeft = new SparkMax(FL_id, MotorType.kBrushless);
        RearLeft  = new SparkMax(RL_id, MotorType.kBrushless);
        FrontRight = new SparkMax(FR_id, MotorType.kBrushless);
        RearRight = new SparkMax(RR_id, MotorType.kBrushless);

        SparkMaxConfig globleConfig = new SparkMaxConfig();
        SparkMaxConfig FrontLeftConfig = new SparkMaxConfig();
        SparkMaxConfig RearLeftConfig = new SparkMaxConfig();
        SparkMaxConfig FrontRightConfig = new SparkMaxConfig();
        SparkMaxConfig RearRightConfig = new SparkMaxConfig();
// Invert the right side motors
    globleConfig
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake);

      FrontRightConfig
        .apply(globleConfig)
        .inverted(true);
     
      RearRightConfig
        .apply(globleConfig)
        .inverted(true);

       FrontLeftConfig
        .apply(globleConfig)
        .inverted(false);

       RearLeftConfig
        .apply(globleConfig)
        .inverted(false);  

    // You may need to change or remove this to match your robot.
  
    FrontLeft.configure(FrontLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    FrontRight.configure(FrontRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    RearLeft.configure(RearLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    RearRight.configure(RearRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    m_drive = new MecanumDrive(FrontLeft::set, RearLeft::set, FrontRight::set, RearRight::set);

    SendableRegistry.addChild(m_drive, FrontLeft);
    SendableRegistry.addChild(m_drive, RearLeft);
    SendableRegistry.addChild(m_drive, FrontRight);
    SendableRegistry.addChild(m_drive, RearRight);

    SmartDashboard.putData(m_drive);
    SmartDashboard.putData(m_field);
    }

    public void driveMechanum(double x, double y, double z) {
        m_drive.driveCartesian(x, y, z);
    }

    private Pose2d m_poseEstimate = new Pose2d();
    private final Field2d m_field = new Field2d();

    public void setPose(Pose2d pose) {
        m_poseEstimate = pose;
        m_field.setRobotPose(pose);
    }

    @Override
    public void periodic() {
        m_field.setRobotPose(m_poseEstimate);
    }

    //@Override
    //public void simulationPeriodic() { //AHHHHHHHHHHHHHHHHHHHHHHHH
    //    m_field.setRobotPose(m_poseEstimate);
    //}

    public Pose2d getPose() {
        return m_poseEstimate;
    }
    
}