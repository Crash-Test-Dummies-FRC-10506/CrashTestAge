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
//import com.studica.frc.AHRS;
//import com.studica.frc.AHRS.NavXComType;

public class Drivetrain extends SubsystemBase {

    private final SparkMax FrontLeft;
    private final SparkMax RearLeft;
    private final SparkMax FrontRight;
    private final SparkMax RearRight;

    private final MecanumDrive m_drive;

    //private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

    public Drivetrain() {
        //m_gyro.reset();

        FrontLeft = new SparkMax(5, MotorType.kBrushless);
  RearLeft  = new SparkMax(4, MotorType.kBrushless);
  FrontRight = new SparkMax(3, MotorType.kBrushless);
  RearRight = new SparkMax(2, MotorType.kBrushless);

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
    }

    public void driveMechanum(double x, double y, double z) {
        m_drive.driveCartesian(x, y, z);
    }

    //private double xtuned = 0;
    //private double ytuned = 0;

    /*public Rotation2d getRot() {
        return Rotation2d.fromDegrees(m_gyro.getYaw());
    }

    public void fieldCentric(double x, double y, double z) {
        Rotation2d rot = getRot();
        
        xtuned = Math.max(Math.min(x, 1), -1);
        ytuned = Math.max(Math.min(y, 1), -1);

        m_drive.driveCartesian(xtuned, ytuned, z, rot);
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

    public Pose2d getPose() {
        return m_poseEstimate;
    }*/
    
}