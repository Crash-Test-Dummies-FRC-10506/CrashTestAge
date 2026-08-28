package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants.CameraConstants;

public class Photon extends SubsystemBase {
    PhotonPipelineResult result;
    PhotonCamera camera;
    PhotonTrackedTarget target;
    double yaw = 0.0;
    public Photon() {
        camera = new PhotonCamera(CameraConstants.kLimelightName);
    }

    @Override
    public void periodic() {
        if (camera.getLatestResult().hasTargets()) {
            for (PhotonTrackedTarget target : result.targets) {
                this.target = target;
                if (target.fiducialId == 16) {
                    yaw = target.getYaw();
                } else {
                    DriverStation.reportWarning("Couldn't get any specific april tags", false);
                }
            }
        } else {
            DriverStation.reportWarning("Camera didn't detect any apriltags", false);
        }
        SmartDashboard.putNumber("April Tag Yaw", yaw);
    }
    
}