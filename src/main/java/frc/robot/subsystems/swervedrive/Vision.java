package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;

public class Vision extends SubsystemBase {
    
    public Vision() {
        PhotonCamera camR = new PhotonCamera("Right(FR)");
        PhotonCamera camL = new PhotonCamera("Left(FL)");
        
    }
}
