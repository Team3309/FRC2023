package friarLib2.vision;

import javax.naming.spi.DirStateFactory.Result;

//Imports for PhotonVision
import org.photonvision.PhotonCamera;
import org.photonvision.common.*;
import org.photonvision.common.dataflow.*;
import org.photonvision.common.dataflow.structures.*;
import org.photonvision.common.hardware.*;
import org.photonvision.common.networktables.*;
import org.photonvision.targeting.*;
import org.photonvision.PhotonUtils;

//WPILib imports for AprilTag
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.jni.AprilTagJNI;
import edu.wpi.first.cscore.CameraServerJNI;


public class Apriltag { 
    static PhotonCamera camera = new PhotonCamera("photonvision"); 
    public static void main(String[]args) {
        var result = camera.getLatestResult();
    }
    boolean hasTargets = result.hasTargets();
    


        
    
}
