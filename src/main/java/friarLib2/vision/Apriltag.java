package friarLib2.vision;

//Imports for PhotonVision
import org.photonvision.PhotonCamera;
import org.photonvision.common.*;
import org.photonvision.common.dataflow.*;
import org.photonvision.common.dataflow.structures.*;
import org.photonvision.common.hardware.*;
import org.photonvision.common.networktables.*;
import org.photonvision.targeting.*;

//WPILib imports for AprilTag
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.jni.AprilTagJNI;


public class Apriltag { 
    PhotonCamera camera = new PhotonCamera("photonvision"); 
    var result = camera.getLatestResult();
    
}
