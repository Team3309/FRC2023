package friarLib2.vision;

//Imports from FIRST for Apriltag
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagDetector.Config;
import edu.wpi.first.apriltag.AprilTagDetector.QuadThresholdParameters;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.jni.AprilTagJNI;

//Imports for PhotonVision
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.common.dataflow.structures.*;
import org.photonvision.common.dataflow.*;
import org.photonvision.*;
import org.photonvision.common.hardware.*;
import org.photonvision.targeting.TargetCorner;
import org.photonvision.common.hardware.VisionLEDMode;

//General imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.nio.file.Path;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class Apriltag implements VisionCamera{

    
    private PhotonCamera camera;
    
    public Apriltag (String name) {
        camera = new PhotonCamera("photonvision");
    }

  /**
     * @return if the camera detects a target
     */
    @Override
    public boolean hasTargets() {
        return camera.getLatestResult().hasTargets();
    }

    /**
     * @return an array of visionTarget objects
     */
    @Override
    public VisionTarget[] getTargets() {
        //Get the list of targets
        PhotonPipelineResult result = camera.getLatestResult();
        List<PhotonTrackedTarget> targetList = result.getTargets();
        PhotonTrackedTarget target = result.getBestTarget();

        //Get info from target.
        double yaw = target.getYaw();
        double pitch = target.getPitch();
        double area = target.getArea();
        double skew = target.getSkew();
        Transform2d pose = target.getCameraToTarget();
        List<TargetCorner> corners = target.getCorners();



        // Get information from target.
        int targetID = target.getFiducialId();
        double poseAmbiguity = target.getPoseAmbiguity();
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

        return target;
    
    }

    /**
     * @return the object with index 0
     */
    @Override
    public VisionTarget getBestTarget() {
        try {
            return getTargets()[0];
        } catch (IndexOutOfBoundsException e) {
            return new VisionTarget();
        }
    }

      /**
     * @param pipeline the pipeline's index, use "driver mode" to enter driver mode
     */
    @Override
    public void setPipeline(String pipeline) {
        if (pipeline.equals("driver mode")) {
            camera.setDriverMode(true);
        } else {
            camera.setDriverMode(false);
            camera.setPipelineIndex(Integer.parseInt(pipeline));
        }
    }

    @Override
    public void setLights(LedMode mode) {
        switch (mode) {
            case on: camera.setLED(VisionLEDMode.kOn); break;
            case off: camera.setLED(VisionLEDMode.kOff); break;
            case blink: camera.setLED(VisionLEDMode.kBlink); break;
            case currentPipeline: camera.setLED(VisionLEDMode.kDefault); break;
        }
    }

    
} 