package friarLib2.vision;

import edu.wpi.first.networktables.NetworkTableInstance;


public class ApriltagCamera implements VisionCamera {

    @Override
    public boolean hasTargets() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(6) == 1;

    }

    @Override
    public VisionTarget[] getTargets() {

        VisionTarget target = new VisionTarget(
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0), 
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0), 
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0), 
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0));

        VisionTarget[] targets = {target};
        
        return targets;
    }

    @Override
    public VisionTarget getBestTarget() {
        return getTargets()[0];
    }

    @Override
    public void setPipeline(String pipelineName) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    }

    @Override
    public void setLights(LedMode mode) {
        switch (mode) {
            case on: NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); break;
            case off: NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); break;
            case blink: NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); break;
            case currentPipeline: NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); break;
        }
    }
}