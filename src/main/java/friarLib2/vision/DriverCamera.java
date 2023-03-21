//package friarLib2.vision;

//import edu.wpi.first.networktables.NetworkTableInstance;
//
//public class DriverCamera implements IVisionCamera {
//
//    @Override
//    public boolean hasTargets() {
//    return NetworkTableInstance.getDefault().getTable("camera").getEntry("pipeline").getDouble(2) == 1;
//    }
//
//    @Override
//    public VisionTarget[] getTargets() {
//        return null;
//    }
//
//    @Override
//    public VisionTarget getBestTarget() {
//        return getTargets()[0];
//    }
//
//    @Override
//    public int getPipeline() {
//        return NetworkTableInstance.getDefault().getTable("camera").getEntry("+")
//    }
//
//}