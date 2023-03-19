package friarLib2.vision;

import edu.wpi.first.networktables.NetworkTableInstance;

public class DriverCamera implements IVisionCamera {

    @Override
    public boolean hasTargets () {
        return NetworkTableInstance.getDefault().getTable("Limelight").getEntry("pipeline").getDouble(0)==2;
    }

    @Override
    public VisionTarget[] getTargets() {
        return null;
    }

    @Override
    public VisionTarget getBestTarget() {
        return getTargets()[0];
    }

    @Override
    public int getPipeline() {
        return NetworkTableInstance.getDefault().getTable("Limelight").getEntry("pipeline").getNumber(-1).intValue();
    }

    @Override
    public void setPipeline(int index) {
        NetworkTableInstance.getDefault().getTable("Limelight").getEntry("pipeline").setNumber(index);
    }

    @Override
    public void setLights(LedMode mode) {
        switch (mode) {
            case off:

        }
    }

    @Override
    public int camMode() {
        NetworkTableInstance.getDefault().getTable("Limelight").getEntry("pipeline").getNumber(1);
    }
}

