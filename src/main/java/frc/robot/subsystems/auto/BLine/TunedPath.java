package frc.robot.subsystems.auto.BLine;

import frc.robot.lib.BLine.Path;

public class TunedPath extends Path {

    private final double HANDOFF_THRESHOLD_METERS;
    private final Path bLinePath;

    public TunedPath(String path, double threshold) {
       this.bLinePath = new Path(path);

        this.HANDOFF_THRESHOLD_METERS = threshold;
    }

    public TunedPath(String path) {
         this.bLinePath = new Path(path);
        this.HANDOFF_THRESHOLD_METERS = 0.0;
    }


    public static  double getHandOffThreshold(TunedPath instance) {
        return instance.HANDOFF_THRESHOLD_METERS;
    }
    public static TunedPath of(String path, double threshold) {
    return new TunedPath(path, threshold);
}
public static Path getPath(TunedPath instance) {
    return instance.bLinePath;
}

}
