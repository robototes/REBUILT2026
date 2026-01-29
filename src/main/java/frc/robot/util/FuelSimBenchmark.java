package frc.robot.util;

import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.lang.reflect.Method;
import java.lang.reflect.Field;

public class FuelSimBenchmark {
    public static void main(String[] args) {
        try {
            FuelSim sim = FuelSim.getInstance();
            sim.clearFuel();

            // Spawn 400 fuels randomly
            for (int i = 0; i < 400; i++) {
                double x = Math.random() * 16.0;
                double y = Math.random() * 8.0;
                sim.spawnFuel(new Translation3d(x, y, 0.075), new Translation3d());
            }

            // Access private fuels list using reflection
            Field fuelsField = FuelSim.class.getDeclaredField("fuels");
            fuelsField.setAccessible(true);
            ArrayList<?> fuels = (ArrayList<?>) fuelsField.get(sim);

            // Access private handleFuelCollisions method
            Method handleCollisions = FuelSim.class.getDeclaredMethod("handleFuelCollisions", ArrayList.class);
            handleCollisions.setAccessible(true);

            long startTime = System.nanoTime();
            int iterations = 1000;

            System.out.println("Starting benchmark with " + fuels.size() + " fuels for " + iterations + " iterations...");

            for (int i = 0; i < iterations; i++) {
                handleCollisions.invoke(null, fuels);
            }

            long endTime = System.nanoTime();
            double durationMs = (endTime - startTime) / 1e6;

            System.out.println("Total time: " + durationMs + " ms");
            System.out.println("Average time per iteration: " + (durationMs / iterations) + " ms");

        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
