package Fleet;

import utils.Observer;
import utils.Vec3;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class Logger implements Observer {
    private File positionsFile, metricsTxt;
    private BufferedWriter posWriter, metricWriter;
    private Simulator sim;
    private FleetState state;

    private int collisionCount;

    public Logger(Simulator sim, FleetState fleetState) throws IOException {
         fleetState.registerObserver(this);

         positionsFile= new File("positions.csv");
         metricsTxt = new File("metrics.txt");

         this.posWriter= new BufferedWriter(new FileWriter(positionsFile ));
         this.metricWriter = new BufferedWriter(new FileWriter(metricsTxt));
         this.sim = sim;

         collisionCount = 0;
    }

    public double computeAvgSpacing() {
        Drone[] drones = sim.getDrones();

        double netSpacing = 0;
        for (int i = 0; i < drones.length; i++) {
            for (int j = 0; j < drones.length; j++) {
                if (i == j) {
                    continue;
                }

                double dist = Vec3.sub(drones[i].getCurrPos(), drones[j].getCurrPos()).getMagnitude();
                if (dist < 2) { // here 2 is d_min
                    collisionCount++;
                }
                netSpacing += dist;
            }
        }

        return netSpacing / (drones.length * (drones.length - 1));
    }

    @Override
    public void update() {
        Drone[] drones = sim.getDrones();

        try {
            for (Drone d : drones) {
                posWriter.write(d.getCurrPos() + "," + d.getCurrVel() +  "," +  d.getController().calcNetThrust().getZ() + "\n");
            }

            // storing final summaries
            metricWriter.write("=================================================\n");
            metricWriter.write("Average Spacing: " + computeAvgSpacing() + "\n Collision Count: "
                    + collisionCount + "\n Communication SuccessRate: " + CommunicationModule.getSuccessfulComms() + "\n");
        }catch (Exception e) {
            System.out.println(e.getMessage());
        }

        resetCollisionCount();
        CommunicationModule.resetSuccessRate();
    }

    void resetCollisionCount() {
        collisionCount = 0;
    }

    public void close() throws IOException{
            posWriter.close();
            metricWriter.close();
    }
}
