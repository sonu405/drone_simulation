package Fleet;

import utils.Observer;
import utils.Vec3;

import java.util.List;
import java.util.ArrayList;

public class Simulator implements Observer {
    private FleetState state;
    ArrayList<Drone>[] neighbourDrones;
    private Drone[] drones;
    private ConfigLoader cl;

    public Simulator(FleetState state, ConfigLoader cl) {
        this.state = state;
        drones = new Drone[cl.getNumOfDrones()];


        Vec3[] offsets = cl.getOffsets();
        double[] droneMasses = cl.getDroneMasses();

        // making each drone after initializing their initial and final positions.
        for (int i = 0; i < cl.getNumOfDrones(); i++) {
            Vec3 initialPos = Vec3.add(cl.getInitialFormationCentre(), offsets[i]);
            Vec3 finalPos   = Vec3.add(cl.getFinalFormationCentre(), offsets[i]);

            drones[i] = new Drone(i, droneMasses[i], initialPos, finalPos, state, this);
        }
    }

    @Override
    public void update() {
        // main logic of updating values with the state change.

        // First step is finding out all the drones that are in the range of each other.
        // For this, we use the Communication Module class.
        neighbourDrones = CommunicationModule.computeNeighbouringDrones(drones, cl.getComm_range(), cl.getPacketLoss());
    }

    public ArrayList<Drone> getNeighbourDrones(int index) {
        return neighbourDrones[index];
    }

    public ConfigLoader getConfigLoader() {
        return cl;
    }
}