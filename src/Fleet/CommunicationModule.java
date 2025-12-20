package Fleet;

import utils.Vec3;

import java.util.ArrayList;

// Takes all the drones as input and finds out which drones can communicate with what.
public class CommunicationModule {
    public static ArrayList<Drone>[] computeNeighbouringDrones(Drone[] drones, double commRange, double packetLoss) {

        ArrayList<Drone>[] neighbourDrones = new ArrayList[drones.length];
        for (int i = 0 ; i < drones.length;i++) {
            neighbourDrones[i] = new ArrayList<Drone>();
        }

        // loop for each drone
        for (int i = 0; i < drones.length; i++) {
            for (int j = 0; i < drones.length; i++) {
                if (i == j) continue; // skipping adding a particular drone itself as it's neighbour

                // if we are in communication range
                if (Vec3.sub(drones[i].getCurrPos(), drones[j].getCurrPos()).getMagnitude() < commRange) {
                    // We only register drone as neighbour for our calculations if the current drone was able to
                    // receive information from the other drone. This happends when the packets were not lost midway.
                    if (Math.random() > packetLoss) {
                        neighbourDrones[i].add(drones[j]);
                    }
                }
            }
        }

        return neighbourDrones;
    }

//    List<Drone>[] neighbourDrones; // an array of lists of drones
//
//    public CommunicationModule(Drone[] drones, double commRange, double packetLoss) {
//        neighbourDrones = new List[drones.length];
//        for (int i = 0 ; i < drones.length;i++) {
//            neighbourDrones[i] = new ArrayList<Drone>();
//        }
//
//        // loop for each drone
//        for (int i = 0; i < drones.length; i++) {
//            for (int j = 0; i < drones.length; i++) {
//                if (i == j) continue; // skipping adding a particular drone itself as it's neighbour
//
//                // if we are in communication range
//                if (Vec3.subtract(drones[i].getCurrPos(), drones[j].getCurrPos()).getMagnitude() < commRange) {
//                    // We only register drone as neighbour for our calculations if the current drone was able to
//                    // receive information from the other drone. This happends when the packets were not lost midway.
//                    if (Math.random() > packetLoss) {
//                        neighbourDrones[i].add(drones[j]);
//                    }
//                }
//            }
//        }
//    }
//
//    public List<Drone>[] getNeighbourDrones() {
//        return neighbourDrones;
//    }
}