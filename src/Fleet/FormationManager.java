package Fleet;

import utils.Vec3;

import java.util.ArrayList;
import java.util.List;

public class FormationManager {
    public static Vec3 computeForce(Drone drone, ArrayList<Drone> neighbourDrones, double posGain, double velGain) {
        Vec3 netForce = new Vec3();

        for (Drone nd: neighbourDrones) {
            Vec3 posTerm = Vec3.sub(drone.getCurrPos(), nd.getCurrPos()).mulScaler(posGain);
            Vec3 velTerm =  Vec3.sub(drone.getCurrVel(), nd.getCurrVel()).mulScaler(velGain);

            netForce.add(Vec3.add(posTerm, velTerm));
        }

        return netForce.mulScaler(-1);
    }
}
