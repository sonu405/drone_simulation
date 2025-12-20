package Fleet;

import utils.Vec3;

import java.util.ArrayList;

public class CollisionAvoidance {
    public static Vec3 computeRepForce(Drone d , ArrayList<Drone> neighbourDrones) {
        Vec3 netRepForce = new Vec3();

        for (Drone nd: neighbourDrones) {
            Vec3 diff = Vec3.sub(d.getCurrPos(), nd.getCurrPos());
            double k_rep =  d.getSim().getConfigLoader().getRepulsionConst();

            netRepForce = netRepForce.add(diff.mulScaler(
                    k_rep /
                    Math.pow(diff.getMagnitude(), 2)
            ));
        }

        return netRepForce;
    }
}
