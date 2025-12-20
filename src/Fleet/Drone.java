package Fleet;

import utils.Matrix3x3;
import utils.Observer;
import utils.Vec3;

import java.lang.Math;
import java.util.ArrayList;

public class Drone extends Thread implements Observer {
    // characteristic attributes
    private int id;
    private double mass;
    private Vec3 currPos, finalPos, currVel, angVel;
    private float deltaT;
    private Matrix3x3 rotMat;

    // state attributes
    private FleetState state;
    private Simulator sim;
    private ArrayList<Drone> neighbourDrones;

    public Drone(int id, double mass, Vec3 initialPos, Vec3 finalPos, FleetState state, Simulator sim) {
        // set itself as an observer of state
        state.registerObserver(this);

        this.id = id;
        this.mass = mass;
        this.currPos = initialPos; // at object creation, curr position = initial position
        this.finalPos = finalPos;
        this.currVel = new Vec3(); // 0 linear velocity
        this.angVel = new Vec3();  // 0 angular velocity

        this.state = state;
        neighbourDrones = new ArrayList<>();
        this.rotMat = new Matrix3x3();
    }

    @Override
    public void update() {
        this.deltaT = state.getDeltaT();
        this.neighbourDrones = sim.getNeighbourDrones(id);
    }

    @Override
    public void run() {
        // the main logic of drone
        Vec3 formationForce = FormationManager.computeForce(this, neighbourDrones, sim.getConfigLoader().getPositionGainConst(),
                sim.getConfigLoader().getVelocityGainConst());
    }

    public Vec3 getCurrPos() {
        return currPos;
    }

    public Vec3 getCurrVel() {
        return this.currVel;
    }

    public Vec3 getFinalPos(){return finalPos;}

    public Simulator getSim() { return sim; }

    public double getMass() {
        return mass;
    }

    // returns angle in radians
    public double getYaw() {
        return Math.atan2(currVel.getY(), currVel.getX());
    }

    public Matrix3x3 getRotMat() {
        return rotMat;
    }

    public Vec3 getAngVel() {
        return angVel;
    }

//    public void getLinearAcc(Vec3 formationForce) {
//        // ma = mg + RT + F_aero + F_rep + F_form
//        Vec3 linearAcc =
//    }
}