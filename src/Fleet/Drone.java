package Fleet;

import utils.Matrix3x3;
import utils.Observer;
import utils.Vec3;

import java.lang.Math;
import java.util.ArrayList;

public class Drone implements Observer {
    // characteristic attributes
    private int id;
    private double mass;
    private Vec3 currPos, finalPos, currVel, angVel;
    private double deltaT;
    private Matrix3x3 rotMat;

    // state attributes
    private FleetState state;
    private Simulator sim;
    private Controller controller;
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
        this.rotMat = new Matrix3x3(); // rotation matrix initialized to I

        this.state = state;
        neighbourDrones = new ArrayList<>();
        this.controller = new Controller(this);
        this.sim = sim;
    }

    @Override
    public void update() {
        System.out.printf("""
                BEFORE UPDATING:
                CurLinVel: %s
                CurrPos:   %s
                -------------
                """, currVel, currPos);

        this.deltaT = state.getDeltaT();
        this.neighbourDrones = sim.getNeighbourDrones(id);


        Vec3 formationForce = FormationManager.computeForce(this, neighbourDrones, sim.getConfigLoader().getPositionGainConst(),
                sim.getConfigLoader().getVelocityGainConst());
        Vec3 aeroDragForce = computeAeroDrag();
        Vec3 repForce = CollisionAvoidance.computeRepForce(this, neighbourDrones);

        Vec3 linearAcc = computeLinearAcc(aeroDragForce, repForce, formationForce, controller.calcNetThrust());

        Vec3 newVel = getCurrVel().add(linearAcc.mulScaler(deltaT));

        setCurrVel(newVel);
        setCurrPos(getCurrPos().add(newVel.mulScaler(deltaT)));

        Vec3 angAcc = computeAngularAcc();
        setAngVel(getAngVel().add(angAcc.mulScaler(deltaT)));
        setRotMat(computeRotMat());

        System.out.printf("""
                        Drone ID: %d 
                        Curr Pos: %s  
                        Final Pos: %s  
                        Vel: %s
                        Acc: %s
                        AngVel: %s
                        angAcc: %s
                        """,
                id, currPos, finalPos, currVel, linearAcc, angVel, angAcc);
    }

    public Vec3 computeLinearAcc(Vec3 aeroDragForce, Vec3 repForce, Vec3 formationForce, Vec3 netThrust) {
        // ma = mg + RT + F_aero + F_rep + F_form

        Vec3 thrustProd = getRotMat().mulVec(netThrust);
        Vec3 linearAcc = aeroDragForce.add(repForce).add(formationForce).add(thrustProd).mulScaler(1.0 / mass);
        linearAcc = linearAcc.add(getSim().getConfigLoader().getGravConst());


        System.out.printf("""
                thrustProd    : %s,
                aeroDragForce : %s,
                repForce      : %s,
                formationForce: %s,
                netThrust     : %s
                """, thrustProd, aeroDragForce, repForce, formationForce, netThrust);

        return linearAcc;
    }

    public Vec3 computeAeroDrag() {
        return getCurrVel().mulScaler(-sim.getConfigLoader().getDragConst());
    }

    public Vec3 computeAngularAcc() {
        Vec3 torque = controller.computeTorque();
        Vec3 I = new Vec3(0.02, 0.02, 0.04);
        Vec3 w = getAngVel();

        System.out.println("Torque: " + torque);

        return  I.mulInv().mul(Vec3.sub(torque, w.cross(I.mul(w))));
    }

    public Matrix3x3 computeRotMat() {
        Vec3 w = getAngVel();

        // Now we use the Rodrigues' Formula
        double rotAngle = w.getMagnitude() * deltaT;
        Vec3 n = w.normalize(); // unit rot axis

        Matrix3x3 I = new Matrix3x3();
        Matrix3x3 unitRotMat = new Matrix3x3(
                0, -n.getZ(), n.getY(),
                n.getZ(), 0, -n.getX(),
                -n.getY(), n.getX(), 0
        );
        Matrix3x3 unitRotMatSq = unitRotMat.mul(unitRotMat);

        Matrix3x3 angExpMat;
        if (rotAngle < 1e-8) {
            angExpMat = I;
        }else {
            angExpMat = I.add(unitRotMat.mulScalar(Math.sin(rotAngle)))
                    .add( unitRotMatSq.mulScalar(1 - Math.cos(rotAngle)) );
        }

        return Matrix3x3.transpose(getRotMat()).mul(angExpMat);
    }


    public Vec3 getCurrPos() {
        return currPos;
    }

    public void setCurrPos(Vec3 currPos) {
        this.currPos = currPos;
    }

    public void setAngVel(Vec3 angVel) {
        this.angVel = angVel;
    }

    public Vec3 getCurrVel() {
        return this.currVel;
    }

    public void setRotMat(Matrix3x3 rotMat) {
        this.rotMat = rotMat;
    }

    public void setCurrVel(Vec3 currVel) {
        this.currVel = currVel;
    }

    public Vec3 getFinalPos() {
        return finalPos;
    }

    public Simulator getSim() {
        return sim;
    }

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

}