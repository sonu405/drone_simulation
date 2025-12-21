package Fleet;

import utils.Vec3;
import java.util.Random;

public class ConfigLoader {
    private double [] droneMasses;
    private double dragConst; // drag constant
    private double repulsionConst; // Repulsion constant
    private double positionGainConst; // position gain constant
    private double velocityGainConst; // velocity gain constant
    private double commRange;
    private double packetLoss;
    private double attitudeGain;
    private double angularRateGain;

    private int numOfDrones;


    // We would have a initial formation centre (C_init) and a final formation centre (C_final)
    // per drone offsets so
    // p_0_i = C_init + offset_i
    // p_d_i = C_final + offset_i
    private Vec3 initialFormationCentre;
    private Vec3 finalFormationCentre;
    private Vec3[] offsets;

    // CONSTANTS
    public final Vec3 g = new Vec3(0,0,-9.81F);

    public ConfigLoader() {
        Random rand = new Random();

        dragConst = rand.nextDouble(0.1 - 0.05) + 0.05; // 0.05 -> 0.1
        repulsionConst = rand.nextDouble(5 - 0.5) + 0.5; // 0.5 -> 5.0
//        positionGainConst = rand.nextDouble(6 - 1) + 1; // 1.0 -> 6.0
//        velocityGainConst = rand.nextDouble(4 - 1) + 1; // 1.0 -> 4.0
        positionGainConst = 3.0;
        velocityGainConst = 2.0;
//        commRange = rand.nextDouble(25 - 5) + 5; // 5.0 -> 25.0
        // AVOIDING calculation of collision forces by removing any communication
        commRange = 0;
        packetLoss = 0.05;
        angularRateGain = 0.15;
        attitudeGain    = 0.8;

        numOfDrones = 1;

        // To move in z direction only from origin
        initialFormationCentre = new Vec3(0,0,0);
        finalFormationCentre = new Vec3(0,0,10);
        offsets = new Vec3[numOfDrones];

        // one drone on x = 0, other on  x = 4
        offsets[0] = new Vec3(0,0,0);
//        offsets[1] = new Vec3(4,0,0);
//        offsets[2] = new Vec3(-10,0,0);
//        offsets[3] = new Vec3(-6,0,0);
//        offsets[4] = new Vec3(-2,0,0);
//        offsets[5] = new Vec3( 2,0,0);
//        offsets[6] = new Vec3( 6,0,0);
//        offsets[7] = new Vec3(10,0,0);
//        offsets[8] = new Vec3(14,0,0);
//        offsets[9] = new Vec3(18,0,0);

        // masses of drones
        droneMasses = new double[numOfDrones]; // 1 to 4.5 kg
        for (int i = 0; i < getNumOfDrones(); i++) {
            droneMasses[i] = rand.nextDouble(4.5-1) + 1;
        }
    }

    public Vec3 getInitialFormationCentre() {
        return initialFormationCentre;
    }

    public Vec3 getFinalFormationCentre() {
        return finalFormationCentre;
    }

    public Vec3[] getOffsets() {
        return offsets;
    }

    public double getDragConst() {
        return dragConst;
    }

    public int getNumOfDrones() {
        return numOfDrones;
    }

    public double getRepulsionConst() {
        return repulsionConst;
    }

    public double getPositionGainConst() {
        return positionGainConst;
    }

    public double getVelocityGainConst() {
        return velocityGainConst;
    }

    public double getComm_range() {
        return commRange;
    }

    public double getPacketLoss() {
        return packetLoss;
    }

    public Vec3 getGravConst() {
        return g;
    }

    public double[] getDroneMasses() {
        return droneMasses;
    }

    public double getAttitudeGain() { return attitudeGain; }

    public double getAngularRateGain() { return angularRateGain; }
}