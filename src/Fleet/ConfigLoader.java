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

        dragConst = 0.2;
        repulsionConst = 1;
        positionGainConst = 3.5;
        velocityGainConst = 2.5;
        commRange = 10;
        packetLoss = 0.05;
        angularRateGain = 0.15; // 0.25
        attitudeGain    = 8.5; // 1.5

        numOfDrones = 5;

        // To move in z direction only from origin
        initialFormationCentre = new Vec3(0,0,0);
        finalFormationCentre = new Vec3(150,90,100);
        offsets = new Vec3[numOfDrones];

        double radius = 50.0;  // Radius of the circle

        // circle formation
        for (int i = 0; i < numOfDrones; i++) {
            double angle = 2 * Math.PI * i / numOfDrones;
            double x = radius * Math.cos(angle);
            double z = radius * Math.sin(angle);
            offsets[i] = new Vec3(x, 0, z);
        }

        // masses of drones
        droneMasses = new double[numOfDrones]; // 1 to 2.5 kg
        for (int i = 0; i < getNumOfDrones(); i++) {
            droneMasses[i] = rand.nextDouble(2.5-1) + 1;
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