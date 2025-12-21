package Fleet;

import utils.Matrix3x3;
import utils.Vec3;

public class Controller {
    Drone d;

    // additional integral error accumalator
    private Vec3 integralError;
    private static final double INTEGRAL_MAX = 10.0; // MAX limit

    public Controller(Drone d) {
        this.d = d;
        this.integralError = new Vec3(0,0,0);
    }

    public Vec3 calcInertialThrust() {

        // error_p
        Vec3 errorPos = Vec3.sub(d.getFinalPos(), d.getCurrPos());

        // Accumalate integral error over time
        double dt = d.getDeltaT();
        integralError = integralError.add(errorPos.mulScaler(dt));

        // Anti-windup: cap the integral term to prevent it from growing too large
        double integralMag = integralError.getMagnitude();
        if (integralMag > INTEGRAL_MAX) {
            integralError = integralError.normalize().mulScaler(INTEGRAL_MAX);
        }

//        if (errorPos.getMagnitude() < 0.1) {
//            // Only reset horizontal integral, keep vertical for hovering
//            integralError = new Vec3(0, 0, integralError.getZ());
//        }

        // Calculate desired velocity
        double k_approach = 0.2;
        double distance = errorPos.getMagnitude();

        if (distance < 3.0) {
            k_approach = 0.15 * (distance / 3.0) + 0.05;
        }

        // velocity desired
        Vec3 velDesired = errorPos.mulScaler(k_approach);

        // capping the desired velocity (no idea about the calculation)
        if (velDesired.getMagnitude() > 2.0) {
            velDesired = velDesired.normalize().mulScaler(2.0);
        }

        // error Vel
        Vec3 errorVel = Vec3.sub(velDesired, d.getCurrVel());

        System.out.printf("""
                Error pos: %s,
                Error vel: %s
                """, errorPos, errorVel);

        // PID control law: a_cmd = k_p * e_p + k_i * integral(e_p) + k_d * e_v
        double integralGainConst = 1.0;  // k_i
        Vec3 proportionalTerm = errorPos.mulScaler(d.getSim().getConfigLoader().getPositionGainConst());
        Vec3 integralTerm = integralError.mulScaler(integralGainConst);
        Vec3 derivativeTerm = errorVel.mulScaler(d.getSim().getConfigLoader().getVelocityGainConst());

//        errorPos = errorPos.mulScaler(d.getSim().getConfigLoader().getPositionGainConst());
//        errorVel = errorVel.mulScaler(d.getSim().getConfigLoader().getVelocityGainConst());


        // Since we add gravity later while the net linear acceleration so removing gravity for now
//        Vec3 desErrorAcc = Vec3.add(errorPos, errorVel);
        Vec3 desErrorAcc =  Vec3.add(Vec3.add(proportionalTerm, integralTerm), derivativeTerm);

        // Inertial component of thrust
        desErrorAcc = desErrorAcc.mulScaler(d.getMass());
        return desErrorAcc;
    }

    public Vec3 calcNetThrust() {
        // Inertial component of thrust
        Vec3 inertialThrust = calcInertialThrust();

        // Thrust of the body
        Vec3 thrustBody = Matrix3x3.transpose(d.getRotMat()).mulVec(inertialThrust);

        return new Vec3(0, 0, thrustBody.getZ());
    }

    public Matrix3x3 computeTargetRotMat() {
        // inertial thrust unit vector
        Vec3 z_b = calcInertialThrust().normalize();

        // worldRef
        Vec3 x_c;
        if (Math.abs(z_b.getZ()) > 0.99) {
            // Thrust is nearly vertical - use x-axis as reference instead of yaw
            // This ensures we have a horizontal reference
            x_c = new Vec3(1, 0, 0);  // Point north

            // Alternative: maintain current body x-direction projected to horizontal
            // Vector3D current_x = R_current.getColumn(0);
            // x_c = new Vector3D(current_x.x, current_x.y, 0).normalize();
        } else {
            // Normal case - use yaw from velocity or target
            x_c = new Vec3(Math.cos(d.getYaw()), Math.sin(d.getYaw()), 0);
        }

        Vec3 y_b = z_b.cross(x_c).normalize();

        Vec3 x_b = y_b.cross(z_b).normalize();

        return new Matrix3x3(x_b, y_b, z_b);
    }

    public Vec3 computeTorque() {
        // first we extract the error vector from mapping of skew symmetric matrix
        // obtained from Vee operator.
        Matrix3x3 RErr = Matrix3x3.transpose(d.getRotMat()).mul(computeTargetRotMat());

        Vec3 e_R = new Vec3 (
            RErr.getC2().getZ() - RErr.getC3().getY(),
            RErr.getC3().getX() - RErr.getC1().getZ(),
            RErr.getC1().getY() - RErr.getC2().getX()
        ).mulScaler(0.5);

        // now we do τ= k_R e_R + k_w (w_target  - w)

        Vec3 wTarget = new Vec3();
        Vec3 angTerm = Vec3.sub(wTarget, d.getAngVel())
                .mulScaler(d.getSim().getConfigLoader().getAngularRateGain());

        Vec3 errorTerm = e_R.mulScaler(d.getSim().getConfigLoader().getAttitudeGain());

        return Vec3.add(errorTerm, angTerm);
    }

}
