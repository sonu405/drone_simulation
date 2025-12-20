package Fleet;

import utils.Matrix3x3;
import utils.Vec3;

public class Controller {
    Drone d;

    public Controller(Drone d) {
        this.d = d;
    }

    public Vec3 calcInertialThrust() {
        // NOTE: Hard coded to 0
        Vec3 velDesired = new Vec3();

        // error_p
        Vec3 errorPos = Vec3.sub(d.getFinalPos(), d.getCurrPos());
        Vec3 errorVel = Vec3.sub(velDesired, d.getCurrVel());

        // desire Linear acceleration
        errorPos = errorPos.mulScaler(d.getSim().getConfigLoader().getPositionGainConst());
        errorVel = errorVel.mulScaler(d.getSim().getConfigLoader().getVelocityGainConst());

        Vec3 desLinearAcc = Vec3.add(
                Vec3.add(errorPos, errorVel),
                d.getSim().getConfigLoader().getGravConst()
        );

        // Inertial component of thrust
        desLinearAcc = desLinearAcc.mulScaler(d.getMass());
        return desLinearAcc;
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
