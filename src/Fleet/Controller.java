package Fleet;

import utils.Matrix3x3;
import utils.Vec3;

public class Controller {
    Drone d;

    public Controller (Drone d) {
        this.d = d;
    }

    public Vec3 calcInertialThrust() {
        Vec3 velDesired= new Vec3();

        // error_p
        Vec3 errorPos = Vec3.sub(d.getFinalPos(), d.getCurrPos());
        Vec3 errorVel = Vec3.sub(velDesired, d.getCurrVel());

        // desire Linear acceleration
        errorPos.mulScaler(d.getSim().getConfigLoader().getPositionGainConst());
        errorVel.mulScaler(d.getSim().getConfigLoader().getVelocityGainConst());

        Vec3 desLinearAcc = Vec3.add(
                Vec3.add(errorPos, errorVel),
                d.getSim().getConfigLoader().getGravConst()
        );

        // Inertial component of thrust
        return desLinearAcc.mulScaler(d.getMass());
    }

    public Vec3 calcNetThrust() {
        // Inertial component of thrust
        Vec3 inertialThrust = calcInertialThrust();

        // Thrust of the body
        Vec3 thrustBody = Matrix3x3.transpose(d.getRotMat()).mulVec(inertialThrust);

       return new Vec3(0,0,thrustBody.getZ());
    }

    public Vec3 computeTorque () {
        // First we calculate the target Rotation matrix
        Vec3 inerThrustHat = calcInertialThrust().toUnitVec();

        Vec3 worldRef = new Vec3(Math.cos(d.getYaw()), Math.sin(d.getYaw()));
        return new Vec3();
    }

}
