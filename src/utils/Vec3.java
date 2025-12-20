package utils;

public class Vec3 {
    private double[] vec;

    public Vec3(double x, double y, double z) {
        vec = new double[3];
        vec[0] = x;
        vec[1] = y;
        vec[2] = z;
    }
    public Vec3() {
        vec = new double[3];
        vec[0] = 0;
        vec[1] = 0;
        vec[2] = 0;
    }

    public static Vec3 add(Vec3 a, Vec3 other) {
        return new Vec3(a.getX() + other.getX(), a.getY() + other.getY(), a.getZ() + other.getZ());
    }
    public static Vec3 sub(Vec3 a, Vec3 other) {
        return new Vec3(a.getX() - other.getX(), a.getY() - other.getY(), a.getZ() - other.getZ());
    }

    public Vec3 add(Vec3 other) {
         double newX =   getX() + other.getX();
         double newY =   getY() + other.getY();
         double newZ =   getZ() + other.getZ();

        return new Vec3(newX, newY, newZ);
    }

    // TODO (CHANGE): RIGHT NOW: IT MUTATES THE STATE
    public Vec3 mulScaler(double scaler) {
        return new Vec3(
            vec[0] * scaler,
            vec[1] * scaler,
            vec[2] * scaler
        );
    }

    public Vec3 cross(Vec3 other) {
        double newX =    getY() * other.getZ() - getZ() * other.getY();
        double newY = - (getX() * other.getZ() - getZ() * other.getX());
        double newZ =    getX() * other.getY() - getY() * other.getX();

        return new Vec3(newX, newY, newZ);
    }

    public double getX() {
        return vec[0];
    }
    public double getY() {
        return vec[1];
    }
    public double getZ() {
        return vec[2];
    }

    public double getMagnitude() {
        return  Math.sqrt(Math.pow(getX(), 2) + Math.pow(getY(), 2) + Math.pow(getZ(), 2));
    }

    public Vec3 normalize() {
        double mag = getMagnitude();

        // TODO: SEND AN ERROR INSTEAD
        if (mag == 0) {
            return new Vec3(0, 0, 0);
        }

        return mulScaler(1.0 / mag); // for unit vector, multiplying 1 / magnitude
    }
}
