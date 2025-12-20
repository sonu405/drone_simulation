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
        vec[0] += other.getX();
        vec[1] += other.getY();
        vec[2] += other.getZ();
        return this;
    }

    public Vec3 mulScaler(double scaler) {
        vec[0] *= scaler;
        vec[1] *= scaler;
        vec[2] *= scaler;
        return this;
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

    public Vec3 toUnitVec() {
        return mulScaler(1 / getMagnitude()); // for unit  vector, multiplying 1 / magnitude
    }
}
