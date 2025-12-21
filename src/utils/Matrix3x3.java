package utils;

public class Matrix3x3 {
    private Vec3 c1;
    private Vec3 c2;
    private Vec3 c3;

    public Matrix3x3(Vec3 c1, Vec3 c2, Vec3 c3) {
        this.c1 = c1;
        this.c2 = c2;
        this.c3 = c3;
    }

    public Matrix3x3(double r1c1, double r1c2, double r1c3,
                     double r2c1, double r2c2, double r2c3,
                     double r3c1, double r3c2, double r3c3
                     ) {
       this.c1 = new Vec3(r1c1, r2c1, r3c1);
       this.c2 = new Vec3(r1c2, r2c2, r3c2);
       this.c3 = new Vec3(r1c3, r2c3, r3c3);
    }

    public Matrix3x3() {
        this.c1 = new Vec3(1, 0, 0);
        this.c2 = new Vec3(0, 1, 0);
        this.c3 = new Vec3(0, 0, 1);
    }

    public static Matrix3x3 transpose(Matrix3x3 m) {
        return new Matrix3x3(
                new Vec3(m.c1.getX(), m.c2.getX(), m.c3.getX()),
                new Vec3(m.c1.getY(), m.c2.getY(), m.c3.getY()),
                new Vec3(m.c1.getZ(), m.c2.getZ(), m.c3.getZ())
        );
    }

    public Vec3 getC1() {
        return c1;
    }

    public Vec3 getC2() {
        return c2;
    }

    public Vec3 getC3() {
        return c3;
    }

    public Matrix3x3 mul(Matrix3x3 other) {
        return new Matrix3x3(
               mulVec(other.getC1()),
               mulVec(other.getC2()),
               mulVec(other.getC3())
        );
    }

    public Vec3 mulVec(Vec3 v) {
        Vec3 newC1 = c1.mulScaler(v.getX());
        Vec3 newC2 = c2.mulScaler(v.getY());
        Vec3 newC3 = c3.mulScaler(v.getZ());

        return Vec3.add(Vec3.add(newC1, newC2), newC3);
    }

    public Matrix3x3 add(Matrix3x3 o) {
        return new Matrix3x3(
            Vec3.add(getC1(), o.getC1()),
            Vec3.add(getC2(), o.getC2()),
            Vec3.add(getC3(), o.getC3())
        );
    }
    public Matrix3x3 mulScalar(double scalar) {
        return new Matrix3x3(
            getC1().mulScaler(scalar),
            getC2().mulScaler(scalar),
            getC3().mulScaler(scalar)
        );
    }
}
