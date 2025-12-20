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
    public Matrix3x3() {
        this.c1 = new Vec3(1,0,1);
        this.c2 = new Vec3(0,1,0);
        this.c3 = new Vec3(0,0,1);
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

    public Vec3 mulVec(Vec3 v) {
        c1.mulScaler(v.getX());
        c2.mulScaler(v.getY());
        c3.mulScaler(v.getZ());

       return Vec3.add(Vec3.add(c1, c2), c3);
    }

    public static Matrix3x3 transpose(Matrix3x3 m) {
        return new Matrix3x3(
                new Vec3(m.c1.getX(), m.c2.getX(), m.c3.getX()),
                new Vec3(m.c1.getY(), m.c2.getY(), m.c3.getY()),
                new Vec3(m.c1.getZ(), m.c2.getZ(), m.c3.getZ())
        );
    }
}
