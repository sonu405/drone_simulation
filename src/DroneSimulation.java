import Fleet.*;
import utils.Vec3;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.Point2D;

public class DroneSimulation extends JPanel {
    private Drone[] drones;

    // Fixed camera settings
    private Vec3 camera = new Vec3(0,300, -400);
    private Vec3 targetCamera = new Vec3(0,50, 0);
    private double focalLength = 500;

    private Simulator sim;

    public DroneSimulation(Simulator sim, FleetState fleetState) {
        setPreferredSize(new Dimension(800, 600));
        setBackground(new Color(135, 206, 235)); // Sky blue

        this.drones = sim.getDrones();

        // Animation timer - 16 -> 60 FPS and 33 -> 30FPS and 200 -> 5FPS
        Timer timer = new Timer(200, e -> {
            fleetState.notifyObervers();
            this.sim = sim;

            repaint();
        });
        timer.start();
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // Draw grid and drones
        drawPerspectiveGrid(g2d);

        for (int i = 0; i < drones.length; i++) {
            double[] angles = drones[i].angleFromCurRotMat();
            drawDrone(g2d, drones[i].getCurrPos(), angles[1], angles[0], angles[2], Color.BLUE);
        }
    }

    private void drawPerspectiveGrid(Graphics2D g2d) {
        int gridSize = 400;
        int gridSpacing = 50;
        g2d.setColor(new Color(100, 100, 100, 150));

        // Lines parallel to X-axis
        for (int z = -gridSize; z <= gridSize; z += gridSpacing) {
            Point2D p1 = worldToScreen(new Vec3(-gridSize, 0, z));
            Point2D p2 = worldToScreen(new Vec3(gridSize, 0, z));
            if (p1 != null && p2 != null) {
                g2d.drawLine((int)p1.getX(), (int)p1.getY(), (int)p2.getX(), (int)p2.getY());
            }
        }

        // Lines parallel to Z-axis
        for (int x = -gridSize; x <= gridSize; x += gridSpacing) {
            Point2D p1 = worldToScreen(new Vec3(x, 0, -gridSize));
            Point2D p2 = worldToScreen(new Vec3(x, 0, gridSize));
            if (p1 != null && p2 != null) {
                g2d.drawLine((int)p1.getX(), (int)p1.getY(), (int)p2.getX(), (int)p2.getY());
            }
        }
    }

    private void drawDrone(Graphics2D g2d, Vec3 pos,
                           double pitch, double roll, double yaw, Color droneColor) {
        // Super simple: just a circle with a direction line

        // Drone center
        Point2D center = worldToScreen(pos);

        // Front point (shows direction)
        Point2D front = worldToScreen(
                new Vec3(
                pos.getX()+ 25 * Math.sin(yaw),
                pos.getY(),
                pos.getZ() - 25 * Math.cos(yaw))
        );

        if (center != null) {
            // Draw direction line
            if (front != null) {
                g2d.setColor(droneColor.darker());
                g2d.setStroke(new BasicStroke(2));
                g2d.drawLine((int)center.getX(), (int)center.getY(), (int)front.getX(), (int)front.getY());
            }

            // Draw drone body (just a circle)
            g2d.setColor(droneColor);
            g2d.fillOval((int)center.getX() - 12, (int)center.getY() - 12, 24, 24);

            // Draw outline
            g2d.setColor(Color.BLACK);
            g2d.setStroke(new BasicStroke(2));
            g2d.drawOval((int)center.getX() - 12, (int)center.getY() - 12, 24, 24);
        }

        g2d.setStroke(new BasicStroke(1));
    }

    // Camera transformation and projection
    private Point2D worldToScreen(Vec3 world) {
        // Transform to camera space
        Vec3 camInWorldCor = Vec3.sub(world, camera);

        // Calculate camera basis vectors
       Vec3 forward = Vec3.sub(targetCamera, camera).normalize();

        // World up vector
        Vec3 worldUp = new Vec3(0, 1, 0);

        // Right = forward × worldUp
        Vec3 right = forward.cross(worldUp).normalize();

        // Up = right × forward
        Vec3 up = right.cross(forward).normalize();

        // Transform to camera basis (dot products)
        double cx = camInWorldCor.dot(right);
        double cy = camInWorldCor.dot(up);
        double cz = -camInWorldCor.dot(forward);

        // Perspective projection
        if (cz >= 0) return null; // Behind camera

        double screenX = (cx * focalLength) / (-cz);
        double screenY = (cy * focalLength) / (-cz);

        // Convert to screen coordinates
        screenX += getWidth() / 2;
        screenY = getHeight() / 2 - screenY;

        return new Point2D.Double(screenX, screenY);
    }
//    private Point2D worldToScreen(Vec3 world) {
//        // Transform to camera space
//        Vec3 camInWorldCor = Vec3.sub(world, camera);
//
//        // Calculate camera basis vectors
//        Vec3 forward = Vec3.sub(targetCamera, camera).normalize();
//
//        // World up vector
//        Vec3 worldUp = new Vec3(0, 1, 0);
//
//        // Right = forward × worldUp
//        Vec3 right = forward.cross(worldUp).normalize();
//
//        // Up = right × forward
//        Vec3 up = right.cross(forward).normalize();
//
//        // Transform to camera basis
//        double cx = camInWorldCor.dot(right);
//        double cy = camInWorldCor.dot(up);
//        double cz = -camInWorldCor.dot(forward);  // Negative because camera looks along -Z
//
//        if (cz >= 0) return null; // Behind camera
//
//        double screenX = (cx * focalLength) / (-cz);
//        double screenY = (cy * focalLength) / (-cz);
//
//        // 5. Convert to screen coordinates
//        screenX += getWidth() / 2;
//        screenY = getHeight() / 2 - screenY;
//
//        return new Point2D.Double(screenX, screenY);
//    }

    public static void main(String[] args) {

        ConfigLoader cl= new ConfigLoader();
        Environment env = new Environment(800,600);

        FleetState fleetState = new FleetState(0.05);


        // TODO: Add env to drone
        Simulator sim = new Simulator(fleetState, cl, env);


        Logger logger = null;
        try {
            logger = new Logger(sim,fleetState);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }


        SwingUtilities.invokeLater(() -> {
            JFrame frame = new JFrame("Multi-Drone 3D Visualization");
            frame.add(new DroneSimulation(sim, fleetState));
            frame.pack();
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);
        });

        try{
            logger.close();
        }catch (Exception e) {
            System.out.println(e.getMessage());
        }
    }
}
