import Fleet.*;
import utils.Vec3;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.Point2D;

public class DroneSimulation extends JPanel {
    private Drone[] drones;

    // Fixed camera settings
    private double cameraX = 0;
    private double cameraY = 300;  // High up
    private double cameraZ = -400; // Back from origin
    private double targetX = 0;    // Look at origin
    private double targetY = 50;
    private double targetZ = 0;
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
            Point2D p1 = worldToScreen(-gridSize, 0, z);
            Point2D p2 = worldToScreen(gridSize, 0, z);
            if (p1 != null && p2 != null) {
                g2d.drawLine((int)p1.getX(), (int)p1.getY(), (int)p2.getX(), (int)p2.getY());
            }
        }

        // Lines parallel to Z-axis
        for (int x = -gridSize; x <= gridSize; x += gridSpacing) {
            Point2D p1 = worldToScreen(x, 0, -gridSize);
            Point2D p2 = worldToScreen(x, 0, gridSize);
            if (p1 != null && p2 != null) {
                g2d.drawLine((int)p1.getX(), (int)p1.getY(), (int)p2.getX(), (int)p2.getY());
            }
        }
    }

    private void drawDrone(Graphics2D g2d, Vec3 pos,
                           double pitch, double roll, double yaw, Color droneColor) {
        // Super simple: just a circle with a direction line

        // Drone center
        Point2D center = worldToScreen(pos.getX(), pos.getY(), pos.getZ());

        // Front point (shows direction)
        Point2D front = worldToScreen(
                pos.getX()+ 25 * Math.sin(yaw),
                pos.getY(),
                pos.getZ() - 25 * Math.cos(yaw)
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
    private Point2D worldToScreen(double worldX, double worldY, double worldZ) {
        // Use fixed camera position
        double camX = cameraX;
        double camY = cameraY;
        double camZ = cameraZ;

        // Camera looks at target point
        double tgtX = targetX;
        double tgtY = targetY;
        double tgtZ = targetZ;

        // Transform to camera space
        // 1. Translate so camera is at origin
        double x = worldX - camX;
        double y = worldY - camY;
        double z = worldZ - camZ;

        // 2. Rotate to camera orientation
        // Calculate camera basis vectors
        double forwardX = tgtX - camX;
        double forwardY = tgtY - camY;
        double forwardZ = tgtZ - camZ;
        double len = Math.sqrt(forwardX*forwardX + forwardY*forwardY + forwardZ*forwardZ);
        forwardX /= len; forwardY /= len; forwardZ /= len;

        // Right = forward × worldUp
        double rightX = forwardY * 0 - forwardZ * 1;  // cross with (0,1,0)
        double rightY = forwardZ * 0 - forwardX * 0;
        double rightZ = forwardX * 1 - forwardY * 0;
        len = Math.sqrt(rightX*rightX + rightY*rightY + rightZ*rightZ);
        rightX /= len; rightY /= len; rightZ /= len;

        // Up = right × forward
        double upX = rightY * forwardZ - rightZ * forwardY;
        double upY = rightZ * forwardX - rightX * forwardZ;
        double upZ = rightX * forwardY - rightY * forwardX;

        // Transform to camera basis
        double cx = x * rightX + y * rightY + z * rightZ;
        double cy = x * upX + y * upY + z * upZ;
        double cz = -(x * forwardX + y * forwardY + z * forwardZ);

        // 3. Perspective projection
        if (cz >= 0) return null; // Behind camera

        double screenX = (cx * focalLength) / (-cz);
        double screenY = (cy * focalLength) / (-cz);

        // 4. Convert to screen coordinates
        screenX += getWidth() / 2;
        screenY = getHeight() / 2 - screenY;

        return new Point2D.Double(screenX, screenY);
    }

    public static void main(String[] args) {

        ConfigLoader cl= new ConfigLoader();
        Environment env = new Environment(1000,1000);

        FleetState fleetState = new FleetState(0.05);


        // TODO: Add env to drone
        Simulator sim = new Simulator(fleetState, cl);

        SwingUtilities.invokeLater(() -> {
            JFrame frame = new JFrame("Multi-Drone 3D Visualization");
            frame.add(new DroneSimulation(sim, fleetState));
            frame.pack();
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);
        });
    }
}
