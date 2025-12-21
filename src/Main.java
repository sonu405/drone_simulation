import Fleet.ConfigLoader;
import Fleet.Environment;
import Fleet.FleetState;
import Fleet.Simulator;

class Main {
    public static void main() {
        ConfigLoader cl= new ConfigLoader();
        Environment env = new Environment(1000,1000);

        FleetState fleetState = new FleetState(0.05);


        // TODO: Add env to drone
        Simulator sim = new Simulator(fleetState, cl);

        for (int i = 0; i < 1000 ; i++) {
            System.out.println("\n\n======= STEP " + i + " ========");
//            try {
//                Thread.sleep(1000);
//            }catch (Exception e) {
//                System.out.println(e.getMessage());
//            }
            fleetState.notifyObervers();
            System.out.println("Slept for 1000 msec");
        }
    }
}
