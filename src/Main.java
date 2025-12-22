import Fleet.*;

class Main {
    public static void main() {
        ConfigLoader cl= new ConfigLoader();
        Environment env = new Environment(100,50);

        FleetState fleetState = new FleetState(0.05);

        // TODO: Add env to drone
        Simulator sim = new Simulator(fleetState, cl, env);

        Logger logger = null;
        try {
            logger = new Logger(sim,fleetState);
        }catch (Exception e){
            System.out.println(e.getMessage());
        }

        for (int i = 0; i < 5000 ; i++) {
            System.out.println("\n\n======= STEP " + i + " ========");
//            try {
//                Thread.sleep(1000);
//            }catch (Exception e) {
//                System.out.println(e.getMessage());
//            }
            fleetState.notifyObervers();
        }

        try{
            logger.close();
        }catch (Exception e) {
            System.out.println(e.getMessage());
        }
    }
}