package Fleet;

import utils.Observer;
import utils.Subject;

import java.util.ArrayList;
import java.util.List;

public class FleetState implements Subject {
    private List<Observer> observers = new ArrayList<Observer>();
    private double deltaT;

    public FleetState(double deltaT) {
        this.deltaT = deltaT;
    }

    @Override
    public void registerObserver(Observer o) {
        observers.add(o);
    }

    @Override
    public void removeObserver(Observer o) {
        observers.remove(o);
    }

    @Override
    public void notifyObervers() {
        for (Observer o: observers) {
            o.update();
        }
    }

    public void setDelta_t(float delta_t) {
        this.deltaT = delta_t;
        notifyObervers();
    }

    public double getDeltaT() {
        return deltaT;
    }
}
