package frc.robot.util;

public class RunningAverage {

    private int numSamples;
    private double avg;

    public RunningAverage(int numSamples) {
        this.avg = 0;
        this.numSamples = numSamples;
    }

    public double updateRunningAverage(double sample) {
        avg -= avg / numSamples;
        avg += sample / numSamples;
        return avg;
    }

    public double getRunningAverage() {
        return avg;
    }

    public void resetAverage(double avg) {
        this.avg = avg;
    }

    public static void main(String[] args) {
        int N = 100;
        double[] value = new double[N];
        double[] processed = new double[N];
        RunningAverage runningAverage = new RunningAverage(5);

        for (int i = 0; i < N; i++) {
            double val = 20 * Math.sin(i / 20.0);
            double noise = (Math.random() - 0.5) * 3;
            value[i] = val + noise;
            processed[i] = runningAverage.updateRunningAverage(value[i]);
            System.out.println(value[i] + "," + processed[i]);
        }
    }
}
