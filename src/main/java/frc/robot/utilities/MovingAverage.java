package frc.robot.utilities;

public class MovingAverage {
    private int size;
    private int count;
    private int current;
    private double[] data;
    private double total;
    
    public MovingAverage(int size) {
        this.size = size;
        this.data = new double[size];
        for (int i=0; i<size; ++i) {
            this.data[i] = 0.0;
        }
        count = 0;
        current = 0;
        total = 0.0;
    }

    public void add(double new_value) {
        total = total - data[current] + new_value;
        data[current] = new_value;
        current = (current + 1) % size;
        count = Math.min(count+1,size);
    }

    public double average() {
        return total/(double)count;
    }

    public double average(int last_n) {
        if (last_n >= count)
            return average();
        double sum = 0.0;
        int i=0;
        while (i < last_n) {
            i += 1;
            sum += data[(current - i) % size];
        }
        return sum/(double)last_n;
    }
}
