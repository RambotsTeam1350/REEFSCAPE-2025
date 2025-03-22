package frc.robot.lib;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;

public class PIDControllerConfigurable extends PIDController {
    private LinearFilter inputFilter;
    private LinearFilter outputFilter;
    private double deadband = 0.0;
    private double lastOutput = 0.0;
    private double outputRateLimit = Double.POSITIVE_INFINITY;
    
    public PIDControllerConfigurable(double kP, double kI, double kD) {
        super(kP, kI, kD);
    }
    
    public PIDControllerConfigurable(double kP, double kI, double kD, double tolerance) {
        super(kP, kI, kD);
        this.setTolerance(tolerance);
    }
    
    /**
     * Sets an input filter to smooth sensor readings.
     * @param taps The number of samples to average (higher = smoother but more lag)
     * @return This object for method chaining
     */
    public PIDControllerConfigurable withInputFilter(int taps) {
        this.inputFilter = LinearFilter.movingAverage(taps);
        return this;
    }
    
    /**
     * Sets an output filter to smooth control signals.
     * @param taps The number of samples to average (higher = smoother but more lag)
     * @return This object for method chaining
     */
    public PIDControllerConfigurable withOutputFilter(int taps) {
        this.outputFilter = LinearFilter.movingAverage(taps);
        return this;
    }
    
    /**
     * Sets a deadband to ignore small errors.
     * @param deadband The minimum error magnitude to respond to
     * @return This object for method chaining
     */
    public PIDControllerConfigurable withDeadband(double deadband) {
        this.deadband = Math.abs(deadband);
        return this;
    }
    
    /**
     * Sets a rate limit on how quickly the output can change.
     * @param rateLimit The maximum change in output per calculation
     * @return This object for method chaining
     */
    public PIDControllerConfigurable withOutputRateLimit(double rateLimit) {
        this.outputRateLimit = Math.abs(rateLimit);
        return this;
    }
    
    @Override
    public double calculate(double measurement) {
        // Apply input filtering if configured
        double filteredMeasurement = (inputFilter != null) ? 
            inputFilter.calculate(measurement) : measurement;
            
        // Call parent calculate method with filtered measurement
        double output = super.calculate(filteredMeasurement);
        
        // Apply deadband
        if (Math.abs(getPositionError()) < deadband) {
            output = 0.0;
        }
        
        // Apply rate limiting
        if (outputRateLimit < Double.POSITIVE_INFINITY) {
            double outputChange = output - lastOutput;
            if (Math.abs(outputChange) > outputRateLimit) {
                output = lastOutput + Math.copySign(outputRateLimit, outputChange);
            }
        }
        
        // Apply output filtering if configured
        if (outputFilter != null) {
            output = outputFilter.calculate(output);
        }
        
        lastOutput = output;
        return output;
    }
    
    @Override
    public double calculate(double measurement, double setpoint) {
        setSetpoint(setpoint);
        return calculate(measurement);
    }
}
