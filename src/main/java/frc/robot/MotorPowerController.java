package frc.robot;

public class MotorPowerController {

  private double Kp,
      Ki,
      integral,
      dTime,
      error,
      timeIncrement,
      lastSensorRead,
      changeSpeed,
      predictionPoint,
      speedNeeded,
      allowedError,
      integralProportionalThreshold;

  /**
   * Sets up the MotorPowerController
   *
   * @param Kp Proportional weight
   * @param Ki Integral weight
   * @param dTime Time into future the controller predicts to adjust speed
   * @param time Time it takes to get from 0 to 1
   * @param allowedError The error allowed before the integral starts to change
   * @param lastSensorRead The last sensor read
   * @param integralProportionalThreshold The error threshold in which the integral starts changing
   *     by smaller increments
   */
  public MotorPowerController(
      double Kp,
      double Ki,
      double dTime,
      double time,
      double allowedError,
      double lastSensorRead,
      double integralProportionalThreshold) {
    this.dTime = dTime * 50; // 50 = code refresh rate (per second)
    this.integralProportionalThreshold =
        integralProportionalThreshold * 2; // multiplies by 2 for math to work i forgot why
    this.allowedError = allowedError;
    this.Ki = Ki;
    this.Kp = Kp;
    this.lastSensorRead = lastSensorRead;
    integral = 0; // starts at 0
    error = 0; // starts at 0
    timeIncrement =
        1 / time
            / 50; // 50 = code refresh rate (per second) this is the maximum amount the integral can
    // change per code frame
  }

  /**
   * Calculates the motor speed needed to reach the setpoint
   *
   * @param setpoint The desired position
   * @param sensorRead The current position
   * @return The motor speed needed to reach the setpoint
   */
  public double calculateMotorPowerController(double setpoint, double sensorRead) {
    error = setpoint - sensorRead; // calculates error

    if (Math.abs(error)
        > allowedError) { // if error is greater than allowed error then integral is added
      // proportional to the error
      if (error < 0 && integral > -1)
        integral -=
            Math.min(
                timeIncrement,
                timeIncrement * 2 / (1 / Math.abs(error) * integralProportionalThreshold));
      else if (integral < 1)
        integral +=
            Math.min(
                timeIncrement,
                timeIncrement * 2 / (1 / Math.abs(error) * integralProportionalThreshold));
    }
    integral = Math.min(1, Math.max(-1, integral)); // limits integral to -1 to 1

    predictionPoint =
        (sensorRead - lastSensorRead) * dTime
            + sensorRead; // predicts position dTime seconds in the future
    changeSpeed = (setpoint - predictionPoint) * Kp; // changes speed proportional to the error
    if (dTime
        == 0) { // if dTime is 0 then changeSpeed is 0 this corrects it doubling the proportional
      // (KP) value
      changeSpeed = 0;
    }

    // calculates speed needed and limits it from -1 to 1
    speedNeeded = Math.min(1, Math.max(-1, ((error * Kp) + integral * Ki) + (changeSpeed)));

    // sets last sensor read to current sensor read for next loop
    lastSensorRead = sensorRead;
    return speedNeeded;
  }
}
