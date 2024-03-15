/*
 * Author(s) - Wafflesnack
 * History
 * jch - 2/25/2024 - copied and switched to SparkFlex controller
 * jch - 2/1/2024 - Fixed feet() and getFPS()
 * waffle - 2/1/2019
 *
 * Desc:
 * Standardize SparkMax Encoder interface with other various encoders.
 */

package frc.io.hdw_io.util;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

public class Encoder_Flex {

  private double tpf;
  private RelativeEncoder neoEnc;

  /** Interface to Spark Max controller with NEO motor encoders */
  public Encoder_Flex(CANSparkFlex neoCtlr, double _tpf) {
    tpf = _tpf;
    neoEnc = neoCtlr.getEncoder();
    // neoEnc.setPositionConversionFactor(tpf);    //Feet - Ticks per foot
  }

  /**
   * @return Encoder rotations.
   */
  public double rotations() {
    return neoEnc.getPosition();
  }

  /**
   * @return calculated feet from ticks.
   */
  public double feet() {
    return tpf == 0.0 ? 0.0 : rotations() * 1024 / tpf;
  }

  /**
   * @return calculated degrees from ticks.
   */
  public double degrees() {
    return rotations() * 360.0;
  }

  /**
   * @return calcuate meters from feet.
   */
  public double meters() {
    return Units.feetToMeters(feet());
  }

  /** Reset encoder count to zero. */
  public void reset() {
    neoEnc.setPosition(0.0);
  }

  /**
   * @return the existiing ticks per foot, tpf.
   */
  public double getTPF() {
    return tpf;
  }

  /**
   * @param tpf - Set ticks per foot.
   */
  public void setTPF(double tpf) {
    this.tpf = tpf;
    // neoEnc.setPositionConversionFactor(tpf);
  }

  /**
   * RPM / 60 = RPS, RPS / 1024 tpr = tps, => RPM * (1024 / 60)[17.067] / tpf = FPS :: tpf != 0
   *
   * @return Feet per seconds from RPM & tpf: RPM * k / tpf = FPS k = (1024 / 60) = 17.0667
   */
  public double getFPS() {
    return tpf == 0.0 ? 0.0 : (17.0667) * (neoEnc.getVelocity() / tpf);
  }

  /**
   * Get the velocity of the motor. This returns the native units of 'RPM' by default, and can be
   * changed by a scale factor using setVelocityConversionFactor().
   *
   * @return Number the RPM of the motor by default
   */
  public double getSpeed() {
    return neoEnc.getVelocity();
  }
}
