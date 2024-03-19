package frc.io.joysticks.util;
/*
Original Author: Sherya
Rewite Author: Jim Hofmann
History:
JCH - 11/6/2019 - rework
S - 3/6/2017 - Original release
TODO: more testing.  maybe add an array handler?
Desc: Allows use of various joystick/gamepad configurations.
Constructor get JS_ID and axisID.  However, if the it needs to pass a default (axis may not
exist for some combinations) then in axisID pass 10 * default value + 100 ( 10 * -default - 100).
IE., to default 0.0 pass 100.  For -1.0 pass -110.  For 1.0 pass 110.
*/

import edu.wpi.first.wpilibj.Joystick;
import frc.util.PropMath;

public class Axis {

  private Joystick joystick;
  private int axisID;
  private boolean exists;
  private double exDefault; // doesn't exist, default to 0
  private double axisInDB; // input deadband
  private double axisOutMn; // output min
  private double axisOutMx; // output max
  private int axisApp; // 0=Linear, 1=sqrt, 2=sq

  /**
   * Construct a new joystick axis object, initialized.
   *
   * @param injoystick joystick ID
   * @param inaxisID axis ID
   */
  public Axis(Joystick injoystick, int inaxisID) {
    joystick = injoystick;
    axisID = inaxisID;
    exists = joystick != null;
    exDefault = 0; // default to 0
    axisInDB = 0.0; // input deadband
    axisOutMn = 0.0; // output min
    axisOutMx = 1.0; // output max
    axisApp = 0; // 0=Linear, 1=sqrt, 2=sq
  }

  // Constructor, defaults set to does not exist & 0.0

  /** Construct a new joystick axis object. Needs initialized. Will return 0.0 as default. */
  public Axis() {
    this.exists = false;
    this.exDefault = 0.0;
  }

  // Constructor, defaults set to does not exist & passed value

  /**
   * Construct a new joystick axis object. Needs initialized but set default value to be return.
   *
   * @param exDefault value to return.
   */
  public Axis(double exDefault) {
    this.exists = false;
    this.exDefault = exDefault;
  }

  /**
   * Assign a new joystick axis
   *
   * @param injoystick joystick ID, If null set !exist and use default value 0.0.
   * @param inAxisID axis ID
   */
  public void setAxis(Joystick injoystick, int inAxisID) {
    joystick = injoystick;
    axisID = inAxisID;
    exists = joystick != null;
    exDefault = 0.0;
  }

  /** Clear assignment. Joystick = null & axisID = 0. */
  public void setAxis() {
    setAxis(null, 0);
  }

  /**
   * @return the raw joystick value
   */
  public double getRaw() {
    return exists ? joystick.getRawAxis(axisID) : exDefault;
  }

  /**
   * @return the rawGet modified with DB, interpolated between DB & 1.0 to outMn & Mx, is Clamped to
   *     the Mn & Mx values if set and had an app applied: 0=linear, 1=sqrt, 2=sqrd.
   *     <p>Configuration is set at istantiation, individually or by setJS
   */
  public double get() {
    if (exists) {
      double tmp = getRaw();
      if (Math.abs(getRaw()) > axisInDB) {
        return PropMath.span2(getRaw(), axisInDB, 1.0, axisOutMn, axisOutMx, true, axisApp);
      }
      return 0.0;
    } else {
      return exDefault;
    }
  }

  /**
   * Configure all parameters for the joystick get() method
   *
   * @param axInDB if the raw absolute value is L.T. this return 0.0
   * @param axOutMn the minimum value to return if in DB
   * @param axOutMx the maximum value to return if in DB
   * @param axApp applied appication: 0=linear, 1=sqrt, 2=sqrd
   */
  public void setJS(double axInDB, double axOutMn, double axOutMx, int axApp) {
    axisInDB = axInDB;
    axisOutMn = axOutMn;
    axisOutMx = axOutMx;
    axisApp = axApp;
  }

  public void setInDB(double axInDB) {
    axisInDB = axInDB;
  }

  public void setOutMn(double axOutMn) {
    axisOutMn = axOutMn;
  }

  public void setOutMx(double axOutMx) {
    axisOutMx = axOutMx;
  }

  public void setApp(int axApp) {
    axisApp = axApp;
  }
}
