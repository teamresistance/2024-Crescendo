package frc.util;

// Combines SimpleProp and some BotMath methods

/**
 * Proportional math. Linear interpolation around SP by PB
 */
public class PropMath {
	
	private double kSP, kPB, kDB, outMn, outMx;
	private boolean k180 = false; // Use Normalize180 for 0 - 360 continuous
	
	/**
	 * Constructor, for a simple Prop only control. (Using defaults)
	 *
	 * <p>kSP = 0.0 kPB = 1.0 kDB = 0.0 outMn = 0.0 outMx = 1.0 k180 = false
	 *
	 * <p>Change defaults with setters, setSP, ...
	 */
	public PropMath() {
		kSP = 0.0;
		kPB = 1.0;
		kDB = 0.0;
		outMn = 0.0;
		outMx = 1.0;
		k180 = false;
	}
	
	/**
	 * Constructor, for a simple Prop only control.
	 *
	 * @param _kSP   Setpoint
	 * @param _kPB   Propband, error when output should equal 1.0, 100%
	 * @param _kDB   Deadband, +/- unit around SP that the output goes to 0.
	 * @param _outMn Minimum output when within DB
	 * @param _outMx Maximum output when within DB
	 * @param _k180  Handle input as continuous, 0 - 360
	 */
	public PropMath(
		double _kSP, double _kPB, double _kDB, double _outMn, double _outMx, boolean _k180) {
		kSP = _kSP;
		kPB = _kPB;
		kDB = _kDB;
		outMn = _outMn;
		outMx = _outMx;
		k180 = _k180;
	}
	
	/**
	 * Constructor, for a simple Prop only control. (Original, Legacy)
	 *
	 * @param _kSP   Setpoint
	 * @param _kPB   Propband, error when output should equal 1.0, 100%
	 * @param _kDB   Deadband, +/- unit around SP that the output goes to 0.
	 * @param _outMn Minimum output when within DB
	 * @param _outMx Maximum output when within DB
	 * @param _k180  Handle input as continuous, 0 - 360
	 */
	public PropMath(double _kSP, double _kPB, double _kDB, double _outMn, double _outMx) {
		kSP = _kSP;
		kPB = _kPB;
		kDB = _kDB;
		outMn = _outMn;
		outMx = _outMx;
		k180 = false;
	}
	
	/**
	 * Constructor, for a simple Prop only control.
	 *
	 * @param parm  0=SP, 1=PB, 2=DB, 3=outMn, 4=outMx
	 * @param _k180 Handle input as continuous, 0 - 360
	 */
	public PropMath(double[] parm, boolean _k180) {
		kSP = parm[0];
		kPB = parm[1];
		kDB = parm[2];
		outMn = parm[3];
		outMx = parm[4];
		k180 = _k180;
	}
	
	/**
	 * Constructor, for a simple Prop only control. (Original, Legacy)
	 *
	 * @param parm  0=SP, 1=PB, 2=DB, 3=outMn, 4=outMx
	 * @param _k180 Handle input as continuous, 0 - 360
	 */
	public PropMath(double[] parm) {
		kSP = parm[0];
		kPB = parm[1];
		kDB = parm[2];
		outMn = parm[3];
		outMx = parm[4];
		k180 = false;
	}
	
	/**
	 * Calculate a simple proportional response.
	 *
	 * @param inFB feedback
	 * @param prnt Print diagnostic info
	 * @return proportional outpput
	 */
	public double calcProp(double inFB, boolean prnt) {
		double err = inFB - kSP; // error
		if (k180) err = normalizeTo180(err); // normalize if continuous around -180 to 180
		if (prnt) System.out.println("FB: " + inFB + "  SP: " + kSP + "  err: " + err);
		if (prnt) System.out.println("Mn: " + outMn + "  Mx: " + outMx);
		
		if (Math.abs(err) < kDB || kPB == 0.0) return 0.0; // In deadband or PB is 0
		
		err /= kPB; // else calc proportional, neg. else pos.
		if (prnt) System.out.print("Prop err: " + err);
		
		err =
			err < 0
				? span(err, -1.0, 0.0, -outMx, -outMn, true, 0)
				: // Neg.
				span(err, 0.0, 1.0, outMn, outMx, true, 0); // else Pos.
		if (prnt) System.out.println("  Span err: " + err);
		return err;
	}
	
	// Set k's
	public void setSP(double _kSP) {
		kSP = _kSP;
	}
	
	public void setPB(double _kPB) {
		kPB = _kPB;
	}
	
	public void setDB(double _kDB) {
		kDB = _kDB;
	}
	
	public void setOutMn(double _outMn) {
		outMn = _outMn;
	}
	
	public void setOutMx(double _outMx) {
		outMx = _outMx;
	}
	
	public void set180(boolean _k180) {
		k180 = _k180;
	}
	
	// Get k's
	public double getSP() {
		return kSP;
	}
	
	public double getPB() {
		return kPB;
	}
	
	public double getDB() {
		return kDB;
	}
	
	public double getOutMn() {
		return outMn;
	}
	
	public double getOutMx() {
		return outMx;
	}
	
	public boolean get180() {
		return k180;
	}
	
	/**
	 * Span between inLo & inHi to outLo & outHi
	 *
	 * @param inVal    value to be spanned
	 * @param inLo_X1  lowest input value must be lower than inHi
	 * @param inHi_X2  highest input value
	 * @param outLo_Y1 output value asso w/ inLo
	 * @param outHi_Y2 output value asso w/ outHi
	 * @return output y = mx + b
	 */
	public static double span(
		double inVal, double inLo_X1, double inHi_X2, double outLo_Y1, double outHi_Y2) {
		
		if (inHi_X2 == inLo_X1) return 0.0; // Invalid values
		if (outHi_Y2 == outLo_Y1) return outLo_Y1;
		
		double m = (outHi_Y2 - outLo_Y1) / (inHi_X2 - inLo_X1); // slope = rise / run
		double b = outLo_Y1 - (m * inLo_X1); // intercept b = y - (m * x)
		return (m * inVal) + b; // Calc y = (m * X) + b
	}
	
	/**
	 * Span between inLo & inHi to outLo & outHi and apply conditioning 0-none, 1-sqrt, 2=sq.
	 * Conditioning is applied towards 0.
	 *
	 * <p>IE 0.25 sqd is 0.06, -0.25 sqd is -0.06 not -0.56
	 *
	 * @param inVal    value to be spanned
	 * @param inLo_X1  lowest input value must be lower than inHi
	 * @param inHi_X2  highest input value
	 * @param outLo_Y1 output value asso w/ inLo
	 * @param outHi_Y2 output value asso w/ outHi
	 * @param clamp    limit output to outLo to outHi
	 * @param app      Adjust output. 0-linear, 1-sqrt, 2-sqr
	 * @return output spanned outLo to outHi
	 */
	public static double span(
		double inVal,
		double inLo_X1,
		double inHi_X2,
		double outLo_Y1,
		double outHi_Y2,
		boolean clamp,
		int app) {
		boolean isNeg = Math.abs(outHi_Y2) < Math.abs(outLo_Y1) ? true : false;
		double y =
			(isNeg
				? span(inVal, inLo_X1, inHi_X2, -1.0, 0.0)
				: span(inVal, inLo_X1, inHi_X2, 0.0, 1.0)); // Span for percentage
		
		switch (app) {
			case 0: // Linear (Do nothing)
				break;
			case 1: // Square root
				y = sign(y) * Math.sqrt(Math.abs(y)); // apply app with sign
				break;
			case 2: // Square
				y = sign(y) * (y * y);
				break;
			default:
				System.out.println("Bad span app - " + app);
				break;
		}
		
		y =
			(isNeg
				? span(y, -1.0, 0.0, outLo_Y1, outHi_Y2)
				: span(y, 0.0, 1.0, outLo_Y1, outHi_Y2)); // Span applied app to output
		
		if (clamp) y = clamp(y, outLo_Y1, outHi_Y2); // and clamp
		
		return y;
	}
	
	/**
	 * @param num to check
	 * @return -1.0 if num less than 0 else +1.0
	 */
	public static double sign(double num) {
		return num < 0 ? -1.0 : 1.0;
	}
	
	/**
	 * Span the input value.
	 *
	 * <p>If the value is negative span is between -inHi / -inLo to -outHi / -outLo
	 *
	 * <p>If the value is positive span is between inLo / inHi to outLo / outHi
	 *
	 * @param inVal value to be spanned
	 * @param inLo  lowest input value must be lower than inHi
	 * @param inHi  highest input value
	 * @param outLo output value asso w/ inLo
	 * @param outHi output value asso w/ outHi
	 * @param clamp limit output to outLo to outHi
	 * @param app   Adjust output. 0-linear, 1-sqrt, 2-sqr
	 * @return output spanned outLo to outHi
	 */
	public static double span2(
		double inVal, double inLo, double inHi, double outLo, double outHi, boolean clamp, int app) {
		double tmp =
			inVal < 0
				? span(inVal, -inHi, -inLo, -outHi, -outLo, true, app)
				: // Neg.
				span(inVal, inLo, inHi, outLo, outHi, true, app); // else Pos.
		return tmp;
	}
	
	/**
	 * Clamp input between val1 and val2
	 *
	 * @param inVal value to clamp between value 1 & 2
	 * @param val1  value 1
	 * @param val2  value 2
	 * @return clamped value
	 */
	public static double clamp(double inVal, double lmt1, double lmt2) {
		double tmp = Math.min(lmt1, lmt2); // Get smaller limit
		if (inVal < tmp) { // If value LT limit return lower limit
			return tmp;
		}
		
		tmp = Math.max(lmt1, lmt2); // Get larger limit
		if (inVal > tmp) { // If value GT limit return larger limit
			return tmp;
		}
		
		return inVal; // else return value
	}
	
	/**
	 * Normalize angle between -180 & 180 continuously
	 *
	 * @param inAngle Angle to evaluate
	 * @return Noralized angle from-180 to +180, continuously
	 */
	public static double normalizeTo180(double inAngle) {
		double tmpD = inAngle % 360.0; // Modulo 0 to 360
		if (tmpD < -180.0) { // If LT -180 add 360 for complement angle
			tmpD += 360.0;
		} else if (tmpD > 180) { // If GT +180 substract 360 for complement angle
			tmpD -= 360;
		}
		return tmpD;
	}
	
	/**
	 * Scan thru an array, find the first GT input. Use segment to interpolate input.
	 *
	 * @param inVal   value to evaluate
	 * @param arInOut 2d array of XY points. [X | Y}[pts]
	 * @return interpolated value
	 */
	public static double segLine(double inVal, double arInOut[][]) {
		int arLen = arInOut[0].length - 1; // Number of segments
		if (inVal < arInOut[0][0]) return arInOut[1][0]; // If LT min return min
		if (inVal > arInOut[0][arLen]) return arInOut[1][arLen]; // If GT max return max
		int x = 0;
		while (++x < arLen && inVal >= arInOut[0][x])
			; // Find 1st segment GT input
		return span(
			inVal,
			arInOut[0][x - 1],
			arInOut[0][x], // Span value against segment
			arInOut[1][x - 1],
			arInOut[1][x],
        true,
        0);
  }
}
