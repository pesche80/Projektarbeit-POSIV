

package com.paad.compass;



import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;

import org.ejml.data.DenseMatrix64F;
import org.ejml.simple.SimpleMatrix;


public class CalcAndFilterData {
	
	private double fx;
	private double fy;
	private double fz;
	private double p;
	private double q;
	private double r;
	private double phi_a;
	private double theta_a;
	private double phi;
	private double theta;
	private double psi;
	
	// define and initialize Matrix Objects with start values
	SimpleMatrix H = new SimpleMatrix(2, 3, true, new double[]{
				1, 0, 0,
				0, 1, 0
	});
	SimpleMatrix Q = new SimpleMatrix(3, 3, true, new double[]{
				0.0001, 0,      0,
				0,      0.0001, 0,
				0,		0,		0.1
	});
	SimpleMatrix R = new SimpleMatrix(2, 2, true, new double[]{
				6, 0,
				0, 6
	});
	SimpleMatrix x = new SimpleMatrix(3, 1, true, new double[]{
				0,
				0,
				0
	});
	SimpleMatrix P = new SimpleMatrix(3, 3, true, new double[]{
				10,	0,	0,
				0,	10,	0,
				0,	0,	10
	});
	

	private final double G = SensorManager.STANDARD_GRAVITY;
	
	public CalcAndFilterData(){
		
	}
	
	/**
	 * Calculates the acceleration angles
	 * @param _fx	coordinate x from sensor
	 * @param _fy	coordinate y from sensor
	 * @return		Array from doubles Phi = _phiTheta[0] and Theta = _phiTheta[1]
	 */
	public double[] EulerAccel(double _fx, double _fy){
		double[] phiTheta_a = new double[2];
		fx = _fx;
		fy = _fy;
		
		phi_a = Math.asin(fy / G);
		theta_a = Math.asin(-fx / (G*Math.cos(phiTheta_a[0])));
		
		phiTheta_a[0] = phi_a;
		phiTheta_a[1] = theta_a;

		return phiTheta_a;
	}
	
	

	/**
	 * Filters data with kalman filter
	 * @param p
	 * @param q
	 * @param r
	 * @param dt
	 * @return Array [phi theta psi]
	 */
	public double[] EulerEKF(double p, double q, double r, double dt){
		double[] phiThetaPsi = new double[3];
		double[][] a = aJacob(x, p, q, r, dt);
		
		
		// TODO: Kalman Filter
		
		phiThetaPsi[0] = phi;
		phiThetaPsi[1] = theta;
		phiThetaPsi[2] = psi;
		
		return phiThetaPsi;
	}

	/**
	 * 
	 * @param xHat
	 * @param p2
	 * @param q2
	 * @param r2
	 * @param dt
	 * @return
	 */
	private SimpleMatrix aJacob(SimpleMatrix xHat, double p2, double q2, double r2, double dt) {
		//double[][] a = new double[][]{{0, 0, 0},{0, 0, 0},{0, 0, 0}};
		double phi2 = xHat.get(0, 0);
		double theta2 = xHat.get(1, 0);
		double value = 0;
		SimpleMatrix a = new SimpleMatrix(3, 3, true, new double[]{
				0,	0,	0,
				0,	0,	0,
				0,	0,	0
		});
		
		SimpleMatrix dtMat = new SimpleMatrix(3, 3, true, new double[] {
				dt,	dt,	dt,
				dt,	dt,	dt,
				dt,	dt,	dt
		});
		
		
		value = q2 * Math.cos(phi2) * Math.tan(theta2) -
							r2 * Math.sin(phi2) * Math.tan(theta2);
		a.set(0, 0, value);
		
		// sec(x) = 1/cox(x)
		value = q2 * Math.sin(phi2) * (1/ (Math.cos(theta2)*Math.cos(theta2))) +
							r2 *Math.cos(phi2)* 1/ (Math.cos(theta2)*Math.cos(theta2));
		a.set(0, 1, value);
		
		a.set(0, 2, 0);		
		
		
		
		value = -q2 * Math.sin(phi2) - r2 * Math.cos(phi2);
		a.set(1, 0, value);
		
		a.set(1, 1, 0);
		
		a.set(1, 2, 0);
		
		
		
		value = q2 * Math.cos(phi2) * (1/ Math.cos(theta2)) -
							r2 * Math.sin(phi2) * (1/ Math.cos(theta2));
		a.set(2, 0, value);
				
		value = q2 * Math.sin(phi2) * (1/ Math.cos(theta2)) * Math.tan(theta2) +
							r2 * Math.cos(phi2)  * (1/ Math.cos(theta2)) * Math.tan(theta2);
		a.set(2, 1, value);
		
		a.set(2, 2, 0);
		
		a = SimpleMatrix.identity(3).plus(a.mult(dtMat));
		
		return a;
	}
	


}
