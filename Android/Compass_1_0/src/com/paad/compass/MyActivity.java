package com.paad.compass;

import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;




public class MyActivity extends Activity {
	
	private SensorManager sensorManager;
	
	
	private float roll = 0;
	private float pitch = 0;
	private float yaw = 0;
	private double[] phiTheta_a;
	private double[] phiThetaPsi;
	
	private CompassView cv;
	
	private float[] aValues = new float[3]; // aktuelle Beschleunigungsdaten
	private float[] gValues = new float[3]; // aktuelle Gyrodaten
	private float timestamp; // Zeitstempel für Berechnung dT
	private static final float NS2S = 1.0f / 1000000000.0f; // Umrechnung nsec
															// in sec
	float dT = 0; // Zeitänderung
	
	// eintragen eines Sensor-Event-Listener
	private final SensorEventListener sensorEventListener = new SensorEventListener() {

		public void onAccuracyChanged(Sensor sensor, int accuracy) {
		}

		public void onSensorChanged(SensorEvent event) { // Bei Sensor-Event
															// aufgerufen
			// Daten von Beschleunigungssensor oder Gyro?
			if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) 
				aValues = event.values;
			if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) { 
				gValues = event.values;
				if (timestamp != 0) {
					dT = (event.timestamp - timestamp) * NS2S;
				}
				timestamp = event.timestamp;
			}

			// Beschleunigungsdaten auslesen
			double fx = aValues[0];
			double fy = aValues[1];
			// double fz = aValues[2];

			// Drehwinkel auslesen
			double p = gValues[0];
			double q = gValues[1];
			double r = gValues[2];

			// Berechnung der Haltung

			CalcAndFilterData eAccel = new CalcAndFilterData();
			phiTheta_a = eAccel.EulerAccel(fx, fy);
			phiThetaPsi = eAccel.EulerEKF(p, q, r, dT);
			
			pitch = (float)Math.toDegrees(phiThetaPsi[0]);
			roll = (float)Math.toDegrees(phiThetaPsi[1]);
			yaw = (float)Math.toDegrees(phiThetaPsi[2]);
			
			
//			pitch_r = (float)Math.asin (fy / g);
//			pitch = -((360/(2*pi)) * pitch_r);
//			roll = ((360/(2*pi)) * (float) Math.asin (-fx / (g*Math.cos(pitch_r))));
//			
			cv.setPitch(pitch);
			cv.setRoll(roll);
			
			
			//cv.invalidate();
			cv.invalidate();
		}
			
	};
	
  /** Called when the activity is first created. */
  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.main);
    
    cv = (CompassView) this.findViewById(R.id.compassView);

	sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);

	Sensor accelerometer = sensorManager
			.getDefaultSensor(Sensor.TYPE_ACCELEROMETER); // Beschleunigungssensor aktivieren
	sensorManager.registerListener(sensorEventListener, accelerometer,
			SensorManager.SENSOR_DELAY_FASTEST);
	
	Sensor gyro = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE); // Gyro aktivieren
	sensorManager.registerListener(sensorEventListener, gyro,
			SensorManager.SENSOR_DELAY_FASTEST);
    
   // cv.setPitch((float) -45);
    
    
    
    
  }
}