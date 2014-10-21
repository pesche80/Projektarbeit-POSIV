package com.paad.compass;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import android.os.Environment;

/**
 * The Java class SaveASCII allow the user to save an array of double (1D) into
 * an ASCII file.
 * 
 * All the values will be save in a column. The generated file can be easily
 * read by Matlab using "readcsv". Note there is no coma in the generated file
 * since there is only one column.
 * 
 * Here is an example how to save a file :
 * 
 * SaveASCII ObjectName = new SaveASCII(doubleArrayToSave, "AndroidDirectory",
 * "filename.ext") ; ObjectName.saveFile();
 * 
 * The only method is "saveFile" which save the array specified at the object
 * creation.
 * 
 * Author : Christian Andrié (original code from Sensortest-master on GitHub)
 * Date : 22th September 2014 Version : 1.00
 */

public class SaveASCII {

	private final double[] mData; // Data to send
	private File mDir; // Path
	private final String mFileName; // File name

	// Constructor
	public SaveASCII(double[] data, String directory, String fileName) {
		mData = data;
		mDir = Environment.getExternalStorageDirectory();
		mDir = new File(mDir, directory);
		mFileName = fileName;
	}

	// Method
	public void saveFile() {
		// Check if the external storage is accessible ("external storage" also
		// means "physically internal storage accessible by a file manager")
		if (Environment.MEDIA_MOUNTED.equals(Environment.getExternalStorageState())
				&& !Environment.MEDIA_MOUNTED_READ_ONLY.equals(Environment
						.getExternalStorageState()) && mData.length > 0) {
			String fname = mFileName;

			File path = new File(mDir, fname);

			try {
				PrintWriter out = new PrintWriter(new BufferedWriter(new FileWriter(path)));
				for (int j = 0; j < mData.length; j++) {
					out.print(mData[j]);
					out.println(); // New line after each data
				}
				out.close();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}
}
