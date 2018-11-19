package sensor.finta.cucc;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PointF;
import android.graphics.Rect;
import android.graphics.RectF;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.PowerManager;
import android.os.SystemClock;
import android.support.v4.util.Pair;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Display;
import android.view.View;
import android.view.WindowManager;
import android.widget.ImageView;
import android.widget.TextView;

//import com.kircherelectronics.fsensor.filter.BaseFilter;
//import com.kircherelectronics.fsensor.filter.averaging.MeanFilter;
//import com.kircherelectronics.fsensor.filter.averaging.MedianFilter;
import com.jjoe64.graphview.GraphView;
import com.jjoe64.graphview.series.DataPoint;
import com.jjoe64.graphview.series.LineGraphSeries;
import com.kircherelectronics.fsensor.filter.fusion.OrientationFusion;
import com.kircherelectronics.fsensor.filter.fusion.OrientationKalmanFusion;
import com.kircherelectronics.fsensor.linearacceleration.LinearAcceleration;
import com.kircherelectronics.fsensor.linearacceleration.LinearAccelerationFusion;
//import com.kircherelectronics.fsensor.linearacceleration.LinearAccelerationFusion;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Timer;
import java.util.TimerTask;
import java.util.Vector;

public class MainActivity extends AppCompatActivity implements SensorEventListener{
    //private LinearAcceleration linearAccelerationFilter;
    private LinearAcceleration linearAccelerationFilterKalman;
    private OrientationFusion orientationFusionKalman;

    private float[] acceleration;
    private float[] magnetic;
    //private float[] linearAcceleration;
    private float[] rotation;

    private SensorManager sensorManager;
    //private Sensor accelerationSensor;
    //private Sensor magneticSensor;
    //private Sensor gyroscopeSensor;

    private float startTime = 0;
    private int count = 0;

    private int vWidth;
    private int vHeight;
    private Canvas mCanvas;
    private Paint mPaint = new Paint();
    private Paint mPaintText = new Paint(Paint.UNDERLINE_TEXT_FLAG);
    private Bitmap mBitmap;
    private ImageView mImageView;
    private int mColorBackground;
    private int halfWidth;
    private int halfHeight;
    private boolean first = true;
    private Timer timer = new Timer();
    private Timer timer2 = new Timer();

    private WindowManager mWindowManager;
    private Display mDisplay;
    private static final float sFriction = 0.0f;
    private long mLastT;
    private float mLastDeltaT;
    private long mSensorTimeStamp;
    private long mCpuTimeStamp;


    private float mPosX = 0.0f;
    private float mPosY = 0.0f;
    private float mAccelX = 0.0f;
    private float mAccelY = 0.0f;
    private float mLastPosX = 0.0f;
    private float mLastPosY = 0.0f;
    private float mOneMinusFriction = 1.0f;


    private Vector<Pair<Long, Float>> dataX = new Vector < Pair<Long, Float> > ();
    private Vector<Pair<Long, Float>> dataY = new Vector < Pair<Long, Float> > ();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        init();
    }

    @Override
    public void onResume() {
        super.onResume();

        //sensorManager.registerListener(this, accelerationSensor, SensorManager.SENSOR_DELAY_FASTEST);
        //sensorManager.registerListener(this, magneticSensor, SensorManager.SENSOR_DELAY_FASTEST);
        //sensorManager.registerListener(this, gyroscopeSensor, SensorManager.SENSOR_DELAY_FASTEST);
        registerSensors();
        startTime =0;
        count = 0;
        orientationFusionKalman.startFusion();
    }

    @Override
    public void onPause() {
        orientationFusionKalman.stopFusion();

        unregisterSensors();
        super.onPause();
    }

    private void unregisterSensors() {
        sensorManager.unregisterListener(this);
    }

    private void init() {
        acceleration = new float[4];
        magnetic = new float[3];
        rotation = new float[3];

        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        //accelerationSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        //magneticSensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        //gyroscopeSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        orientationFusionKalman = new OrientationKalmanFusion();
        orientationFusionKalman.setTimeConstant(0.18f);
        linearAccelerationFilterKalman = new LinearAccelerationFusion(orientationFusionKalman);
        linearAccelerationFilterKalman.setTimeConstant(0.18f);

        // Get an instance of the WindowManager
        mWindowManager = (WindowManager) getSystemService(WINDOW_SERVICE);
        mDisplay = mWindowManager.getDefaultDisplay();


        //orientationFusion = new OrientationKalmanFusion();
        //orientationFusion = new OrientationComplimentaryFusion();
        //linearAccelerationFilter = new LinearAccelerationFusion(orientationFusion);


        //view
        mColorBackground = Color.GREEN;
        mPaint.setColor(Color.RED);
        mPaintText.setColor(Color.RED);
        mPaintText.setTextSize(70);
        mImageView = (ImageView) findViewById(R.id.myimageview);
    }

    private float calculateSensorFrequency() {
        // Initialize the start time.
        if (startTime == 0) {
            startTime = System.nanoTime();
        }

        long timestamp = System.nanoTime();

        // Find the sample period (between updates) and convert from
        // nanoseconds to seconds. Note that the sensor delivery rates can
        // individually vary by a relatively large time frame, so we use an
        // averaging technique with the number of sensor updates to
        // determine the delivery rate.
        float hz = (count++ / ((timestamp - startTime) / 1000000000.0f));

        return hz;
    }

    @Override
    public void onSensorChanged(SensorEvent event) {

        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            // Android reuses events, so you probably want a copy
            float[] acceleration = new float[3];
            System.arraycopy(event.values, 0, acceleration, 0, event.values.length);
            orientationFusionKalman.setAcceleration(acceleration);
            acceleration = linearAccelerationFilterKalman.filter(acceleration);

            System.arraycopy(acceleration, 0, this.acceleration, 0, acceleration.length);
            this.acceleration[3] = calculateSensorFrequency();
            mSensorTimeStamp = event.timestamp;
            mCpuTimeStamp = System.nanoTime();

        } else  if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            // Android reuses events, so you probably want a copy
            System.arraycopy(magnetic, 0, this.magnetic, 0, magnetic.length);
            orientationFusionKalman.setMagneticField(this.magnetic);
        } else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
            // Android reuses events, so you probably want a copy
            System.arraycopy(rotation, 0, this.rotation, 0, rotation.length);
            //THIS IS CHANGED!!!!!!!!!!!!
            orientationFusionKalman.setOrientation(this.rotation);
            // Filter the rotation
            //orientationFusionKalman.filter(this.rotation);
        }
        else if (event.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) {
            //Log.v("LINEAR_ACC", "TYPE_LINEAR_ACCELERATION");
        }
    }
    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {}

    private void registerSensors() {

        //linearAccelerationFilterKalman.reset();
        //orientationFusionKalman.reset();

        sensorManager.registerListener(this, sensorManager
                        .getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
                SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(this, sensorManager
                        .getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION),
                SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(this, sensorManager
                        .getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
                SensorManager.SENSOR_DELAY_FASTEST);
        sensorManager.registerListener(this,
                sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
                SensorManager.SENSOR_DELAY_FASTEST);

    }

    public void onImageViewClick(View view)
    {
        mPosX = 0.0f;
        mPosY = 0.0f;
        mAccelX = 0.0f;
        mAccelY = 0.0f;
        mLastPosX = 0.0f;
        mLastPosY = 0.0f;
        mOneMinusFriction = 1.0f;
        dataX = new Vector < Pair<Long, Float> > ();
        dataY = new Vector < Pair<Long, Float> > ();
        timer = new Timer();
        timer2 = new Timer();
        vWidth = view.getWidth();
        vHeight = view.getHeight();
        acceleration = null;
        magnetic = null;
        halfWidth = vWidth / 2;
        halfHeight = vHeight / 2;
        mBitmap = Bitmap.createBitmap(vWidth, vHeight, Bitmap.Config.ARGB_8888);
        mImageView.setImageBitmap(mBitmap);
        mCanvas = new Canvas(mBitmap);
        mCanvas.drawColor(mColorBackground);
        mPaint.setStyle(Paint.Style.STROKE);
        mCanvas.drawCircle(halfWidth, halfHeight, 400, mPaint);
        // mCanvas.save();
        init();
        Draw();

        timer.scheduleAtFixedRate(new TimerTask(){
            @Override
            public void run() {
                MainActivity.this.runOnUiThread(new Runnable() {
                    public void run() {
                        Draw();
                        CalculatePositions();
                        dataX.add(new Pair<Long, Float>(new Long(System.nanoTime()), new Float(mPosX)));
                        dataY.add(new Pair<Long, Float>(new Long(System.nanoTime()), new Float(mPosY)));
                    }
                });
            }

        }, 500, 10);

        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                timer.cancel();
                GraphView graphX = (GraphView) findViewById(R.id.graphx);
                GraphView graphY = (GraphView) findViewById(R.id.graphy);
                graphX.removeAllSeries();
                graphY.removeAllSeries();

                //X graph
                for(int i = 0; i < dataX.size(); ++i)
                {
                    dataX.setElementAt(new Pair<Long, Float>((long)i, dataX.get(i).second), i);
                }

                long max = Long.MIN_VALUE;
                long min = Long.MAX_VALUE;
                for(int i=0; i < dataX.size(); i++){
                    if(dataX.get(i).first > max){
                        max = dataX.get(i).first;
                    }

                    if(dataX.get(i).first < min){
                        min = dataX.get(i).first;
                    }
                }

                graphX.getViewport().setYAxisBoundsManual(true);
                graphX.getViewport().setXAxisBoundsManual(true);
                graphX.getViewport().setMinY(-1.0);
                graphX.getViewport().setMaxY(1.0);
                graphX.getViewport().setMinX(min);
                graphX.getViewport().setMaxX(max);
                // enable scaling and scrolling
                graphX.getViewport().setScalable(true);
                graphX.getViewport().setScalableY(true);

                int fixXSize = dataX.size();
                DataPoint[] datap = new DataPoint[fixXSize];
                for(int i = 0; i < fixXSize; ++i){
                    datap[i] = new DataPoint(dataX.get(i).first, dataX.get(i).second);
                }
                LineGraphSeries<DataPoint> series = new LineGraphSeries<DataPoint>(datap);
                graphX.addSeries(series);


                //Y graph
                for(int i = 0; i < dataY.size(); ++i)
                {
                    dataY.setElementAt(new Pair<Long, Float>((long)i, dataY.get(i).second), i);
                }

                max = Long.MIN_VALUE;
                min = Long.MAX_VALUE;

                for(int i=0; i < dataY.size(); i++){
                    if(dataY.get(i).first > max){
                        max = dataY.get(i).first;
                    }

                    if(dataY.get(i).first < min){
                        min = dataY.get(i).first;
                    }
                }
                graphY.getViewport().setYAxisBoundsManual(true);
                graphY.getViewport().setXAxisBoundsManual(true);
                graphY.getViewport().setMinY(-1.0);
                graphY.getViewport().setMaxY(1.0);
                graphY.getViewport().setMinX(min);
                graphY.getViewport().setMaxX(max);
                graphY.getViewport().setScalable(true);
                graphY.getViewport().setScalableY(true);

                int fixYSize = dataY.size();
                datap = new DataPoint[fixYSize];
                for(int i = 0; i < fixYSize; ++i){
                    datap[i] = new DataPoint(dataY.get(i).first, dataY.get(i).second);
                }
                series = new LineGraphSeries<DataPoint>(datap);
                graphY.addSeries(series);


                float maxXPos = Long.MIN_VALUE;
                float maxYPos = Long.MIN_VALUE;

                for(int i = 0; i < dataX.size(); ++i)
                {
                    if(Math.abs(dataX.get(i).second) > maxXPos)
                    {
                        maxXPos = Math.abs(dataX.get(i).second);
                    }
                }

                for(int i = 0; i < dataY.size(); ++i)
                {
                    if(Math.abs(dataY.get(i).second) > maxYPos)
                    {
                        maxYPos = Math.abs(dataY.get(i).second);
                    }
                }

                final float toTextX = maxXPos;
                final float toTextY = maxYPos;

                MainActivity.this.runOnUiThread(new Runnable() {
                    public void run() {
                        ((TextView)findViewById(R.id.labX)).setText((Float.toString(toTextX) + "  "));
                        ((TextView)findViewById(R.id.labY)).setText((Float.toString(toTextY) + "  "));
                    }
                });
            }
        }, 2000);
    }



    private void Draw(){
        float x;
        float y;
        float z;
        x = acceleration[0];
        y = acceleration[1];
        z = acceleration[2];

        mBitmap = Bitmap.createBitmap(vWidth, vHeight, Bitmap.Config.ARGB_8888);
        mImageView.setImageBitmap(mBitmap);
        mCanvas = new Canvas(mBitmap);
        mCanvas.drawColor(mColorBackground);
        mPaint.setStyle(Paint.Style.STROKE);
        mCanvas.drawCircle(halfWidth, halfHeight, 400, mPaint);
        mCanvas.drawCircle(halfWidth + x * 100, halfHeight + y * 100, 30, mPaintText);

        //((TextView)findViewById(R.id.labX)).setText((Float.toString(x)));
        //((TextView)findViewById(R.id.labY)).setText((Float.toString(y)));
        //((TextView)findViewById(R.id.label3)).setText((Float.toString(z)));

        mImageView.invalidate();
    }

    private void CalculatePositions() {

        final long now = mSensorTimeStamp + (System.nanoTime() - mCpuTimeStamp);
        final float sx = acceleration[0];
        final float sy = acceleration[1];

        final long t = now;
        if (mLastT != 0) {
            final float dT = (float) (t - mLastT) * (1.0f / 1000000000.0f);
            if (mLastDeltaT != 0) {
                final float dTC = dT / mLastDeltaT;




                final float ax = -sx;
                final float ay = -sy;

                /*
                 * Time-corrected Verlet integration The position Verlet
                 * integrator is defined as x(t+t) = x(t) + x(t) - x(t-t) +
                 * a(t)t2 However, the above equation doesn't handle variable
                 * t very well, a time-corrected version is needed: x(t+t) =
                 * x(t) + (x(t) - x(t-t)) * (t/t_prev) + a(t)t2 We also add
                 * a simple friction term (f) to the equation: x(t+t) = x(t) +
                 * (1-f) * (x(t) - x(t-t)) * (t/t_prev) + a(t)t2
                 */

                final float dTdT = dT * dT;
                final float x = mPosX + mOneMinusFriction * dTC * (mPosX - mLastPosX) + mAccelX
                        * dTdT;
                final float y = mPosY + mOneMinusFriction * dTC * (mPosY - mLastPosY) + mAccelY
                        * dTdT;
                mLastPosX = mPosX;
                mLastPosY = mPosY;
                mPosX = x;
                mPosY = y;
                mAccelX = ax;
                mAccelY = ay;

                ((TextView)findViewById(R.id.posX)).setText((Float.toString(mPosX) + "  "));
                ((TextView)findViewById(R.id.posY)).setText((Float.toString(mPosY) + "  "));
            }
            mLastDeltaT = dT;
        }
        mLastT = t;
    }

}


