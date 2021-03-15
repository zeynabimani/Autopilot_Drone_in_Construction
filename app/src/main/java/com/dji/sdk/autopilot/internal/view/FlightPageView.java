package com.dji.sdk.autopilot.internal.view;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.provider.MediaStore;
import android.util.Base64;
import android.util.Log;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.RadioGroup;
import android.widget.Switch;

import com.chaquo.python.PyObject;
import com.chaquo.python.Python;
import com.chaquo.python.android.AndroidPlatform;
import com.dji.sdk.autopilot.R;
import com.dji.sdk.autopilot.internal.controller.DJISampleApplication;
import com.dji.sdk.autopilot.internal.utils.DialogUtils;
import com.dji.sdk.autopilot.internal.utils.ModuleVerificationUtil;
import com.dji.sdk.autopilot.internal.utils.ToastUtils;
import com.dji.sdk.autopilot.internal.utils.VideoFeedView;
import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.MultiFormatReader;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Reader;
import com.google.zxing.Result;
import com.google.zxing.common.HybridBinarizer;

import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

import dji.common.camera.SettingsDefinitions;
import dji.common.error.DJIError;
import dji.common.flightcontroller.LocationCoordinate3D;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.FlightCoordinateSystem;
import dji.common.flightcontroller.virtualstick.RollPitchControlMode;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.flightcontroller.virtualstick.YawControlMode;
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.common.model.LocationCoordinate2D;
import dji.common.util.CommonCallbacks;
import dji.midware.data.manager.P3.DJIPayloadUsbDataManager;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.flightcontroller.FlightController;

import static org.opencv.imgproc.Imgproc.getStructuringElement;

public class FlightPageView extends Activity {

    //flight parameters
    private float pitch;   //[-15,15]
    private float roll;     //[-15,15]
    private float yaw;      //[-30,30]
    private float throttle; //[-2,2]
    private float fb = 2, rl = 2, ud = 2, velocity = 4;
    private int up_sec, fb_sec, rl_sec;
    private Thread flightThread;
    private FlightPageView.SendVirtualStickDataTask sendVirtualStickDataTask;
    private Timer sendVirtualStickDataTimer;
    private FlightController flightController;
    private boolean set_home_location, set_qr_location;
    private LocationCoordinate2D home_location, qr_location;

    //camera parameters
    private Timer timer = new Timer();
    private long timeCounter = 0;
    private long hours = 0;
    private long minutes = 0;
    private long seconds = 0;
    private String time = "";
    private FlightPageView.GimbalRotateTimerTask gimbalRotationTimerTask;
    private Timer rotateGimbalTimer;
    private VideoFeeder.VideoDataListener videoDataListener;
    private VideoFeedView primaryVideoFeed;

    //log parameters
    private File logDataFile;
    private int myCounter;

    //image processing parameters
    private boolean shutdown_collison = true, start_detecting_process, start_landing_process, should_save_photo = false;
    private PyObject nn_params;
    private double mDist;
    private boolean writeLandingLog, wasInOtherCases;
    private int notFound_Timer = 0;
    private float search_velocity_horizontal = 0.2f, search_velocity_vertical = 0.15f;
    private long search_duration = 2000;
    private FlightPageView.direction last_dir = FlightPageView.direction.NONE;

    private enum direction {
        FRONT,
        BACK,
        RIGHT,
        LEFT,
        NONE
    }

    private long startTime, endTime;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        setContentView(R.layout.__flight_page);
        super.onCreate(savedInstanceState);
        getWindow().setSoftInputMode(WindowManager.LayoutParams.SOFT_INPUT_STATE_ALWAYS_HIDDEN);
        init();
    }

    public void init() {
        flightController = ModuleVerificationUtil.getFlightController();
        if (flightController == null) {
            return;
        }
        OpenCVLoader.initDebug();
        setUpImageListener();
        setUpCoordinateListener();
        setUpCollisionListener();
        setCoordinateSystem();
        enable_stick();
        createLogFile("Flight-data-log.txt");
        primaryVideoFeed = (VideoFeedView) findViewById(R.id.my_primary_video_feed);
        videoDataListener = primaryVideoFeed.registerLiveVideo(VideoFeeder.getInstance().getPrimaryVideoFeed(), true);

        onClick();
    }

    public void onClick() {
        findViewById(R.id.takeoff_button).setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                auto_take_off();
//                record_video();
            }
        });
        findViewById(R.id.land_button).setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                auto_land();
//                stop_recording();
            }
        });
        findViewById(R.id.go_button).setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                flight();
            }
        });
        findViewById(R.id.stop_go_button).setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                if (flightThread != null)
                    flightThread.interrupt();
            }
        });

        findViewById(R.id.record_video_button).setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                record_video();
            }
        });
        findViewById(R.id.stop_record_button).setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                stop_recording();
            }
        });
        findViewById(R.id.shoot_photo_button).setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                should_save_photo = true;
                shoot_single_photo();
            }
        });

        findViewById(R.id.find_floor_button).setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                start_detecting_process = true;
            }
        });
        findViewById(R.id.landing_procedure_button).setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                start_landing_process = true;
            }
        });
        findViewById(R.id.collision_check_button).setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                shutdown_collison = false;
            }
        });
        findViewById(R.id.cancel_button).setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                start_landing_process = false;
                start_detecting_process = false;
                shutdown_collison = true;
                roll = 0;
                pitch = 0;
                throttle = 0;
            }
        });

        findViewById(R.id.save_message_button).setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                try {
                    EditText message = (EditText) findViewById(R.id.message);
                    String message_text = message.getText().toString();
                    save_message_in_file(message_text);

                } catch (NumberFormatException ex) {
                    return;
                }
            }
        });

        findViewById(R.id.gimbal_up_button).setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                camera_up();
                try {
                    Thread.sleep(4000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                stop_camera_rotation();
            }
        });
        findViewById(R.id.gimbal_down_button).setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                camera_down();
                try {
                    Thread.sleep(4000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                stop_camera_rotation();
            }
        });

        findViewById(R.id.set_home_pose_button).setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                try {
                    EditText right_left = (EditText) findViewById(R.id.set_home_x);
                    EditText front_back = (EditText) findViewById(R.id.set_home_y);
                    fb = Float.parseFloat(front_back.getText().toString());
                    rl = Float.parseFloat(right_left.getText().toString());
                    set_home_location = false;
                } catch (NumberFormatException ex) {
                    return;
                }
            }
        });
        findViewById(R.id.set_home_pose_by_loc_button).setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                LocationCoordinate2D coordinate = flightController.getState().getHomeLocation();
                home_location = new LocationCoordinate2D(coordinate.getLatitude(), coordinate.getLongitude());
                set_home_location = true;
            }
        });
        findViewById(R.id.set_qr_pose_by_loc_button).setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                LocationCoordinate2D coordinate = flightController.getState().getHomeLocation();
                qr_location = new LocationCoordinate2D(coordinate.getLatitude(), coordinate.getLongitude());
                set_qr_location = true;
            }
        });
    }

    @Override
    protected void onDestroy() {
        if (null != sendVirtualStickDataTimer) {
            if (sendVirtualStickDataTask != null) {
                sendVirtualStickDataTask.cancel();

            }
            sendVirtualStickDataTimer.cancel();
            sendVirtualStickDataTimer.purge();
            sendVirtualStickDataTimer = null;
            sendVirtualStickDataTask = null;
        }
        DJIPayloadUsbDataManager.getInstance().setDataListener(null);
        super.onDestroy();
    }

    public void flight() {
        try {
            //TODO
            Switch s = (Switch) findViewById(R.id.speed_switch);
            if (s.isChecked()) {
                velocity = 4;
            } else {
                velocity = 1;
            }

            EditText floor_num = (EditText) findViewById(R.id.target_floor);
            ud = Float.parseFloat(floor_num.getText().toString()) * 1;
//            velocity = Float.parseFloat(speed.getText().toString());

            if (set_home_location && set_qr_location) {
                getDistance(home_location, qr_location);
            }

            throttle = 0;
            roll = 0;
            pitch = 0;

            setUpListeners();

            FlightThread ft = new FlightThread();
            flightThread = new Thread(ft);
            flightThread.start();

        } catch (NumberFormatException ex) {
            return;
        }
    }

    public class FlightThread implements Runnable {
        public void run() {
            setUpFlight();
        }
    }

    public void setUpFlight() {
        if (velocity == 0) {
            velocity = 4;
        }
        velocity /= 10;
        //up
        if (ud != 0) {
            up_sec = Math.abs((int) (ud * 1000 / velocity));   //ms
            if (ud > 0)
                throttle = velocity;
            else
                throttle = -1 * velocity;

            try {
                Thread.sleep(up_sec);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            throttle = 0;
        }
        //rl
        if (rl != 0) {
            rl_sec = Math.abs((int) (rl * 1000 / velocity));  //ms
            if (rl > 0)
                pitch = velocity;
            else
                pitch = -1 * velocity;

            try {
                Thread.sleep(rl_sec);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            pitch = 0;
        }
        //fb
        if (fb != 0) {
            fb_sec = Math.abs((int) (fb * 1000 / velocity));  //ms
            if (fb > 0)
                roll = velocity;
            else
                roll = -1 * velocity;

            try {
                Thread.sleep(fb_sec);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            roll = 0;
        }
        throttle = 0;
        roll = 0;
        pitch = 0;
    }

    private void setUpListeners() {
        if (null == sendVirtualStickDataTimer) {
            sendVirtualStickDataTask = new FlightPageView.SendVirtualStickDataTask();
            sendVirtualStickDataTimer = new Timer();
            sendVirtualStickDataTimer.schedule(sendVirtualStickDataTask, 100, 200);
        }
    }

    private class SendVirtualStickDataTask extends TimerTask {
        @Override
        public void run() {
            if (ModuleVerificationUtil.isFlightControllerAvailable()) {
                DJISampleApplication.getAircraftInstance()
                        .getFlightController()
                        .sendVirtualStickFlightControlData(new FlightControlData(
                                        pitch,
                                        roll,
                                        yaw,
                                        throttle),
                                new CommonCallbacks.CompletionCallback() {
                                    @Override
                                    public void onResult(DJIError djiError) {

                                    }
                                });
            }
        }
    }

    public void auto_take_off() {
        flightController.startTakeoff(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {
                DialogUtils.showDialogBasedOnError(FlightPageView.this, djiError);
            }
        });
    }

    public void auto_land() {
        flightController.startLanding(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {
                DialogUtils.showDialogBasedOnError(FlightPageView.this, djiError);
            }
        });
    }

    public void enable_stick() {
        flightController.setVirtualStickModeEnabled(true, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {
                DialogUtils.showDialogBasedOnError(FlightPageView.this, djiError);
            }
        });
    }

    public void setCoordinateSystem() {
        flightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
        flightController.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
        flightController.setVerticalControlMode(VerticalControlMode.VELOCITY);
        flightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.BODY);
    }

    public void record_video() {
        if (ModuleVerificationUtil.isCameraModuleAvailable()) {
            DJISampleApplication.getProductInstance()
                    .getCamera()
                    .setMode(SettingsDefinitions.CameraMode.RECORD_VIDEO,
                            new CommonCallbacks.CompletionCallback() {
                                @Override
                                public void onResult(DJIError djiError) {
//                                    ToastUtils.setResultToToast("SetCameraMode to recordVideo");
                                }
                            });
            DJISampleApplication.getProductInstance()
                    .getCamera()
                    .startRecordVideo(new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {
                            //success so, start recording
                            if (null == djiError) {
                                ToastUtils.setResultToToast("Start Record");
                                timer = new Timer();
                                timer.schedule(new TimerTask() {
                                    @Override
                                    public void run() {
                                        timeCounter = timeCounter + 1;
                                        hours = TimeUnit.MILLISECONDS.toHours(timeCounter);
                                        minutes =
                                                TimeUnit.MILLISECONDS.toMinutes(timeCounter) - (hours * 60);
                                        seconds = TimeUnit.MILLISECONDS.toSeconds(timeCounter) - ((hours
                                                * 60
                                                * 60) + (minutes * 60));
                                        time = String.format("%02d:%02d:%02d", hours, minutes, seconds);
                                    }
                                }, 0, 1);
                            } else {
//                                Log.d("Debug", "ERRRRRRRRRRRR" + djiError);
                                ToastUtils.setResultToToast(djiError.toString());
                            }
                        }
                    });
        }
    }

    public void stop_recording() {
        if (ModuleVerificationUtil.isCameraModuleAvailable()) {
            DJISampleApplication.getProductInstance()
                    .getCamera()
                    .stopRecordVideo(new CommonCallbacks.CompletionCallback() {
                        @Override
                        public void onResult(DJIError djiError) {
                            ToastUtils.setResultToToast("Stop Record");
                            timer.cancel();
                            timeCounter = 0;
                        }
                    });
        }
    }

    public void shoot_single_photo(){
        DJISampleApplication.getProductInstance()
                .getCamera()
                .setMode(SettingsDefinitions.CameraMode.SHOOT_PHOTO,
                        new CommonCallbacks.CompletionCallback() {
                            @Override
                            public void onResult(DJIError djiError) {

                            }
                        });

        DJISampleApplication.getProductInstance()
                .getCamera()
                .startShootPhoto(new CommonCallbacks.CompletionCallback() {
                    @Override
                    public void onResult(DJIError djiError) {
                        if (null == djiError) {
                            ToastUtils.setResultToToast("Ok");
                        } else {
                            ToastUtils.setResultToToast(djiError.getDescription());
                        }
                    }
                });
    }

    private void camera_down() {
        if (rotateGimbalTimer == null) {
            rotateGimbalTimer = new Timer();
            gimbalRotationTimerTask = new FlightPageView.GimbalRotateTimerTask(-25);
            rotateGimbalTimer.schedule(gimbalRotationTimerTask, 0, 100);
        }
    }

    private void camera_up() {
        if (rotateGimbalTimer == null) {
            rotateGimbalTimer = new Timer();
            gimbalRotationTimerTask = new FlightPageView.GimbalRotateTimerTask(25);
            rotateGimbalTimer.schedule(gimbalRotationTimerTask, 0, 100);
        }
    }

    private void stop_camera_rotation() {
        if (rotateGimbalTimer != null) {
            if (gimbalRotationTimerTask != null) {
                gimbalRotationTimerTask.cancel();
            }
            rotateGimbalTimer.cancel();
            rotateGimbalTimer.purge();
            gimbalRotationTimerTask = null;
            rotateGimbalTimer = null;
        }

        if (ModuleVerificationUtil.isGimbalModuleAvailable()) {
            DJISampleApplication.getProductInstance().getGimbal().
                    rotate(null, new CommonCallbacks.CompletionCallback() {

                        @Override
                        public void onResult(DJIError error) {

                        }
                    });
        }
    }

    private class GimbalRotateTimerTask extends TimerTask {
        float pitchValue;

        GimbalRotateTimerTask(float pitchValue) {
            super();
            this.pitchValue = pitchValue;
        }

        @Override
        public void run() {
            if (ModuleVerificationUtil.isGimbalModuleAvailable()) {
                DJISampleApplication.getProductInstance().getGimbal().
                        rotate(new Rotation.Builder().pitch(pitchValue)
                                .mode(RotationMode.SPEED)
                                .yaw(Rotation.NO_ROTATION)
                                .roll(Rotation.NO_ROTATION)
                                .time(0)
                                .build(), new CommonCallbacks.CompletionCallback() {

                            @Override
                            public void onResult(DJIError error) {

                            }
                        });
            }
        }
    }

    public void save_message_in_file(String message_text) {
        try {
            logInFile(message_text);
            logInFile("\n");
        } catch (Exception e) {
            Log.e("Debug", "File write failed: " + e.toString());
        }
    }

    private void logInFile(String data) throws Exception {
        FileOutputStream stream = new FileOutputStream(logDataFile, true);
        stream.write(data.getBytes());
        stream.close();
    }

    private void createLogFile(String filepath) {
        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
        logDataFile = new File(path, filepath);
        try {
            logInFile("----------------------------------------------------\n");
        } catch (Exception e) {
            Log.e("Debug", "File write failed: " + e.toString());
        }
    }

    private void setUpImageListener() {
        FlightPageView.MyImageProcessing img = new FlightPageView.MyImageProcessing();
    }

    private void setUpCoordinateListener() {
        FlightPageView.MyShowCoordinate coor = new FlightPageView.MyShowCoordinate();
    }

    private void setUpCollisionListener() {
        FlightPageView.MyCollisionAvoidance coll = new FlightPageView.MyCollisionAvoidance();
    }

    private class MyShowCoordinate {
        private Handler hUpdate;
        private Runnable rUpdate;

        public MyShowCoordinate() { // Constructor
            hUpdate = new Handler();
            rUpdate = new Runnable() {
                @Override
                public void run() {
                    //related to read coordinate
                    LocationCoordinate3D coordinate = flightController.getState().getAircraftLocation();

                    Long currentTime = Calendar.getInstance().getTimeInMillis();
                    //labels: time - worldX - worldY - worldZ - alt - velocityX - velocityY - velocityZ - roll - pitch - yaw - throttle
                    String data = currentTime + "~" + coordinate.getLatitude() + "~" + coordinate.getLongitude() + "~" + coordinate.getAltitude() + "~" + mDist + "~" +
                            flightController.getState().getVelocityX() + "~" + flightController.getState().getVelocityY() + "~" + flightController.getState().getVelocityZ() + "~" +
                            roll + "~" + pitch + "~" + yaw + "~" + throttle + "\n";
                    try {
                        logInFile(data);
                    } catch (Exception e) {
                        Log.e("Debug", "File write failed: " + e.toString());
                    }
                }
            };

            Thread tUpdate = new Thread() {
                public void run() {
                    while (true) {
                        hUpdate.post(rUpdate);
                        try {
                            sleep(200);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                }
            };
            tUpdate.start();
        }
    }

    private class MyCollisionAvoidance {
        private Handler hUpdate;
        private Runnable rUpdate;

        public MyCollisionAvoidance() { // Constructor
            hUpdate = new Handler();
            rUpdate = new Runnable() {
                @Override
                public void run() {
                    Bitmap bmp32 = primaryVideoFeed.getBitmap();

                    //Initialize python
                    if (!Python.isStarted()) {
                        Python.start(new AndroidPlatform(FlightPageView.this));
                    }

                    if (!shutdown_collison && bmp32 != null) {
//                        Long t1 = Calendar.getInstance().getTimeInMillis();
                        if (nn_params == null) {
                            load_NN_params();
                        }
                        String imageString = getStringImage(bmp32);
                        Python py = Python.getInstance();
                        PyObject test = py.getModule("myTest");
                        PyObject answer = test.callAttr("test_simple", imageString, nn_params.asList().get(0), nn_params.asList().get(1), nn_params.asList().get(2), nn_params.asList().get(3), nn_params.asList().get(4), nn_params.asList().get(5));

                        if (answer.asList().get(0).toBoolean() == true) {
                            System.out.println("__COLLISION__");
                            String im = answer.asList().get(1).toString();
                            Bitmap bmp = getBitmapFromString(im.substring(2, im.length() - 1));
                            MediaStore.Images.Media.insertImage(FlightPageView.this.getContentResolver(), bmp32, null, null);
                            MediaStore.Images.Media.insertImage(FlightPageView.this.getContentResolver(), bmp, null, null);

                            shoot_single_photo();

                            ToastUtils.setResultToToast("collision found");
                            auto_land();
//                            dir_message.setText("collision found");

                            //log file
                            save_message_in_file("collision found");
                            shutdown_collison = true;
                        }
//                        Long t2 = Calendar.getInstance().getTimeInMillis();
//                        Log.d("Debug", "Collision time: " + String.valueOf(t2 - t1));
                    }
                }

                public void load_NN_params() {
                    Python py = Python.getInstance();
                    PyObject NN = py.getModule("NN");
                    nn_params = NN.callAttr("load_NN");
                }

                public String getStringImage(Bitmap bmp) {
                    ByteArrayOutputStream baos = new ByteArrayOutputStream();
                    bmp.compress(Bitmap.CompressFormat.PNG, 100, baos);
                    byte[] imageBytes = baos.toByteArray();
                    String encodedImg = android.util.Base64.encodeToString(imageBytes, Base64.DEFAULT);
                    return encodedImg;
                }

                public Bitmap getBitmapFromString(String encodedImg) {
                    byte[] decodedString = Base64.decode(encodedImg, Base64.DEFAULT);
                    return BitmapFactory.decodeByteArray(decodedString, 0, decodedString.length);
                }
            };

            Thread tUpdate = new Thread() {
                public void run() {
                    while (true) {
                        hUpdate.post(rUpdate);
                        try {
                            sleep(4000);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                }
            };
            tUpdate.start();
        }
    }

    private class MyImageProcessing {
        private Handler hUpdate;
        private Runnable rUpdate;

        public MyImageProcessing() { // Constructor
            hUpdate = new Handler();
            rUpdate = new Runnable() {
                @Override
                public void run() {
                    Bitmap bmp32 = primaryVideoFeed.getBitmap();

                    if (bmp32 != null) {
                        Mat src = new Mat();
                        Utils.bitmapToMat(bmp32, src);

//                        Log.d("Debug", "h: " + String.valueOf(src.rows()));
//                        Log.d("Debug", "w: " + String.valueOf(src.cols()));

//                        Long t1 = Calendar.getInstance().getTimeInMillis();
//                        Long t2 = Calendar.getInstance().getTimeInMillis();
//                        Log.d("Debug", "short time: " + String.valueOf(t2 - t1));

//                        Size sz =   new Size(480, 240);
//                        Imgproc.resize( croppedimage, resizeimage, sz )

                        Mat hsv = new Mat(src.rows(), src.cols(), CvType.CV_8U, new Scalar(1));
                        Imgproc.cvtColor(src, hsv, Imgproc.COLOR_RGB2HSV);
                        //make binary image
                        Mat blue_bin = new Mat(src.rows(), src.cols(), CvType.CV_8U, new Scalar(1));
                        Mat orange_bin = new Mat(src.rows(), src.cols(), CvType.CV_8U, new Scalar(1));
                        Core.inRange(hsv, new Scalar(85, 75, 60), new Scalar(110, 255, 255), blue_bin);  //BLUE
                        Core.inRange(hsv, new Scalar(0, 120, 60), new Scalar(25, 245, 255), orange_bin);  //ORANGE
                        //image filtering
                        Size si = new Size(9, 9); // it was 11
                        Mat strel = getStructuringElement(Imgproc.MORPH_RECT, si);
                        Imgproc.morphologyEx(blue_bin, blue_bin, Imgproc.MORPH_DILATE, strel);
                        Imgproc.morphologyEx(orange_bin, orange_bin, Imgproc.MORPH_DILATE, strel);
                        Mat bin = new Mat(src.rows(), src.cols(), CvType.CV_8U, new Scalar(1));
                        Core.bitwise_and(blue_bin, orange_bin, bin);
                        if (should_save_photo) {
                            MediaStore.Images.Media.insertImage(FlightPageView.this.getContentResolver(), bmp32, null, null);
                            myCounter++;
//                            if(myCounter == 5){
//                                should_save_photo = false;
//                                myCounter = 0;
//                            }
                            should_save_photo = false;
                        }
//                        //show binary image
//                        Bitmap bmp = Bitmap.createBitmap(src.cols(), src.rows(), Bitmap.Config.ARGB_8888);
//                        Utils.matToBitmap(bin, bmp);
//                        processedImg.setImageBitmap(bmp);

                        //find square in image according to noise removal
                        List<MatOfPoint> rects = findSquare(bin, 500); //TODO check limit
                        setUpListeners();
                        mDist = 0;

                        if (start_landing_process) {
                            ToastUtils.setResultToToast("auto landing started");
                            shutdown_collison = true;

//                            dir_message.setText("start_landing_process");
                            if (!writeLandingLog) {
                                save_message_in_file("start_landing_process");
                                writeLandingLog = true;
                            }
                            if (rects.size() != 0) {
                                int res = findRealSquare(rects);
                                Mat croppedImage = hsv.submat(Imgproc.boundingRect(rects.get(res)));
                                mDist = findDist(croppedImage, 300, 2);  //TODO check limit
                                if (mDist != -1) {
                                    Point center = getCenterByMoment(rects, res);
                                    notFound_Timer = 0;
//                                    onceSeen = true;
                                    move_horizontal(center, bin.rows(), bin.cols());
                                }
                            } else {
//                                if (onceSeen) {  //it means that if I once saw it before I try searching
                                //there is no shape
                                if (notFound_Timer == 0) {
                                    startTime = System.currentTimeMillis();
                                    notFound_Timer = 1;
                                }
                                endTime = System.currentTimeMillis();
                                //square
                                squareMove_horizontal();
//                                }
                            }
                        } else if (start_detecting_process) {
                            ToastUtils.setResultToToast("find the floor");

                            if (rects.size() != 0) {
                                notFound_Timer = 0;
//                                dir_message.setText("start_detecting_process");
                                save_message_in_file("start_detecting_process");
//                                onceSeen2 = true;
                                //first QR should be in the center part of the image
                                Point center = getCenterByMoment(rects, findRealSquare(rects));
                                boolean is_aligned = move_vertical(center, bin.rows(), bin.cols());

                                if (is_aligned) {
                                    Rect bigROI = Imgproc.boundingRect(rects.get(findRealSquare(rects)));
                                    Mat croppedImage_bgr = src.submat(bigROI);
                                    //scan qr code
                                    Bitmap bmp_qr = Bitmap.createBitmap(croppedImage_bgr.cols(), croppedImage_bgr.rows(), Bitmap.Config.ARGB_8888);
                                    Utils.matToBitmap(croppedImage_bgr, bmp_qr);
                                    String result = scanQRImage(bmp_qr);
                                    //TODO add number textfield and change here
//                                  if (result != null && result.equalsIgnoreCase("1"))

//                                Log.d("Debug", "-- check roi -- w: " + bigROI.width + " h: " + bigROI.height);
//

                                    Log.d("Debug", bigROI.x + " " + bigROI.y + " " + bigROI.width + " " + bigROI.height);

                                    if ((bigROI.x - 100) > 0)
                                        bigROI.x = bigROI.x - 95;
                                    else
                                        bigROI.x = 5;
                                    if ((bigROI.y - 100) > 0)
                                        bigROI.y = bigROI.y - 95;
                                    else
                                        bigROI.y = 5;

                                    int newY = bigROI.height + bigROI.y + 200;
                                    int newX = bigROI.width + bigROI.x + 200;

                                    if (newX < hsv.cols())
                                        bigROI.width = bigROI.width + 195;
                                    else
                                        bigROI.width = bigROI.width + 195 - Math.abs(newX - hsv.cols());
                                    if (newY < hsv.rows())
                                        bigROI.height = bigROI.height + 195;
                                    else
                                        bigROI.height = bigROI.height + 195 - Math.abs(newY - hsv.rows());

                                    Log.d("Debug", bigROI.x + " " + bigROI.y + " " + (bigROI.width + bigROI.x) + " " + (bigROI.height + bigROI.y));
                                    Log.d("Debug", hsv.rows() + " " + hsv.cols());


                                    Mat croppedImage_hsv = hsv.submat(bigROI);

                                    mDist = findDist(croppedImage_hsv, 500, 1);
                                    if (mDist != -1) {
                                        if (mDist >= 300) {
                                            roll = search_velocity_horizontal;
                                            pitch = 0;
                                            throttle = 0;
                                        } else {
                                            //gimbal
                                            save_message_in_file("find dist to qr: " + mDist);

                                            roll = 0;
                                            pitch = 0;
                                            throttle = 0;
                                            camera_down();
                                            try {
                                                Thread.sleep(4000);
                                            } catch (InterruptedException e) {
                                                e.printStackTrace();
                                            }
                                            stop_camera_rotation();
                                            start_detecting_process = false;
                                            start_landing_process = true;
                                            shutdown_collison = true;
                                        }
                                    }
                                }
                            } else {
//                                if (onceSeen2) {
                                if (notFound_Timer == 0) {
                                    startTime = System.currentTimeMillis();
                                    notFound_Timer = 1;
                                }
                                endTime = System.currentTimeMillis();
                                //square
                                squareMove_vertical();
//                                }
                            }
                        }
                    }
                }

                public void squareMove_horizontal() {
//                    dir_message.setText("square horizontal");
                    switch (last_dir) {
                        case FRONT:
                            pitch = 0;
                            throttle = 0;
                            roll = search_velocity_horizontal;
                            wasInOtherCases = true;
                            start_square_move();
                            break;
                        case BACK:
                            pitch = 0;
                            throttle = 0;
                            roll = -1 * search_velocity_horizontal;
                            wasInOtherCases = true;
                            start_square_move();
                            break;
                        case RIGHT:
                            roll = 0;
                            throttle = 0;
                            pitch = search_velocity_horizontal;
                            wasInOtherCases = true;
                            start_square_move();
                            break;
                        case LEFT:
                            roll = 0;
                            throttle = 0;
                            pitch = -1 * search_velocity_horizontal;
                            wasInOtherCases = true;
                            start_square_move();
                            break;
                        case NONE:
                            int number;
                            if (wasInOtherCases) {
                                number = 3;
                            } else {
                                number = 1;
                            }
                            if ((endTime - startTime) <= number * search_duration) {  //right
                                roll = 0;
                                throttle = 0;
                                pitch = search_velocity_horizontal;
                            } else if ((endTime - startTime) <= (number + 1) * search_duration) { //back
                                pitch = 0;
                                throttle = 0;
                                roll = -1 * search_velocity_horizontal;
                            } else if ((endTime - startTime) <= (number + 2) * search_duration) { //left
                                roll = 0;
                                throttle = 0;
                                pitch = -1 * search_velocity_horizontal;
                            } else if ((endTime - startTime) <= (number + 3) * search_duration) { //front
                                pitch = 0;
                                throttle = 0;
                                roll = search_velocity_horizontal;
                            } else {
                                //go Up to see more space
                                pitch = 0;
                                roll = 0;
                                throttle = search_velocity_vertical;
                                try {
                                    Thread.sleep(500);
                                } catch (InterruptedException e) {
                                    e.printStackTrace();
                                }
                                throttle = 0;
                                notFound_Timer = 0;
                                wasInOtherCases = false;
                            }
                        default:
                    }
                }

                public void squareMove_vertical() {
//                    dir_message.setText("square vertical");
                    if ((endTime - startTime) <= 1 * search_duration) { //up
                        pitch = 0;
                        roll = 0;
                        throttle = search_velocity_vertical;
                    } else if ((endTime - startTime) <= 2 * search_duration) {  //right
                        roll = 0;
                        throttle = 0;
                        pitch = search_velocity_horizontal;
                    } else if ((endTime - startTime) <= 3 * search_duration) { //down
                        pitch = 0;
                        roll = 0;
                        throttle = -1 * search_velocity_vertical;
                    } else if ((endTime - startTime) <= 4 * search_duration) { //left
                        roll = 0;
                        throttle = 0;
                        pitch = -1 * search_velocity_horizontal;
                    } else {
                        //go Back to see more space
                        pitch = 0;
                        throttle = 0;
                        roll = -1 * search_velocity_horizontal;
                        try {
                            Thread.sleep(500);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        roll = 0;
                        notFound_Timer = 0;
                    }
                }

                public void move_horizontal(Point center, int width, int height) {
                    wasInOtherCases = false;
//                    dir_message.setText("horizontal");
                    if (center.y < (double) 2 * width / 5) {
                        //go Front
                        last_dir = FlightPageView.direction.FRONT;
                        throttle = 0;
                        pitch = 0;
                        roll = search_velocity_horizontal;
                    } else if (center.y > (double) 3 * width / 5) {
                        //go Back
                        last_dir = FlightPageView.direction.BACK;
                        throttle = 0;
                        pitch = 0;
                        roll = -1 * search_velocity_horizontal;
                    } else if (center.x > (double) 3 * height / 5) {
                        //go Right
                        last_dir = FlightPageView.direction.RIGHT;
                        throttle = 0;
                        roll = 0;
                        pitch = search_velocity_horizontal;
                    } else if (center.x < (double) 2 * height / 5) {
                        //go Left
                        last_dir = FlightPageView.direction.LEFT;
                        throttle = 0;
                        roll = 0;
                        pitch = -1 * search_velocity_horizontal;
                    } else {
                        if (mDist >= 50) {
                            roll = 0;
                            pitch = 0;
                            throttle = -1 * search_velocity_vertical;
                            try {
                                Thread.sleep(500);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                        } else {
                            roll = 0;
                            pitch = 0;
                            throttle = 0;
                            try {
                                Thread.sleep(100);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            // for at least one more frame we be at center
                            if ((center.x >= (double) 2 * height / 5) && (center.x <= (double) 3 * height / 5) && (center.y >= (double) 2 * width / 5) && (center.y <= (double) 3 * width / 5)) {
                                save_message_in_file("land");
                                auto_land();
                                start_landing_process = false;
                            }
                        }
                    }
                }

                public boolean move_vertical(Point center, int width, int height) {
                    wasInOtherCases = false;
//                    dir_message.setText("vertical");
                    if (center.x > (double) 3 * height / 5) {
                        //go Right
                        Log.d("Debug", "checkkkkk right");
                        last_dir = FlightPageView.direction.RIGHT;
                        throttle = 0;
                        roll = 0;
                        pitch = search_velocity_horizontal;
                        return false;
                    } else if (center.x < (double) 2 * height / 5) {
                        //go Left
                        Log.d("Debug", "checkkkkk left");
                        last_dir = FlightPageView.direction.LEFT;
                        throttle = 0;
                        roll = 0;
                        pitch = -1 * search_velocity_horizontal;
                        return false;
                    } else {
                        return true;
                    }
                }

                public void start_square_move() {
                    if ((endTime - startTime) >= 2 * search_duration && (endTime - startTime) < 3 * search_duration) {
                        last_dir = FlightPageView.direction.NONE;
                    }
                }

                public double findDist(Mat hsv_roi, int noise_lim, int key) {
                    Mat blue_bin = new Mat(hsv_roi.rows(), hsv_roi.cols(), CvType.CV_8U, new Scalar(1));
                    Core.inRange(hsv_roi, new Scalar(85, 75, 60), new Scalar(110, 255, 255), blue_bin);  //BLUE

                    List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
                    Imgproc.findContours(blue_bin, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);
                    double min_area = Double.MAX_VALUE;
                    int min_index = -1;

                    double max_area = 0;
                    int max_index = -1;

                    ArrayList<Integer> indexes = new ArrayList<>();

                    for (int i = 0; i < contours.size(); i++) {
                        double area = Imgproc.contourArea(contours.get(i));
                        if (area > noise_lim) {
                            double epsilon = 0.1 * Imgproc.arcLength(new MatOfPoint2f(contours.get(i).toArray()), true);
                            MatOfPoint2f approx = new MatOfPoint2f();
                            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), approx, epsilon, true);
                            if (approx.toArray().length == 4) {
                                indexes.add(i);
                                if (area < min_area) {
                                    min_area = area;
                                    min_index = i;
                                }
                                if (area > max_area) {
                                    max_area = area;
                                    max_index = i;
                                }
                            }
                        }
                    }
                    double dist = -1;
                    if (key == 1) { //qr
                        if (max_index != -1) {
                            dist = 19128 * Math.pow(max_area, -0.5);
                        }
                    } else { //landing
                        if (min_index != -1) {
                            dist = 9189.1 * Math.pow(min_area, -0.587);
                        }
                    }

                    return dist;
                }

                public List<MatOfPoint> findSquare(Mat bin, int nois_lim) {
                    //connected component
                    List<MatOfPoint> rects = new ArrayList<MatOfPoint>();
                    List<MatOfPoint> r = new ArrayList<MatOfPoint>();
                    List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
                    Imgproc.findContours(bin, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);

                    for (int i = 0; i < contours.size(); i++) {
                        double area = Imgproc.contourArea(contours.get(i));
//                        Log.d("Debug", "are: " + area);
                        if (area > nois_lim) {
                            double epsilon = 0.1 * Imgproc.arcLength(new MatOfPoint2f(contours.get(i).toArray()), true);
                            MatOfPoint2f approx = new MatOfPoint2f();
                            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), approx, epsilon, true);
                            if (approx.toArray().length == 4) {
                                rects.add(new MatOfPoint(approx.toArray()));
                            } else if (approx.toArray().length == 2) {
                                r.add(new MatOfPoint(approx.toArray()));
                            }
                        }
                    }
                    return rects;
                }

                public int findRealSquare(List<MatOfPoint> contours) {
                    int index = 0;

                    if (contours.size() == 1)
                        return 0;
                    else if (contours.size() == 2) { //min
                        if (Imgproc.contourArea(contours.get(0)) < Imgproc.contourArea(contours.get(1)))
                            index = 0;
                        else
                            index = 1;
                    } else {
                        double max_area = 0;
                        for (int i = 0; i < contours.size(); i++) {  //max
                            double area = Imgproc.contourArea(contours.get(i));
                            if (max_area < area) {
                                max_area = area;
                                index = i;
                            }
                        }
                    }
                    return index;
                }

                public Point getCenterByMoment(List<MatOfPoint> contours, int index) {
                    Moments m1;
                    m1 = Imgproc.moments(contours.get(index));

                    return new Point(m1.m10 / (m1.m00 + 1e-5), m1.m01 / (m1.m00 + 1e-5));
                }

                public String scanQRImage(Bitmap bMap) {
                    String contents = null;

                    int[] intArray = new int[bMap.getWidth() * bMap.getHeight()];
                    //copy pixel data from the Bitmap into the 'intArray' array
                    bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());

                    LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArray);
                    BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));

                    Reader reader = new MultiFormatReader();
                    try {
                        Result result = reader.decode(bitmap);
                        contents = result.getText();
                    } catch (Exception e) {
                        Log.e("QrTest", "Error decoding barcode", e);
                    }
                    return contents;
                }
            };

            Thread tUpdate = new Thread() {
                public void run() {
                    while (true) {
                        hUpdate.post(rUpdate);
                        try {
                            sleep(200);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                }
            };
            tUpdate.start();
        }
    }

    public void getDistance(LocationCoordinate2D home, LocationCoordinate2D qr) {
        double pi = 3.14;
        double lat1 = home.getLatitude() * pi / 180;
        double lat2 = qr.getLatitude() * pi / 180;
        double lon1 = home.getLongitude() * pi / 180;
        double lon2 = qr.getLongitude() * pi / 180;
        double deltaLat = lat2 - lat1;
        double deltaLon = lon2 - lon1;
        double x = deltaLon * Math.cos((lat1 + lat2) / 2);
        double y = deltaLat;
        x *= 10000000;
        y *= 10000000;
        Log.d("Debug", "getDistance: " + x + " " + y);
        fb = (float) y;
        rl = (float) x;

    }

}
