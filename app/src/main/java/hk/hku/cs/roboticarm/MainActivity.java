package hk.hku.cs.roboticarm;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;

import android.Manifest;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.os.Handler;
import android.os.ParcelUuid;
import android.os.Parcelable;
import android.util.Log;
import android.view.SurfaceView;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.FrameLayout;
import android.widget.ListView;
import android.widget.NumberPicker;
import android.widget.Spinner;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.DMatch;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.DescriptorExtractor;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.Features2d;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.Timer;
import java.util.TimerTask;
import java.util.UUID;

public class MainActivity extends AppCompatActivity implements CameraBridgeViewBase.CvCameraViewListener2 {

    //declare the variables

    Switch switch_steppers;
    Switch switch_fan;

    Button buttonCalibration;

    NumberPicker np_gripper_close;
    NumberPicker np_gripper_open;
    
    BluetoothAdapter bluetooth;
    String address;
    String name;
    private Set<BluetoothDevice> pairedDevices;
    private static final int ENABLE_BLUETOOTH = 1;
    private ArrayList<BluetoothDevice> deviceList = new ArrayList<BluetoothDevice>();
    String TAG = MainActivity.class.getSimpleName();
    String dStarted = BluetoothAdapter.ACTION_DISCOVERY_STARTED;
    String dFinished = BluetoothAdapter.ACTION_DISCOVERY_FINISHED;

    private BluetoothSocket transferSocket;
    boolean listening = false;
    boolean writing;

    BluetoothDevice device;
    BluetoothSocket clientSocket;

    double homeX;
    double homeY;
    double homeZ;

    volatile double currentX;
    volatile double currentY;
    volatile double currentZ;

    private CameraBridgeViewBase cameraView;

    Mat mRgba;
    Mat mRgbaF;
    Mat mRgbaT;

    Mat matHSV;
    Mat matWhite;
    Mat matAll;
    Mat matHoughLines;
    Mat matGreen;

    int count;

    double flipperInX;
    double flipperInY;
    double flipperInZ;

    double flipperOutX;
    double flipperOutY;
    double flipperOutZ;

    double newChessX;
    double newChessY;
    double newChessZ;

    double transitionFlipperX;
    double transitionFlipperY;
    double transitionFlipperZ;

    double transitionChessBoardX;
    double transitionChessBoardY;
    double transitionChessBoardZ;

    ArrayList<String> chessBoard;
    ArrayList<String> newChessBoard;
    ArrayList<Integer> whiteCounter;
    ArrayList<Integer> blackCounter;
    ArrayList<Integer> noneCounter;

    int w;
    int h;

    Switch autoSwitch;
    Switch aiFirstSwitch;

    boolean isAuto;
    String turnPlayer;
    int dst;
    int chessNo;
    int chessNoDetected;
    int newChessPos;
    int endCount;
    volatile boolean isMoving;
    Handler handler;
    Timer timer;

    ArrayList<Point3D> referencePts;
    int referencePtsCounter;

    Runnable autoLoop = new Runnable() {

        @Override
        public void run() {

            //do this part if automatic mode is on and the robotic arm is not moving
            if (isAuto && !isMoving){
                //Log.d(TAG, "Auto");
                //Log.d("runnable", "turnPlayer: " + turnPlayer);
                //Log.d("runnable", "chessNo: " + chessNo + " chessNoDetected: " + chessNoDetected);

                //start
                if (chessNo == 0){
                    for (int i=0; i<chessBoard.size(); i++){
                        if (chessBoard.get(i) != newChessBoard.get(i)){
                            newChessPos = i;
                        }
                        chessBoard.set(i, newChessBoard.get(i));
                    }
                    chessNo = chessNoDetected;
                }

                //after placing a new chess
                if (chessNo < chessNoDetected){

                    //update the chessboard state and find the new chess index
                    for (int i=0; i<chessBoard.size(); i++){
                        if (chessBoard.get(i) != newChessBoard.get(i)){
                            newChessPos = i;
                        }
                        chessBoard.set(i, newChessBoard.get(i));
                        //Log.d("runnable", "chessNo increase: chessboard " + i + ": " + chessBoard.get(i));
                    }

                    chessNo = chessNoDetected;

                    //Log.d("runnable", "newChessPos: " + newChessPos);

                    //get the indices of the chesses to be flipped
                    ArrayList<Integer> flipList = getFlipList(chessBoard, newChessPos, turnPlayer);

                    //Log.d("runnable", "flipList size: " + flipList.size());

                    //flip the chesses
                    flip(flipList);
                }

                //after flipping, check available moves
                else if (chessNo == chessNoDetected){

                    //Assumption: optimal detection environment
                    //update chessboard state
                    for (int i=0; i<chessBoard.size(); i++){
                        if (chessBoard.get(i) != newChessBoard.get(i)){
                            chessBoard.set(i, newChessBoard.get(i));
                        }
                        //Log.d("runnable", "equal chessNo: chessboard " + i + ": " + chessBoard.get(i));
                    }

                    ArrayList<Integer> placeablePos = getPlaceable(chessBoard, turnPlayer);
                    /*
                    for (int i=0; i<placeablePos.size(); i++){
                        Log.d("runnable", turnPlayer + " Placeable: " + placeablePos.get(i));
                    }

                     */

                    //no available moves
                    if (placeablePos.size() == 0){
                        //swap turn player
                        if (turnPlayer == "White"){
                            //Log.d("runnable", "Change turnplayer: white -> black");
                            turnPlayer = "Black";
                        }
                        else if (turnPlayer == "Black"){
                            //Log.d("runnable", "Change turnplayer: black -> white");
                            turnPlayer = "White";
                        }
                        endCount ++;
                        //both players have no available moves -> game ends
                        if (endCount == 2){
                            //autoSwitch.setChecked(false);
                            //Log.d("runnable", "No more moves");
                            isAuto = false;
                        }
                    }

                    //ai has available moves and control robotic arm to place chess at random available position
                    else if (placeablePos.size() > 0 && turnPlayer == "Black"){
                        dst = placeablePos.get(new Random().nextInt(placeablePos.size()));
                        //Log.d("runnable", "dst: " + dst);
                        //Log.d("runnable", "Place chess: " + dst);
                        if (dst != -1){
                            placeChess(dst);
                        }
                        //dst = -1;
                    }
                }
            }

            //Log.d("runnable", "Autoloop");
            handler.postDelayed(this, 1000);
        }
    };


    static {
        if (OpenCVLoader.initDebug()) {
            Log.d("OpenCV", "failure load");
        } else {
            Log.d("OpenCV", "successful load");
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        setContentView(R.layout.activity_main);

        //set switch steppers
        switch_steppers = findViewById(R.id.switchSteppers);
        switch_steppers.setChecked(false);
        switch_steppers.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    // The toggle is enabled
                    if (transferSocket != null){
                        sendMessages(transferSocket, "M17\r");
                        StringBuilder incoming = new StringBuilder();
                        listenForMessages(clientSocket, incoming);
                        Log.d(TAG, "Steppers on");
                    }
                } else {
                    // The toggle is disabled
                    if (transferSocket != null){
                        sendMessages(transferSocket, "M18\r");
                        StringBuilder incoming = new StringBuilder();
                        listenForMessages(clientSocket, incoming);
                        Log.d(TAG, "Steppers off");
                    }
                }
            }
        });

        //set switch fan
        switch_fan = findViewById(R.id.switchFan);
        switch_fan.setChecked(false);
        switch_fan.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    // The toggle is enabled
                    if (transferSocket != null){
                        sendMessages(transferSocket, "M106\r");
                        StringBuilder incoming = new StringBuilder();
                        listenForMessages(clientSocket, incoming);
                        Log.d(TAG, "Fan on");
                    }
                } else {
                    // The toggle is disabled
                    if (transferSocket != null){
                        sendMessages(transferSocket, "M107\r");
                        StringBuilder incoming = new StringBuilder();
                        listenForMessages(clientSocket, incoming);
                        Log.d(TAG, "Fan off");
                    }
                }
            }
        });

        //set button text
        buttonCalibration = (Button)findViewById(R.id.button_calibration);
        buttonCalibration.setText("Calibration 0");

        //set number picker value
        np_gripper_close = (NumberPicker)findViewById(R.id.numPicker_gripper_close);
        np_gripper_close.setMinValue(0);
        np_gripper_close.setMaxValue(10);
        np_gripper_close.setValue(10);
        np_gripper_close.setOnValueChangedListener(numPickerOnValueChange);

        //set number picker value
        np_gripper_open = (NumberPicker)findViewById(R.id.numPicker_gripper_open);
        np_gripper_open.setMinValue(0);
        np_gripper_open.setMaxValue(10);
        np_gripper_open.setValue(10);
        np_gripper_open.setOnValueChangedListener(numPickerOnValueChange);

        //set home position to (0, 180, 180)
        homeX = 0;
        homeY = 180;
        homeZ = 180;

        //set current position of the robotic arm to home position
        currentX = homeX;
        currentY = homeY;
        currentZ = homeZ;

        //initialize bluetooth attributes
        bluetooth = BluetoothAdapter.getDefaultAdapter();
        initBluetooth();

        ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.ACCESS_FINE_LOCATION}, 1);

        device = bluetooth.getRemoteDevice("AB:90:78:56:4B:ED");

        //set camera values
        cameraView = (JavaCameraView) findViewById(R.id.cameraView);
        cameraView.setMaxFrameSize(1080, 1980);
        //cameraView.setLayoutParams(new FrameLayout.LayoutParams(1080, 1980));
        cameraView.setVisibility(SurfaceView.VISIBLE);
        cameraView.setCvCameraViewListener(this);

        //initialize handler and timer
        handler = new Handler();
        timer = new Timer();

        //set switch to check who take a move first
        aiFirstSwitch = (Switch)findViewById(R.id.switchTurnPlayer);
        aiFirstSwitch.setChecked(false);

        //set switch for automatic mode
        autoSwitch = (Switch)findViewById(R.id.switchAuto);
        autoSwitch.setChecked(false);
        autoSwitch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    //set all the values for the start of a game

                    //Log.d(TAG, "Checked");
                    isAuto = true;
                    if (!aiFirstSwitch.isChecked()){
                        turnPlayer = "White";
                    }
                    else{
                        turnPlayer = "Black";
                    }
                    chessNo = 0;
                    //chessNoDetected = 0;
                    count = 0;
                    newChessPos = 0;
                    endCount = 0;
                    isMoving = false;
                    handler.postDelayed(autoLoop, 1000);
                    //timer.cancel();
                } else {
                    //turn off the automatic mode

                    //Log.d(TAG, "Not Checked");
                    isAuto = false;
                    handler.removeCallbacks(autoLoop);
                    //timer.cancel();
                }
            }
        });

        //initialize variables
        isAuto = autoSwitch.isChecked();
        turnPlayer = "White";
        dst = -1;
        chessNo = 0;
        chessNoDetected = 0;
        isMoving = false;

        chessBoard = new ArrayList<String>();
        newChessBoard = new ArrayList<String>();
        referencePts = new ArrayList<Point3D>();
        whiteCounter = new ArrayList<Integer>();
        blackCounter = new ArrayList<Integer>();
        noneCounter = new ArrayList<Integer>();
        for (int i=0; i<8; i++){
            for (int j=0; j<8; j++){
                chessBoard.add("None");
                newChessBoard.add("None");
                whiteCounter.add(0);
                blackCounter.add(0);
                noneCounter.add(0);
            }
        }
        for (int i=0; i<15; i++){
            referencePts.add(new Point3D());
        }
    }

    public void onSaveInstanceState(Bundle savedInstanceState) {
        super.onSaveInstanceState(savedInstanceState);
    }

    //initialize bluetooth
    private void initBluetooth() {
        if (!bluetooth.isEnabled()) {
            // Bluetooth isn’t enabled, prompt the user to turn it on.
            Intent intent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(intent, ENABLE_BLUETOOTH);
        }
        else {
            Toast.makeText(getApplicationContext(), "Bluetooth is enabled", Toast.LENGTH_LONG).show();
        }
    }

    //check if bluetooth is turned on
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode == ENABLE_BLUETOOTH)
            if (resultCode == RESULT_OK) {
                Toast.makeText(getApplicationContext(), "Bluetooth is enabled", Toast.LENGTH_LONG).show();
            }
            else{
                Toast.makeText(getApplicationContext(), "Please turn on Bluetooth", Toast.LENGTH_LONG).show();
                initBluetooth();
            }
    }

    private final BroadcastReceiver discoveryResult = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String remoteDeviceName = intent.getStringExtra(BluetoothDevice.EXTRA_NAME);
            BluetoothDevice remoteDevice = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
            deviceList.add(remoteDevice);
            Log.d(TAG, "Discovered "  + remoteDeviceName);
        }
    };

    //connect to server socket of bluetooth device
    private void connectToServerSocket(BluetoothDevice device, UUID uuid) {
        try{
            clientSocket = device.createRfcommSocketToServiceRecord(uuid);
            clientSocket.connect();
            Log.d(TAG, "Connect to " + uuid);

            // Start listening for messages.
            StringBuilder incoming = new StringBuilder();
            listenForMessages(clientSocket, incoming);

            // Add a reference to the socket used to send messages.
            transferSocket = clientSocket;

            //Write messages.
            //sendMessages(transferSocket);
        } catch (IOException e) {
            Log.e("bluetooth", "Bluetooth connectToServerSocket IOException", e);
        }
    }

    //disconnect from the socket
    private void closeSocket(BluetoothSocket socket){
        try{
            socket.close();
            Log.d("bluetooth", "Closed");
        }
        catch(IOException e){
            Log.e("bluetooth", "Bluetooth closeSocket IOException", e);
        }
    }

    //send message via output stream
    private void sendMessages(BluetoothSocket socket, String msg) {
        writing = true;
        byte[] buffer = new byte[1024];
        buffer = msg.getBytes();
        try {
            OutputStream os = socket.getOutputStream();
            os.write(buffer);
            Log.d("bluetooth", "msg: " + msg);
        } catch (IOException e) {
            Log.e("bluetooth", "sendMessages IOException", e);
        }
    }

    //receive message via input stream
    private String listenForMessages(BluetoothSocket socket, StringBuilder incoming) {
        listening = true;
        int bufferSize = 1024;
        byte[] buffer = new byte[bufferSize];
        String result = "";
        try {
            InputStream is = socket.getInputStream();
            int bytesRead = -1;
            while (listening) {
                bytesRead = is.read(buffer);
                Log.d(TAG, "bytesread: " + bytesRead);
                if (bytesRead != -1) {

                    while ((bytesRead == bufferSize) &&
                            (buffer[bufferSize - 1] != 0)){
                        result = result + new String(buffer, 0, bytesRead - 1);
                        bytesRead = is.read(buffer);
                    }
                    result = result + new String(buffer, 0, bytesRead - 1);
                    incoming.append(result);
                    Log.d("InStream", result);
                    listening = false;
                }
                //socket.close();
            }
        } catch (IOException e) {
            Log.e("bluetooth", "listenForMessages IOException", e);
        }
        finally {
            return result;
        }
    }

    private final NumberPicker.OnValueChangeListener numPickerOnValueChange
            = new NumberPicker.OnValueChangeListener() {
        @Override
        public void onValueChange(NumberPicker view, int oldValue, int newValue) {
            //Toast.makeText(view.getContext(), String.valueOf(newValue), Toast.LENGTH_LONG).show();
        }
    };

    //connect to the server socket by UUID
    public void buttonOpen(View view) {
        //Toast.makeText(this, "Button open", Toast.LENGTH_LONG).show();
        if (clientSocket == null){
            connectToServerSocket(device, UUID.fromString("00001101-0000-1000-8000-00805f9b34fb"));
        }
    }

    //close the socket connection
    public void buttonClose(View view) {
        //Toast.makeText(this, "Button close", Toast.LENGTH_LONG).show();
        if (clientSocket != null){
            closeSocket(clientSocket);
        }
    }

    //move the robotic arm to transition flipper
    public void buttonTransitionFlipper(View view) {
        //Toast.makeText(this, "Button bottom", Toast.LENGTH_LONG).show();
        if (transferSocket != null){
            sendMessages(transferSocket, "G1 X" + String.valueOf(transitionFlipperX) + " Y" + String.valueOf(transitionFlipperY) + " Z" + String.valueOf(transitionFlipperZ) + "\r");
            StringBuilder incoming = new StringBuilder();
            String response = listenForMessages(clientSocket, incoming);
            currentX = transitionFlipperX;
            currentY = transitionFlipperY;
            currentZ = transitionFlipperZ;
        }
    }

    //move the robotic arm to home position
    public void buttonHome(View view) {
        //Toast.makeText(this, "Button home", Toast.LENGTH_LONG).show();
        if (transferSocket != null){
            sendMessages(transferSocket, "G1 X" + String.valueOf(homeX) + " Y" + String.valueOf(homeY) + " Z" + String.valueOf(homeZ) + "\r");
            StringBuilder incoming = new StringBuilder();
            String response = listenForMessages(clientSocket, incoming);
            currentX = homeX;
            currentY = homeY;
            currentZ = homeZ;
        }
    }

    //record the current position for calibration
    public void buttonCalibration(View view) {
        //Toast.makeText(this, "Button rest", Toast.LENGTH_LONG).show();
        if (transferSocket != null){
            //reference pt -> 8+7
            referencePts.set(referencePtsCounter, new Point3D(currentX, currentY, currentZ));
            if (referencePtsCounter < 14){
                referencePtsCounter ++;
            }
            else{
                referencePtsCounter = 0;
                transitionChessBoardX = (referencePts.get(0).get(0) + referencePts.get(14).get(0)) / 2;
                transitionChessBoardY = (referencePts.get(0).get(1) + referencePts.get(14).get(1)) / 2;
                transitionChessBoardZ = referencePts.get(0).get(2) + 40;
            }
            buttonCalibration.setText("Calibration " + String.valueOf(referencePtsCounter));
        }
    }

    //move the robotic arm to the transition state of the chessboard
    public void buttonTransitionChessboard(View view) {
        //Toast.makeText(this, "Button end stop", Toast.LENGTH_LONG).show();

        if (transferSocket != null){
            sendMessages(transferSocket, "G1 X" + String.valueOf(transitionChessBoardX) + " Y" + String.valueOf(transitionChessBoardY) + " Z" + String.valueOf(transitionChessBoardZ) + "\r");
            StringBuilder incoming = new StringBuilder();
            String response = listenForMessages(clientSocket, incoming);
            currentX = transitionChessBoardX;
            currentY = transitionChessBoardY;
            currentZ = transitionChessBoardZ;
        }
    }

    //turn off the vacuum to put down object
    public void buttonGripperOpen(View view) {
        //Toast.makeText(this, "Button gripper open", Toast.LENGTH_LONG).show();
        if (transferSocket != null){
            int gripperOpen = np_gripper_open.getValue();
            sendMessages(transferSocket, "M5 T" + gripperOpen + "\r");
            StringBuilder incoming = new StringBuilder();
            String response = listenForMessages(clientSocket, incoming);
        }
    }

    //turn on the vacuum to pick up object
    public void buttonGripperClose(View view) {
        //Toast.makeText(this, "Button gripper close", Toast.LENGTH_LONG).show();
        if (transferSocket != null){
            int gripperClose = np_gripper_close.getValue();
            sendMessages(transferSocket, "M3 T" + gripperClose + "\r");
            StringBuilder incoming = new StringBuilder();
            String response = listenForMessages(clientSocket, incoming);
        }
    }

    //move the robotic arm forward for short distance
    public void buttonPlusY(View view) {
        //Toast.makeText(this, "Button + (Y)", Toast.LENGTH_LONG).show();
        if (transferSocket != null){
            sendMessages(transferSocket, "G1 X" + String.valueOf(currentX) + " Y" + String.valueOf(currentY + 2.0) + " Z" + String.valueOf(currentZ) + "\r");
            StringBuilder incoming = new StringBuilder();
            String response = listenForMessages(clientSocket, incoming);
            currentY += 2;
        }
    }

    //move the robotic arm forward for long distance
    public void buttonYPlus(View view) {
        //Toast.makeText(this, "Button Y+", Toast.LENGTH_LONG).show();
        if (transferSocket != null){
            sendMessages(transferSocket, "G1 X" + String.valueOf(currentX) + " Y" + String.valueOf(currentY + 10.0) + " Z" + String.valueOf(currentZ) + "\r");
            StringBuilder incoming = new StringBuilder();
            String response = listenForMessages(clientSocket, incoming);
            Log.d("InStream", "response: " + response);
            currentY += 10;
        }
    }

    //move the robotic arm to the right for short distance
    public void buttonPlusX(View view) {
        //Toast.makeText(this, "Button + (X)", Toast.LENGTH_LONG).show();
        if (transferSocket != null){
            sendMessages(transferSocket, "G1 X" + String.valueOf(currentX + 2.0) + " Y" + String.valueOf(currentY) + " Z" + String.valueOf(currentZ) + "\r");
            StringBuilder incoming = new StringBuilder();
            String response = listenForMessages(clientSocket, incoming);
            currentX += 2;
        }
    }

    //move the robotic arm to the right for long distance
    public void buttonXPlus(View view) {
        //Toast.makeText(this, "Button X+", Toast.LENGTH_LONG).show();
        if (transferSocket != null){
            sendMessages(transferSocket, "G1 X" + String.valueOf(currentX + 10.0) + " Y" + String.valueOf(currentY) + " Z" + String.valueOf(currentZ) + "\r");
            StringBuilder incoming = new StringBuilder();
            String response = listenForMessages(clientSocket, incoming);
            currentX += 10;
        }
    }

    //move the robotic arm to the left for short distance
    public void buttonMinusX(View view) {
        //Toast.makeText(this, "Button - (X)", Toast.LENGTH_LONG).show();
        if (transferSocket != null){
            sendMessages(transferSocket, "G1 X" + String.valueOf(currentX - 2.0) + " Y" + String.valueOf(currentY) + " Z" + String.valueOf(currentZ) + "\r");
            StringBuilder incoming = new StringBuilder();
            String response = listenForMessages(clientSocket, incoming);
            currentX -= 2;
        }
    }

    //move the robotic arm to the left for long distance
    public void buttonXMinus(View view) {
        //Toast.makeText(this, "Button X-", Toast.LENGTH_LONG).show();
        if (transferSocket != null){
            sendMessages(transferSocket, "G1 X" + String.valueOf(currentX - 10.0) + " Y" + String.valueOf(currentY) + " Z" + String.valueOf(currentZ) + "\r");
            StringBuilder incoming = new StringBuilder();
            String response = listenForMessages(clientSocket, incoming);
            currentX -= 10;
        }
    }

    //move the robotic arm backward for short distance
    public void buttonMinusY(View view) {
        //Toast.makeText(this, "Button - (Y)", Toast.LENGTH_LONG).show();
        if (transferSocket != null){
            sendMessages(transferSocket, "G1 X" + String.valueOf(currentX) + " Y" + String.valueOf(currentY - 2.0) + " Z" + String.valueOf(currentZ) + "\r");
            StringBuilder incoming = new StringBuilder();
            String response = listenForMessages(clientSocket, incoming);
            currentY -= 2;
        }
    }

    //move the robotic arm backward for long distance
    public void buttonYMinus(View view) {
        //Toast.makeText(this, "Button Y-", Toast.LENGTH_LONG).show();
        if (transferSocket != null){
            sendMessages(transferSocket, "G1 X" + String.valueOf(currentX) + " Y" + String.valueOf(currentY - 10.0) + " Z" + String.valueOf(currentZ) + "\r");
            StringBuilder incoming = new StringBuilder();
            String response = listenForMessages(clientSocket, incoming);
            currentY -= 10;
        }
    }

    //move the robotic arm upward for short distance
    public void buttonPlusZ(View view) {
        //Toast.makeText(this, "Button + (Z)", Toast.LENGTH_LONG).show();
        if (transferSocket != null){
            sendMessages(transferSocket, "G1 X" + String.valueOf(currentX) + " Y" + String.valueOf(currentY) + " Z" + String.valueOf(currentZ + 2.0) + "\r");
            StringBuilder incoming = new StringBuilder();
            String response = listenForMessages(clientSocket, incoming);
            currentZ += 2;
        }
    }

    //move the robotic arm upward for long distance
    public void buttonZPlus(View view) {
        //Toast.makeText(this, "Button Z+", Toast.LENGTH_LONG).show();
        if (transferSocket != null){
            sendMessages(transferSocket, "G1 X" + String.valueOf(currentX) + " Y" + String.valueOf(currentY) + " Z" + String.valueOf(currentZ + 10.0) + "\r");
            StringBuilder incoming = new StringBuilder();
            String response = listenForMessages(clientSocket, incoming);
            currentZ += 10;
        }
    }

    //move the robotic arm downward for short distance
    public void buttonMinusZ(View view) {
        //Toast.makeText(this, "Button - (Z)", Toast.LENGTH_LONG).show();
        if (transferSocket != null){
            sendMessages(transferSocket, "G1 X" + String.valueOf(currentX) + " Y" + String.valueOf(currentY) + " Z" + String.valueOf(currentZ - 2.0) + "\r");
            StringBuilder incoming = new StringBuilder();
            String response = listenForMessages(clientSocket, incoming);
            currentZ -= 2;
        }
    }

    //move the robotic arm downward for long distance
    public void buttonZMinus(View view) {
        //Toast.makeText(this, "Button Z-", Toast.LENGTH_LONG).show();
        if (transferSocket != null){
            sendMessages(transferSocket, "G1 X" + String.valueOf(currentX) + " Y" + String.valueOf(currentY) + " Z" + String.valueOf(currentZ - 10.0) + "\r");
            StringBuilder incoming = new StringBuilder();
            String response = listenForMessages(clientSocket, incoming);
            currentZ -= 10;
        }
    }

    @Override
    public void onPause() {
        super.onPause();
        if (cameraView != null)
            cameraView.disableView();
    }

    public void onDestroy() {
        super.onDestroy();
        if (cameraView != null)
            cameraView.disableView();
    }

    @Override
    public void onPointerCaptureChanged(boolean hasCapture) {

    }

    //initialize variables when camera view starts
    @Override
    public void onCameraViewStarted(int width, int height) {
        w = width;
        h = height;
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mRgbaF = new Mat(height, width, CvType.CV_8UC4);
        mRgbaT = new Mat(width, width, CvType.CV_8UC4);
        matHSV = new Mat(width, height, CvType.CV_16UC4);
        matWhite = new Mat(width, height, CvType.CV_16UC4);
        matAll = new Mat(width, height, CvType.CV_16UC4);
    }

    @Override
    public void onCameraViewStopped() {
        mRgba.release();
    }

    //do for each frame captured
    //chessboard and chess detection by colors and shapes
    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        // TODO Auto-generated method stub

        //chessboard detection
        ArrayList<ArrayList<Point>> verticalPts = new ArrayList<ArrayList<Point>>();
        ArrayList<ArrayList<Point>> horizontalPts = new ArrayList<ArrayList<Point>>();
        mRgba = inputFrame.rgba();
        Imgproc.cvtColor(mRgba, matHSV, Imgproc.COLOR_BGR2HSV);

        matGreen = new Mat();
        Scalar minGreen = new Scalar ( 40, 127, 25 );
        Scalar maxGreen = new Scalar ( 70, 255, 255 );
        Core.inRange (matHSV, minGreen, maxGreen, matGreen);

        matHoughLines = new Mat();
        Imgproc.Canny(matGreen, matGreen, 150, 150, 3);

        Imgproc.HoughLines(matGreen, matHoughLines,1,Math.PI/180,90);
        //Log.d(TAG, "rows: " + String.valueOf(matHoughLines.rows()));
        //Log.d(TAG, "cols: " + String.valueOf(matHoughLines.cols()));
        for (int i=0; i<matHoughLines.rows(); i++){
            for (int j=0; j<matHoughLines.cols(); j++){
                double[] vec = matHoughLines.get(i, j);
                double r = vec[0];
                double theta = vec[1];
                //Log.d(TAG, "rho: " + String.valueOf(r));
                //Log.d(TAG, "theta: " + String.valueOf(theta));
                double sin = Math.sin(theta);
                double cos = Math.cos(theta);
                double x0 = cos * r;
                double y0 = sin * r;
                double x1 = x0 + 1000 * (-sin);
                double y1 = y0 + 1000 * (cos);
                double x2 = x0 - 1000 * (-sin);
                double y2 = y0 - 1000 * (cos);
                Point startPt = new Point(x1, y1);
                Point endPt = new Point(x2, y2);
                if (theta >= 0.0 && theta <=0.1 || theta <= 3.14 && theta >= 3.04){
                    ArrayList<Point> points = new ArrayList<Point>();
                    points.add(startPt);
                    points.add(endPt);
                    horizontalPts.add(new ArrayList<Point>(points));
                }
                else if (theta >= 1.47 && theta <= 1.67){
                    ArrayList<Point> points = new ArrayList<Point>();
                    points.add(startPt);
                    points.add(endPt);
                    verticalPts.add(new ArrayList<Point>(points));
                }
                //Imgproc.line(mRgba, start,end,new Scalar(255, 0, 0),2);
            }
        }

        //Log.d(TAG, "Horizontal: " + String.valueOf(horizontalPts.size()));
        //Log.d(TAG, "Vertical: " + String.valueOf(verticalPts.size()));

        ArrayList<Point> intersections = new ArrayList<Point>();
        for (int hori=0; hori<horizontalPts.size(); hori++){
            for (int vert=0; vert<verticalPts.size(); vert++){
                Point intersection = getIntersection(horizontalPts.get(hori).get(0), horizontalPts.get(hori).get(1), verticalPts.get(vert).get(0), verticalPts.get(vert).get(1));
                intersections.add(intersection);
                //Imgproc.circle(mRgba, intersection, 3, new Scalar(255, 0, 0), 2);
            }
        }

        //get cluster points
        ArrayList<Point> clusterPoints = getCluster(intersections);

        //Log.d(TAG, "cluster: " + clusterPoints.size());

        //sort clusters
        clusterPoints = sortCorners(clusterPoints);

        ///*
        //white chess detection
        ArrayList<Point> whiteLocations = new ArrayList<Point>();

        Imgproc.cvtColor( inputFrame.rgba(), matHSV, Imgproc.COLOR_BGR2HSV);

        Scalar minWhite = new Scalar ( 0, 0, 200 );
        Scalar maxWhite = new Scalar ( 360, 60, 255 );
        Core.inRange (matHSV, minWhite, maxWhite, matWhite);

        Mat circlesWhite = new Mat();

        Imgproc.blur(matWhite, matWhite, new Size(9, 9), new Point(2, 2));
        Imgproc.HoughCircles(matWhite, circlesWhite, Imgproc.CV_HOUGH_GRADIENT, 2, 40, 100, 70, 15, 30);

        //Log.d(TAG, "white: " + String.valueOf(circlesWhite.cols()));

        if (circlesWhite.cols() > 0){
            for (int x=0; x < circlesWhite.cols(); x++ ) {
                double circleCoord[] = circlesWhite.get(0, x);

                if (circleCoord == null) {
                    break;
                }

                Point center = new Point((int) circleCoord[0], (int) circleCoord[1]);
                int r = (int) circleCoord[2];

                Imgproc.circle(mRgba, center, 3, new Scalar(0, 0, 0), 5);
                Imgproc.circle(mRgba, center, r, new Scalar(0, 0, 0), 2);

                //Log.d(TAG, "White Coordinate: " + String.valueOf(circleVec[0]) + ", " + String.valueOf(circleVec[1]));

                Point whiteGrid = getGrid(clusterPoints, center);
                //chessBoard.set((int) (8 * whiteGrid.y + whiteGrid.x), "White");
                whiteLocations.add(whiteGrid);
            }
        }

        //black chess detection
        ArrayList<Point> blackLocations = new ArrayList<Point>();
        matAll = inputFrame.gray();
        Mat circlesAll = new Mat();
        //Imgproc.blur(matAll, matAll, new Size(9, 9), new Point(2, 2));
        Imgproc.HoughCircles(matAll, circlesAll, Imgproc.CV_HOUGH_GRADIENT, 2, 40, 100, 70, 15, 30);

        //Log.d(TAG, "all: " + String.valueOf(circlesAll.cols()));

        if (circlesAll.cols() > 0){
            for (int x=0; x < circlesAll.cols(); x++ ) {
                double circleCoord[] = circlesAll.get(0, x);

                if (circleCoord == null) {
                    break;
                }

                Point center = new Point((int) circleCoord[0], (int) circleCoord[1]);
                int r = (int) circleCoord[2];

                //Log.d(TAG, "Black Coordinate: " + String.valueOf(circleVec[0]) + ", " + String.valueOf(circleVec[1]));

                Point blackGrid = getGrid(clusterPoints, center);
                if (!whiteLocations.contains(blackGrid)) {
                    //chessBoard.set((int) (8 * blackGrid.y + blackGrid.x), "Black");
                    Imgproc.circle(mRgba, center, 3, new Scalar(255, 0, 0), 5);
                    Imgproc.circle(mRgba, center, r, new Scalar(255, 0, 0), 2);
                    blackLocations.add(blackGrid);
                }
            }
        }

        circlesWhite.release();
        circlesAll.release();

        //Log.d(TAG, "count: " + count);

        //count<11 -> color++ white->black->none(remaining)
        for (int x=0; x<8; x++){
            for (int y=0; y<8; y++){
                if (whiteLocations.contains(new Point(x, y))){
                    //chessBoard.set(8 * y + x, "White");
                    whiteCounter.set(8 * y + x, whiteCounter.get(8 * y + x) + 1);
                }

                else if (blackLocations.contains(new Point(x, y))){
                    //chessBoard.set(8 * y + x, "Black");
                    blackCounter.set(8 * y + x, blackCounter.get(8 * y + x) + 1);
                }

                else{
                    //chessBoard.set(8 * y + x, "None");
                    noneCounter.set(8 * y + x, noneCounter.get(8 * y + x) + 1);
                }
            }
        }

        //while the robotic arm is not moving, count++
        if (!isMoving){
            count ++;
        }
        //else set count to 10
        else{
            count = 10;
        }
        //Update chessBoard
        if (count == 10){
            //Log.d(TAG, "count = 10, update newChessBoard");
            chessNoDetected = 0;
            //Log.d(TAG, "chessboard size: " + chessBoard.size());
            for (int i=0; i<newChessBoard.size(); i++){
                if (!isMoving){
                    if (whiteCounter.get(i) > 5){
                        newChessBoard.set(i, "White");
                        chessNoDetected ++;
                    }

                    else if (blackCounter.get(i) > 5){
                        newChessBoard.set(i, "Black");
                        chessNoDetected ++;
                    }

                    else if (noneCounter.get(i) > 5){
                        newChessBoard.set(i, "None");
                    }
                }

                //Log.d(TAG, "chessBoard " + i + ": " + chessBoard.get(i));
                //Log.d(TAG, "newChessPos: " + newChessPos);
                //Log.d(TAG, "reset color counter");
                whiteCounter.set(i, 0);
                blackCounter.set(i, 0);
                noneCounter.set(i, 0);
            }

            count = 0;
        }
        //Log.d(TAG, "isAuto = " + isAuto);

        //set green frame to signal that the chessboard is captured fine
        if (clusterPoints.size() == 81){
            //Log.d(TAG, "Rectangle");
            Imgproc.rectangle(mRgba, new Point(0, 0), new Point(w, h), new Scalar(0, 255, 0), 3);
        }

        ///*
        // Rotate mRgba 90 degrees
        Core.transpose(mRgba, mRgbaT);
        Imgproc.resize(mRgbaT, mRgbaF, mRgbaF.size(), 0,0, 0);
        Core.flip(mRgbaF, mRgba, 1 );
        return mRgba;
         //*/

    }

    //MyTask to schedule messages that sends to the Arduino bluetooth receiver
    public class MyTask extends TimerTask {
        private String msg;
        private String action;
        private double x;
        private double y;
        private double z;

        public MyTask(String msg, double x, double y, double z) {
            this.msg = msg;
            this.action = "MOVE";
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public MyTask(String msg) {
            this.msg = msg;
            this.action = "GRIPPER/END";
        }

        @Override
        public void run() {
            if (msg != "" && transferSocket != null){
                sendMessages(transferSocket, msg);
                if (this.action == "MOVE"){
                    currentX = this.x;
                    currentY = this.y;
                    currentZ = this.z;
                }
                Log.d("task scheduler", "Send msg: " + msg);
            }
            else{
                isMoving = false;
                Log.d("task scheduler", "isMoving: " + isMoving);
            }
        }
    }

    //place a new chess at index dst
    public void placeChess(int dst) {
        isMoving = true;
        Log.d("placeChessTest", "set ismoving to true");

        int waitingTime = 2000;
        int delay = waitingTime;

        String msgMoveToTransitionFlipper = "G1 X" + String.valueOf(transitionFlipperX) + " Y" + String.valueOf(transitionFlipperY) + " Z" + String.valueOf(transitionFlipperZ) + "\r";
        timer.schedule(new MyTask(msgMoveToTransitionFlipper, transitionFlipperX, transitionFlipperY, transitionFlipperZ), delay);
        Log.d("placeChessTest", "move to transition flipper");

        delay += waitingTime;

        String msgMoveToNewChess2 = "G1 X" + String.valueOf(newChessX) + " Y" + String.valueOf(newChessY) + " Z" + String.valueOf(newChessZ + 50) + "\r";
        timer.schedule(new MyTask(msgMoveToNewChess2, newChessX, newChessY, newChessZ + 50), delay);
        Log.d("placeChessTest", "move to new chess");

        delay += waitingTime;

        String msgMoveToNewChess = "G1 X" + String.valueOf(newChessX) + " Y" + String.valueOf(newChessY) + " Z" + String.valueOf(newChessZ) + "\r";
        timer.schedule(new MyTask(msgMoveToNewChess, newChessX, newChessY, newChessZ), delay);
        Log.d("placeChessTest", "move to new chess");

        delay += waitingTime;

        int gripperClose = np_gripper_close.getValue();
        String msgGripperClose = "M3 T" + gripperClose + "\r";
        timer.schedule(new MyTask(msgGripperClose), delay);
        Log.d("placeChessTest", "gripper close");

        delay += waitingTime;

        timer.schedule(new MyTask(msgMoveToTransitionFlipper, transitionFlipperX, transitionFlipperY, transitionFlipperZ), delay);
        Log.d("placeChessTest", "move to transition flipper");

        delay += waitingTime;

        String msgMoveToTransitionChessBoard = "G1 X" + String.valueOf(transitionChessBoardX) + " Y" + String.valueOf(transitionChessBoardY) + " Z" + String.valueOf(transitionChessBoardZ) + "\r";
        timer.schedule(new MyTask(msgMoveToTransitionChessBoard, transitionChessBoardX, transitionChessBoardY, transitionChessBoardZ), delay);
        Log.d("placeChessTest", "move to transition chessboard");

        delay += waitingTime;

        ArrayList<Double> XYZ = getXYZ(dst);

        String msgMoveToTransitionChessBoard2 = "G1 X" + String.valueOf(XYZ.get(0)) + " Y" + String.valueOf(XYZ.get(1)) + " Z" + String.valueOf(transitionChessBoardZ) + "\r";
        timer.schedule(new MyTask(msgMoveToTransitionChessBoard2, XYZ.get(0), XYZ.get(1), transitionChessBoardZ), delay);

        delay += waitingTime;

        String msgMoveToChessBoard = "G1 X" + String.valueOf(XYZ.get(0)) + " Y" + String.valueOf(XYZ.get(1)) + " Z" + String.valueOf(XYZ.get(2) + 10) + "\r";
        timer.schedule(new MyTask(msgMoveToChessBoard, XYZ.get(0), XYZ.get(1), XYZ.get(2) + 10), delay);

        delay += waitingTime;

        int gripperOpen = np_gripper_open.getValue();
        String msgGripperOpen = "M5 T" + gripperOpen + "\r";
        timer.schedule(new MyTask(msgGripperOpen), delay);
        Log.d("placeChessTest", "gripper open");

        delay += waitingTime * 2;

        timer.schedule(new MyTask(msgMoveToTransitionChessBoard, transitionChessBoardX, transitionChessBoardY, transitionChessBoardZ), delay);
        Log.d("placeChessTest", "move to transition chessboard");

        delay += waitingTime;

        String msgMoveToRest = "G1 X" + String.valueOf(transitionFlipperX) + " Y" + String.valueOf(transitionFlipperY) + " Z" + String.valueOf(transitionFlipperZ) + "\r";
        timer.schedule(new MyTask(msgMoveToRest, transitionFlipperX, transitionFlipperY, transitionFlipperZ), delay);
        Log.d("flipTest", "move to rest");

        delay += waitingTime;

        timer.schedule(new MyTask(""), delay);
        Log.d("placeChessTest", "set ismoving to false");

        //dst = -1;
        //Log.d("placeChessTest", "set dst to -1");
        //isMoving = false;

        return;
    }

    //flip the chesses with the indices stored in flipList
    public void flip(ArrayList<Integer> flipList) {
        isMoving = true;
        Log.d("flipTest", "set ismoving to true");

        int waitingTime = 2000;
        int delay = waitingTime;

        String msgMoveToTransitionChessBoard = "G1 X" + String.valueOf(transitionChessBoardX) + " Y" + String.valueOf(transitionChessBoardY) + " Z" + String.valueOf(transitionChessBoardZ) + "\r";
        timer.schedule(new MyTask(msgMoveToTransitionChessBoard, transitionChessBoardX, transitionChessBoardY, transitionChessBoardZ), delay);
        Log.d("flipTest", "move to transition chessboard");

        delay += waitingTime;

        for (int i=0; i<flipList.size(); i++){
            ArrayList<Double> XYZ = getXYZ(flipList.get(i));

            String msgMoveToTransitionChessBoard2 = "G1 X" + String.valueOf(XYZ.get(0)) + " Y" + String.valueOf(XYZ.get(1)) + " Z" + String.valueOf(transitionChessBoardZ) + "\r";
            timer.schedule(new MyTask(msgMoveToTransitionChessBoard2, XYZ.get(0), XYZ.get(1), transitionChessBoardZ), delay);

            delay += waitingTime;

            String msgMoveToChessBoard = "G1 X" + String.valueOf(XYZ.get(0)) + " Y" + String.valueOf(XYZ.get(1)) + " Z" + String.valueOf(XYZ.get(2)) + "\r";
            timer.schedule(new MyTask(msgMoveToChessBoard, XYZ.get(0), XYZ.get(1), XYZ.get(2)), delay);

            delay += waitingTime;

            int gripperClose = np_gripper_close.getValue();
            String msgGripperClose = "M3 T" + gripperClose + "\r";
            timer.schedule(new MyTask(msgGripperClose), delay);
            Log.d("flipTest", "gripper close");

            delay += waitingTime;

            timer.schedule(new MyTask(msgMoveToTransitionChessBoard2, XYZ.get(0), XYZ.get(1), transitionChessBoardZ), delay);

            delay += waitingTime;

            timer.schedule(new MyTask(msgMoveToTransitionChessBoard, transitionChessBoardX, transitionChessBoardY, transitionChessBoardZ), delay);
            Log.d("flipTest", "move to transition chessboard");

            delay += waitingTime;

            String msgMoveToTransitionFlipper = "G1 X" + String.valueOf(transitionFlipperX) + " Y" + String.valueOf(transitionFlipperY) + " Z" + String.valueOf(transitionFlipperZ) + "\r";
            timer.schedule(new MyTask(msgMoveToTransitionFlipper, transitionFlipperX, transitionFlipperY, transitionFlipperZ), delay);
            Log.d("flipTest", "move to transition flipper");

            delay += waitingTime;

            String msgMoveToFlipperIn = "G1 X" + String.valueOf(flipperInX) + " Y" + String.valueOf(flipperInY) + " Z" + String.valueOf(flipperInZ) + "\r";
            timer.schedule(new MyTask(msgMoveToFlipperIn, flipperInX, flipperInY, flipperInZ), delay);
            Log.d("flipTest", "move to flipper in");

            delay += waitingTime;

            int gripperOpen = np_gripper_open.getValue();
            String msgGripperOpen = "M5 T" + gripperOpen + "\r";
            timer.schedule(new MyTask(msgGripperOpen), delay);
            Log.d("flipTest", "gripper open");

            delay += waitingTime * 2;

            timer.schedule(new MyTask(msgMoveToTransitionFlipper, transitionFlipperX, transitionFlipperY, transitionFlipperZ), delay);
            Log.d("flipTest", "move to transition flipper");

            delay += waitingTime;

            String msgMoveToFlipperOut = "G1 X" + String.valueOf(flipperOutX) + " Y" + String.valueOf(flipperOutY) + " Z" + String.valueOf(flipperOutZ) + "\r";
            timer.schedule(new MyTask(msgMoveToFlipperOut, flipperOutX, flipperOutY, flipperOutZ), delay);
            Log.d("flipTest", "move to flipper out");

            delay += waitingTime;

            gripperClose = np_gripper_close.getValue();
            msgGripperClose = "M3 T" + gripperClose + "\r";
            timer.schedule(new MyTask(msgGripperClose), delay);
            Log.d("flipTest", "gripper close");

            delay += waitingTime;

            timer.schedule(new MyTask(msgMoveToTransitionFlipper, transitionFlipperX, transitionFlipperY, transitionFlipperZ), delay);
            Log.d("flipTest", "move to transition flipper");

            delay += waitingTime;

            timer.schedule(new MyTask(msgMoveToTransitionChessBoard, transitionChessBoardX, transitionChessBoardY, transitionChessBoardZ), delay);
            Log.d("flipTest", "move to transition chessboard");

            delay += waitingTime;

            timer.schedule(new MyTask(msgMoveToTransitionChessBoard2, XYZ.get(0), XYZ.get(1), transitionChessBoardZ), delay);

            delay += waitingTime;

            msgMoveToChessBoard = "G1 X" + String.valueOf(XYZ.get(0)) + " Y" + String.valueOf(XYZ.get(1)) + " Z" + String.valueOf(XYZ.get(2) + 10) + "\r";
            timer.schedule(new MyTask(msgMoveToChessBoard, XYZ.get(0), XYZ.get(1), XYZ.get(2) + 10), delay);

            delay += waitingTime;

            gripperOpen = np_gripper_open.getValue();
            msgGripperOpen = "M5 T" + gripperOpen + "\r";
            timer.schedule(new MyTask(msgGripperOpen), delay);
            Log.d("flipTest", "gripper open");

            delay += waitingTime * 2;

            timer.schedule(new MyTask(msgMoveToTransitionChessBoard2, XYZ.get(0), XYZ.get(1), transitionChessBoardZ), delay);

            delay += waitingTime;
        }

        //String msgMoveToTransitionChessBoard = "G1 X" + String.valueOf(transitionChessBoardX) + " Y" + String.valueOf(transitionChessBoardY) + " Z" + String.valueOf(transitionChessBoardZ) + "\r";
        timer.schedule(new MyTask(msgMoveToTransitionChessBoard, transitionChessBoardX, transitionChessBoardY, transitionChessBoardZ), delay);
        Log.d("flipTest", "move to transition chessboard");

        delay += waitingTime;

        String msgMoveToRest = "G1 X" + String.valueOf(transitionFlipperX) + " Y" + String.valueOf(transitionFlipperY) + " Z" + String.valueOf(transitionFlipperZ) + "\r";
        timer.schedule(new MyTask(msgMoveToRest, transitionFlipperX, transitionFlipperY, transitionFlipperZ), delay);
        Log.d("flipTest", "move to rest");

        delay += waitingTime;

        if (turnPlayer == "White"){
            Log.d("flipTest", "turn player change: white -> black");
            turnPlayer = "Black";
        }
        else if (turnPlayer == "Black"){
            Log.d("flipTest", "turn player change: black -> white");
            turnPlayer = "White";
            Log.d("flipTest", "set dst to -1");
            dst = -1;
        }
        endCount = 0;
        timer.schedule(new MyTask(""), delay);
        return;
    }

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.d(TAG, "base loader call back success");
                    cameraView.enableView();
                }
                break;
                default: {
                    super.onManagerConnected(status);
                }
                break;
            }
        }
    };

    @Override
    public void onResume() {
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION, this, mLoaderCallback);
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
        //OpenCVLoader.initAsync( OpenCVLoader.OPENCV_VERSION, this, mLoaderCallback );
    }

    //record the position of the entrance of the flipper for calibration
    public void buttonFlipperIn(View view) {
        //Toast.makeText(this, "Calibrate Flipper In", Toast.LENGTH_LONG).show();
        if (clientSocket != null){
            flipperInX = currentX;
            flipperInY = currentY;
            flipperInZ = currentZ;
        }
    }

    //record the position of the exit of the flipper for calibration
    public void buttonFlipperOut(View view) {
        //Toast.makeText(this, "Calibrate Flipper Out", Toast.LENGTH_LONG).show();
        if (clientSocket != null){
            flipperOutX = currentX;
            flipperOutY = currentY;
            flipperOutZ = currentZ;

            //transitionFlipperX = (flipperOutX + flipperInX) / 2;
            //transitionFlipperY = (flipperOutY + flipperInY) / 2;
            transitionFlipperX = flipperOutX;
            transitionFlipperY = flipperOutY;
            transitionFlipperZ = flipperOutZ + (flipperInZ - flipperOutZ) * 2;
        }
    }

    //record the position to pick up a new chess for calibration
    public void buttonNewChess(View view) {
        //Toast.makeText(this, "Calibrate New Chess", Toast.LENGTH_LONG).show();
        if (clientSocket != null){
            newChessX = currentX;
            newChessY = currentY;
            newChessZ = currentZ;
        }
    }

    //calculate the intersection of 2 lines
    public Point getIntersection(Point pt1, Point pt2, Point pt3, Point pt4){
        double intersectX;
        double intersectY;
        double denominator;
        double x1 = pt1.x;
        double y1 = pt1.y;
        double x2 = pt2.x;
        double y2 = pt2.y;
        double x3 = pt3.x;
        double y3 = pt3.y;
        double x4 = pt4.x;
        double y4 = pt4.y;

        denominator = (x1 - x2) * (y3 -y4) - (y1 - y2) * (x3 - x4);
        intersectX = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator;
        intersectY = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator;
        Point intersect = new Point(intersectX, intersectY);

        return intersect;
    }

    //cluster the points within a short range
    public ArrayList<Point> getCluster(ArrayList<Point> intersections){
        if (intersections.size() < 81){
            return new ArrayList<Point>(1);
        }
        ArrayList<Point> cluster = new ArrayList<Point>();
        ArrayList<Integer> counters = new ArrayList<Integer>();
        cluster.add(intersections.get(0));
        counters.add(1);

        for (int intersect=1; intersect<intersections.size(); intersect++){
            boolean addFlag = true;
            for (int i=0; i<cluster.size(); i++){
                double diffX = Math.abs(intersections.get(intersect).x - cluster.get(i).x);
                double diffY = Math.abs(intersections.get(intersect).y - cluster.get(i).y);
                if (diffX <= 30 && diffY <= 30){
                    int currentCount = counters.get(i);
                    double totalX = cluster.get(i).x * currentCount + intersections.get(intersect).x;
                    double totalY = cluster.get(i).y * currentCount + intersections.get(intersect).y;
                    Point newCentroid = new Point(totalX / (currentCount + 1), totalY / (currentCount + 1));
                    cluster.set(i, newCentroid);
                    counters.set(i, currentCount + 1);
                    addFlag = false;
                    break;
                }
            }
            if (addFlag){
                cluster.add(intersections.get(intersect));
                counters.add(1);
            }
        }
        //Log.d(TAG, "Cluster size: " + cluster.size());

        Collections.sort(cluster, new Comparator<Point>(){
            public int compare(Point pt1, Point pt2) {
                int compareY = Double.compare(pt1.y, pt2.y);
                return compareY;
            }
        });

         //*/

        return cluster;
    }

    //sort the points to match the chessboard index
    public ArrayList<Point> sortCorners(ArrayList<Point> corners){
        if (corners.size() != 81){
            return corners;
        }

        ArrayList<Point> tmp;
        for (int i=0; i<9; i++){
            tmp = new ArrayList<Point>(corners.subList(9 * i + 0, 9 * i + 9));
            Collections.sort(tmp, new Comparator<Point>(){
                public int compare(Point pt1, Point pt2) {
                    int compareX = Double.compare(pt1.x, pt2.x);
                    return compareX;
                }
            });

            for (int j=0; j<9; j++){
                corners.set(9 * i + j, tmp.get(j));
            }
        }

        return corners;
    }

    //get the grid index of the chess
    public Point getGrid(ArrayList<Point> corners, Point chess){
        if (corners.size() != 81){
            return new Point();
        }

        int gridY = -1;
        int gridX = -1;

        for (int y=0; y<8; y++){
            if (chess.y > corners.get(y * 9 + 4).y && chess.y < corners.get((y + 1) * 9 + 4).y){
                gridY = y;
                break;
            }
        }
        for (int x=0; x<8; x++){
            if (chess.x > corners.get(36 + x).x && chess.x < corners.get(36 + x + 1).x){
                gridX = x;
                break;
            }
        }

        if (gridY != -1 && gridX != -1){
            //chessBoard.set(8 * gridY + gridX, value);
            //Log.d(TAG, "X: " + gridX + " Y: " + gridY);
            return new Point(gridX, gridY);
        }

        return new Point();
    }

    //get the indices of placeable grid
    public ArrayList<Integer> getPlaceable(ArrayList<String> chessBoard, String target){
        ArrayList<Integer> placeablePos = new ArrayList<Integer>();
        //target = placeable color = turn player color
        //opposite = reference point to find placeable
        String opposite;
        if (target == "White"){
            opposite = "Black";
        }
        else{
            opposite = "White";
        }
        for (int i=0; i<chessBoard.size(); i++){
            if (i == 0 || i == 7 || i == 56 || i == 63){
                continue;
            }
            if (chessBoard.get(i) == opposite){
                int oppositeX = i % 8;
                int oppositeY = i / 8;
                for (int j=0; j< chessBoard.size(); j++){
                    if (i == j || placeablePos.contains(j) || chessBoard.get(j) != "None"){
                        continue;
                    }
                    int targetX = j % 8;
                    int targetY = j / 8;
                    int diffX = Math.abs(oppositeX - targetX);
                    int diffY = Math.abs(oppositeY - targetY);
                    boolean placeable = false;
                    //Vertical positions
                    if (diffX == 1 && diffY == 0){
                        //Target North to opposite
                        if (oppositeX > targetX){
                            for (int k=i + 1; Math.floorMod(k, 8)!=0 && k<64 && k>=0; k++){
                                if (chessBoard.get(k) == "None"){
                                    placeable = false;
                                    break;
                                }
                                else if (chessBoard.get(k) == target){
                                    placeable = true;
                                    break;
                                }
                            }
                        }
                        //Target South to opposite
                        else if (oppositeX < targetX){
                            for (int k=i - 1; Math.floorMod(k, 8)!=7 && k<64 && k>=0; k--){
                                if (chessBoard.get(k) == "None"){
                                    placeable = false;
                                    break;
                                }
                                else if (chessBoard.get(k) == target){
                                    placeable = true;
                                    break;
                                }
                            }
                        }
                    }

                    //Horizontal positions
                    else if (diffX == 0 && diffY == 1){
                        //Target East to opposite
                        if (oppositeY > targetY){
                            for (int k=i + 8; k<64 && k>=0; k+=8){
                                if (chessBoard.get(k) == "None"){
                                    placeable = false;
                                    break;
                                }
                                else if (chessBoard.get(k) == target){
                                    placeable = true;
                                    break;
                                }
                            }
                        }
                        //Target West to opposite
                        else if (oppositeY < targetY){
                            for (int k=i - 8; k>=0 && k<64; k-=8){
                                if (chessBoard.get(k) == "None"){
                                    placeable = false;
                                    break;
                                }
                                else if (chessBoard.get(k) == target){
                                    placeable = true;
                                    break;
                                }
                            }
                        }
                    }

                    //Diagonal positions
                    else if (diffX == 1 && diffY == 1){
                        //Target North East to opposite
                        if (oppositeX > targetX && oppositeY > targetY){
                            for (int k=i + 9; Math.floorMod(k, 64)%8>=targetX && Math.floorMod(k, 64)/8>=targetY && k<64 && k>=0; k+=9){
                                if (chessBoard.get(k) == "None"){
                                    placeable = false;
                                    break;
                                }
                                else if (chessBoard.get(k) == target){
                                    placeable = true;
                                    break;
                                }
                            }
                        }
                        //Target North West to opposite
                        else if (oppositeX > targetX && oppositeY < targetY){
                            for (int k=i - 7; Math.floorMod(k, 64)%8>=targetX && Math.floorMod(k, 64)/8<=targetY && k<64 && k>=0; k-=7){
                                if (chessBoard.get(k) == "None"){
                                    placeable = false;
                                    break;
                                }
                                else if (chessBoard.get(k) == target){
                                    placeable = true;
                                    break;
                                }
                            }
                        }
                        //Target South East to opposite
                        if (oppositeX < targetX && oppositeY > targetY){
                            for (int k=i + 7; Math.floorMod(k, 64)%8<=targetX && Math.floorMod(k, 64)/8>=targetY && k<64 && k>=0; k+=7){
                                if (chessBoard.get(k) == "None"){
                                    placeable = false;
                                    break;
                                }
                                else if (chessBoard.get(k) == target){
                                    placeable = true;
                                    break;
                                }
                            }
                        }
                        //Target South West to opposite
                        else if (oppositeX < targetX && oppositeY < targetY){
                            for (int k=i - 9; Math.floorMod(k, 64)%8<=targetX && Math.floorMod(k, 64)/8<=targetY && k<64 && k>=0; k-=9){
                                if (chessBoard.get(k) == "None"){
                                    placeable = false;
                                    break;
                                }
                                else if (chessBoard.get(k) == target){
                                    placeable = true;
                                    break;
                                }
                            }
                        }
                    }

                    if (placeable){
                        placeablePos.add(j);
                    }
                }
            }
        }

        //return index of placeable positions
        return placeablePos;
    }

    //get the indices of chesses to be flipped
    public ArrayList<Integer> getFlipList(ArrayList<String> chessBoard, int newChessIndex, String opposite){
        ArrayList<Integer> flipList = new ArrayList<Integer>();

        int x = newChessIndex % 8;
        int y = newChessIndex / 8;
        //String opposite = turnPlayer;
        Log.d("getFlipList", "opposite: " + opposite);

        String target;
        if (opposite == "White"){
            target = "Black";
        }
        else{
            target = "White";
        }

        Log.d("getFlipList", "target: " + target);

        /*
        else if (opposite == "Black"){
            target = "White";
        }
        else{
            return new ArrayList<Integer>();
        }

         */

        ArrayList<Integer> tmp = new ArrayList<Integer>();

        //right
        for (int i=newChessIndex-8; i<64 && i>=0; i-=8){
            if (chessBoard.get(i) == target){
                tmp.add(i);
            }
            else if(chessBoard.get(i) == opposite){
                for (int j=0; j<tmp.size(); j++){
                    if (!flipList.contains(tmp.get(j))){
                        flipList.add(tmp.get(j));
                    }
                }
                break;
            }
            else{
                break;
            }
        }
        tmp.clear();

        //left
        for (int i=newChessIndex+8; i<64 && i>=0; i+=8){
            if (chessBoard.get(i) == target){
                tmp.add(i);
            }
            else if(chessBoard.get(i) == opposite){
                for (int j=0; j<tmp.size(); j++){
                    if (!flipList.contains(tmp.get(j))){
                        flipList.add(tmp.get(j));
                    }
                }
                break;
            }
            else{
                break;
            }
        }
        tmp.clear();

        //up
        for (int i=newChessIndex-1; Math.floorMod(i, 8)!=7 && i<64 && i>=0; i--){
            if (chessBoard.get(i) == target){
                tmp.add(i);
            }
            else if(chessBoard.get(i) == opposite){
                for (int j=0; j<tmp.size(); j++){
                    if (!flipList.contains(tmp.get(j))){
                        flipList.add(tmp.get(j));
                    }
                }
                break;
            }
            else{
                break;
            }
        }
        tmp.clear();

        //down
        for (int i=newChessIndex+1; Math.floorMod(i, 8)!=0 && i<64 && i>=0; i++){
            if (chessBoard.get(i) == target){
                tmp.add(i);
            }
            else if(chessBoard.get(i) == opposite){
                for (int j=0; j<tmp.size(); j++){
                    if (!flipList.contains(tmp.get(j))){
                        flipList.add(tmp.get(j));
                    }
                }
                break;
            }
            else{
                break;
            }
        }
        tmp.clear();

        //top right
        for (int i=newChessIndex-9; Math.floorMod(i, 64)%8<x && Math.floorMod(i, 64)/8<y && i<64 && i>=0 && Math.floorMod(i, 8) != 7; i-=9){
            if (chessBoard.get(i) == target){
                tmp.add(i);
            }
            else if(chessBoard.get(i) == opposite){
                for (int j=0; j<tmp.size(); j++){
                    if (!flipList.contains(tmp.get(j))){
                        flipList.add(tmp.get(j));
                    }
                }
                break;
            }
            else{
                break;
            }
        }
        tmp.clear();

        //top left
        for (int i=newChessIndex+7; Math.floorMod(i, 64)%8<x && Math.floorMod(i, 64)/8>y && i<64 && i>=0 && Math.floorMod(i, 8) != 7; i+=7){
            if (chessBoard.get(i) == target){
                tmp.add(i);
            }
            else if(chessBoard.get(i) == opposite){
                for (int j=0; j<tmp.size(); j++){
                    if (!flipList.contains(tmp.get(j))){
                        flipList.add(tmp.get(j));
                    }
                }
                break;
            }
            else{
                break;
            }
        }
        tmp.clear();

        //bottom right
        for (int i=newChessIndex-7; Math.floorMod(i, 64)%8>x && Math.floorMod(i, 64)/8<y && i<64 && i>=0 && Math.floorMod(i, 8) != 0; i-=7){
            if (chessBoard.get(i) == target){
                tmp.add(i);
            }
            else if(chessBoard.get(i) == opposite){
                for (int j=0; j<tmp.size(); j++){
                    if (!flipList.contains(tmp.get(j))){
                        flipList.add(tmp.get(j));
                    }
                }
                break;
            }
            else{
                break;
            }
        }
        tmp.clear();

        //bottom left
        for (int i=newChessIndex+9; Math.floorMod(i, 64)%8>x && Math.floorMod(i, 64)/8>y && i<64 && i>=0 && Math.floorMod(i, 8) != 0; i+=9){
            if (chessBoard.get(i) == target){
                tmp.add(i);
            }
            else if(chessBoard.get(i) == opposite){
                for (int j=0; j<tmp.size(); j++){
                    if (!flipList.contains(tmp.get(j))){
                        flipList.add(tmp.get(j));
                    }
                }
                break;
            }
            else{
                break;
            }
        }

        return flipList;
    }

    //calculate the 3D coordinates of a given grid index
    public ArrayList<Double> getXYZ(int index){
        ArrayList<Double> XYZ = new ArrayList<Double>();

        /*
        XYZ.add(chessBoardXYZ.get(index).get(0));
        XYZ.add(chessBoardXYZ.get(index).get(1));
        XYZ.add(chessBoardXYZ.get(index).get(2));
        return XYZ;

         */

        ///*
        if (index < 0 || index > 63){
            return XYZ;
        }

        int x = index % 8;
        int y = index / 8;

        double indexX = referencePts.get(7-x).get(0);
        if (x < 4){
            indexX -= (referencePts.get(7).get(0) - referencePts.get(7+y).get(0)) * (x+1) / 4;
        }
        else{
            indexX += (referencePts.get(7).get(0) - referencePts.get(7+y).get(0)) * (x-3) / 4;
        }
        double indexY = referencePts.get(7+y).get(1);
        //indexY -= (referencePts.get(7).get(1) - referencePts.get(7-y).get(1));
        double indexZ = referencePts.get(0).get(2);
        for (int i=0; i<referencePts.size(); i++){
            indexZ = Math.min(indexZ, referencePts.get(i).get(2));
        }

        XYZ.add(indexX);
        XYZ.add(indexY);
        XYZ.add(indexZ);

        return XYZ;

         //*/
    }

    //Point3D class to store the (x, y, z) coordinates of a position
    public class Point3D{
        double x;
        double y;
        double z;

        public Point3D(){
            this.x = 0.0;
            this.y = 0.0;
            this.z = 0.0;
        }

        public Point3D(double x, double y, double z){
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public double get(int index){
            if (index == 0){
                return this.x;
            }
            else if (index == 1){
                return this.y;
            }
            else if (index == 2){
                return this.z;
            }
            else{
                return 0.0;
            }
        }
    }
}