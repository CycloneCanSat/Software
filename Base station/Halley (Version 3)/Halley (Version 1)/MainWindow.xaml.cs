using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.IO;
using System.IO.Ports;
using System.Timers;
using System.Threading;
using System.ComponentModel;
using Microsoft.Maps.MapControl.WPF;
using AForge;
using WebcamControl;
using Microsoft.Expression.Encoder;
using Microsoft.Expression.Encoder.Devices;
using LiveCharts;
using System.Windows.Threading;

namespace Halley__Version_1_
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public class tempGraph
    {
        public double temperature { get; set; }
        public DateTime time { get; set; }
    }
    public class pressureGraph
    {
        public double pressure { get; set; }
        public DateTime time { get; set; }
    }
    public class data
    {
        public string input {set; get;}
        public int snr { set; get; }
        public int rssi { set; get; }
        public int sampleNumber { set; get; }
        public double internal_temperature { set; get; }
        public double barometric_pressure { set; get; }
        public double external_temperature { set; get; }
        public double humidity { set; get; }
        public int hours { set; get; }
        public int minutes { set; get; }
        public int seconds { set; get; }
        public int gpsFix { set; get; }
        public double longitude { set; get; }
        public double latitude { set; get; }
        public double altitude { set; get; }
        public double heading { set; get; }
        public double pitch { set; get; }
        public double roll { set; get; }
        public double agriculturalViability { set; get; }
        public double pressureAtms { set; get; }
        public double altitudeFromPressure { set; get; }
        public double dewPoint { set; get; }
        public double groundBarometricPressure { set; get; }
        public double groundTemperature { set; get; }
        public double groundLatitude { set; get; }
        public double groundLongitude { set; get; }
        public double groundAltitude { set; get; }
    }

    public partial class MainWindow : Window
    {
        List<data> allData = new List<data>();
        const int expectedLength = 26;
        StreamWriter writer = File.AppendText("CANSAT-DATA.txt");
        public SerialPort serial = new SerialPort();
        public bool connected = false;
        public double numberOfItems = 0;
        public double longAverage = 0;
        public double latAverage = 0;
        public int counter = 0;
        public int counter2;
        public bool newAvailable = false;
        System.Timers.Timer timer = new System.Timers.Timer();
        public bool elapsed = false;
        public const int defaultBaudRate = 115200;
        public readonly BackgroundWorker bw = new BackgroundWorker();

        private readonly DispatcherTimer _timer2;
        public Func<double, string> XFormatter2 { get; set; }
        public Func<double, string> YFormatter2 { get; set; }
        public SeriesCollection Series2 { get; set; }

        private readonly DispatcherTimer _timer;
        private DateTime _currentDate = DateTime.Now;
        public SeriesCollection Series { get; set; }
        public Func<double, string> YFormatter { get; set; }
        public Func<double, string> XFormatter { get; set; }
        private void TimerOnTick(object sender, EventArgs eventArgs)
        {
            _currentDate= DateTime.Now;
            if(Series[0].Values.Count > 10)
            {
                Series[0].Values.RemoveAt(0);
            }
            if (Series[1].Values.Count > 10)
            {
                Series[1].Values.RemoveAt(0);
            }
            if (Series[2].Values.Count > 10)
            {
                Series[2].Values.RemoveAt(0);
            }
            if (allData.Count() > 0)
            {
                Series[0].Values.Add(new tempGraph
                {
                    temperature = allData[allData.Count() - 1].external_temperature,
                    time = _currentDate
                }
                );
                Series[1].Values.Add(new tempGraph
                {
                    temperature = allData[allData.Count() - 1].internal_temperature,
                    time = _currentDate
                }
                );
                Series[2].Values.Add(new tempGraph
                {
                    temperature = allData[allData.Count() - 1].groundTemperature,
                    time = _currentDate
                }
                );
            }
            /*foreach (var series in Series)
            {
                if (series.Values.Count > 10) series.Values.RemoveAt(0);
                series.Values.Add(new tempGraph
                {
                    temperature = _,
                    time = _currentDate
                });
            }*/
        }
        private void Timer2OnTick(object sender, EventArgs eventArgs)
        {
            _currentDate = DateTime.Now;
            if (Series2[0].Values.Count > 10)
            {
                Series2[0].Values.RemoveAt(0);
            }
            if (Series2[1].Values.Count > 10)
            {
                Series2[1].Values.RemoveAt(0);
            }
            if (allData.Count() > 0)
            {
                Series2[0].Values.Add(new pressureGraph
                {
                    pressure = allData[allData.Count() - 1].barometric_pressure,
                    time = _currentDate
                }
                );
                Series2[1].Values.Add(new pressureGraph
                {
                    pressure = allData[allData.Count() - 1].groundBarometricPressure,
                    time = _currentDate
                }
                );
            }
        }
        public MainWindow()
        {
            InitializeComponent();
            timer.Start();
            setUpBaudRates();

            var config = new SeriesConfiguration<tempGraph>();
            config.Y(model => model.temperature);
            config.X(model => model.time.ToOADate());
            Series = new SeriesCollection(config) { new LineSeries { Values = new ChartValues<tempGraph>() } };
            Series.Add(new LineSeries { Values = new ChartValues<tempGraph>() });
            Series.Add(new LineSeries { Values = new ChartValues<tempGraph>() });
            Series[0].Title = "External Temperature";
            Series[1].Title = "Internal Temperature";
            Series[2].Title = "Ground Temperature";
            XFormatter = val => DateTime.FromOADate(val).ToString("mm.ss");
            YFormatter = val => val + " °";
            DataContext = this;
            _timer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(1000) };
            _timer.Tick += TimerOnTick;
            _timer.Start();

            var config2 = new SeriesConfiguration<pressureGraph>();
            config2.Y(model => model.pressure);
            config2.X(model => model.time.ToOADate());
            Series2 = new SeriesCollection(config2) { new LineSeries { Values = new ChartValues<pressureGraph>() } };
            Series2.Add(new LineSeries { Values = new ChartValues<pressureGraph>() });
            Series2[0].Title = "Barometric Pressure";
            Series2[1].Title = "Ground Barometric Pressure";
            XFormatter2 = val => DateTime.FromOADate(val).ToString("mm.ss");
            YFormatter2 = val => val + "hPa";
            DataContext = this;
            _timer2 = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(1000) };
            _timer2.Tick += Timer2OnTick;
            _timer2.Start();
            this.Show();
            setUpPossibleInputsForSerialPort();
            ///AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
            Binding binding_1 = new Binding("SelectedValue");
            binding_1.Source = VideoDevicesComboBox;
            WebcamCtrl.SetBinding(Webcam.VideoDeviceProperty, binding_1);

            Binding binding_2 = new Binding("SelectedValue");
            binding_2.Source = AudioDevicesComboBox;
            WebcamCtrl.SetBinding(Webcam.AudioDeviceProperty, binding_2);

            // Create directory for saving video files
            string videoPath = @"C:\VideoClips";

            if (!Directory.Exists(videoPath))
            {
                Directory.CreateDirectory(videoPath);
            }
            // Create directory for saving image files
            string imagePath = @"C:\WebcamSnapshots";

            if (!Directory.Exists(imagePath))
            {
                Directory.CreateDirectory(imagePath);
            }
            //AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
            // Set some properties of the Webcam control
            WebcamCtrl.VideoDirectory = videoPath;
            WebcamCtrl.ImageDirectory = imagePath;
            WebcamCtrl.FrameRate = 72;
            WebcamCtrl.FrameSize = new System.Drawing.Size(1080, 720);

            // Find available a/v devices
            var vidDevices = EncoderDevices.FindDevices(EncoderDeviceType.Video);
            var audDevices = EncoderDevices.FindDevices(EncoderDeviceType.Audio);
            VideoDevicesComboBox.ItemsSource = vidDevices;
            AudioDevicesComboBox.ItemsSource = audDevices;
            VideoDevicesComboBox.SelectedIndex = 0;
            AudioDevicesComboBox.SelectedIndex = 0;
            releaseServo.IsEnabled = false;
            servoArm.IsEnabled = false;
            t1.IsEnabled = false;
            t2.IsEnabled = false;
            bw.DoWork += Bw_DoWork;
            bw.RunWorkerCompleted += Bw_RunWorkerCompleted;
            bw.RunWorkerAsync();
            
        }
        private void StartCaptureButton_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                // Display webcam video
                WebcamCtrl.StartPreview();
            }
            catch
            {
                MessageBox.Show("Device is in use by another application");
            }
        }

        private void StopCaptureButton_Click(object sender, RoutedEventArgs e)
        {
            // Stop the display of webcam video.
            WebcamCtrl.StopPreview();
        }

        private void StartRecordingButton_Click(object sender, RoutedEventArgs e)
        {
            // Start recording of webcam video to harddisk.
            WebcamCtrl.StartRecording();
        }

        private void StopRecordingButton_Click(object sender, RoutedEventArgs e)
        {
            // Stop recording of webcam video to harddisk.
            WebcamCtrl.StopRecording();
        }

        private void TakeSnapshotButton_Click(object sender, RoutedEventArgs e)
        {
            // Take snapshot of webcam video.
            WebcamCtrl.TakeSnapshot();
        }

        public void setUpPossibleInputsForSerialPort()
        {
            bool doIt = true;
            if (serialPortSelectionBox.IsDropDownOpen == true || serialPortSelectionBox.Text != "")
                doIt = false;
            if (doIt)
            {
                serialPortSelectionBox.Items.Clear();
                string[] portsAvailable;
                portsAvailable = SerialPort.GetPortNames();
                for (int i = 0; i < portsAvailable.Length; i++)
                    serialPortSelectionBox.Items.Add(portsAvailable[i]);

                if(portsAvailable.Length == 1)
                {
                    serialPortSelectionBox.Text = portsAvailable[0];
                    serialPortBaudRate.Text = Convert.ToString(defaultBaudRate);
                    connectToSerial();
                }
            }
        }
        public void setUpBaudRates()
        {
            bool doIt = true;
            if (serialPortBaudRate.Text != "" || serialPortBaudRate.IsDropDownOpen == true)
                doIt = false;
            if (doIt)
            {
                serialPortBaudRate.Items.Clear();
                if (SerialPort.GetPortNames().Length > 0)
                {
                    int[] possibleBaudRates =
                    {
                4800, 9600, 19200, 38400, 57600, 115200
                 };
                    for (int i = 0; i < possibleBaudRates.Length; i++)
                    {
                        serialPortBaudRate.Items.Add(possibleBaudRates[i]);
                    }
                }
                else
                {
                    serialPortBaudRate.IsEnabled = false;
                    serialPortSelectionBox.IsEnabled = false;
                    connectSerialButton.IsEnabled = false;
                }
            }
        }
        public void connectToSerial()
        {
            if (connected)
                MessageBox.Show("Please disconnect from other devices before connecting to this one");
            if (serialPortSelectionBox.Text == "")
                MessageBox.Show("You must choose a valid port");
            else
            {
                serial.PortName = serialPortSelectionBox.Text;
                serial.BaudRate = Convert.ToInt32(serialPortBaudRate.Text);
                serial.Open();
                connected = true;
                connectSerialButton.IsEnabled = false;
                changeButtons();
                _timer.Start();
                _timer2.Start();
            }
            serial.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(receive);
            changeButtons();
        }

        private void connectSerialButton_Click(object sender, RoutedEventArgs e)
        {
            connectToSerial();
        }
        private void Bw_DoWork(object sender, DoWorkEventArgs e)
        {
            Thread.Sleep(50);
        }
        /*
        // TEST SYSTEM - changing something in the UI
        public void changeStuff()
        {
            dataNoDataLabel.Content = Convert.ToString(counter);
            counter++;
        }
        */ 
        private void Bw_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e)
        {
            // DO STUFF HERE
            if(!connected)
            {
                setUpPossibleInputsForSerialPort();

                Thread.Sleep(50);
                setUpBaudRates();
                dataNoDataLabel.Content = "NO DATA";
                dataNoDataLabel.Foreground = new SolidColorBrush(Colors.Red);
                Thread.Sleep(50);
                disableButtons();

                Thread.Sleep(50);
            }
            else
            {
                string input = "";
                bool newReceived = false;
                //Check for receiving data
                /*try
                {
                    input = serial.ReadLine();
                    writer.WriteLine(input);
                    newReceived = true;
                }
                catch (TimeoutException)
                {
                    MessageBox.Show("HELLO");
                    newReceived = false;
                    counter2++;
                    dataNoDataLabel.Content = "NO DATA";
                    dataNoDataLabel.Foreground = new SolidColorBrush(Colors.Red);
                    if (counter2 > 10)
                    {
                        dataNoDataLabel.Content = "NO DATA";
                        dataNoDataLabel.Foreground = new SolidColorBrush(Colors.Red);
                    }
                }
                */
                if(newAvailable)
                {
                    input = serial.ReadLine();
                    writer.WriteLine(input);
                    newReceived = true;
                }
                else
                {
                    
                    if (elapsed)
                    {
                        dataNoDataLabel.Content = "NO DATA";
                        dataNoDataLabel.Foreground = new SolidColorBrush(Colors.Red);
                    }
                    
                }
                newAvailable = false;
                Thread.Sleep(50);
                //DECODE DATA
                bool valid = false;
                if(newReceived)
                {
                    SerialOutputs.AppendText("\n");
                    SerialOutputs.AppendText("\n");
                    SerialOutputs.AppendText(input);
                    SerialOutputs.CaretIndex = SerialOutputs.Text.Length;
                    SerialOutputs.ScrollToEnd();
                    valid = checkValid(input);
                    Thread.Sleep(50);
                    if (valid)
                    {
                        decodeData(input);
                        Thread.Sleep(50);
                        counter++;
                        dataNoDataLabel.Content = "RECEIVING DATA";
                        dataNoDataLabel.Foreground = new SolidColorBrush(Colors.Green);
                        elapsed = false;
                        timer.Stop();
                        timer.Start();
                        timer.Interval = 2000;
                        timer.AutoReset = true;
                        timer.Elapsed += Timer_Elapsed;
                    }
                    else
                    {
                        dataNoDataLabel.Content = "NO DATA";
                    }
                }
                if (allData.Count() > 0)
                {
                    updateThings();
                }
            }
            bw.RunWorkerAsync();
        }

        private void Timer_Elapsed(object sender, ElapsedEventArgs e)
        {
            elapsed = true;
        }

        public bool checkValid(string input)
        {
            string[] inputs = input.Split(',');
            if (inputs.Length != 26)
                return false;
            else
                return true;
        }
        public void updateThings()
        {
            /*
            for reference....
             Location newTest = new Location();
             newTest.Latitude = 0;
             newTest.Longitude = 0;
             Pushpin pin = new Pushpin();
             map.Children.Add(pin);
             */
            int temp = allData.Count() - 1;
            internalTempBox.Text = Convert.ToString(allData[temp].internal_temperature);
            pressureBox.Text = Convert.ToString(allData[temp].barometric_pressure);
            externalTempBox.Text = Convert.ToString(allData[temp].external_temperature);
            humidityBox.Text = Convert.ToString(allData[temp].humidity);
            altitudeBox.Text = Convert.ToString(allData[temp].altitudeFromPressure);
            dewPointBox.Text = Convert.ToString(allData[temp].dewPoint);
            rapBox.Text = Convert.ToString(allData[temp].agriculturalViability);
            satsBox.Text = Convert.ToString(allData[temp].gpsFix);
            bool temp2 = false;
            if (satsBox.Text != "0")
            {
                longBox.IsEnabled = true;
                latBox.IsEnabled = true;
                altitudeGPSBox.IsEnabled = true;
                temp2 = true;
                longBox.Text = Convert.ToString(allData[temp].longitude);
                latBox.Text = Convert.ToString(allData[temp].latitude);
                altitudeGPSBox.Text = Convert.ToString(allData[temp].altitude);
            }
            else
            {
                longBox.IsEnabled = false;
                latBox.IsEnabled = false;
                altitudeGPSBox.IsEnabled = false;
            }
            headingBox.Text = Convert.ToString(allData[temp].heading);
            pitchBox.Text = Convert.ToString(allData[temp].pitch);
            rollBox.Text = Convert.ToString(allData[temp].roll);
            string hours = allData[temp].hours.ToString("D2");
            string minutes = allData[temp].minutes.ToString("D2");
            string seconds = allData[temp].seconds.ToString("D2");
            string time = hours + ":" + minutes + ":" + seconds;
            timeBox.Text = time;
            groundTempBox.Text = Convert.ToString(allData[temp].groundTemperature);
            groundPressure.Text = Convert.ToString(allData[temp].groundBarometricPressure);
            snrBox.Text = Convert.ToString(allData[temp].snr);
            rssiBox.Text = Convert.ToString(allData[temp].rssi);
            dataReceivedCounter.Text = Convert.ToString(counter);
            if (temp2)
            {
                Location newLocation = new Location();
                newLocation.Latitude = allData[temp].latitude;
                newLocation.Longitude = allData[temp].longitude;
                Pushpin pin = new Pushpin();
                pin.Background = new SolidColorBrush(Colors.AliceBlue);
                map.Children.Add(pin);
                Location groundLocation = new Location();
                groundLocation.Latitude = allData[temp].groundLatitude;
                groundLocation.Longitude = allData[temp].groundLongitude;
                Pushpin pin2 = new Pushpin();
                pin2.Background = new SolidColorBrush(Colors.Red);
                map.Children.Add(pin2);
                double longSum = longAverage * numberOfItems;
                longSum += newLocation.Longitude + groundLocation.Longitude;
                numberOfItems += 2;
                longAverage = longSum / (double)numberOfItems;
                double latSum = latAverage * (numberOfItems - 2);
                latSum += newLocation.Latitude + groundLocation.Latitude;
                latAverage = latSum / numberOfItems;
                Location center = new Location();
                center.Latitude = latAverage;
                center.Longitude = longAverage;
                map.SetView(center, 12);
            }
        }
        public void decodeData(string input)
        {
            string[] inputs = input.Split(',');
            data newData = new data();
            newData.snr = Convert.ToInt32(inputs[0]);
            newData.rssi = Convert.ToInt32(inputs[1]);
            newData.sampleNumber = Convert.ToInt32(inputs[2]);
            newData.internal_temperature = Convert.ToDouble(inputs[3]);
            newData.barometric_pressure = Convert.ToDouble(inputs[4]);
            newData.external_temperature = Convert.ToDouble(inputs[5]);
            newData.humidity= Convert.ToDouble(inputs[6]);
            newData.hours = Convert.ToInt32(inputs[7]);
            newData.minutes = Convert.ToInt32(inputs[8]);
            newData.seconds = Convert.ToInt32(inputs[9]);
            newData.gpsFix = Convert.ToInt32(inputs[10]);
            newData.longitude = Convert.ToDouble(inputs[11]);
            newData.latitude = Convert.ToDouble(inputs[12]);
            newData.altitude = Convert.ToDouble(inputs[13]);
            newData.heading = Convert.ToDouble(inputs[14]);
            newData.pitch = Convert.ToDouble(inputs[15]);
            newData.roll = Convert.ToDouble(inputs[16]);
            newData.agriculturalViability = Convert.ToDouble(inputs[17]);
            newData.pressureAtms = Convert.ToDouble(inputs[18]);
            newData.altitudeFromPressure = Convert.ToDouble(inputs[19]);
            newData.dewPoint = Convert.ToDouble(inputs[20]);
            newData.groundTemperature = Convert.ToDouble(inputs[22]);
            newData.groundBarometricPressure = Convert.ToDouble(inputs[21]);
            newData.groundLatitude = Convert.ToDouble(inputs[23]);
            newData.groundLongitude = Convert.ToDouble(inputs[24]);
            newData.groundAltitude = Convert.ToDouble(inputs[25]);
            allData.Add(newData);
        }
        public void disableButtons()
        {
            connectSerialButton.IsEnabled = true;
            serialPortBaudRate.IsEnabled = true;
            serialPortSelectionBox.IsEnabled = true;
            disconnectSerialButton.IsEnabled = false;
            dataReceivedCounter.IsEnabled = false;
            snrBox.IsEnabled = false;
            rssiBox.IsEnabled = false;
            samplesReceivedLabel.IsEnabled = false;
            samplesReceivedLabel_Copy.IsEnabled = false;
            samplesReceivedLabel_Copy1.IsEnabled = false;
            map.IsEnabled = true;
            label1.IsEnabled = false;
            label1_Copy.IsEnabled = false;
            qfeInput.IsEnabled = false;
            setQFE.IsEnabled = false;
            setQFECurrent.IsEnabled = false;
            longSent.IsEnabled = false;
            latSent.IsEnabled = false;
            sendCoordinates.IsEnabled = false;
            returnToHome.IsEnabled = false;
            servoArm.IsEnabled = false;
            t1.IsEnabled = false;
            releaseServo.IsEnabled = false;
            t2.IsEnabled = false;
        }
        public void changeButtons()
        {
            disconnectSerialButton.IsEnabled = true;
            dataReceivedCounter.IsEnabled = true;
            snrBox.IsEnabled = true;
            rssiBox.IsEnabled = true;
            samplesReceivedLabel.IsEnabled = true;
            samplesReceivedLabel_Copy.IsEnabled = true;
            samplesReceivedLabel_Copy1.IsEnabled = true;
            map.IsEnabled = true;
            label1.IsEnabled = true;
            label1_Copy.IsEnabled = true;
            qfeInput.IsEnabled = true;
            setQFE.IsEnabled = true;
            setQFECurrent.IsEnabled = true;
            longSent.IsEnabled = true;
            latSent.IsEnabled = true;
            sendCoordinates.IsEnabled = true;
            returnToHome.IsEnabled = true;
            servoArm.IsEnabled = true;
            releaseServo.IsEnabled = false;
            t1.IsEnabled = true;
            t2.IsEnabled = false;
        }

        private void disconnectSerialButton_Click(object sender, RoutedEventArgs e)
        {
            serial.Close();
            connected = false;
            _timer.Stop();
            _timer2.Stop();
        }

        private void sendCoordinates_Click(object sender, RoutedEventArgs e)
        {
            string toBeSent = "3," + longSent.Text + "," + latSent.Text;
            serial.WriteLine(toBeSent);
        }

        private void returnToHome_Click(object sender, RoutedEventArgs e)
        {
            double tempLat = allData[allData.Count() - 1].groundLatitude;
            double tempLong = allData[allData.Count() - 1].groundLongitude;
            string toBeSent = "4," + tempLong + "," + tempLat;
        }

        private void setQFE_Click(object sender, RoutedEventArgs e)
        {
            string toBeSent = "1" + qfeInput.Text;
            serial.WriteLine(toBeSent);
        }

        private void setQFECurrent_Click(object sender, RoutedEventArgs e)
        {
            string toBeSent = "0";
            serial.WriteLine(toBeSent);
        }

        private void servoArm_Click(object sender, RoutedEventArgs e)
        {
            //serial.Write("256akq20a4");
            servoArm.IsEnabled = false;
            releaseServo.IsEnabled = true;
            t1.IsEnabled = false;
            t2.IsEnabled = true;
        }

        private void releaseServo_Click(object sender, RoutedEventArgs e)
        {

            //serial.Write("256akq20a4");
            releaseServo.IsEnabled = false;
            t2.IsEnabled = false;
        }
        public void receive(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
        {
            newAvailable = true;
        }
    }
}
