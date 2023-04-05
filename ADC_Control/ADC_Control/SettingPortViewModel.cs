using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.ComponentModel.DataAnnotations;
using System.Configuration;
using System.IO.Ports;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace ADC_Control
{
    public sealed class SettingPortViewModel : INotifyPropertyChanged
    {
        static SettingPortViewModel()
        {
            BaundRateValues = new()
        {
            110,
            300,
            600,
            1200,
            2400,
            4800,
            9600,
            14400,
            19200,
            38400,
            57600,
            115200,
            128000,
            256000
        };            
        }
        public SettingPortViewModel()
        {
            SerialPort port = new SerialPort();
            BaundRate = BaundRateValues.First();
            ParityValues = Enum.GetValues(typeof(Parity)).OfType<Parity>().ToList();
            DataBitsValues = new() { 5, 6, 7, 8, 9 };
            DataBits = 5;
            StopBitsValues = Enum.GetValues(typeof(StopBits)).OfType<StopBits>().ToList();
        }

        public List<Parity> ParityValues { get; init; }
        private Parity parity;
        public Parity Parity
        {
            get => parity;
            set
            {
                parity = value;
                OnPropertyChanged();
            }
        }

        public static List<int> BaundRateValues { get; private set; }
        private int baundRate;
        public int BaundRate 
        {
            get => baundRate;
            set
            {
                baundRate = value;
                OnPropertyChanged();
            }
        }

        public List<int> DataBitsValues { get; init; }
        private int dataBits;
        [Range(5, 9)]
        public int DataBits 
        {
            get => dataBits;
            set
            {
                dataBits = value;
                OnPropertyChanged();
            }
        }

        public List<StopBits> StopBitsValues { get; init; }

        private StopBits stopBits;
        public StopBits StopBits 
        {
            get => stopBits;
            set
            {
                stopBits = value;
                OnPropertyChanged();
            }
        }

        public void ResetChangedProperties()
        {
            IsChangedProperties = false;
        }

        private bool isChangedProperties;
        public bool IsChangedProperties
        {
            get=> isChangedProperties;
            private set
            {
                isChangedProperties = value;
                OnPropertyChanged();
            }
        }


        public void SetPropertiesPort(SerialPort port)
        {
            port.BaudRate = BaundRate;
            port.DataBits = DataBits;
            port.Parity = Parity;
            port.StopBits = StopBits;
        }
        private void OnPropertyChanged([CallerMemberName] string name = "")
        {
            if(name != "IsChangedProperties")
                IsChangedProperties = true;
            PropertyChanged?.Invoke(this, new(name));
        }
        public event PropertyChangedEventHandler? PropertyChanged;
    }
}
