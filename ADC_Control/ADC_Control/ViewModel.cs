using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO.Ports;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Input;
using ADC_Control.Properties;
using ADC_Control_Library;
using InteractiveDataDisplay.WPF;

namespace ADC_Control
{
    sealed public class ViewModel:INotifyPropertyChanged, IDisposable
    {
        #region Constructor
        public ViewModel() 
        {
            //инициализация компонентов
            Graphs = new();
            ADC = new(new ADCLogger());
            SettingPortVM = new();
            tokenSource = new();
            Logger = NLog.LogManager.GetCurrentClassLogger();

            //"Привязка" к изменению свойств
            SettingPortVM.PropertyChanged += (sender, e) => UpdatePortCommand();
            SettingPortVM.PropertyChanged += ClosePortIfSelectPortChanged;
            ADC.PropertyChanged += (sender, e) => UpdatePortCommand();
            ADC.PropertyChanged += CallPropertyChangedByGraphs;
            PropertyChanged += (sender, e) => UpdatePortCommand();
        }



        #endregion //Constructor

        #region VisualProperties

        private PointGraph? selectGraph;
        /// <summary>
        /// Выбранный график
        /// </summary>
        public PointGraph? SelectGraph 
        {
            get => selectGraph;
            set
            {
                selectGraph = value;
                OnPropertyChanged();
            }
        }

        /// <summary>
        /// Коллекция графикоа
        /// </summary>
        public List<PointGraph> Graphs { get; init; }

        private string? selectPort; 
        /// <summary>
        /// выбранный порт для связи с АЦП
        /// </summary>
        public string? SelectPort 
        {
            get => selectPort;
            set
            {
                selectPort = value;
                OnPropertyChanged();
            }
        }

        private bool? isMirrorTest;
        /// <summary>
        /// Прошёл ли порт проверку на связь с микроконтроллером
        /// </summary>
        public bool? IsMirrorTest
        {
            get => isMirrorTest;
            private set
            {
                isMirrorTest = value;
                OnPropertyChanged();
            }
        }


        private StringBuilder console = new();
        /// <summary>
        /// Консоль для вывода
        /// </summary>
        public StringBuilder Console
        {
            get => console;
            private set
            {
                console = value;
                OnPropertyChanged();
            }
        }

        private TimeSpan timeConvertation;
        /// <summary>
        /// Время для непрерывной конвертации
        /// </summary>
        public TimeSpan TimeConvertation
        {
            get => timeConvertation;
            set
            {
                timeConvertation = value;
                OnPropertyChanged();
            }
        }

        /// <summary>
        /// Свойства АЦП
        /// </summary>
        public List<IPropertiesTableElementable> ADCProperties
        {
            get => ADC.ADCProperties;
        }

        /// <summary>
        /// Доступные порты
        /// </summary>
        public List<string>? Ports { get => ADC.Ports; }

        /// <summary>
        /// Настройки порта для связи с АЦП
        /// </summary>
        public SettingPortViewModel SettingPortVM { get; init; }

        #endregion //VisualProperties

        #region Command

        #region Head

        private UniversalCommand? updatePortsCommand;
        /// <summary>
        /// Обновить доступные порты для подключения
        /// </summary>
        public UniversalCommand UpdatePortsCommand => updatePortsCommand ??= new(UpdatePorts);

        private UniversalCommand? openPortCommand;
        /// <summary>
        /// Открыть порт для связи
        /// </summary>
        public UniversalCommand OpenPortCommand => openPortCommand ??= new(OpenPort, CanOpenPort);

        private UniversalCommand? closePortCommand;
        /// <summary>
        /// Закрыть порт
        /// </summary>
        public UniversalCommand ClosePortCommand => closePortCommand ??= new(ClosePort, CanClosePort);

        private UniversalCommand? updatePortParametersCommand;
        /// <summary>
        /// Открыть другой порт
        /// </summary>
        public UniversalCommand UpdatePortParametersCommand => updatePortParametersCommand ??=new(UpdatePortParameters, CanUpdatePortParameters);

        private UniversalCommand? updateADCPropertiesCommand;
        /// <summary>
        /// Считать свойства с АЦП модуля
        /// </summary>
        public UniversalCommand UpdateADCPropertiesCommand => updateADCPropertiesCommand ??= new(UpdateADCProperties, CanInvokeADCOperation);

        private UniversalCommand? writeADCPropertiesCommand;
        /// <summary>
        /// Записать свойства на АЦП модуль
        /// </summary>
        public UniversalCommand WriteADCPropertiesCommand=> writeADCPropertiesCommand ??= new(WriteADCProperties, CanInvokeADCOperation);


        private UniversalCommand? testMirrorCommand;
        /// <summary>
        /// Проверить связь с микроконтроллером
        /// </summary>
        public UniversalCommand TestMirrorCommand => testMirrorCommand ??= new(
            (obj) => IsMirrorTest = ADC.TestMirror(10, GetToken()),
            (obj) => SelectPort != null && !ADC.Port.IsOpen);

        private UniversalCommand? cancelADCOpertaionCommand;

        /// <summary>
        /// Отменить выполняемую операцию
        /// </summary>
        public UniversalCommand CancelADCOpertaionCommand => cancelADCOpertaionCommand ??= new(
            (obj) => { tokenSource.Cancel(); isRunningOperation = false; },
            (obj) => isRunningOperation);

        private UniversalCommand? calibrationADCInsideCommand;
        
        /// <summary>
        /// Откалибровать АЦП на 0. Выполнять перед измерениями
        /// </summary>
        public UniversalCommand CalibrationADCInsideCommand => calibrationADCInsideCommand ??= new((obj) => ADC.CalibrationInside(), CanInvokeADCOperation);

        private UniversalCommand? calibrationADCOutsideCommand;

        /// <summary>
        /// Откалибровать АЦП с учётом проводов. Выполнять при КЗ
        /// </summary>
        public UniversalCommand CalibrationADCOutsideCommand => calibrationADCOutsideCommand ??= new((obj) => ADC.CalibrationOutside(), CanInvokeADCOperation);

        private UniversalCommand? calibrationADCScaleCommand;
        
        /// <summary>
        /// Откалибровать масштаб АЦП
        /// </summary>
        public UniversalCommand CalibrationADCScaleCommand => calibrationADCScaleCommand ??= new((obj) => ADC.CalibrationScale(), CanInvokeADCOperation);

        private UniversalCommand? convertADCCommand;

        /// <summary>
        /// Конвертирует 1 значение сигнала
        /// </summary>
        public UniversalCommand ConvertADCCommand => convertADCCommand ??= new(
            (obj) => WriteStringInConsole(string.Format(ADC.Convert(GetToken()).ToString())), CanInvokeADCOperation);

        private UniversalCommand? convertADCToTimeCommand;
        /// <summary>
        /// Конвертирует по времени в график
        /// </summary>
        public UniversalCommand ConvertADCToTimeCommand => convertADCToTimeCommand ??= new(ConvertADCToTime, CanInvokeADCOperation);
        #endregion //Head

        #region Body
        private void UpdatePorts(object? parameter)
        {
            ADC.UpdatePorts();
        }
        private bool CanOpenPort(object? parameter) => SelectPort != null && !ADC.Port.IsOpen && SettingPortVM.StopBits != StopBits.None;
        private void OpenPort(object? parameter)
        {
            try
            {
                if (SettingPortVM.SavePortSettingsCommand.CanExecute(null))
                    SettingPortVM.SavePortSettingsCommand.Execute(null);

                Logger.Info(Resources.LogClosePortStart);
                ADC.Port.PortName = selectPort;
                SettingPortVM.SetPropertiesPort(ADC.Port);
                SettingPortVM.ResetChangedProperties();
                ADC.Port.Open();
                UpdatePortCommand();
                IsMirrorTest = ADC.TestMirror(10, GetToken());
                if (IsMirrorTest == true)
                    UpdateADCProperties(null);
                Logger.Info(string.Format(Resources.LogClosePortFinish, SelectPort));
            }
            catch (Exception)
            {
                Logger.Error(string.Format(Resources.LogOpenPortError, SelectPort));
            }
        }

        private bool CanClosePort(object? parameter) => SelectPort != null && ADC.Port.IsOpen;
        private void ClosePort(object? parameter)
        {
            Logger.Info(string.Format(Resources.LogClosePortStart, ADC.Port.PortName));
            ADC.Port.Close();
            Logger.Info(Resources.LogClosePortStart);
            UpdatePortCommand();
            IsMirrorTest = null;
        }


        private bool CanUpdatePortParameters(object? parameter) => SelectPort != null && ADC.Port.IsOpen && SettingPortVM.IsChangedProperties;
        private void UpdatePortParameters(object? parameter)
        {
            ClosePort(parameter);
            OpenPort(parameter);
        }

        private bool CanInvokeADCOperation(object? parameter)
        {
            return ADC.Port.IsOpen && !isRunningOperation && IsMirrorTest == true;
        }

        private void ConvertADCToTime(object? parameter)
        {

            if (short.TryParse(TimeConvertation.TotalMilliseconds.ToString(), out short time))
            {
                try
                {
                    Logger.Info(Resources.LogStartConvertGraph);
                    var result = ADC.ConvertToTime(time, GetToken());
                    Logger.Info(Resources.LogEndConvertGraph);

                }
                catch (Exception)
                {

                    Logger.Info(Resources.LogErrorConvertGraph);
                }
            }
        }

        public void UpdateADCProperties(object? parameter)
        {
            Logger.Info(Resources.LogReadADCPropertiesStart);
            ADC.UpdateFromADCProperties();
            Logger.Info(Resources.LogReadADCPropertiesFinish);
            OnADCPropertiesChanged();
        }

        public void WriteADCProperties(object? parameter)
        {
            Logger.Info(Resources.LogWriteADCPropertiesStart);
            ADC.WriteToADCProperties();
            Logger.Info(Resources.LogWriteADCPropertiesFinish);
        }

        #endregion //Body

        #endregion //Command

        #region PrivateProperties

        private bool isRunningOperation = false;
        private CancellationTokenSource tokenSource;
        private ADCManager ADC { get; init; }

        private NLog.Logger Logger { get; init; }
        #endregion //PrivateProperties

        #region PrivateMethods

        private CancellationToken GetToken()
        {
            isRunningOperation = true;
            return tokenSource.Token;
        }

        private void WriteStringInConsole(string message)
        {
            Console.Append(message);
            OnPropertyChanged("Console");
        }

        private void OnADCPropertiesChanged() => OnPropertyChanged("ADCProperties");

        private void ClosePortIfSelectPortChanged(object? sender, PropertyChangedEventArgs e)
        {
            if (e.PropertyName == "SelectPort")
                ClosePort(null);
        }

        private void CallPropertyChangedByGraphs(object? sender, PropertyChangedEventArgs e)
        {
            OnPropertyChanged("Graphs");
        }

        private void UpdatePortCommand()
        {
            UpdatePortParametersCommand.OnCanExecuteChanged();
            OpenPortCommand.OnCanExecuteChanged();
            ClosePortCommand.OnCanExecuteChanged();
            UpdatePortParametersCommand.OnCanExecuteChanged();
        }

        #endregion //PrivateMethods
        private void OnPropertyChanged([CallerMemberName] string name = "") => PropertyChanged?.Invoke(this, new(name));

        public void Dispose()
        {
            tokenSource.Dispose();
            ADC.Dispose();
        }

        public event PropertyChangedEventHandler? PropertyChanged;
    }
}
