using System;
using System.CodeDom;
using System.Collections.Generic;
using System.Data;
using System.Diagnostics.Contracts;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ADC_Control_Library
{
    /// <summary>
    /// Элементы таблицы свйоств АЦП
    /// </summary>
    public interface IPropertiesTableElementable
    {
        /// <summary>
        /// Название свойства
        /// </summary>
        public ADCProperty Property { get; }

        public ADCPropertyMode Mode { get; }


        public object? SelectValue { get; set; }
        /// <summary>
        /// Доступные значения
        /// </summary>
        public List<object> Values { get; }
        /// <summary>
        /// Конвертация строкового значения свйоства в значение для АЦП 
        /// </summary>
        /// <param name="value"></param>
        /// <returns></returns>
        public byte ConvertValueToCode(object value);
        public object ConvertCodeToValue(byte code);
    }
    /// <summary>
    /// Элемент таблицы по перечислению
    /// </summary>
    public class PropertiesTableElementByEnum : IPropertiesTableElementable
    {
        /// <summary>
        /// Элемент таблицы по перечислению
        /// </summary>
        /// <param name="property">Имя свойства</param>
        /// <param name="code">Код свойства</param>
        /// <param name="enumType">Тип перечисления</param>
        protected internal PropertiesTableElementByEnum(ADCProperty property, Type enumType, ADCPropertyMode mode = ADCPropertyMode.ReadAndWrite)
        {
            Property = property;
            EnumType = enumType;
            Mode = mode;
            Values = Enum.GetNames(enumType).ToList<object>();
        }
        /// <summary>
        /// ися Свойства
        /// </summary>
        public ADCProperty Property { get; init; }

        /// <summary>
        /// Значения
        /// </summary>
        public List<object> Values { get; init; }

        public object? SelectValue { get; set; }

        public ADCPropertyMode Mode { get; init; }

        /// <summary>
        /// Тип перечисления
        /// </summary>
        private Type EnumType { get; init; }

        public object ConvertCodeToValue(byte code)
        {
            return Enum.ToObject(EnumType, code);
        }

        /// <summary>
        /// Конвертация названия свойства в ключ
        /// </summary>
        /// <param name="value"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentException"></exception>
        public byte ConvertValueToCode(object value)
        {
            if (Enum.TryParse(EnumType, value.ToString(), out object? result))
                return Convert.ToByte(result);
            throw new ArgumentException("Not find enum value", "value");
        }
    }

    public class PropertiesTableDoubleValueElement : IPropertiesTableElementable
    {
        public PropertiesTableDoubleValueElement(ADCProperty property, Dictionary<double, byte> values, ADCPropertyMode mode = ADCPropertyMode.ReadAndWrite)
        {
            Property = property;
            DicValues = values;
            Mode = mode;
            Values = values.Select(i=> (object)i.Key).ToList();
            DicValues = values;
        }
        public ADCProperty Property { get; init; }

        public object? SelectValue { get; set; }

        public List<object> Values { get; init; }

        public ADCPropertyMode Mode { get; init; }

        private Dictionary<double, byte> DicValues { get; init; }

        public object ConvertCodeToValue(byte code)
        {
            return code;
        }

        public byte ConvertValueToCode(object value)
        {
            if(value is double d)
                if(DicValues.TryGetValue(d, out byte res))
                    return res;
            throw new ArgumentException("Not find", "value");
        }
    }

    public class PropertiesTableByteElement : IPropertiesTableElementable
    {
        public PropertiesTableByteElement(ADCProperty property, ADCPropertyMode mode)
        {
            Property = property;
            Mode = mode;
            Values = new();
            for (int i = 0; i <= byte.MaxValue; i++)
                Values.Add(i);
        }

        public ADCProperty Property { get; init; }

        public ADCPropertyMode Mode { get; init; }

        public object? SelectValue { get; set; }

        public List<object> Values { get; init; }

        public object ConvertCodeToValue(byte code)
        {
            throw new NotImplementedException();
        }

        public byte ConvertValueToCode(object value)
        {
            if(value is byte bt)
                return bt;
            throw new ArgumentException("Value not is byte type", "value");
        }
    }
}
