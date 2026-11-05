using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace Acquire
{
    /// <summary>
    /// 极简 JSON：只为本工程的 meta.json 服务（写入 + 自检时解析校验）。
    /// 不引第三方库，保持 .NET Framework 4.0 原生依赖。
    /// </summary>
    internal sealed class JsonObject : List<KeyValuePair<string, object>>
    {
        public void Add(string key, object value)
        {
            Add(new KeyValuePair<string, object>(key, value));
        }

        public object Get(string key, object fallback)
        {
            for (int i = 0; i < Count; i++)
            {
                if (string.Equals(this[i].Key, key, StringComparison.Ordinal)) return this[i].Value;
            }
            return fallback;
        }
    }

    internal static class MiniJson
    {
        public static string Serialize(object value)
        {
            StringBuilder sb = new StringBuilder(2048);
            WriteValue(sb, value, 0);
            sb.Append("\r\n");
            return sb.ToString();
        }

        private static void WriteValue(StringBuilder sb, object value, int indent)
        {
            if (value == null)
            {
                sb.Append("null");
                return;
            }
            if (value is string)
            {
                WriteString(sb, (string)value);
                return;
            }
            if (value is bool)
            {
                sb.Append(((bool)value) ? "true" : "false");
                return;
            }
            if (value is double)
            {
                double d = (double)value;
                if (double.IsNaN(d) || double.IsInfinity(d)) sb.Append("null");
                else sb.Append(d.ToString("0.######", CultureInfo.InvariantCulture));
                return;
            }
            if (value is float)
            {
                sb.Append(((float)value).ToString("0.######", CultureInfo.InvariantCulture));
                return;
            }
            if (value is int || value is long || value is short || value is byte || value is uint)
            {
                sb.Append(Convert.ToInt64(value, CultureInfo.InvariantCulture).ToString(CultureInfo.InvariantCulture));
                return;
            }
            JsonObject obj = value as JsonObject;
            if (obj != null)
            {
                WriteObject(sb, obj, indent);
                return;
            }
            IDictionary dict = value as IDictionary;
            if (dict != null)
            {
                JsonObject converted = new JsonObject();
                foreach (DictionaryEntry entry in dict)
                    converted.Add(Convert.ToString(entry.Key, CultureInfo.InvariantCulture), entry.Value);
                WriteObject(sb, converted, indent);
                return;
            }
            IEnumerable seq = value as IEnumerable;
            if (seq != null)
            {
                WriteArray(sb, seq, indent);
                return;
            }
            WriteString(sb, Convert.ToString(value, CultureInfo.InvariantCulture));
        }

        private static void WriteObject(StringBuilder sb, JsonObject obj, int indent)
        {
            if (obj.Count == 0)
            {
                sb.Append("{}");
                return;
            }
            sb.Append("{\r\n");
            for (int i = 0; i < obj.Count; i++)
            {
                Indent(sb, indent + 1);
                WriteString(sb, obj[i].Key);
                sb.Append(": ");
                WriteValue(sb, obj[i].Value, indent + 1);
                if (i < obj.Count - 1) sb.Append(',');
                sb.Append("\r\n");
            }
            Indent(sb, indent);
            sb.Append('}');
        }

        private static void WriteArray(StringBuilder sb, IEnumerable seq, int indent)
        {
            List<object> items = new List<object>();
            foreach (object item in seq) items.Add(item);
            if (items.Count == 0)
            {
                sb.Append("[]");
                return;
            }
            sb.Append("[\r\n");
            for (int i = 0; i < items.Count; i++)
            {
                Indent(sb, indent + 1);
                WriteValue(sb, items[i], indent + 1);
                if (i < items.Count - 1) sb.Append(',');
                sb.Append("\r\n");
            }
            Indent(sb, indent);
            sb.Append(']');
        }

        private static void Indent(StringBuilder sb, int level)
        {
            sb.Append(new string(' ', level * 2));
        }

        private static void WriteString(StringBuilder sb, string text)
        {
            sb.Append('"');
            for (int i = 0; i < text.Length; i++)
            {
                char c = text[i];
                switch (c)
                {
                    case '"': sb.Append("\\\""); break;
                    case '\\': sb.Append("\\\\"); break;
                    case '\r': sb.Append("\\r"); break;
                    case '\n': sb.Append("\\n"); break;
                    case '\t': sb.Append("\\t"); break;
                    default:
                        if (c < ' ') sb.Append("\\u").Append(((int)c).ToString("x4", CultureInfo.InvariantCulture));
                        else sb.Append(c);
                        break;
                }
            }
            sb.Append('"');
        }

        public static object Parse(string text)
        {
            if (text == null) throw new ArgumentNullException("text");
            int index = 0;
            object value = ParseValue(text, ref index);
            SkipWhitespace(text, ref index);
            if (index != text.Length) throw new FormatException("JSON 尾部存在多余字符，位置 " + index);
            return value;
        }

        private static object ParseValue(string s, ref int i)
        {
            SkipWhitespace(s, ref i);
            if (i >= s.Length) throw new FormatException("JSON 意外结束");
            char c = s[i];
            if (c == '{') return ParseObject(s, ref i);
            if (c == '[') return ParseArray(s, ref i);
            if (c == '"') return ParseString(s, ref i);
            if (c == 't') { Expect(s, ref i, "true"); return true; }
            if (c == 'f') { Expect(s, ref i, "false"); return false; }
            if (c == 'n') { Expect(s, ref i, "null"); return null; }
            return ParseNumber(s, ref i);
        }

        private static JsonObject ParseObject(string s, ref int i)
        {
            JsonObject obj = new JsonObject();
            i++;
            SkipWhitespace(s, ref i);
            if (i < s.Length && s[i] == '}')
            {
                i++;
                return obj;
            }
            while (true)
            {
                SkipWhitespace(s, ref i);
                if (i >= s.Length || s[i] != '"') throw new FormatException("JSON 对象键必须是字符串，位置 " + i);
                string key = ParseString(s, ref i);
                SkipWhitespace(s, ref i);
                if (i >= s.Length || s[i] != ':') throw new FormatException("JSON 缺少冒号，位置 " + i);
                i++;
                object value = ParseValue(s, ref i);
                obj.Add(key, value);
                SkipWhitespace(s, ref i);
                if (i >= s.Length) throw new FormatException("JSON 对象未闭合");
                if (s[i] == ',') { i++; continue; }
                if (s[i] == '}') { i++; return obj; }
                throw new FormatException("JSON 对象出现意外字符 " + s[i] + "，位置 " + i);
            }
        }

        private static List<object> ParseArray(string s, ref int i)
        {
            List<object> list = new List<object>();
            i++;
            SkipWhitespace(s, ref i);
            if (i < s.Length && s[i] == ']')
            {
                i++;
                return list;
            }
            while (true)
            {
                list.Add(ParseValue(s, ref i));
                SkipWhitespace(s, ref i);
                if (i >= s.Length) throw new FormatException("JSON 数组未闭合");
                if (s[i] == ',') { i++; continue; }
                if (s[i] == ']') { i++; return list; }
                throw new FormatException("JSON 数组出现意外字符 " + s[i] + "，位置 " + i);
            }
        }

        private static string ParseString(string s, ref int i)
        {
            i++; // opening quote
            StringBuilder sb = new StringBuilder();
            while (true)
            {
                if (i >= s.Length) throw new FormatException("JSON 字符串未闭合");
                char c = s[i++];
                if (c == '"') return sb.ToString();
                if (c != '\\')
                {
                    sb.Append(c);
                    continue;
                }
                if (i >= s.Length) throw new FormatException("JSON 转义序列不完整");
                char esc = s[i++];
                switch (esc)
                {
                    case '"': sb.Append('"'); break;
                    case '\\': sb.Append('\\'); break;
                    case '/': sb.Append('/'); break;
                    case 'b': sb.Append('\b'); break;
                    case 'f': sb.Append('\f'); break;
                    case 'n': sb.Append('\n'); break;
                    case 'r': sb.Append('\r'); break;
                    case 't': sb.Append('\t'); break;
                    case 'u':
                        if (i + 4 > s.Length) throw new FormatException("JSON \\u 转义不完整");
                        sb.Append((char)int.Parse(s.Substring(i, 4), NumberStyles.HexNumber, CultureInfo.InvariantCulture));
                        i += 4;
                        break;
                    default: throw new FormatException("JSON 未知转义 \\" + esc);
                }
            }
        }

        private static double ParseNumber(string s, ref int i)
        {
            int start = i;
            if (i < s.Length && (s[i] == '-' || s[i] == '+')) i++;
            while (i < s.Length && (char.IsDigit(s[i]) || s[i] == '.' || s[i] == 'e' || s[i] == 'E' || s[i] == '+' || s[i] == '-')) i++;
            if (i == start) throw new FormatException("JSON 数字格式非法，位置 " + start);
            string token = s.Substring(start, i - start);
            double value;
            if (!double.TryParse(token, NumberStyles.Float, CultureInfo.InvariantCulture, out value))
                throw new FormatException("JSON 数字无法解析：" + token);
            return value;
        }

        private static void SkipWhitespace(string s, ref int i)
        {
            while (i < s.Length && (s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n')) i++;
        }

        private static void Expect(string s, ref int i, string token)
        {
            if (i + token.Length > s.Length || string.CompareOrdinal(s, i, token, 0, token.Length) != 0)
                throw new FormatException("JSON 期望 " + token + "，位置 " + i);
            i += token.Length;
        }
    }
}
