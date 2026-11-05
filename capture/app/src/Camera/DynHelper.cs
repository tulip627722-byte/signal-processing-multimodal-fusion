using System;
using System.Collections;
using System.Globalization;
using System.Reflection;

namespace Acquire
{
    /// <summary>
    /// 极简反射调用助手：只在 RealSense 这种"编译期不能硬依赖"的地方用。
    /// 失败时给出带成员名的清晰异常，方便界面把原因原样显示出来。
    /// </summary>
    internal static class DynHelper
    {
        public static object Get(object target, string property)
        {
            object value;
            string error;
            if (!TryGet(target, property, out value, out error)) throw new MissingMemberException(error);
            return value;
        }

        public static object GetOptional(object target, string property)
        {
            object value;
            string error;
            TryGet(target, property, out value, out error);
            return value;
        }

        public static bool TryGet(object target, string property, out object value, out string error)
        {
            value = null;
            error = null;
            if (target == null)
            {
                error = "目标为 null，无法读取属性 " + property;
                return false;
            }
            Type type = target.GetType();
            PropertyInfo prop = type.GetProperty(property, BindingFlags.Public | BindingFlags.Instance);
            if (prop != null)
            {
                value = prop.GetValue(target, null);
                return true;
            }
            FieldInfo field = type.GetField(property, BindingFlags.Public | BindingFlags.Instance);
            if (field != null)
            {
                value = field.GetValue(target);
                return true;
            }
            error = type.FullName + " 没有属性 " + property;
            return false;
        }

        public static object Call(object target, string method, params object[] args)
        {
            object result;
            string error;
            if (!TryCall(target, method, args, out result, out error)) throw new MissingMethodException(error);
            return result;
        }

        /// <summary>依次尝试多个方法名，都找不到时返回 null（不抛异常）。</summary>
        public static object CallFirst(object target, string[] methods, params object[] args)
        {
            if (target == null) return null;
            foreach (string name in methods)
            {
                object result;
                string error;
                if (TryCall(target, name, args, out result, out error)) return result;
            }
            return null;
        }

        public static bool TryCall(object target, string method, object[] args, out object result, out string error)
        {
            result = null;
            error = null;
            if (target == null)
            {
                error = "目标为 null，无法调用 " + method;
                return false;
            }
            Type type = target.GetType();
            MethodInfo selected = Select(type, method, args, out error);
            if (selected == null) return false;
            try
            {
                result = selected.Invoke(target, args);
                return true;
            }
            catch (TargetInvocationException ex)
            {
                Exception inner = ex.InnerException ?? ex;
                throw new InvalidOperationException(type.Name + "." + method + " 调用失败：" + inner.Message, inner);
            }
        }

        public static object New(Type type, params object[] args)
        {
            if (type == null) throw new ArgumentNullException("type");
            ConstructorInfo selected = null;
            string detail = null;
            foreach (ConstructorInfo ctor in type.GetConstructors())
            {
                ParameterInfo[] ps = ctor.GetParameters();
                if (ps.Length != args.Length) continue;
                if (!ArgsMatch(ps, args)) continue;
                selected = ctor;
                break;
            }
            if (selected == null)
            {
                detail = type.FullName + " 没有匹配的构造函数（" + args.Length + " 个参数）";
                throw new MissingMethodException(detail);
            }
            try
            {
                return selected.Invoke(args);
            }
            catch (TargetInvocationException ex)
            {
                Exception inner = ex.InnerException ?? ex;
                throw new InvalidOperationException(type.Name + " 构造失败：" + DescribeException(inner), inner);
            }
        }

        public static object EnumValue(Assembly asm, string typeName, string member)
        {
            Type type = asm.GetType(typeName, false);
            if (type == null) throw new MissingMemberException("找不到类型 " + typeName);
            return Enum.Parse(type, member, true);
        }

        public static IEnumerable Enumerate(object collection)
        {
            if (collection == null) return new object[0];
            IEnumerable seq = collection as IEnumerable;
            return seq ?? new object[0];
        }

        public static string DescribeException(Exception ex)
        {
            while (ex is TargetInvocationException && ex.InnerException != null) ex = ex.InnerException;
            if (ex is DllNotFoundException)
                return "缺少原生库（" + ex.Message.Trim() + "），需要安装 Intel RealSense SDK 2.0 或把 realsense2.dll 放到 exe 旁";
            if (ex is BadImageFormatException)
                return "位数不匹配（" + ex.Message.Trim() + "），请确认使用 x64 版 SDK";
            return ex.GetType().Name + ": " + ex.Message;
        }

        private static MethodInfo Select(Type type, string method, object[] args, out string error)
        {
            error = null;
            MethodInfo best = null;
            int bestScore = -1;
            foreach (MethodInfo candidate in type.GetMethods(BindingFlags.Public | BindingFlags.Instance))
            {
                if (!string.Equals(candidate.Name, method, StringComparison.Ordinal)) continue;
                ParameterInfo[] ps = candidate.GetParameters();
                if (ps.Length != args.Length) continue;
                int score;
                if (!ScoreArgs(ps, args, out score)) continue;
                if (score > bestScore)
                {
                    best = candidate;
                    bestScore = score;
                }
            }
            if (best == null)
                error = type.FullName + " 没有匹配的 " + method + "(" + args.Length + " 个参数)";
            return best;
        }

        private static bool ArgsMatch(ParameterInfo[] ps, object[] args)
        {
            int score;
            return ScoreArgs(ps, args, out score);
        }

        private static bool ScoreArgs(ParameterInfo[] ps, object[] args, out int score)
        {
            score = 0;
            for (int i = 0; i < ps.Length; i++)
            {
                Type p = ps[i].ParameterType;
                object a = args[i];
                if (a == null)
                {
                    if (p.IsValueType && Nullable.GetUnderlyingType(p) == null) return false;
                    continue;
                }
                Type at = a.GetType();
                if (p == at || p.IsAssignableFrom(at))
                {
                    score += 2;
                    continue;
                }
                if (p.IsEnum && at == typeof(string)) { score += 1; continue; }
                if (IsNumeric(p) && IsNumeric(at)) { score += 1; continue; }
                return false;
            }
            return true;
        }

        private static bool IsNumeric(Type type)
        {
            return type == typeof(int) || type == typeof(uint) || type == typeof(long) || type == typeof(ulong) ||
                   type == typeof(short) || type == typeof(ushort) || type == typeof(byte) || type == typeof(sbyte) ||
                   type == typeof(double) || type == typeof(float);
        }

        public static string Format(object value)
        {
            return Convert.ToString(value, CultureInfo.InvariantCulture);
        }
    }
}
