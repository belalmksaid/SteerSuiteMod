using System;
using System.Runtime.InteropServices;

namespace SteerSuiteAdapter
{
    
    public class SteerAdapter
    {
        /// <summary>
        /// The native methods in the DLL's unmanaged code.
        /// </summary>
        internal static class UnsafeNativeMethods
        {
            const string _dllLocation = "steerplugin.dll";
            [DllImport(_dllLocation, EntryPoint = "optimize", CallingConvention = CallingConvention.Cdecl)]
            public static extern double optimize(double a);
            [DllImport(_dllLocation, EntryPoint = "optimize2", CallingConvention = CallingConvention.Cdecl)]
            public static extern double optimize2( double[] points, UInt64 point_length, UInt64[] faces, UInt64 face_length);
            [DllImport(_dllLocation, EntryPoint = "optimize3", CallingConvention = CallingConvention.Cdecl)]
            public static extern double optimize3(double[] aabbs, UInt64 num_aabbs, UInt64 aabb_to_optimize);
        }

        /// <summary>
        /// Simulate N games in the DLL.
        /// </summary>
        public double optimize(double num)
        {
            return UnsafeNativeMethods.optimize(num);
        }

        public double optimize2(double[] points, UInt64 point_length, UInt64[] faces, UInt64 face_length)
        {
            return UnsafeNativeMethods.optimize2(points, point_length, faces, face_length);
        }

        public double optimize3(double[] aabbs, UInt64 num_aabbs, UInt64 aabb_to_optimize)
        {
            return UnsafeNativeMethods.optimize3(aabbs, num_aabbs, aabb_to_optimize);
        }
    }
}

