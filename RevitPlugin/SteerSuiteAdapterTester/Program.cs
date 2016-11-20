using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using SteerSuiteAdapter;

namespace SteerSuiteAdapterTester
{
    class Program
    {
        static void Main(string[] args)
        {
            SteerAdapter sa = new SteerAdapter();
            Console.WriteLine("Hello, World!");
            Console.WriteLine("optimize says: " + sa.optimize(-5.3));

            UInt64 size = 99;
            double[] points = new double[size];
            UInt64[] faces = new UInt64[size];
            for ( UInt64 i=0; i < size; i++)
            {
                points[i] = Math.Sqrt(i + 2);
            }
            for (UInt64 i = 0; i < size; i++)
            {
                faces[i] = (i + 2);
            }
            sa.optimize2(points, size, faces, size);

            UInt64 aabb_size = 6*6;
            double[] aabbs = new double[aabb_size];
            double[] stuff = { 7.60943188553042, 8.26559986453306,0, 13.1233595800525,3.31788163576327, 11.5199813732961,
                                7.93751587503174, 16.4676996020659,0, 13.1233595800525,3.31788163576327, 3.9740496147659,
                                15.8115316230632, 16.4676996020659,0, 13.1233595800525,3.64596562526458, 11.5199813732961,
                                12.8587757175514, 16.1396156125646,0, 13.1233595800525,10.8638133942934, 11.5199813732961,
                                7.93751587503175, 11.2183557700449,0, 13.1233595800525,10.8638133942934, 11.5199813732961,
                                11.6416899500466, 12.2978579290492,0, 13.1233595800525,8.98526797053594, 9.64143594953857

};
            
            for (UInt64 i = 0; i < aabb_size; i++)
            {
                aabbs[i] = stuff[i];
            }
            sa.optimize3(aabbs, aabb_size, 2);
        }
    }
}
