using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


namespace ConsoleApp1
{
    class Vertex
    {
        private double[] pos_3D;
        private double[] pos_2D;

        public Vertex(double[] pos)
        {
            pos_3D = pos;
            pos_2D = new double[2];
            calcPos(0, 0, 0);
        }
        void calcPos(double x, double y, double z)
        {
            double distance = Math.Sqrt(Math.Pow((pos_3D[0] - x),2) + Math.Pow((pos_3D[1] - y), 2) + Math.Pow((pos_3D[2] - z), 2)) / 100;
            double change = 1 / (1 + distance);
            pos_2D[0] = pos_3D[0] * distance;
            pos_2D[1] = pos_3D[1] * distance;
        }

        void setPos(double[] pos)
        {
            pos_3D = pos;
            calcPos(0, 0, 0);
        }

        double[] getPos_3D()
        {
            return pos_3D;
        }

        double[] getPos_2D()
        {
            return pos_2D;
        }
    }
}
