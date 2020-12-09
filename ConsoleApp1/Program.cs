using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Drawing;
using System.Windows.Forms;

namespace ConsoleApp1
{
    class Program
    {
        static void Main(string[] args)
        {
            double[] pos = new double[]{ 1, 0, 1 };
            Vertex v1 = new Vertex(pos);
            pos[0] = 2;
            Vertex v2 = new Vertex(pos);

            pos[0] = -1;
            pos[2] = 3;
            Vertex v3 = new Vertex(pos);

            pos[0] = -2;
            pos[2] = 2;
            Vertex v4 = new Vertex(pos);

            Pen blackPen = new Pen(Color.Black, 3);
            Graphics e = CreateGraphics();
            e.DrawLine(blackPen, x1, y1, x2, y2);
        }
    }
}
