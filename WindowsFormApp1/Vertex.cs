using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WindowsFormsApp1
{
    class Vertex
    {
        private double[] pos_3D;
        private double[] pos_2D;
        private double[] size;

        public Vertex(double[] pos, double sizeX, double sizeY, double[] cameraPos, double[] cameraRot)
        {
            size = new double[2];
            size[0] = sizeX;
            size[1] = sizeY;
            pos_3D = pos;

            pos_2D = new double[] {size[0], size[1]};
            calcPos(cameraPos, cameraRot);
        }
        private double getLength(double[] vec)
        {
            return Math.Sqrt(Math.Pow((vec[0]), 2) + Math.Pow((vec[1]), 2) + Math.Pow((vec[2]), 2));
        }

        private double[] vectorDiff(double[] vec1, double[] vec2)
        {
            return new double[] { vec1[0] - vec2[0], vec1[1] - vec2[1], vec1[2] - vec2[2] };
        }

        void calcPos(double[] cameraPos, double[] cameraRot)
        {
            double[] dTransform = multMatrix(multMatrix(multMatrix(vectorDiff(pos_3D, cameraPos), new double[][] { new double[] { Math.Cos(cameraRot[2]), Math.Sin(cameraRot[2]), 0 }, new double[] { -Math.Sin(cameraRot[2]), Math.Cos(cameraRot[2]),0 }, new double[] { 0, 0, 1 } }), new double[][] { new double[] { Math.Cos(cameraRot[1]), 0, -Math.Sin(cameraRot[1]) }, new double[] { 0, 1, 0 }, new double[] { Math.Sin(cameraRot[1]), 0, Math.Cos(cameraRot[1]) } }),new double[][] { new double[] { 1, 0, 0 }, new double[] { 0, Math.Cos(cameraRot[0]), Math.Sin(cameraRot[0]) }, new double[] { 0, -Math.Sin(cameraRot[0]), Math.Cos(cameraRot[0]) } });

            double[] notEZ = new double[] { 0, 0, 1000 };

            pos_2D[0] = size[0] + notEZ[2] * dTransform[0] / dTransform[2] + notEZ[0];
            pos_2D[1] = size[1] +  notEZ[2] * dTransform[1] / dTransform[2] + notEZ[1];
            /*
            double[] cameraDir = new double[] { Math.Cos(cameraRot[0]) * Math.Sin(cameraRot[1]), Math.Sin(cameraRot[0]) * Math.Sin(cameraRot[1]), Math.Cos(cameraRot[1]) };
           
            double distance = (Math.Sqrt(Math.Pow((pos_3D[0] - cameraPos[0]), 2) + Math.Pow((pos_3D[1] - cameraPos[1]), 2) + Math.Pow((pos_3D[2] - cameraPos[2]), 2))) / 100;
            double[] direction = new double[] { pos_3D[0] - cameraPos[0], pos_3D[1] - cameraPos[1], pos_3D[2] - cameraPos[2] };
            double theta = Math.Acos((cameraDir[0] * direction[0] + cameraDir[1] * direction[1] + cameraDir[2] * direction[2])/(getLength(cameraDir) * getLength(direction)));
            
            
            if (dotProduct(direction, cameraDir) > 0)
            {
                
                double[] line = new double[] { direction[0], direction[1], direction[2], cameraPos[0], cameraPos[1], cameraPos[2] };
                Console.WriteLine("check check");
                double[] plane = new double[] { cameraDir[0], cameraDir[1], cameraDir[2], cameraPos[0] + cameraDir[0] * size[0], cameraPos[1] + cameraDir[1] * size[0], cameraPos[2] + cameraDir[2] * size[0] };

                // cameraDir[0] (x - cameraPos[0] - cameraRot[0] * size[0]) + cameraDir[1] (y - cameraPos[1] - cameraRot[1] * size[0]) + cameraDir[2] (z - cameraPos[2] - cameraRot[2] * size[0]) = 0
                // x = cameraPos[0] + t * direction[0]
                // y = cameraPos[1] + t * direction[1]
                // z = cameraPos[2] + t * direction[2]

                // cameraDir[0] (t * direction[0] - cameraDir[0] * size[0]) + cameraDir[1] (t * direction[1] - cameraDir[1] * size[0]) + cameraDir[2] (t * direction[2] - cameraDir[2] * size[0]) = 0
                // direction[0]^2 t + direction[1]^2 t + direction[2]^2 t = size[0] ( direction[0] * cameraRot[0] +  direction[1] * cameraRot[1] +  direction[2] * cameraRot[2])
                // t = size[0] ( direction[0] * cameraDir[0] +  direction[1] * cameraDir[1] +  direction[2] * cameraDir[2]) / (direction[0]^2 + direction[1]^2 + direction[2]^2)

                double t_value = 25 * (Math.Pow(cameraDir[0],2) + Math.Pow(cameraDir[1], 2) + Math.Pow(cameraDir[2], 2)) / (direction[0] * cameraDir[0] + direction[1] * cameraDir[1] + direction[2] * cameraDir[2]);

                double[] intersection = new double[] { cameraPos[0] + t_value * direction[0], cameraPos[1] + t_value * direction[1], cameraPos[2] + t_value * direction[2]};

                Console.WriteLine("ROTATE \t {0} \t {1} \t {2}", cameraDir[0], cameraDir[1], cameraDir[2]);
                Console.WriteLine("INTERSECTION \t {0} \t {1} \t {2}", intersection[0], intersection[1], intersection[2]);
                Console.WriteLine("POINT \t {0} \t {1} \t {2}", pos_3D[0], pos_3D[1], pos_3D[2]);
                pos_2D[0] = size[0] + (size[0] * direction[0] / intersection[2]);
                pos_2D[1] = size[1] + (size[0] * direction[1] / intersection[2]);
                Console.WriteLine("DISTANCE \t {0}", distance);
            }
            */
        }

        private double[] rotate(double[] theta)
        {
            double[] vec = new double[] { 0, 0, 1 };
            double[][] rXMatrix = new double[][] { new double[] { 1, 0, 0 }, new double[] { 0, Math.Cos(theta[0]), -Math.Sin(theta[0]) }, new double[] { 0, Math.Sin(theta[0]), Math.Cos(theta[0]) } };
            double[][] rYMatrix = new double[][] { new double[] { Math.Cos(theta[1]), 0, Math.Sin(theta[1]) }, new double[] { 0, 1, 0 }, new double[] { -Math.Sin(theta[1]), 0, Math.Cos(theta[1]) } };
            double[][] rZMatrix = new double[][] { new double[] { Math.Cos(theta[2]), -Math.Sin(theta[2]), 0 }, new double[] { Math.Sin(theta[2]), Math.Cos(theta[2]), 0 }, new double[] { 0, 0, 1 } };
            return multMatrix(multMatrix(multMatrix(vec, rZMatrix), rYMatrix), rXMatrix);
        }

        private double[] multMatrix(double[] vect, double[][] mat)
        {
            double[] result = new double[mat[0].Length];
            for (int i = 0; i < vect.Length; i++)
            {
                result[i] = vect[0] * mat[i][0] + vect[1] * mat[i][1] + vect[2] * mat[i][2];
            }
            return result;
        }

        private double dotProduct(double[] vec1, double[] vec2)
        {
            return vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2];
        }

        public void setPos3D(double[] pos, double[] cameraPos, double[] cameraRot)
        {
            pos_3D = pos;
            calcPos(cameraPos, cameraRot);
        }

        public void shiftPos3D(double[] pos, double[] cameraPos, double[] cameraRot)
        {
            for(int i = 0; i < 3; i++)
            {
                pos_3D[i] = pos_3D[i] + pos[i];
            }

            calcPos(cameraPos, cameraRot);
        }

        public double[] getPos_3D()
        {
            return pos_3D;
        }

        public double[] getPos_2D()
        {
            return pos_2D;
        }

        public Boolean equals(Vertex v)
        {
            double[] vPos = v.getPos_3D();
            if (vPos[0] == pos_3D[0])
            {
                if (vPos[1] == pos_3D[1])
                {
                    if (vPos[2] == pos_3D[2])
                    {
                        //Console.WriteLine("equals");
                        return true;
                    }
                    else
                    {
                        //Console.WriteLine("vPos[2] != pos_3D[2]");
                        return false;
                    }
                }
                else
                {
                    //Console.WriteLine("vPos[1] != pos_3D[1]");
                    return false;
                }
            }
            else
            {
                //Console.WriteLine("vPos[0] != pos_3D[0]");
                return false;
            }
        }
    }
}
