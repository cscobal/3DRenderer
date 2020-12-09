using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Collections;

namespace WindowsFormsApp1
{
    public partial class Form1 : Form
    {
        private Boolean mouseDown = false;
        private double mouseX = 0;
        private double mouseY = 0;
        private ArrayList objs;
        private System.Drawing.Graphics graphicsObj;
        private Pen myPen;
        private double[] cameraPos;
        private double[] cameraRot;

        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_Load(object sender, EventArgs e)
        {

        }
        private void Form1_Paint(object sender, PaintEventArgs e)
        {

        }

        private void Form1_Paint_1(object sender, PaintEventArgs e)
        {
            cameraPos = new double[] { 0, 0, 0 };
            cameraRot = new double[] {0, 0, 0};
            graphicsObj = this.CreateGraphics();
            myPen = new Pen(System.Drawing.Color.Black, 1);

            ArrayList vert3 = new ArrayList();
            Vertex vc1 = new Vertex(new double[] { -50, -50, 600 }, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
            vert3.Add(vc1);
            Vertex vc2 = new Vertex(new double[] { -50, 50, 600 }, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
            vert3.Add(vc2);

            Vertex vc3 = new Vertex(new double[] { 50, 50, 600 }, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
            vert3.Add(vc3);

            Vertex vc4 = new Vertex(new double[] { 50, -50, 600 }, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
            vert3.Add(vc4);
            
            Vertex vc11 = new Vertex(new double[] { 50, -50, 400 }, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
            vert3.Add(vc11);
           Vertex vc12 = new Vertex(new double[] { -50, 50, 400 }, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
           vert3.Add(vc12);
            Vertex vc14 = new Vertex(new double[] { -50, -50, 400 }, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
            vert3.Add(vc14);
            Vertex vc15 = new Vertex(new double[] { 50, 50, 400 }, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
            vert3.Add(vc15);

            Vertex vc16 = new Vertex(new double[] { 0, 0, 100 }, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
            vert3.Add(vc16);

            /*
              
             
             */

            /*
            Vertex vc13 = new Vertex(new double[] { 150, 150, 200 }, ClientRectangle.Width / 2, ClientRectangle.Height / 2);
            //vert3.Add(vc13);

            Vertex vc14 = new Vertex(new double[] { 150, -150, 200 }, ClientRectangle.Width / 2, ClientRectangle.Height / 2);
            //vert3.Add(vc14);
            //Vertex vc5 = new Vertex(new double[] { 0.970142500145332, 0,  0.242535625036333 }, ClientRectangle.Width / 2, ClientRectangle.Height / 2);

            Vertex vc7 = new Vertex(new double[] { 9, -9, -18 }, ClientRectangle.Width / 2, ClientRectangle.Height / 2);

            Vertex vc5 = new Vertex(new double[] { 0, -15, -15 }, ClientRectangle.Width / 2, ClientRectangle.Height / 2);
            Vertex vc6 = new Vertex(new double[] { -9, -9, -18 }, ClientRectangle.Width / 2, ClientRectangle.Height / 2);
            Vertex vc8 = new Vertex(new double[] { 0, -63, 92 }, ClientRectangle.Width / 2, ClientRectangle.Height / 2);


            double[] pos4 = vc7.getPos_2D();
            double[] pos5 = vc5.getPos_2D();
            double[] pos6 = vc6.getPos_2D();
            double[] pos7 = vc8.getPos_2D();
            graphicsObj.DrawLine(myPen, ClientRectangle.Width / 2, ClientRectangle.Height / 2, (float)pos7[0], (float)pos7[1]);
            graphicsObj.DrawLine(myPen, (float)pos4[0], (float)pos4[1], (float)pos5[0], (float)pos5[1]);
            graphicsObj.DrawLine(myPen, (float)pos5[0], (float)pos5[1], (float)pos6[0], (float)pos6[1]);
            graphicsObj.DrawLine(myPen, (float)pos6[0], (float)pos6[1], (float)pos4[0], (float)pos4[1]);
       */




            ArrayList vert = new ArrayList();
 
             vc1 = new Vertex(new double[] { -450, -50, 500 }, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
            vert.Add(vc1);
             vc2 = new Vertex(new double[] { -450, 50, 500 }, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
            vert.Add(vc2);

             vc3 = new Vertex(new double[] { -350, 50, 500 }, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
            vert.Add(vc3);

             vc4 = new Vertex(new double[] { -350, -50, 500 }, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
            vert.Add(vc4);

             vc11 = new Vertex(new double[] { -350, -50, 400 }, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
            vert.Add(vc11);
             vc12 = new Vertex(new double[] { -450, 50, 400 }, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
            vert.Add(vc12);
             vc14 = new Vertex(new double[] { -450, -50, 400 }, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
            vert.Add(vc14);
             vc15 = new Vertex(new double[] { -350, 50, 400 }, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
            vert.Add(vc15);

            double[] pos = new double[] { 75, 75, 225 };
            //Vertex v1 = new Vertex(pos, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
           // vert.Add(v1);
            pos = new double[] { 75, -25, 225 };
           // Vertex v2 = new Vertex(pos, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
          //  vert.Add(v2);

            pos = new double[] { -25, 75, 225 };
           // Vertex v3 = new Vertex(pos, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
           // vert.Add(v3);
           
            pos = new double[] { -25, -25, 225 };
            //Vertex v4 = new Vertex(pos, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
           // vert.Add(v4);
            /*

            pos = new double[] { 25, 25, 87.5 };
            Vertex v21 = new Vertex(pos, ClientRectangle.Width / 2, ClientRectangle.Height / 2);
            vert.Add(v21);
            pos = new double[] { 0, 0, -37.5 };
            Vertex v22 = new Vertex(pos, ClientRectangle.Width / 2, ClientRectangle.Height / 2);
            vert.Add(v22);

            pos = new double[] { 12.5, 250/4 + 25, 25 };
            Vertex v23 = new Vertex(pos, ClientRectangle.Width / 2, ClientRectangle.Height / 2);
            vert.Add(v23);

            pos = new double[] { 12.5, -150/4 - 25, 25 };
            Vertex v24 = new Vertex(pos, ClientRectangle.Width / 2, ClientRectangle.Height / 2);
            vert.Add(v24);
            
            pos = new double[] { -25, 75, 75 };
            Vertex v25 = new Vertex(pos, ClientRectangle.Width / 2, ClientRectangle.Height / 2);
            vert.Add(v25);

            pos = new double[] { -25, -25, 75 };
            Vertex v26 = new Vertex(pos, ClientRectangle.Width / 2, ClientRectangle.Height / 2);
            vert.Add(v26);
            */
            /*
            pos = new double[] { 50, 50, 125 };
             Vertex v5 = new Vertex(pos, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
             vert.Add(v5);
             pos = new double[] { 50, -50, 125 };
             Vertex v6 = new Vertex(pos, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
             vert.Add(v6);

             pos = new double[] { -50, 50, 125 };
             Vertex v7 = new Vertex(pos, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
             vert.Add(v7);

             pos = new double[] { -50, -50, 125 };
             Vertex v8 = new Vertex(pos, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
             vert.Add(v8);
          
            */
            objs = new ArrayList();
            Object3D testObj = new Object3D(vert);
            Object3D testObj3 = new Object3D(vert3);
            //testObj.shiftObj(new double[] { 200, 100, 100 });
            //testObj.rotate(new double[] { 0 ,0,0});
            objs.Add(testObj3);
            //objs.Add(testObj);
            
            

            /*
            ArrayList vert1 = new ArrayList();
            pos = new double[] { 200, 0, 0 };
            Vertex v11 = new Vertex(pos, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
            vert1.Add(v11);
            pos = new double[] { 240, 0, 0 };
            Vertex v12 = new Vertex(pos, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
            vert1.Add(v12);

            pos = new double[] { 180, 40, 0 };
            Vertex v13 = new Vertex(pos, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
            vert1.Add(v13);

            pos = new double[] { 260, 40, 0 };
            Vertex v14 = new Vertex(pos, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
            vert1.Add(v14);


            pos = new double[] { 220, 60, 0 };
            Vertex v15 = new Vertex(pos, ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
            vert1.Add(v15);
            */
            // Object3D testObj2 = new Object3D(vert1);
            //  objs.Add(testObj2);
            drawObjects();
      

           

        }



        private void drawObjects()
        {
            graphicsObj.Clear(Color.White);
            for (int i = 0; i < objs.Count; i ++)
            {
                Console.WriteLine("check");
                ((Object3D)objs[i]).setCameraPos(cameraPos, cameraRot);
                ArrayList edges = ((Object3D)objs[i]).getEdges();
                ArrayList faces = ((Object3D)objs[i]).getFaces();
                /*
                foreach (var e in edges)
                {
                    Vertex[] edge = (Vertex[])e;
                    double[] pos1 = (edge[0]).getPos_2D();
                    double[] pos2 = (edge[1]).getPos_2D();

                    g.DrawLine(p, (float)pos1[0], (float)pos1[1], (float)pos2[0], (float)pos2[1]);
                    System.Threading.Thread.Sleep(500);

                }
                */
                int count1 = 0;
                int count2 = 0;
                
               
                ArrayList face = new ArrayList();
                ArrayList otherFaces = new ArrayList();
                for(int j = 0; j < objs.Count; j++)
                {
                    if(i != j)
                    {
                        ArrayList objFaces = ((Object3D)objs[j]).getFaces();
                        otherFaces.Add(objFaces);
                    }
                }
                ArrayList visFaces = ((Object3D)objs[i]).detectVisibleFaces(cameraPos, this, otherFaces);
                // Console.WriteLine("NUMBER  FACES \t {0}", faces.Count);
                //Console.WriteLine("NUMBER VISIBLE FACES \t {0}", visFaces.Count);
                /*
                Console.WriteLine("camera position \t {0}  \t {1} \t {2}", cameraPos[0], cameraPos[1], cameraPos[2]);
                foreach (var f in faces)
                {
    
                    face = (ArrayList)f;
        
                   // ((Object3D)obj).printList(face);
                    foreach (var e in face)
                    {
                        Vertex[] edge = (Vertex[])e;
                        edge[0].setPos3D(edge[0].getPos_3D(), cameraPos,cameraRot);
                        edge[1].setPos3D(edge[1].getPos_3D(), cameraPos, cameraRot);
                        double[] pos1 = (edge[0]).getPos_2D();
                        double[] pos2 = (edge[1]).getPos_2D();

                        graphicsObj.DrawLine(myPen, (float)pos1[0], (float)pos1[1], (float)pos2[0], (float)pos2[1]);
                    }
            

                }
                */
               foreach (var f in visFaces)
               {
                //   Console.WriteLine("check2");
                    face = (ArrayList)f;
                   //  Console.WriteLine("CHECKINGGG");
                     ((Object3D)objs[i]).printList(face);
                    foreach (var e in face)
                    {
                        Vertex[] edge = (Vertex[])e;
                        double windowSize = (this.ClientRectangle.Width / 2) * (this.ClientRectangle.Height / 2);
                        double[] pos1 = (edge[0]).getPos_2D();
                        double[] pos2 = (edge[1]).getPos_2D();
                        if (pos1[0] < windowSize && pos1[1] < windowSize && pos2[0] < windowSize && pos2[1] < windowSize)
                        { 
                        graphicsObj.DrawLine(myPen, (float)pos1[0], (float)pos1[1], (float)pos2[0], (float)pos2[1]);
                        }

                   }
                  

                 }
                 
                /*
             face = (ArrayList)faces[0];
             ArrayList edgesToShow = ((Object3D)obj).showFace(face, new double[]{ -150, 0, 0}, new double[]{ 0, 0, 0});
             Console.WriteLine("eyyy");
             foreach (var e in edgesToShow)
             {
                 double[][] edge = (double[][])e;
                 Vertex vert1 = new Vertex(edge[0], ClientRectangle.Width / 2, ClientRectangle.Height / 2);
                 Vertex vert2 = new Vertex(edge[1], ClientRectangle.Width / 2, ClientRectangle.Height / 2);

                 double[] pos1 = (vert1).getPos_2D();
                 double[] pos2 = (vert2).getPos_2D();

                 g.DrawLine(p, (float)pos1[0], (float)pos1[1], (float)pos2[0], (float)pos2[1]);

             }
             */
            }

        }





        private void Form1_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.KeyCode == Keys.W)
            {
                cameraPos[2] += 5;
            }
            else if (e.KeyCode == Keys.S)
            {
                cameraPos[2] -= 5;
            }
            else if (e.KeyCode == Keys.A)
            {
                cameraPos[0] -= 5;
            }
            else if (e.KeyCode == Keys.D)
            {
                cameraPos[0] += 5;
            }
            else if (e.KeyCode == Keys.Space)
            {
                cameraPos[1] -= 5;
            }
            else if (e.KeyCode == Keys.ShiftKey)
            {
                cameraPos[1] += 5;
            }
            else if (e.KeyCode == Keys.Insert)
            {
                cameraRot[0] += 5.0/180.0 * Math.PI;
            }
            else if (e.KeyCode == Keys.Delete)
            {
                cameraRot[0] -= 5.0 / 180.0 * Math.PI;
            }
            else if (e.KeyCode == Keys.Home)
            {
                cameraRot[1] += 5.0 / 180.0 * Math.PI;
            }
            else if (e.KeyCode == Keys.End)
            {
                cameraRot[1] -= 5.0 / 180.0 * Math.PI;
            }
            else if (e.KeyCode == Keys.PageUp)
            {
                cameraRot[2] += 5.0 / 180.0 * Math.PI;
            }
            else if (e.KeyCode == Keys.PageDown)
            {
                cameraRot[2] -= 5.0 / 180.0 * Math.PI;
            }
            Console.WriteLine("key pressed");
   
            drawObjects();
        }
        public void drawEdges(ArrayList edges, System.Drawing.Color color)
        {
            for(int i = 0; i < edges.Count; i++)
            {

               // Console.WriteLine("checking cast");
                Vertex v1 = new Vertex(((double[][])edges[i])[0], ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
                Vertex v2 = new Vertex(((double[][])edges[i])[1], ClientRectangle.Width / 2, ClientRectangle.Height / 2, cameraPos, cameraRot);
               // Console.WriteLine("cst checked");
                double[] pos1 = v1.getPos_2D();
                double[] pos2 = v2.getPos_2D();
                Pen aPen = new Pen(color, 1);
                graphicsObj.DrawLine(aPen, (float)pos1[0], (float)pos1[1], (float)pos2[0], (float)pos2[1]);
            }
        }
        public void clear()
        {
            graphicsObj.Clear(Color.White);
        }
    }
}
