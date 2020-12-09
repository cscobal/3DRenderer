using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Collections;

namespace WindowsFormsApp1
{
    class Object3D
    {
        private ArrayList verts = new ArrayList();
        private ArrayList surfVert = new ArrayList();
        private ArrayList edges = new ArrayList();
        private ArrayList faces = new ArrayList();
        private double[] center = new double[3];
        private double maxPossibleDotP;
        private double[] cameraPos;
        double[] normDirection;
        private double[] cameraRot;
        private Form1 form;

        public Object3D(ArrayList v)
        {
            
            double[][] testing = twoPlaneIntersection(new double[] {1, 1, 1, 1}, new double[] {1, 2, 3, 4});
            double[][] line1 = new double[][] { new double[] { 2, 1 }, new double[] { 3, 2 }, new double[] { 4, 3}};
            double[][] line2 = new double[][] { new double[] { 1, 2 }, new double[] { 2, 4 }, new double[] { -4, -1 } };
            //Console.WriteLine("T_VALUE PART OF LINE \t {0} \t {1} \t {2}", testing[0][0], testing[1][0], testing[2][0]);
            //Console.WriteLine("CONSTANT PART OF LINE \t {0} \t {1} \t {2}", testing[0][1], testing[1][1], testing[2][1]);
            double[] intPoint = lineToLineIntersection(line1, line2);
            System.Threading.Thread.Sleep(10000);

            cameraPos = new double[] { 0, 0, 0 };
            cameraRot = new double[] { 0, 0 };
            verts = v;
            normDirection = new double[] { 0, 1, 0 };
            
            center = new double[] { 0, 0, 0 };
            foreach (var ver in verts)
            {
                Vertex vert = (Vertex)ver;
                double[] pos = vert.getPos_3D();
                center[0] += pos[0];
                center[1] += pos[1];
                center[2] += pos[2];
            }
            center[0] = center[0] / verts.Count;
            center[1] = center[1] / verts.Count;
            center[2] = center[2] / verts.Count;
            Vertex maxV = (Vertex)verts[0];
            ArrayList hullV = new ArrayList();
            foreach (var ver in verts)
            {
                Vertex vert;
                vert = (Vertex)ver;
                double[] pos = vert.getPos_3D();
                double[] maxPos = maxV.getPos_3D();

                if (Math.Pow((pos[0] - center[0]), 2) + Math.Pow((pos[1] - center[1]), 2) + Math.Pow((pos[2] - center[2]), 2) > Math.Pow((maxPos[0] - center[0]), 2) + Math.Pow((maxPos[1] - center[1]), 2) + Math.Pow((maxPos[2] - center[2]), 2))
                {
                    maxV = vert;
                }

            }

            ArrayList tempV = new ArrayList();
            foreach (var vertices in verts)
            {
                if (!maxV.equals((Vertex)vertices))
                {
                    tempV.Add(vertices);
                }
            }
            hullV.Add(maxV);
            tempV.Remove(maxV);
            double[] maxP = maxV.getPos_3D();
            Vertex maxVert = (Vertex)tempV[0];
            maxPossibleDotP = -2 * Math.Pow(getLength(vectorDiff(maxVert.getPos_3D(), center)),4);
      
            double dotP = 0;
            if (tempV.Count > 0)
            {
                foreach (var vert1 in tempV)
                {
                    Vertex v1 = (Vertex)vert1;
                    double[] pos1 = new double[3];
                    double[] tempPos1 = v1.getPos_3D();
                    double[] pos2 = new double[3];
                    double[] tempPos2 = maxV.getPos_3D();
                    for (int i = 0; i < 3; i++)
                    {
                        pos1[i] = tempPos1[i] - center[i];
                        pos2[i] = tempPos2[i] - center[i];
                    }
                    if ((pos1[0] * pos2[0]) + (pos1[1] * pos2[1]) + (pos1[2] * pos2[2]) >= dotP)
                    {
                        maxVert = v1;
                        dotP = (pos1[0] * pos2[0]) + (pos1[1] * pos2[1]) + (pos1[2] * pos2[2]);
                    }

                }
            }
            hullV.Add(maxVert);
            tempV.Remove(maxVert);
            Vertex[] firstEdge = new Vertex[] { (Vertex)hullV[0], (Vertex)hullV[1] };
            //edges.Add(firstEdge);
            double[] e1v1 = firstEdge[0].getPos_3D();
            double[] e1v2 = firstEdge[1].getPos_3D();
           double[] direction = multScalar(1,getDirection2D(firstEdge, center));
           // System.Console.WriteLine("RESULT \t {0} \t {1} \t {2}", getDirection2D(firstEdge, center)[0], getDirection2D(firstEdge, center)[1], getDirection2D(firstEdge, center)[2]);
            recurseEdge(firstEdge, direction);
          
            /*
            ArrayList tempVerts = new ArrayList();
            foreach (var vs in verts)
            {
                tempVerts.Add(vs);
            }
            ArrayList tempEdges = new ArrayList();
            foreach (var e in edges)
            {
                tempEdges.Add(e);
            }
           findFaces(tempVerts, tempEdges);
           */
        }

        private Boolean checkEdge(Vertex[] edge)
        {
            foreach (var eArray in edges)
            {
                Vertex[] e = (Vertex[])eArray;
                if ((e[0].equals(edge[0]) && e[1].equals(edge[1])) || (e[1].equals(edge[0]) && e[0].equals(edge[1])))
                {
                    return true;
                }
            }
            return false;
        }

        public void setCameraPos(double[] pos, double[] cameraRot)
        {
            cameraPos = pos;
            foreach(var v in verts)
            {
                Vertex vert = (Vertex)v;
                vert.setPos3D(vert.getPos_3D(), cameraPos, cameraRot);
            }
        }

        private double[] vertexVector (Vertex v)
        {
            double[] edgePos1 = new double[3];
            double[] tempPos1 = v.getPos_3D();
            for (int i = 0; i < 3; i++)
            {
                edgePos1[i] = tempPos1[i] - center[i];
            }
            return edgePos1;
        }
       
        private void recurseEdge(Vertex[] edgeTemp, double[] directionTemp)
        {
            Boolean first = true;
            ArrayList edgeQueue = new ArrayList();
            ArrayList dirQueue = new ArrayList();
            edgeQueue.Add(edgeTemp);
            ArrayList vertQueue = new ArrayList();
            ArrayList directionList = new ArrayList();
            edges.Add(edgeTemp);
            dirQueue.Add(directionTemp);
            while (edgeQueue.Count > 0 || vertQueue.Count > 0)
            {
                //Console.WriteLine("COUNT \t {0}", edgeQueue.Count);
                double[] direction;
                Vertex[] edge;
                edge = (Vertex[])edgeQueue[0];
                edgeQueue.Remove(edge);
                double[] e2v1 = edge[0].getPos_3D();
                double[] e2v2 = edge[1].getPos_3D();
    
                direction = (double[])dirQueue[0];

                dirQueue.Remove(direction);
                double maxDotP = -Math.Pow(getLength(vertexVector(edge[0])), 2);
                
                ArrayList vertList = new ArrayList();
                double[] edgePos1 = new double[3];
                double[] tempPos1 = edge[0].getPos_3D();
                double[] edgePos2 = new double[3];
                double[] tempPos2 = edge[1].getPos_3D();
                for (int i = 0; i < 3; i++)
                {
                    edgePos1[i] = tempPos1[i] - center[i];
                    edgePos2[i] = tempPos2[i] - center[i];
                }
                //Console.WriteLine("DOTP CHECK \t {0}", dotProduct(direction, edgeVector(edge)));
                foreach (var vert1 in verts)
                {
                    Vertex v1 = (Vertex)vert1;
                    if (!v1.equals(edge[0]) && !v1.equals(edge[1]))
                    {
                        e2v1 = v1.getPos_3D();
                        
                       // Console.WriteLine("VERTEX CHECKING \t {0} \t {1} \t {2} ", e2v1[0], e2v1[1], e2v1[2]);

                        double[] pos = vertexVector(v1);

                        e2v1 = edge[0].getPos_3D();
                        e2v2 = edge[1].getPos_3D();
                        //Console.WriteLine("STARTING EDGE VERTEX 1 \t {0} \t {1} \t {2} \t VERTEX 2 \t {3} \t {4} \t {5}", e2v1[0], e2v1[1], e2v1[2], e2v2[0], e2v2[1], e2v2[2]);
                        double dotP;
                        if (first)
                        {
                             dotP = dotProductPlane(edge, v1, direction, 1);
                           // edgeQueue.Add(edge);
                           // dirQueue.Add(direction);
                        }
                        else
                        {
                             dotP = dotProductPlane(edge, v1, direction, -1);
                        }
                       // Console.WriteLine("DOTP CHECKING \t {0} \t {1}", dotP, maxDotP);
                        if(direction[2] < 0)
                        {
                            e2v1 = v1.getPos_3D();
                        }
                        if (dotP > maxDotP)
                        {
                            vertList = new ArrayList();
                            vertList.Add(v1);
                            maxDotP = dotP;
                        }
                        else if (dotP == maxDotP)
                        {
                            vertList.Add(v1);
                        }
                   }
                }
                first = false;
                
            foreach(var v in vertList)
                {
                    Vertex vert = (Vertex)v;
                    e2v1 = vert.getPos_3D();
                    //Console.WriteLine(" VERTICES \t {0} \t {1} \t {2} ", e2v1[0], e2v1[1], e2v1[2]);
                }
           //     Console.WriteLine("past");
                ArrayList newEdges = new ArrayList();
                newEdges.Add(edge);
                double[] pointCenter = new double[3];
                if(vertList.Count == 0)
                {
                    newEdges = new ArrayList();
                }
                   else if (vertList.Count == 1)
                {
           
                    Vertex v = (Vertex)vertList[0];
                    Vertex[] edge1 = new Vertex[] { edge[0], v };
                    newEdges.Add(edge1);
                    Vertex[] edge2 = new Vertex[] { edge[1], v };
                    newEdges.Add(edge2);
                    double[] pos = new double[3];
                    foreach (var ver in edge)
                    {
                        Vertex vert = (Vertex)ver;
                        pos = vert.getPos_3D();
                        for (int i = 0; i < 3; i++)
                        {
                            pointCenter[i] = pointCenter[i] + pos[i];
                        }
                    }
                    pos = v.getPos_3D();
                    for (int i = 0; i < 3; i++)
                    {
                        pointCenter[i] = (pointCenter[i] + pos[i])/3;
                    }
                    if (inFaces(edge) || inFaces(edge1) || inFaces(edge2))
                    {
                        newEdges = new ArrayList();
                    }
                }
                else
                {
                   
                    double minEdge1 = -1;
                    double minEdge2 = -1;
                    Vertex minVert1 = (Vertex)vertList[0];
                    Vertex minVert2 = (Vertex)vertList[0];

                    //More than  one treat as 2D case
                    
                    foreach(var v in edge)
                    {
                        Vertex vert = (Vertex)v;
                        double[] pos = vert.getPos_3D();
                        for(int i = 0; i < 3; i++)
                        {
                            pointCenter[i] = pointCenter[i] + pos[i];
                        }
                    }

                    for(int j = 0; j < vertList.Count; j++)
                    {
                        Vertex vert = (Vertex)vertList[j];
                      double[] pos = vert.getPos_3D();
                        for (int i = 0; i < 3; i++)
                        {
                            pointCenter[i] = pointCenter[i] + pos[i];
                        }
                    }

                    for(int i = 0; i < 3; i++)
                    {
                        pointCenter[i] = pointCenter[i] / (vertList.Count + 2);
                    }


                    double[][] plane = getPlane(edge, (Vertex)vertList[0]);
                    vertList.Add(edge[1]);
                    Vertex end = edge[1];
                    double[] vector = vectorDiff(vertexVector(edge[0]), vertexVector(edge[1]));
                    Vertex nextVert = edge[0];
                    foreach(var v in vertList)
                    {
                        Vertex tempV = (Vertex)v;
                        
                    }
                    double[] e1v1 = edge[0].getPos_3D();
                    double[] e1v2 = edge[1].getPos_3D();
                   Vertex[] startEdge = edge;
                    
                    // System.Threading.Thread.Sleep(5000);
                    while (vertList.Count > 0 && !nextVert.equals(end))
                    {
                        if (inFaces(edge))
                        {
                            newEdges = new ArrayList();
                            vertList = new ArrayList();
                        }
                        else
                        {
                         
                            double maxDotProduct = maxPossibleDotP;
                            
                            foreach (var v in vertList)
                            {
                                Vertex vert = (Vertex)v;
                
                                double dotProd = dotProduct(vector, vectorDiff(vertexVector(vert), vertexVector(edge[0])));
                           
                                if (dotProd > maxDotProduct)
                                {
                                    maxDotProduct = dotProd;
                                    nextVert = vert;
                                }
                            }
   
                            //edge[1]= edge[0];
                            //edge[0] = nextVert;
                            edge = new Vertex[] { nextVert, edge[0] };
 
                            vector = vectorDiff(vertexVector(edge[0]), vertexVector(edge[1]));

                            newEdges.Add(edge);
                            vertList.Remove(nextVert);
                        }
                    }
         
                    if(!nextVert.equals(end))
                    {
                        newEdges = new ArrayList();
                    }
                    e2v1 = startEdge[0].getPos_3D();
                    e2v2 = startEdge[1].getPos_3D();
                    //Console.WriteLine("NEW EDGES VERTEX 1 \t {0} \t {1} \t {2} \t VERTEX 2 \t {3} \t {4} \t {5}", e2v1[0], e2v1[1], e2v1[2], e2v2[0], e2v2[1], e2v2[2]);


                   // printDouble(pointCenter);
                    /*
                    foreach (var maxVert in vertList)
                    {


                        Vertex v = (Vertex)maxVert;
                        if (dotProduct(vertexVector(v), vertexVector(edge[0])) >= minEdge1)
                        {
                            minEdge1 = dotProduct(vertexVector(v), vertexVector(edge[0]));
                            minVert1 = v;
                        }
                        if (dotProduct(vertexVector(v), vertexVector(edge[1])) >= minEdge2)
                        {
                            minEdge2 = dotProduct(vertexVector(v), vertexVector(edge[1]));
                            minVert2 = v;
                        }
                        /*
                            Vertex[] edge1 = new Vertex[] { edge[0], v };
                        double[] e1v1 = edge1[0].getPos_3D();
                        double[] e1v2 = edge1[1].getPos_3D();
                        Console.WriteLine("CHECKING EDGE 1 VERTEX 1 \t {0} \t {1} \t {2} \t VERTEX 2 \t {3} \t {4} \t {5}", e1v1[0], e1v1[1], e1v1[2], e1v2[0], e1v2[1], e1v2[2]);

                        Vertex[] edge2 = new Vertex[] { edge[1], v };
                        double[] e2v1 = edge2[0].getPos_3D();
                        double[] e2v2 = edge2[1].getPos_3D();
                        Console.WriteLine("CHECKING EDGE 2 VERTEX 1 \t {0} \t {1} \t {2} \t VERTEX 2 \t {3} \t {4} \t {5}", e2v1[0], e2v1[1], e2v1[2], e2v2[0], e2v2[1], e2v2[2]);

                        double dotP1 = dotProduct(edgeVector(edge), vertexVector(v));
                        if (!checkEdge(edge1) && dotP1 > 0 && !checkEdge(new Vertex[] { edge[1], v}))
                        {
                            double[][] dir = getDirection(edge1);
                            edges.Add(edge1);
                            edgeQueue.Add(edge1);
                            dirQueue.Add(dir);
                            /*
                            else
                            {
                                vertQueue.Add(edge1);
                                directionList.Add(dir);
                            }
                            */
                    /*
                }

                if (!checkEdge(edge2) && dotP1 < 0 && !checkEdge(new Vertex[] { edge[0], v }))
                {
                    double[][] dir = getDirection(edge2);

                    edges.Add(edge2);
                    edgeQueue.Add(edge2);
                    dirQueue.Add(dir);
                    /*
                    else
                    {
                        vertQueue.Add(edge2);
                        directionList.Add(dir);
                    }
                    */
                    /*
                }
                */
                    /*
                    }
                    Vertex[] edge1 = new Vertex[] { edge[0], minVert1 };
                    if (minEdge1 != -1 && !checkEdge(edge1) )
                    {
                        
                        double[][] dir = getDirection(edge1);
                        edges.Add(edge1);
                        edgeQueue.Add(edge1);
                        dirQueue.Add(dir);
                    }
                    Vertex[] edge2 = new Vertex[] { edge[1], minVert2 };
                    if (minEdge2 != -1 && !checkEdge(edge2) )
                    {
                        
                        double[][] dir = getDirection(edge2);
                        edges.Add(edge2);
                        edgeQueue.Add(edge2);
                        dirQueue.Add(dir);
                    }
                    */
                }

                if (!checkFaces(newEdges) && newEdges.Count > 2)
                {
                    foreach (var e in newEdges)
                    {
                        
                        Vertex[] newEdge = (Vertex[])e;

                        if (!checkEdge(newEdge))
                        {
                            edges.Add(newEdge);
                            double[] dir = multScalar(1, getDirection2D(newEdge, pointCenter));
                            edgeQueue.Add(newEdge);
                            dirQueue.Add(dir);
                        }

                    }

                   
                    faces.Add(newEdges);
                }

                
                
            }
/*
                    edge = (Vertex[])vertQueue[0];
                    vertQueue.Remove(edge);
                    direction = (double[][])directionList[0];
                    directionList.Remove(direction);
                    if (!checkEdge(edge) && getDotP(edge) >= 0)
                    {
                        double[][] dir = getDirection(edge);
                        edges.Add(edge);
                        edgeQueue.Add(edge);
                        dirQueue.Add(dir);
                    }
                */
                

               
            
        }

        private Boolean inFaces(Vertex[] edge)
        {
            int count = 0;
            foreach(var f in faces)
            {
                ArrayList face = (ArrayList)f;
                foreach(var e in face)
                {
                    if(edgeEquals(edge, (Vertex[])e))
                    {
                        count++;
                    }
                }
            }
            if(count == 2)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        private double[] getDirection2D(Vertex[] edge, double[] faceCenter)
        {
            Vertex ver1 = edge[0];
            Vertex ver2 = edge[1];
 
            double[] v1Pos = vectorDiff(ver1.getPos_3D(), faceCenter);
            double[] v2Pos = vectorDiff(ver2.getPos_3D(), faceCenter);

     
            double v1Len = getLength(v1Pos);
            double v2Len = getLength(v2Pos);
            double[] newVec = multScalar(-1, vectorDiff(v1Pos, v2Pos));

            double dirLen = getLength(newVec);
   
            double dotP = (v1Pos[0] * v2Pos[0]) + (v1Pos[1] * v2Pos[1]) + (v1Pos[2] * v2Pos[2]);
    
            double semiPerimeter = (v1Len + v2Len + dirLen) / 2;

            double altitude = (2 / dirLen) * Math.Sqrt(semiPerimeter * (semiPerimeter - dirLen) * (semiPerimeter - v1Len) * (semiPerimeter - v2Len));
            double difference = Math.Sqrt(Math.Pow(v1Len, 2) - Math.Pow(altitude, 2));
            double[] endPoint = addVector(v1Pos, multScalar(difference / dirLen, newVec));

            double endLen = getLength(endPoint);
            return new double []{Math.Round(endPoint[0]/endLen,5), Math.Round(endPoint[1]/endLen, 5), Math.Round(endPoint[2]/endLen,5) };
        }

        private double[] getNormDirection(double[][] edge, double[] faceCenter)
        {
            
            double[] v1Pos = vectorDiff(edge[0], faceCenter);
            double[] v2Pos = vectorDiff(edge[1], faceCenter);


            double v1Len = getLength(v1Pos);
            double v2Len = getLength(v2Pos);
            double[] newVec = multScalar(-1, vectorDiff(v1Pos, v2Pos));

            double dirLen = getLength(newVec);

            double dotP = (v1Pos[0] * v2Pos[0]) + (v1Pos[1] * v2Pos[1]) + (v1Pos[2] * v2Pos[2]);

            double semiPerimeter = (v1Len + v2Len + dirLen) / 2;

            double altitude = (2 / dirLen) * Math.Sqrt(semiPerimeter * (semiPerimeter - dirLen) * (semiPerimeter - v1Len) * (semiPerimeter - v2Len));
  
            double difference = Math.Sqrt(Math.Pow(v1Len, 2) - Math.Pow(altitude, 2));
            double[] endPoint = addVector(v1Pos, multScalar(difference / dirLen, newVec));
       
            return new double[] { Math.Round(endPoint[0], 5), Math.Round(endPoint[1], 5), Math.Round(endPoint[2], 5) };
        }

        private double[] getNearestPoint(Vertex[] edge, double[] pos)
        {
            Vertex ver1 = edge[0];
            Vertex ver2 = edge[1];

            double[] v1Pos = vectorDiff(ver1.getPos_3D(), pos);
            double[] v2Pos = vectorDiff(ver2.getPos_3D(), pos);


            double v1Len = getLength(v1Pos);
            double v2Len = getLength(v2Pos);
            double[] newVec = vectorDiff(v1Pos, v2Pos);

            double dirLen = getLength(newVec);

            double dotP = (v1Pos[0] * v2Pos[0]) + (v1Pos[1] * v2Pos[1]) + (v1Pos[2] * v2Pos[2]);

            double semiPerimeter = (v1Len + v2Len + dirLen) / 2;

            double altitude = (2 / dirLen) * Math.Sqrt(semiPerimeter * (semiPerimeter - dirLen) * (semiPerimeter - v1Len) * (semiPerimeter - v2Len));
            double difference = Math.Max(0, Math.Min(getLength(newVec), Math.Sqrt(Math.Pow(v1Len, 2) - Math.Pow(altitude, 2))));
          
            double[] endPoint = addVector(v1Pos, multScalar(-difference / dirLen, newVec));
           
            return endPoint;
        }

        private double dotProductPlane(Vertex[] edge, Vertex vert, double[] direction, double scalar)
        {

            double[] pos1 = edgeVector(new Vertex[] { edge[0], edge[0] });

            double[] pos2 = edgeVector(new Vertex[] { edge[1], edge[0] });
            double[] pos3 = edgeVector(new Vertex[] { vert, edge[0] });

            double[] dir1 = edgeVector(edge);

            pos3 = vectorDiff(pos3,  multScalar(dotProduct(pos3, dir1)/getLength(dir1), normalize(dir1)));
            double[] crossP = multScalar(1, new double[] { (dir1[1] * direction[2]) - (dir1[2] * direction[1]), (dir1[2] * direction[0]) - (dir1[0] * direction[2]), (dir1[0] * direction[1]) - (dir1[1] * direction[0]) });
            
            if (scalar * dotProduct(pos3, crossP) < 0 && scalar > 0)
            {
                return -7500;
            }
            //return dotProduct(addVector(vertShift, pos3), direction);
            return Math.Round(dotProduct(pos3, direction)/getLength(pos3), 5);
            //shift onto plane

        }
        private void printD(double dub)
        {
            Console.WriteLine("PRINTDUB \t {0}", dub);
        }
        private void printDouble(double[] vertex)
        {
            Console.WriteLine("PRINTDOUBLE \t {0} \t {1} \t {2}", vertex[0], vertex[1], vertex[2]);
        }

        private double[][] getPlane(Vertex[] edge, Vertex vert)
        {
            double[] dir1 = normalize(edgeVector(edge));
            double[] pos = new double[3];
            double[] tempPos = vert.getPos_3D();
            for (int i = 0; i < 3; i++)
            {
                pos[i] = tempPos[i] - center[i];

            }
            double[] component = multScalar(dotProduct(dir1, pos), dir1);
            double[] dir2 = normalize(vectorDiff(pos, component));
            return new double[][] { dir1, dir2 };
        }

        private double[] multScalar(double scalar, double[] vector)
        {
            double[] result = new double[vector.Length];
            for(int i = 0; i < vector.Length; i++)
            {
                result[i] = vector[i] * scalar;
            }
            return result;
        }

        private double[] addScalar(double scalar, double[] vector)
        {
            double[] result = new double[vector.Length];
            for (int i = 0; i < vector.Length; i++)
            {
                result[i] = vector[i] + scalar;
            }
            return result;
        }

        private double[] subtractScalar(double scalar, double[] vector)
        {
            double[] result = new double[vector.Length];
            for (int i = 0; i < vector.Length; i++)
            {
                result[i] = vector[i] - scalar;
            }
            return result;
        }

        private double[] addVector(double[] v1, double[] v2)
        {
            double[] result = new double[v1.Length];
            for (int i = 0; i < v1.Length; i++)
            {
                result[i] = v1[i] + v2[i];
            }
            return result;
        }

        private double[] vectorDiff(double[] v1, double[] v2)
        {
            double[] result = new double[v1.Length];
            for (int i = 0; i < v1.Length; i++)
            {
                result[i] = v1[i] - v2[i];
            }
            return result;
        }

        private void findFaces(ArrayList possV, ArrayList possE)
        {
            //find shortest unique cycles from starting vertex using edges
            //choose a direction from starting vertex
            //imagine a plane going through that edge from center
            //find the cross product of the plane
            //find the directions that have largest and smallest dot product with normal vector
            //follow the directions on the new v1 and v2 with starting edge planes. the path must stay on the plane or there would be a shorter path


            if (possE.Count > 0)
            {
                Vertex[] startEdge = (Vertex[])possE[0];
                

                Vertex start = startEdge[0];
                double[] st = start.getPos_3D();
                Console.WriteLine("START \t {0} \t {1} \t {2}", st[0], st[1], st[2]);
                ArrayList usedEdges = new ArrayList();
                foreach (var e in possE)
                {
                    Vertex[] ed = (Vertex[])e;

                    if (inEdge(start, (Vertex[])e))
                    {
                        Vertex[] edge = (Vertex[])e;
                        double[] e1v1 = edge[0].getPos_3D();
                        double[] e1v2 = edge[1].getPos_3D();
                        Console.WriteLine("CHECKING VERTEX 1 \t {0} \t {1} \t {2} \t VERTEX 2 \t {3} \t {4} \t {5}", e1v1[0], e1v1[1], e1v1[2], e1v2[0], e1v2[1], e1v2[2]);
                        usedEdges.Add(e);
                    }
                }

                while (usedEdges.Count > 0)
                {

                    Vertex[] edge = (Vertex[])usedEdges[0];
                    ArrayList rightList = new ArrayList();
                    
                    if (!edge[0].equals(start))
                    {
                        Vertex temp = edge[0];
                        edge[0] = start;
                        edge[1] = temp;
                    }


                    double[][] direction = getDirection(edge);
                    double[] crossP = new double[] { (direction[0][1] * direction[1][2]) - (direction[0][2] * direction[1][1]), (direction[0][2] * direction[1][0]) - (direction[0][0] * direction[1][2]), (direction[0][0] * direction[1][1]) - (direction[0][1] * direction[1][0]) };

                    double minDotP = 1;
                    double maxDotP = 1;
                    Vertex[] leftPath = (Vertex[])possE[0];
                    Vertex[] rightPath = (Vertex[])possE[0];
                    possE.Remove(usedEdges[0]);
                    foreach (var tempEdge in possE)
                    {
                        Vertex[] tempE = (Vertex[])tempEdge;
                        if (inEdge(edge[1], (Vertex[])tempEdge))
                        {
                            if (!edge[1].equals(tempE[0]))
                            {
                                Vertex temp = ((Vertex[])tempEdge)[0];
                                ((Vertex[])tempEdge)[0] = edge[1];
                                ((Vertex[])tempEdge)[1] = temp;
                            }
                            double[] pos1 = new double[3];
                            double[] pos2 = new double[3];
                            double[] tempPos1 = ((Vertex[])tempEdge)[0].getPos_3D();
                            double[] tempPos2 = ((Vertex[])tempEdge)[1].getPos_3D();
                            for (int i = 0; i < 3; i++)
                            {
                                pos1[i] = tempPos1[i] - center[i];
                                pos2[i] = tempPos2[i] - center[i];
                            }
                            double[] pos = new double[] { pos1[0] - pos2[0], pos1[1] - pos2[1], pos1[2] - pos2[2] };
                            double dotP = dotProduct(edgeVector(edge), pos);
                            if (dotP <= minDotP && dotProduct(crossP, pos) >= 0)
                            {
                                minDotP = dotP;
                                leftPath = ((Vertex[])tempEdge);
                            }
                            if (dotP <= maxDotP && dotProduct(crossP, pos) <= 0)
                            {
                                maxDotP = dotP;
                                rightPath = ((Vertex[])tempEdge);
                            }
                        }
                    }
                    possE.Add(usedEdges[0]);
                    if (!edgeEquals(edge, rightPath))
                    {
                        rightList.Add(edge);
                        ArrayList leftList = new ArrayList();

                        
                        if (dotProduct(crossP, edgeVector(rightPath)) < 0 || maxDotP != minDotP)
                        {
                            //follow right & left path
                            double[] e1v1 = edge[0].getPos_3D();
                            double[] e1v2 = edge[1].getPos_3D();
                            e1v1 = rightPath[0].getPos_3D();
                            e1v2 = rightPath[1].getPos_3D();

                            double[] rightDirection = getNorm(edge, rightPath);
                            e1v1 = rightPath[0].getPos_3D();
                            e1v2 = rightPath[1].getPos_3D();
                            rightList.Add(rightPath);
                            Vertex[] newRight = rightPath;
                            int iter = 0;
                            while (!inEdge(start, rightPath))
                            {
                                double minAngle = 2 * Math.PI;
                                foreach (var tempEdge in possE)
                                {
                                    e1v1 = ((Vertex[])tempEdge)[0].getPos_3D();
                                    e1v2 = ((Vertex[])tempEdge)[1].getPos_3D();
                                   
                                    if (inEdge(rightPath[1], (Vertex[])tempEdge))
                                    {

                                        if (!edgeEquals(rightPath, (Vertex[])tempEdge))
                                        {

                                            if (!rightPath[1].equals(((Vertex[])tempEdge)[0]))
                                            {
                                                Vertex temp = ((Vertex[])tempEdge)[0];
                                                ((Vertex[])tempEdge)[0] = rightPath[1];
                                                ((Vertex[])tempEdge)[1] = temp;
                                            }


                                            if (dotProduct(edgeVector((Vertex[])tempEdge), rightDirection) == 0)
                                            {
                                                double angle = Math.Acos(dotProduct(edgeVector(rightPath), edgeVector((Vertex[])tempEdge)) / getLength(edgeVector(rightPath)));

                                                if (angle <= minAngle)
                                                {
                                                    minAngle = angle;
                                                    newRight = (Vertex[])tempEdge;

                                                }
                                            }
                                        }
                                    }
                                }


                                rightPath = newRight;
                                rightList.Add(rightPath);
                                Console.WriteLine("RIGHTLIST COUNT \t {0}", rightList.Count);
                                System.Threading.Thread.Sleep(100);
             
                            }

                            if (!checkFaces(rightList))
                            {

                                faces.Add(rightList);
                            }
                        }
                    }

                    usedEdges.Remove(usedEdges[0]);
                    System.Threading.Thread.Sleep(100);
                    Console.WriteLine("COUNT OF THING \t {0}", usedEdges.Count);
                }
                
                possV.Remove(start);
                ArrayList tempEdges = new ArrayList();
                foreach (var e in possE)
                {
                    tempEdges.Add(e);
                }
                foreach (var e in tempEdges)
                {
                    if (inEdge(start, (Vertex[])e))
                    {
                        possE.Remove(e);
                    }
                }
                Console.WriteLine("OUT \t {0}", usedEdges.Count);
                findFaces(possV, possE);
            }
            

        }

        private Boolean checkFaces(ArrayList testFace)
        {
            foreach (var f in faces)
            {
                ArrayList face = (ArrayList)f;
                if (face.Count == testFace.Count)
                {
                    int count = 0;
                    for (int i = 0; i < face.Count; i++)
                    {
                        for(int j = 0; j < face.Count; j++)
                        {
                            if(edgeEquals((Vertex[])face[j], (Vertex[])testFace[i]))
                            {
                                count++;
                            }
                        }
                    }
                    if(count == face.Count)
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        private double[] getNorm(Vertex[] edge1, Vertex[] edge2)
        {
            double[] direction1 = edgeVector(edge1);
            double[] direction2 = edgeVector(edge2);
            double dotP = dotProduct(direction1, direction2);
            double[] unitDir1 = normalize(direction1);
            double[] unitDir2 = new double[3];
            for (int i = 0; i < 3; i++)
            {
                unitDir2[i] = direction2[i] - unitDir1[i] * dotP;
            }
            double[][] direction = new double[][] { unitDir1, normalize(unitDir2) };
            return normalize(new double[] { (direction[0][1] * direction[1][2]) - (direction[0][2] * direction[1][1]), (direction[0][2] * direction[1][0]) - (direction[0][0] * direction[1][2]), (direction[0][0] * direction[1][1]) - (direction[0][1] * direction[1][0]) });
        }

        private double[] getCrossP(double[] direction1, double[] direction2)
        {
  
            double dotP = dotProduct(direction1, direction2);
            double[] unitDir1 = normalize(direction1);
            double[] unitDir2 = new double[3];
            for (int i = 0; i < 3; i++)
            {
                unitDir2[i] = direction2[i] - unitDir1[i] * dotP;
            }
            double[][] direction = new double[][] { unitDir1, normalize(unitDir2) };
            return normalize(new double[] { (direction[0][1] * direction[1][2]) - (direction[0][2] * direction[1][1]), (direction[0][2] * direction[1][0]) - (direction[0][0] * direction[1][2]), (direction[0][0] * direction[1][1]) - (direction[0][1] * direction[1][0]) });
        }

        private double[] edgeVector(Vertex[] edge)
        {
            double[] pos1 = new double[3];
            double[] pos2 = new double[3];
            double[] tempPos1 = edge[0].getPos_3D();
            double[] tempPos2 = edge[1].getPos_3D();
            for (int i = 0; i < 3; i++)
            {
                pos1[i] = tempPos1[i] - center[i];
                pos2[i] = tempPos2[i] - center[i];
            }
            return new double[] { pos1[0] - pos2[0], pos1[1] - pos2[1], pos1[2] - pos2[2] };
        }


        private double dotProduct(double[] dP1, double[] dP2)
        {
            double[] p1 = dP1;
            double[] p2 = dP2;
            return Math.Round((p1[0] * p2[0]) + (p1[1] * p2[1]) + (p1[2] * p2[2]),15);
        }

        private double getLength(double[] n)
        {
            double result = 0;
            foreach (var elem in n)
            {
                result += Math.Pow(elem, 2);
            }
            return Math.Sqrt(result);
        }

        private double[] normalize(double[] n)
        {
            double result = 0;
            double[] endResult = new double[n.Length];
            
            foreach(var elem in n)
            {
                result += Math.Pow(elem, 2);
            }
            double len = Math.Sqrt(result);
            for(int i = 0; i < n.Length; i++)
            {
                endResult[i] = n[i] / len;
            }
            return endResult;
        }

        private Boolean edgeEquals(Vertex[] edge1, Vertex[] edge2)
        {
            if((edge1[0].equals(edge2[0]) && edge1[1].equals(edge2[1])) || (edge1[1].equals(edge2[0]) && edge1[0].equals(edge2[1])))
            {
                return true;
            }
            return false;
        }

        private double getDotP(Vertex[] edge)
        {
            double[] edgePos1 = new double[3];
            double[] tempPos1 = edge[0].getPos_3D();
            double[] edgePos2 = new double[3];
            double[] tempPos2 = edge[1].getPos_3D();
            for (int i = 0; i < 3; i++)
            {
                edgePos1[i] = tempPos1[i] - center[i];
                edgePos2[i] = tempPos2[i] - center[i];
            }
            return (edgePos1[0] * edgePos2[0]) + (edgePos1[1] * edgePos2[1]) + (edgePos1[2] * edgePos2[2]);
        }

        private Boolean inEdge(Vertex v, Vertex[] edge)
        {
            if (v.equals(edge[0]) || v.equals(edge[1]))
            {
                return true;
            }
            return false;
        }

        private Boolean inEdges(Vertex v)
        {
            foreach (var e in edges)
            {
                Vertex[] edge = (Vertex[])e;
                if (v.equals(edge[0]) || v.equals(edge[1]))
                {
                    return true;
                }
            }
            return false;
        }

        private double[][] getDirection(Vertex[] edge)
        {
            Vertex ver1 = edge[0];
            Vertex ver2 = edge[1];


            double[] v1Pos = new double[3];
            double[] tempPos1 = ver1.getPos_3D();
            double[] v2Pos = new double[3];
            double[] tempPos2 = ver2.getPos_3D();
            for (int i = 0; i < 3; i++)
            {
                v1Pos[i] = tempPos1[i] - center[i];
                v2Pos[i] = tempPos2[i] - center[i];
            }
            double v1Len = Math.Sqrt(Math.Pow(v1Pos[0], 2) + Math.Pow(v1Pos[1], 2) + Math.Pow(v1Pos[2], 2));
            double v2Len = Math.Sqrt(Math.Pow(v2Pos[0], 2) + Math.Pow(v2Pos[1], 2) + Math.Pow(v2Pos[2], 2));

            double[] newVec = new double[] { v1Pos[0] - v2Pos[0], v1Pos[1] - v2Pos[1], v1Pos[2] - v2Pos[2] };

            double dirLen = Math.Sqrt(Math.Pow(newVec[0], 2) + Math.Pow(newVec[1], 2) + Math.Pow(newVec[2], 2));
            double dotP = (v1Pos[0] * v2Pos[0]) + (v1Pos[1] * v2Pos[1]) + (v1Pos[2] * v2Pos[2]);

            double theta1 = Math.Acos(dotP / (v1Len * v2Len));

            double theta2 = Math.Asin(v1Len * Math.Sin(theta1) / dirLen);

            double result = Math.Cos(theta2) * v1Len;

            double[] endPoint = new double[] { Math.Round(v1Pos[0] + result * newVec[0] / dirLen, 5), Math.Round(v1Pos[1] + result * newVec[1] / dirLen,5), Math.Round(v1Pos[2] + result * newVec[2] / dirLen,5) };
            double endLen = Math.Sqrt(Math.Pow(endPoint[0], 2) + Math.Pow(endPoint[1], 2) + Math.Pow(endPoint[2], 2));
            double[][] direction = new double[][] { new double[] { newVec[0] / dirLen, newVec[1] / dirLen, newVec[2] / dirLen }, new double[] { endPoint[0] / endLen, endPoint[1] / endLen, endPoint[2] / endLen } };

            return direction;
        }

        public ArrayList getVert()
        {
            return verts;
        }
        public ArrayList getEdges()
        {
            return edges;
        }
        public ArrayList getFaces()
        {
            return faces;
        }
        public void printList(ArrayList face)
        {
            int count = 1;
            Console.WriteLine("FACE");
            foreach(var f in face)
            {
                Vertex[] edge = (Vertex[])f;
                double[] pos1 = edge[0].getPos_3D();
                double[] pos2 = edge[1].getPos_3D();
                Console.WriteLine("VERTEX {0}: \t {1} \t {2} \t {3} \t VERTEX {4}: \t {5} \t {6} \t {7}", count, pos1[0], pos1[1], pos1[2], count+1, pos2[0], pos2[1], pos2[2]);
                count++;
            }
        }

        public void printVList(ArrayList vList)
        {
            int count = 1;
            Console.WriteLine("vList");
            foreach (var v in vList)
            {
                Vertex vert = (Vertex)v;
                double[] pos = vert.getPos_3D();
                Console.WriteLine("VERTEX {0}: \t {1} \t {2} \t {3}", count, pos[0], pos[1], pos[2]);
                count++;
            }
        }

        public void printVArray(Vertex[] vArray)
        {
            foreach(var v in vArray)
            {
                printDouble(v.getPos_3D());
            }
        }

        public void printVert(ArrayList vert)
        {
            Console.WriteLine("VERTICES");
            foreach (var v in vert)
            {
                Vertex vt = (Vertex)v;
                double[] pos1 = vt.getPos_3D();
                Console.WriteLine("VERTEX: \t {1} \t {2} \t {3} ", pos1[0], pos1[1], pos1[2]);
            }
            Console.WriteLine("VERTICES?");
        }

        public double[] getCenter()
        {
            return center;
        }

        public void cursorRotate(double[] oldPos, double[] newPos)
        {
            double[] theta = new double[] { Math.Acos(Math.Abs(oldPos[0] - center[0])/Math.Abs(center[0] - newPos[0])) , Math.Acos( Math.Abs(oldPos[1] - center[1])/ Math.Abs(center[1] - newPos[1]) ), 0};
            Console.WriteLine("ROTATE OBJECT");
            printDouble(theta);
            System.Threading.Thread.Sleep(500);
            rotate(theta);
        }
        

        public void rotate(double[] theta)
        {
            double[][] rXMatrix = new double[][] { new double[] { 1, 0, 0 }, new double[] { 0, Math.Cos(theta[0]), -Math.Sin(theta[0]) }, new double[] { 0, Math.Sin(theta[0]), Math.Cos(theta[0]) } };
            double[][] rYMatrix = new double[][] {  new double[] { Math.Cos(theta[1]),0, Math.Sin(theta[1]) }, new double[] { 0, 1, 0 }, new double[] { -Math.Sin(theta[1]), 0, Math.Cos(theta[1]) } };
            double[][] rZMatrix = new double[][] { new double[] { Math.Cos(theta[2]), -Math.Sin(theta[2]),0 }, new double[] {  Math.Sin(theta[2]), Math.Cos(theta[2]),0 }, new double[] { 0, 0, 1 }};

            foreach(var v in verts)
            {
                Vertex vert = (Vertex)v;
                double[] pos = vectorDiff(vert.getPos_3D(), center);
                vert.setPos3D(addVector(multMatrix(multMatrix(multMatrix(pos, rZMatrix),  rYMatrix), rXMatrix), center), cameraPos, cameraRot);
            }
        }
        private double[] multMatrix(double[] vect, double[][] mat)
        {
            double[] result = new double[mat[0].Length];
            for(int i = 0; i < vect.Length; i++)
            {
                result[i] = vect[0] * mat[i][0] + vect[1] * mat[i][1] + vect[2] * mat[i][2];
            }
            return result;
        }


        private Vertex[] edgeToVertex(ArrayList edgeList)
        {

            return null;
        }

        public ArrayList detectVisibleFaces(double[] pos, Form1 form1, ArrayList otherFaces)
        {
            form = form1;
            ArrayList visibleFacesList = new ArrayList();
            for (int i = 0; i < faces.Count; i++)
            {
                Boolean visible = true;
                Vertex[] coneFace = edgesToList((ArrayList)faces[i]);
                ArrayList coneVertices = new ArrayList();
                foreach (var v in coneFace)
                {
                    coneVertices.Add(v.getPos_3D());
                }
                for (int j = 0; j < faces.Count; j++)
                {
                    if (j != i && visible)
                    {
                        Vertex[] checkFace = edgesToList((ArrayList)faces[j]);
                        ArrayList checkVertices = new ArrayList();
                        foreach (var v in checkFace)
                        {
                            checkVertices.Add(v.getPos_3D());
                        }
                        if (checkFaceConeIntersection(coneVertices, checkVertices, pos))
                        {
                            visible = false;
                        }
                    }
                }
                for (int k = 0; k < otherFaces.Count; k++)
                {
                    ArrayList objFace = (ArrayList)otherFaces[k];
                    for (int j = 0; j < objFace.Count; j++)
                    {
                        if (visible)
                        {
                            Vertex[] checkFace = edgesToList((ArrayList)objFace[j]);
                            ArrayList checkVertices = new ArrayList();
                            foreach (var v in checkFace)
                            {
                                checkVertices.Add(v.getPos_3D());
                            }
                            if (checkFaceConeIntersection(coneVertices, checkVertices, pos))
                            {
                                visible = false;
                            }
                        }
                    }
                }

                if (visible)
                {
                    visibleFacesList.Add(faces[i]);
                }
            }
            //loop through all faces
            
            //loop through all faces

            return visibleFacesList;
        }

        private ArrayList copyList(ArrayList myList)
        {
            ArrayList copiedList = new ArrayList();
            foreach(var v in myList)
            {
                copiedList.Add(v);
            }
            return copiedList;
        }

        private double[][] twoPlaneIntersection(double[] plane1, double[] plane2)
        {
            double[][] line = new double[3][];
            //plane1[0][0] (x - plane1[1][0]) + plane1[0][1] (y - plane1[1][1]) + plane1[0][2] (z - plane1[1][2]) = 0
            //plane2[0][0] (x - plane2[1][0]) + plane2[0][1] (y - plane2[1][1]) + plane2[0][2] (z - plane2[1][2]) = 0

            //x = (plane1[0][1] (plane1[1][1] - y) + plane1[0][2] (plane1[1][2] - z)) / plane1[0][0] + plane1[1][0]
            //x = (plane2[0][1] (plane2[1][1] - y) + plane2[0][2] (plane2[1][2] - z)) / plane2[0][0] + plane2[1][0]

            //z[0] = 1
            //z[1] = 0

            //y[0] = (t_1_z / t_1_x  - t_2_z / t_2_x) / (t_2_y / t_2_x - t_1_y / t_1_x)
            //y[1] = (d_1 - d_2) / (t_2_y / t_2_x - t_1_y / t_1_x)

            //x[1] = - t_1_y / t_1_x * y[1] - d_1
            //x[0] = - t_1_y / t_1_x * y[0] - t_1_z / t_1_x

            line[1] = new double[] {(plane1[2] / plane1[0] - plane2[2] /plane2[0]) / (plane2[1] / plane2[0] - plane1[1] / plane1[0]) , (plane1[3] - plane2[3]) / (plane2[1] / plane2[0] - plane1[1] / plane1[0]) };
            line[0] = new double[] { -plane1[1] / plane1[0] * line[1][0] - plane1[2] / plane1[0] , - plane1[1] / plane1[0] * line[1][1] - plane1[3]};

            line[2] = new double[] { 1, 0 };
            Console.WriteLine("test4");
            //(plane1[0][1] (plane1[1][1] - y) + plane1[0][2] (plane1[1][2] - z)) / plane1[0][0] + plane1[1][0] = (plane2[0][1] (plane2[1][1] - y) + plane2[0][2] (plane2[1][2] - z)) / plane2[0][0] + plane2[1][0]
            return line;
        }


        //convert face to plane first
        private double[][] edgeToEdgeOnPlane(double[][] edge, double[] cameraPos, double[][] facePlane)
        {
            double[][] retEdge = new double[2][];
            //get plane edgePlane from edge and camera position
            double[] crossP = getCrossP(vectorDiff(edge[0], edge[1]), vectorDiff(edge[0], cameraPos));
            double[] edgePlane = parametricPlaneToNormal(new double[][] { crossP, edge[0] });
            double[] facePlaneNormal = parametricPlaneToNormal(facePlane);
            //convert edgePlane to normal form
            //convert facePlane to normal form
            double[][] intLine = twoPlaneIntersection(edgePlane, facePlaneNormal);

            double[] firstVert = vectorDiff(edge[0], cameraPos);
            double[] secondVert = vectorDiff(edge[1], cameraPos);
            double[][] firstVertLine = new double[][] { new double[] { firstVert[0], cameraPos[0] }, new double[] { firstVert[1], cameraPos[1] }, new double[] { firstVert[2], cameraPos[2] } };
            double[][] secondVertLine = new double[][] { new double[] { secondVert[0], cameraPos[0] }, new double[] { secondVert[1], cameraPos[1] }, new double[] { secondVert[2], cameraPos[2] } };
            //get plane intersection line intLine from planeToPlaneIntersection
            double[] firstIntersection = lineToLineIntersection(intLine, firstVertLine);
            double[] secondIntersection = lineToLineIntersection(intLine, secondVertLine);

            //get line point intersection of intLine and line from cameraPos to each vertex of edge
            //return those
            retEdge[0] = firstIntersection;
            retEdge[1] = secondIntersection;
            return retEdge;
        }

        private double[] lineToLineIntersection(double[][] line1, double[][] line2)
        {
            double[] intersection = new double[3];
            //line 1
            //x = t_1 * a1_1 + x1_0
            //y = t_1 * a1_2 + y1_0
            //z = t_1 * a1_3 + z1_0

            //line 2
            //x = t_2 * a2_1 + x2_0
            //y = t_2 * a2_2 + y2_0
            //z = t_2 * a2_3 + z2_0

            //t_2 = (y2_0 - y1_0 - (x2_0 - x1_0) * a1_2 / a1_1) / (a2_1 * a1_2 / a1_1 - a2_2)

            //t_1 = (t_2 * a2_1 + x2_0 - x1_0) / a1_1
            //(x2_0 - x1_0) * a1_2 / a1_1  + y1_0 - y2_0 = t_2 (a2_2 -a2_1 * a1_2 / a1_1) 
            //t_2 = ((x2_0 - x1_0) * a1_2 / a1_1 + y1_0 - y2_0) / (a2_2 -a2_1 * a1_2 / a1_1)
            double t2_value = ((line2[0][1] - line1[0][1]) * line1[1][0] / line1[0][0] + line1[1][1] - line2[1][1]) / (line2[1][0] - (line2[0][0] * line1[1][0] / line1[0][0]));
            double t1_value = (t2_value * line2[0][0] + line2[0][1] - line1[0][1]) / line1[0][0];

            double x_1 = t1_value * line1[0][0] + line1[0][1];
            double x_2 = t2_value * line2[0][0] + line2[0][1];

            double y_1 = t1_value * line1[1][0] + line1[1][1];
            double y_2 = t2_value * line2[1][0] + line2[1][1];

            double z_1 = t1_value * line1[2][0] + line1[2][1];
            double z_2 = t2_value * line2[2][0] + line2[2][1];

            if(x_1 != x_2)
            {
                Console.WriteLine("X VALUES ARE INCONSISTENT \t {0} \t {1}", x_1, x_2);
            }
            else if(y_1 != y_2)
            {
                Console.WriteLine("Y VALUES ARE INCONSISTENT \t {0} \t {1}", y_1, y_2);
            }
            else if(z_1 != z_2)
            {
                Console.WriteLine("Z VALUES ARE INCONSISTENT \t {0} \t {1}", z_1, z_2);
            }
            else
            {
                intersection[0] = x_1;
                intersection[1] = y_1;
                intersection[2] = z_1;
                //Console.WriteLine("INTERSECTION \t {0} \t {1} \t {2}", intersection[0], intersection[1], intersection[2]);
            }

            return intersection;
        }

        private double[] parametricPlaneToNormal(double[][] plane)
        {
            double[] retPlane = new double[4];
            double constant = -plane[0][0] * plane[1][0] - plane[0][1] * plane[1][1] - plane[0][2] * plane[1][2];
            retPlane[0] = plane[0][0];
            retPlane[1] = plane[0][1];
            retPlane[2] = plane[0][2];
            retPlane[3] = constant;
            return retPlane;
        }

        private Boolean checkFaceConeIntersection(ArrayList coneVertices, ArrayList checkVertices, double[] pos)
        {
            double maxConeDist = 0;
            for(int i = 0; i < coneVertices.Count; i++)
            {
                if(getDistance(pos, (double[])coneVertices[i]) > maxConeDist)
                {
                    maxConeDist = getDistance(pos, (double[])coneVertices[i]);
                }
            }
            double minCheckDist = -maxPossibleDotP;
            for (int i = 0; i < checkVertices.Count; i++)
            {
                if (getDistance(pos, (double[])checkVertices[i]) < minCheckDist)
                {
                    minCheckDist = getDistance(pos, (double[])checkVertices[i]);
                }
            }
          
            if (minCheckDist < maxConeDist)
            {
                //check whether face goes thru cone
                double[][] checkPlane = faceToPlane(checkVertices);

                ArrayList backupCheckVertices = copyList(checkVertices);
                ArrayList intersectionVertices = getIntersection(checkPlane, coneVertices, pos);
                ArrayList backupIntersectionVertices = copyList(intersectionVertices);
                //form.clear();

       
                ArrayList intersectionEdges = getFaceEdges(intersectionVertices);

                ArrayList faceEdges = getFaceEdges(checkVertices);
                if (intersectionEdges == null)
                {
                    return false;
                }
                else if (intersectionEdges.Count == 1)
                {
                    if (((double[][])intersectionEdges[0])[1] == null)
                    {
                        if (checkPolygonPointIntersection(faceEdges, intersectionEdges, backupCheckVertices, backupIntersectionVertices, checkPlane[0]))
                        {
                            return true;
                        }
                        else
                        {
                            return false;
                        }
                    }
                    else if (checkPolygonIntersection(intersectionEdges, faceEdges, backupIntersectionVertices, backupCheckVertices, checkPlane[0]))
                    {
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
                else
                {
                    if (checkPolygonIntersection(intersectionEdges, faceEdges, backupIntersectionVertices, backupCheckVertices, checkPlane[0]))
                    {
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
            else
            {
                return false;
            }
        }
        private Boolean checkPolygonPointIntersection(ArrayList firstPolygon, ArrayList secondPolygon, ArrayList firstVertices, ArrayList secondVertices, double[] planeNorm)
        {
            //iterate thru list of edges
            for (int i = 0; i < firstPolygon.Count; i++)
            {
                double[][] firstEdge = (double[][])firstPolygon[i];
                //find line perpendicular to this and plane

                double[] edgeDir = vectorDiff(firstEdge[0], firstEdge[1]);
                double[] perp = getCrossP(planeNorm, edgeDir);
                double[] midpoint = getMidpoint(firstEdge[0], firstEdge[1]);
                double[] firstVertCheck = new double[3];
                for (int j = 0; j < firstVertices.Count; j++)
                {
                    double[] vert = vectorDiff((double[])firstVertices[j], midpoint);
                    double[] checkVert = multScalar(dotProduct(vert, perp) / Math.Pow(getLength(perp), 2), perp);
                    if (getLength(checkVert) != 0)
                    {
                        firstVertCheck = checkVert;
                    }
                }
                ArrayList secondVerticesProj = new ArrayList();
                Boolean intersection = false;
                for (int j = 0; j < secondVertices.Count; j++)
                {
                    double[] vert = vectorDiff((double[])secondVertices[j], midpoint);
                    double[] checkVert = multScalar(dotProduct(vert, perp) / Math.Pow(getLength(perp), 2), perp);
                    if (dotProduct(checkVert, firstVertCheck) > 0)
                    {

                        // form.drawEdges(tempList, System.Drawing.Color.Pink);
                        //  System.Threading.Thread.Sleep(100);
                        intersection = true;
                    }

                }
                if (!intersection)
                {
                    return false;
                }
            }
            return true;
        }


        private Boolean checkPolygonIntersection(ArrayList firstPolygon, ArrayList secondPolygon, ArrayList firstVertices, ArrayList secondVertices, double[] planeNorm)
        {
            //iterate thru list of edges
            for(int i = 0; i < firstPolygon.Count; i++)
            {
                double[][] firstEdge = (double[][])firstPolygon[i];
                //find line perpendicular to this and plane

                double[] edgeDir = vectorDiff(firstEdge[0], firstEdge[1]);
                double[] perp = getCrossP(planeNorm, edgeDir);
                double[] midpoint = getMidpoint(firstEdge[0], firstEdge[1]);
                double[] firstVertCheck = new double[3];
               for(int j = 0; j < firstVertices.Count; j++)
                {
                    double[] vert = vectorDiff((double[])firstVertices[j], midpoint);
                    double[] checkVert = multScalar(dotProduct(vert, perp) / Math.Pow(getLength(perp), 2), perp);
                    if(getLength(checkVert) != 0)
                    {
                        firstVertCheck = checkVert;
                    }
                }
                ArrayList secondVerticesProj = new ArrayList();
                Boolean intersection = false;
                for (int j = 0; j < secondVertices.Count; j++)
                {
                    double[] vert = vectorDiff((double[])secondVertices[j], midpoint);
                    double[] checkVert = multScalar(dotProduct(vert, perp) / Math.Pow(getLength(perp), 2), perp);
                    if (dotProduct(checkVert, firstVertCheck) > 0)
                    {
                       
                        // form.drawEdges(tempList, System.Drawing.Color.Pink);
                        //  System.Threading.Thread.Sleep(100);
                        intersection = true;
                    }

                }
                if(!intersection)
                {
                    return false;
                }
            
           
            }
            return true;
        }
        
        private double[] getMidpoint(double[] point1, double[] point2)
        {
            return divScalar(2, addVector(point1, point2));
        }

        private double[] divScalar(double scalar, double[] vec)
        {
            vec[0] = vec[0] / scalar;
            vec[1] = vec[1] / scalar;
            vec[2] = vec[2] / scalar;
            return vec;
        }
     
        private ArrayList getIntersection(double[][] plane, ArrayList vertices, double[] pos)
        {
            ArrayList intersectionVertices = new ArrayList();
            for(int i = 0; i < vertices.Count; i++)
            {
                double[] vertPos = (double[])vertices[i];
                double[][] line = new double[][] { vectorDiff(vertPos, pos), pos };
                double[] intersection = planeLineIntersection(plane, line);

                if (intersection != null)
                {
                   // System.Threading.Thread.Sleep(2000);
                    intersectionVertices.Add(intersection);
                }
            }
            return intersectionVertices;
        }

        private ArrayList getFaceEdges(ArrayList vertices)
        {
            ArrayList faceEdges = new ArrayList();
            double[] center = new double[3];
            for (int i = 0; i < vertices.Count; i++)
            {
                center[0] += ((double[])vertices[i])[0];
                center[1] += ((double[])vertices[i])[1];
                center[2] += ((double[])vertices[i])[2];
            }
            center[0] = center[0] / vertices.Count;
            center[1] = center[1] / vertices.Count;
            center[2] = center[2] / vertices.Count;
            double maxDist = 0;
            int index = -1;
            for (int i = 0; i < vertices.Count; i++)
            {
                double[] vert = (double[])vertices[i];
                if(getDistance(vert, center) > maxDist)
                {
                    index = i;
                    maxDist = getDistance(vert, center);
                }
            }
            if(index == -1)
            {
                Console.WriteLine("INDEX IS -1, EVERY POINT OF FACE IS THE SAME");
                foreach(var v in vertices)
                {
                    printDouble((double[])v);
                }
                if (vertices.Count > 0)
                {
                    faceEdges.Add(new double[][] { center, null });
                    return faceEdges;
                }
                else
                {
                    return null;
                }
                //System.Windows.Forms.Application.Exit();
            }
            else
            {
                double[] firstVert = (double[])vertices[index];
                vertices.Remove(vertices[index]);
                if (vertices.Count == 0)
                {
                    if (vertices.Count > 0)
                    {
                        faceEdges.Add(new double[][] { center, null });
                        return faceEdges;
                    }
                    else
                    {
                        return null;
                    }
                }
                else
                {
                    //need to find thru diff means, use dot product
                    double[] secondVert = new double[3];
                    double maxDotP = maxPossibleDotP;
                    index = -1;
                    for (int i = 0; i < vertices.Count; i++)
                    {
                        double[] vert = (double[])vertices[i];
                        if (dotProduct(vectorDiff(firstVert, center), vectorDiff(vert, center)) > maxDotP)
                        {
                            index = i;
                            maxDotP = dotProduct(vectorDiff(firstVert, center), vectorDiff(vert, center));
                        }
                    }
                    if (index == -1)
                    {
                        Console.WriteLine("INDEX IS -1, MAXIMUM DOT P IS LESS THAN MINIMUM SHOULD BE");
                        return null;
                    }
                    else
                    {
                        secondVert = (double[])vertices[index];
                        faceEdges.Add(new double[][] { firstVert, secondVert });
                        vertices.Remove(vertices[index]);
                        double[] edgeDir = vectorDiff(secondVert, firstVert);
                        double[] lineTail = secondVert;
                        while (vertices.Count > 0)
                        {
                            maxDotP = maxPossibleDotP;
                            index = -1;
                            for (int i = 0; i < vertices.Count; i++)
                            {
                                double[] vert = vectorDiff((double[])vertices[i], lineTail);
                                double dotP = dotProduct(edgeDir, vert)/getLength(vert);
                                if (dotP > maxDotP)
                                {
                                    maxDotP = dotP;
                                    index = i;
                                }
                            }
                            if (index == -1)
                            {
                                Console.WriteLine("INDEX IS -1, MAXIMUM DOT P IS LESS THAN MINIMUM SHOULD BE FOR CONSTRUCTING FACE");
                                return null;
                            }
                            else
                            {
                                double[] nextVert = (double[])vertices[index];
                                faceEdges.Add(new double[][] { lineTail, nextVert });
                                edgeDir = vectorDiff(nextVert, lineTail);
                                vertices.Remove(vertices[index]);
                                lineTail = nextVert;
                            }
                            
                        }
                        faceEdges.Add(new double[][] { lineTail, firstVert });
                    }
                }
            }
            return faceEdges;
        }

        private double[] planeLineIntersection(double[][] plane, double[][] line)
        {
            double[] intersectionPoint = new double[3];
            // plane[0][0] * (x - plane[1][0]) + plane[0][1] * (y - plane[1][1]) + plane[0][2] * (z - plane[1][2]) = 0
            // x = line[0][0] * t + line[1][0]
            // y = line[0][1] * t + line[1][1]
            // z = line[0][2] * t + line[1][2]
            // plane[0][0] * (line[0][0] * t + line[1][0] - plane[1][0]) + plane[0][1] * (line[0][1] * t + line[1][1] - plane[1][1]) + plane[0][2] * (line[0][2] * t + line[1][2] - plane[1][2]) = 0
            // t (plane[0][0] * line[0][0] + plane[0][1] * line[0][1] + plane[0][2] * line[0][2]) = plane[0][0] * (plane[1][0] - line[1][0]) + plane[0][1] * (plane[1][1] - line[1][1]) + plane[0][2] * (plane[1][2] - line[1][2])
            double temp = (plane[0][0] * line[0][0] + plane[0][1] * line[0][1] + plane[0][2] * line[0][2]);
            if (temp == 0)
            {
                if (plane[0][0] * (line[1][0] - plane[1][0]) + plane[0][1] * (line[1][1] - plane[1][1]) + plane[0][2] * (line[1][2] - plane[1][2]) == 0)
                {
                    return line[1];
                }
                else
                {
                    return null;
                }
            }
            else
            {
                double t = (plane[0][0] * (plane[1][0] - line[1][0]) + plane[0][1] * (plane[1][1] - line[1][1]) + plane[0][2] * (plane[1][2] - line[1][2])) / (plane[0][0] * line[0][0] + plane[0][1] * line[0][1] + plane[0][2] * line[0][2]);
                intersectionPoint[0] = line[0][0] * t + line[1][0];
                intersectionPoint[1] = line[0][1] * t + line[1][1];
                intersectionPoint[2] = line[0][2] * t + line[1][2];

                if (getLength(vectorDiff(intersectionPoint, line[1])) > (getLength(line[0])))
                {
                    return null;
                }
                else if (dotProduct(line[0], vectorDiff(intersectionPoint, line[1])) < 0)
                {
                    return null;
                }
                else
                {
                    Console.WriteLine("plane norm \t {0} \t {1} \t {2}", plane[0][0], plane[0][1], plane[0][2]);
                    Console.WriteLine("intersection point \t {0} \t {1} \t {2}", intersectionPoint[0], intersectionPoint[1], intersectionPoint[2]);
                    Console.WriteLine("line point \t {0} \t {1} \t {2}", addVector(line[0], line[1])[0], addVector(line[0], line[1])[1], addVector(line[0], line[1])[2]);
                    return intersectionPoint;
                }
            }
        }

        public ArrayList findVisibleFaces(double[] pos)
        {
            ArrayList visibleFacesList = new ArrayList();
            for(int i = 0; i < faces.Count; i++)
            {
                Boolean visible = true;
                Vertex[] coneFace = edgesToList((ArrayList)faces[i]);
                ArrayList vertList = new ArrayList();
                foreach (var v in coneFace)
                {
                    if (!vertList.Contains(v.getPos_3D()))
                    {
                        vertList.Add(v.getPos_3D());
                    }
                }
                for(int j = 0; j < faces.Count; j++)
                {
                    if(i != j)
                    {
                        Vertex[] checkFace = edgesToList((ArrayList)faces[j]);
                        if(checkIntersection(vertList, (ArrayList)faces[j], pos))
                        {
                            printD(101010101);
                            printD(i);
                            printD(j);
                            visible = false;
                        }
                    }
                }
                if(visible)
                {
                    visibleFacesList.Add((ArrayList)faces[i]);
                }
            }
            return visibleFacesList;
        }

        public ArrayList visibleFaces(double[]pos)
        {
            ArrayList retFaces = new ArrayList();
            foreach(var f in faces)
            {
                Console.WriteLine("FACE CHECKING");
                Boolean visible = true;
                ArrayList face = (ArrayList)f;
                double[] faceCenter = new double[3];
                Vertex[] faceBound = edgesToList(face);
                ArrayList vList = new ArrayList();
                
                foreach (var v in faceBound)
                {
                    vList.Add((Vertex)v);
                    faceCenter = addVector(faceCenter, ((Vertex)v).getPos_3D());
            
                }
                faceCenter = multScalar(1/((double)faceBound.Length), faceCenter);

                ArrayList bounds = findBounds(pos, face, faceCenter);
                

                if (checkEdgeBounds(pos, bounds))
                {

                    visible = false;
                    Console.WriteLine("FAILED");
                        
                    printVList(vList);
                }
                
                if (visible)
                {
                    retFaces.Add(face);
                }
            }
            

            return retFaces;
        }
        private void printVertex(Vertex v)
        {
            double[] pos = v.getPos_3D();
            Console.WriteLine("VERTEX PRINT \t {0} \t {1} \t {2}", pos[0], pos[1], pos[2]);
        }

        //FINISH
        private ArrayList findBounds(double[]pos, ArrayList face, double[] faceCenter)
        {
  
            ArrayList index = new ArrayList();
            double minDist = Math.Pow(getLength(vectorDiff(faceCenter, pos)),2);
            double maxDist = 0;
            double[] minPoint = new double[3];
            double[] maxPoint = new double[3];
            for(int i = 0; i < face.Count; i++ )
            {
                Vertex[] edge = (Vertex[])face[i];
                double[] checkP = getNearestPoint(edge, pos);
                double dist = getDistance(pos, checkP);
                if(dist < minDist)
                {
                    minDist = dist;
                    minPoint = checkP;
                }

                if(dist > maxDist)
                {
                    maxDist = dist;
                    maxPoint = checkP;
                }
            }

            double[] dir1 = normalize(vectorDiff(pos, faceCenter));

       
            double[] vPos = vectorDiff(minPoint, center);

            double[][] plane = getPlane(pos, faceCenter, vPos);
            double[] norm = getCrossP(plane[0], plane[1]);
            double[][] objBase = getPlane(pos, faceCenter, norm);
            double[][] pEquation = new double[][] { plane[0], minPoint };
            ArrayList shapePoints = new ArrayList();

            ArrayList ratio = new ArrayList();
            ArrayList upperBounds = new ArrayList();
            double[] baseCenter = { 0, 0, 0 };
            Vertex[] faceBound = edgesToList(face);
            double proj = getProjection(vectorDiff(minPoint, pos), vectorDiff(faceCenter, pos));
            foreach (var v in faceBound)
            {
                Vertex vert = (Vertex)v;
                double[] posV = vert.getPos_3D();
                double mult = proj / getProjection(vectorDiff(posV, pos), vectorDiff(faceCenter, pos));
                double[] c1 = getVectorProjection(vectorDiff(posV, pos), vectorDiff(faceCenter, pos));
                double[] components = multScalar(mult, addVector(vectorDiff(getVectorProjection(vectorDiff(posV, pos), vectorDiff(faceCenter, pos)), pos), vectorDiff(posV, getVectorProjection(vectorDiff(posV, pos), vectorDiff(faceCenter, pos)))));
                double[] point = addVector(pos, components);

                shapePoints.Add(point);
                ratio.Add(getLength(vectorDiff(pos, point))/ getLength(vectorDiff(pos, vert.getPos_3D())));
                baseCenter = addVector(baseCenter, point);
                upperBounds.Add(vert.getPos_3D());
            }
            baseCenter = multScalar(1/((double)faceBound.Length), baseCenter);
            double baseDist = getLength(vectorDiff(pos, baseCenter));
           
            ArrayList bounds = new ArrayList();
            bounds.Add(baseCenter);
            bounds.Add(maxPoint);
            bounds.Add(shapePoints);
            bounds.Add(upperBounds);
            bounds.Add(minPoint);
            bounds.Add(face);
            //find 2nd part

            return bounds;
        }

        private Vertex[] edgesToList(ArrayList face)
        {
            ArrayList tempVerts = new ArrayList();
            foreach(var e in face)
            {
                Vertex[] edge = (Vertex[])e;
                for(int i = 0; i < 2; i++)
                {
                    if(!tempVerts.Contains(edge[i]))
                    {
                        tempVerts.Add(edge[i]);
                    }
                }
            }
            Vertex[] vList = new Vertex[tempVerts.Count];
            for(int i = 0; i < tempVerts.Count; i++)
            {
                vList[i] = (Vertex)tempVerts[i];
            }
            return vList;
        }

        private double[][] faceToPlane(ArrayList facePoints)
        {

            double[][] plane = new double[2][];
            double[] center = getListCenter(facePoints);

            plane[1] = center;
          
            printD(facePoints.Count);
            double[] p1 = (double[])facePoints[0];
            double[] p2 = (double[])facePoints[1];

            double[] v1 = vectorDiff(center, p1);
            double[] v2 = vectorDiff(center, p2);
  
            double[] crossP = getCrossP(v1, v2);
            plane[0] = crossP;
            
            return plane;
        }

        private ArrayList planePolygon(double[][] plane, ArrayList pointList, double[] viewP)
        {
            double planeConstant = plane[0][0] * plane[1][0] + plane[0][1] * plane[1][1] + plane[0][2] * plane[1][2];
            ArrayList facePoints = new ArrayList();
            double[][] pList = arrayListToArray(pointList);
            foreach (var p in pList)
            {
                double[] point = vectorDiff(p, viewP);
                double t_value = planeConstant/(plane[0][0] * point[0] + plane[0][1] * point[1] + plane[0][2] * point[2]);
                double[] intersection = multScalar(t_value, point);
                if (getDistance(new double[] { 0, 0, 0} , intersection) <= getDistance(new double[] { 0, 0, 0 }, point) && getDistance(point, intersection) <= getDistance(new double[] { 0, 0, 0 }, point))
                {
                    facePoints.Add(intersection);
                }
            }
            return facePoints;
        }

        private double[][] arrayListToArray(ArrayList dList)
        {
            double[][] retArray = new double[dList.Count][];
            for(int i = 0; i < dList.Count; i++)
            {
                retArray[i] = (double[])dList[i];
            }
            return retArray;
        }
        
        private double[] getListCenter(ArrayList pList)
        {
            double[] center = new double[] { 0, 0, 0 };
            for (int i = 0; i < pList.Count; i++)
            {
                center = addVector((double[])pList[i], center);
            }
            center = multScalar(1.0 / ((double)pList.Count), center);
            return center;
        }

        private Boolean checkIntersection(ArrayList coneFace, ArrayList checkPolygon, double[] viewP)
        {
            ArrayList checkFace = new ArrayList();
            Vertex[] face = edgesToList(checkPolygon);
            foreach(var v in face)
            {
                checkFace.Add(vectorDiff(v.getPos_3D(), viewP));
            }
            double[][] checkPlane = faceToPlane(checkFace);
            ArrayList facePoints = planePolygon(checkPlane, coneFace, viewP);
            ArrayList checkPoly = new ArrayList();
            
            foreach(var e in checkPolygon)
            {
                Vertex[] currE = (Vertex[])e;
                ArrayList edg = new ArrayList();
                edg.Add(currE[0].getPos_3D());
                edg.Add(currE[1].getPos_3D());
                checkPoly.Add(edg);
            }
            Console.WriteLine("check polygon");
            foreach (var e in checkPoly)
            {
                printDList((ArrayList)e);
                Console.WriteLine("next edge");
            }
            Console.WriteLine("check plane");
            printDouble(checkPlane[0]);
            printDouble(checkPlane[1]);
            Console.WriteLine("original cone");
            printDArray(arrayListToArray(coneFace));
            if(facePoints.Count > 1)
            { 
            ArrayList conePoly = vertToEdges(facePoints);
            Console.WriteLine("Cone polygon");
            foreach (var e in conePoly)
            {
                printDList((ArrayList)e);
                Console.WriteLine("next edge");
            }

                foreach (var e in conePoly)
                {

                    ArrayList edge = (ArrayList)e;
                    Console.WriteLine("PRINTING OUT THE IMPORTANT PART");
                    printDouble((double[])edge[0]);
                    printDouble((double[])edge[1]);
                    double[] line = vectorDiff((double[])edge[0], (double[])edge[1]);
                    double[] perpLine = multScalar(-1,getCrossP(line, checkPlane[0]));
                    printDouble(perpLine);

                    double maxD = 0;
                    double[] maxP = (double[])edge[0];


                    foreach (var ed in conePoly)
                    {
                        ArrayList currEdge = (ArrayList)ed;
                        double[] p1 = getVectorProjection(vectorDiff((double[])currEdge[0], (double[])edge[0]), perpLine);
                        printDouble(p1);
                        if (getDistance(p1, getVectorProjection(cameraPos, perpLine)) >= maxD)
                        {
                            maxD = getDistance(p1, getVectorProjection(cameraPos, perpLine));
                            maxP = p1;
                        }
                    }

                    Console.WriteLine("SEPARATE");
                    Boolean separate = true;
                    foreach (var ed in checkPoly)
                    {
                        ArrayList currEdge = (ArrayList)ed;
                        double[] p1 = getVectorProjection(vectorDiff((double[])currEdge[0], (double[])edge[0]), perpLine);
                        printDouble(p1);
                        if (getDistance(maxP, p1) >= maxD || getDistance(getVectorProjection(cameraPos, perpLine), p1) >= maxD)
                        {
                            Console.WriteLine("failed1");
                            printD(maxD);
                            printD(getDistance(maxP, p1));
                            printD(getDistance(p1, getVectorProjection((double[])edge[0], perpLine)));
                            printDouble(p1);
                            printDouble(maxP);
                            printDouble(getVectorProjection((double[])edge[0], perpLine));
                            
                        }
                        else
                        {
                            separate = false;
                        }
                    }
                    if(separate)
                    {
                        return false;
                    }



                }
                foreach (var e in checkPoly)
                {

                    ArrayList edge = (ArrayList)e;
                    double[] line = vectorDiff((double[])edge[0], (double[])edge[1]);
                    double[] perpLine = multScalar(1, getCrossP(line, checkPlane[0]));
                    Console.WriteLine("PRINTING OUT THE IMPORTANT PART");
                    printDouble((double[])edge[0]);

                    double maxD = 0;
                    double[] maxP = (double[])edge[0];

                    Console.WriteLine("checking CHECKCKCKCK");
                    foreach (var ed in checkPoly)
                    {
                        ArrayList currEdge = (ArrayList)ed;
                        double[] p1 = getVectorProjection(vectorDiff((double[])currEdge[0], (double[])edge[0]), perpLine);
                        printDouble(p1);
                        if (getDistance(p1, getVectorProjection(cameraPos, perpLine)) >= maxD)
                        {
                            maxD = getDistance(p1, getVectorProjection(cameraPos, perpLine));
                            maxP = p1;
                        }
                    }
                    Boolean separate = true;
                    Console.WriteLine("otherhrher");
                    foreach (var ed in conePoly)
                    {
                        ArrayList currEdge = (ArrayList)ed;
                        double[] p1 = getVectorProjection(vectorDiff((double[])currEdge[0], (double[])edge[0]), perpLine);
                        printDouble(p1);
                        if (getDistance(maxP, p1) >= maxD || getDistance(getVectorProjection(cameraPos, perpLine), p1) >= maxD)
                        {
                            printD(maxD);
                            printD(getDistance(maxP, p1));
                            printD(getDistance(p1, getVectorProjection(cameraPos, perpLine)));
                            printDouble(p1);
                            printDouble(maxP);
                            printDouble(getVectorProjection(cameraPos, perpLine));
                            Console.WriteLine("failed2");
                            
                        }
                        else
                        {
                            separate = false;
                        }
                    }
                    if(separate)
                    {
                        return false;
                    }



                }
            }
           
            return true;
        }

        private void printDList(ArrayList vertices)
        {
            foreach(var v in vertices)
            {
                printDouble((double[])v);
            }
        }
        private void printDArray(double[][] vertices)
        {
            foreach (var v in vertices)
            {
                printDouble(v);
            }
        }

        private ArrayList vertToEdges(ArrayList vertices)
        {
      
            ArrayList edges = new ArrayList();
            double[] center = getListCenter(vertices);
            double[][] plane = faceToPlane(vertices);
            int max = -1;
            double maxDist = maxPossibleDotP;
           
        
            for(int i = 0; i < vertices.Count; i++)
            {
                double[] currP = (double[])vertices[i];
                if(getDistance(center, currP) > maxDist)
                {
                    maxDist = getDistance(cameraPos, currP);
                    max = i;
                }
            }

            if (max != -1)
            {
                double[] maxP = (double[])vertices[max];
                double[] finalP = maxP;
                double vCount = vertices.Count;
                vertices.Remove(vertices[max]);
                double[] dir = vectorDiff(maxP, center);
                Console.WriteLine("WRONG SECTION");
                printDouble(dir);
                printDouble(maxP);
                while (vertices.Count > 0)
                {
                    //System.Threading.Thread.Sleep(500);
                    max = -1;
                    maxDist = maxPossibleDotP;
                    for (int i = 0; i < vertices.Count; i++)
                    {
                        double[] currP = (double[])vertices[i];
                        printDouble(currP);
                        printDouble(center);
                        printDouble(dir);
                        double dotP = dotProduct(dir, vectorDiff(currP, center))/getLength(vectorDiff(currP, center));
                        printD(dotP);
                        if (dotP >= maxDist)
                        {
                            
                            maxDist = dotP;
                            max = i;
                        }
                    }
                    if (max != -1)
                    {
                        double[] nextP = (double[])vertices[max];
                        vertices.Remove(vertices[max]);
                        ArrayList newEdge = new ArrayList();
                        newEdge.Add(maxP);
                        newEdge.Add(nextP);
                        edges.Add(newEdge);
                        maxP = nextP;
                        dir = vectorDiff(maxP, center);
                    }
                }
               // System.Threading.Thread.Sleep(1000);
                if (vCount > 2)
                {
                    ArrayList edge = new ArrayList();
                    edge.Add(maxP);
                    edge.Add(finalP);
                    edges.Add(edge);
                }
            }
            else
            {
                Console.WriteLine("CHECKING FOR WEIRD");
            }
            return edges;
        }

        
        private Boolean checkEdgeBounds(double[] point, ArrayList bounds)
        {
            //first section
            Console.WriteLine("checkEdgeBounds");
            double[] baseCenter = (double[])bounds[0];
            double[] endCenter = multScalar(dotProduct(vectorDiff(point, baseCenter), vectorDiff(point, (double[])bounds[1])) / (Math.Pow(getLength(vectorDiff(point, baseCenter)), 2)), vectorDiff(point, baseCenter));
            ArrayList face = (ArrayList)bounds[5];
            Vertex[] vList = edgesToList(face);
            foreach(var e in edges)
            {
                
                Vertex[] edge = (Vertex[])e;
                if (!inFace(face, edge))
                {
                    Console.WriteLine("CHECKING EDGE/FACE");
                    printEdge(edge);
                    printList(face);
                    if (getProjection(vectorDiff(edge[0].getPos_3D(), point), vectorDiff(baseCenter, point)) >= 0 && getProjection(vectorDiff(edge[1].getPos_3D(), point), vectorDiff(baseCenter, point)) >= 0)
                    {
                        
                        if (getLength(getVectorProjection(vectorDiff(edge[0].getPos_3D(), point), vectorDiff(baseCenter, point))) <= getLength(vectorDiff(baseCenter, point)) && getLength(getVectorProjection(vectorDiff(edge[1].getPos_3D(), point), vectorDiff(baseCenter, point))) <= getLength(vectorDiff(baseCenter, point)))
                        {
                            Console.WriteLine("CASE 1");
                            if (checkCone(face, edge, point, baseCenter) || checkCone(face, new Vertex[] { edge[1], edge[0]}, point, baseCenter))
                            {
                                return true;
                            }
                        }
                        else if (getLength(getVectorProjection(vectorDiff(edge[0].getPos_3D(), point), vectorDiff(baseCenter, point))) <= getLength(vectorDiff(baseCenter, point)) && getLength(getVectorProjection(vectorDiff(edge[1].getPos_3D(), point), vectorDiff(baseCenter, point))) > getLength(vectorDiff(baseCenter, point)))
                        {
                            Console.WriteLine("CASE 2");
                            if (checkCone(face, edge, point, baseCenter) || checkBase(face, new Vertex[] { edge[1], edge[0]}, point, endCenter))
                            {
                                return true;
                            }
                        }
                        else if (getLength(getVectorProjection(vectorDiff(edge[1].getPos_3D(), point), vectorDiff(baseCenter, point))) <= getLength(vectorDiff(baseCenter, point)) && getLength(getVectorProjection(vectorDiff(edge[0].getPos_3D(), point), vectorDiff(baseCenter, point))) > getLength(vectorDiff(baseCenter, point)))
                        {
                            Console.WriteLine("CASE 3");
                            if (checkBase(face, edge, point, baseCenter) || checkCone(face, new Vertex[] { edge[1], edge[0] }, point, endCenter))
                            {
                                return true;
                            }
                        }
                        else if (getLength(getVectorProjection(vectorDiff(edge[0].getPos_3D(), point), vectorDiff(endCenter, point))) <= getLength(vectorDiff(endCenter, point)) && getLength(getVectorProjection(vectorDiff(edge[1].getPos_3D(), point), vectorDiff(endCenter, point))) <= getLength(vectorDiff(endCenter, point)))
                        {
                            Console.WriteLine("CASE 4");
                            if (checkBase(face, edge, point, baseCenter) || checkBase(face, new Vertex[] { edge[1], edge[0] }, point, endCenter))
                            {
                                return true;
                            }
                        }
                        else if (getLength(getVectorProjection(vectorDiff(edge[0].getPos_3D(), point), vectorDiff(endCenter, point))) <= getLength(vectorDiff(endCenter, point)) && getLength(getVectorProjection(vectorDiff(edge[1].getPos_3D(), point), vectorDiff(endCenter, point))) > getLength(vectorDiff(endCenter, point)))
                        {
                            Console.WriteLine("CASE 5");
                            if (checkBase(face, edge, point, endCenter))
                            {
                                return true;
                            }
                        }
                        else if (getLength(getVectorProjection(vectorDiff(edge[1].getPos_3D(), point), vectorDiff(endCenter, point))) <= getLength(vectorDiff(endCenter, point)) && getLength(getVectorProjection(vectorDiff(edge[0].getPos_3D(), point), vectorDiff(baseCenter, point))) > getLength(vectorDiff(baseCenter, point)))
                        {

                            Console.WriteLine("CASE 6");
                            if (checkBase(face, new Vertex[] { edge[1], edge[0] }, point, endCenter))
                            {
                                return true;
                            }
                        }
                        Console.WriteLine("CASE 6.5");
                    }
                    else if(getProjection(vectorDiff(edge[0].getPos_3D(), point), vectorDiff(baseCenter, point)) >= 0)
                    {
                        Console.WriteLine("CASE 7");
                        if (checkCone(face, edge, point, baseCenter))
                        {

                            return true;
                        }
                    }
                    else if (getProjection(vectorDiff(edge[1].getPos_3D(), point), vectorDiff(baseCenter, point)) >= 0)
                    {
                        Console.WriteLine("CASE 8");
                        if (checkCone(face, new Vertex[] { edge[1], edge[0] }, point, baseCenter))
                        {
                            
                            return true;
                        }
                    }
                }
            }
            return false;
        }

        private Boolean checkCone(ArrayList face, Vertex[] edge, double[] point, double[] baseCenter)
        {
            Console.WriteLine("ConeCheck");
            Vertex[] vList = edgesToList(face);

            foreach (var e in face)
            {
                Vertex[] edgeFace = (Vertex[])e;
                double[][] planeTemp = getPlane(point, edgeFace[0].getPos_3D(), edgeFace[0].getPos_3D());
                double[][] plane = new double[][] { point, planeTemp[0], planeTemp[1] };
                double[][] line = new double[][] { edge[0].getPos_3D(), vectorDiff(edge[0].getPos_3D(), edge[1].getPos_3D()) };
                double[] intersection = planeIntersection(plane, line);
                if (intersection != null && !pointEquals(intersection, edge[0].getPos_3D()) && !pointEquals(intersection, edge[1].getPos_3D()))
                {
                    if (getLength(getVectorProjection(intersection, baseCenter)) <= getLength(baseCenter))
                    {
                        if ((Math.Round(dotProduct(vectorDiff(edgeFace[0].getPos_3D(), point), vectorDiff(intersection, point)) / getLength(vectorDiff(intersection, point)), 5) > Math.Round(dotProduct(vectorDiff(edgeFace[0].getPos_3D(), point), vectorDiff(edgeFace[1].getPos_3D(), point)) / getLength(vectorDiff(edgeFace[1].getPos_3D(), point)), 5) || Math.Round(dotProduct(vectorDiff(edgeFace[1].getPos_3D(), point), vectorDiff(intersection, point)) / getLength(vectorDiff(intersection, point)), 5) > Math.Round(dotProduct(vectorDiff(edgeFace[1].getPos_3D(), point), vectorDiff(edgeFace[0].getPos_3D(), point)) / getLength(vectorDiff(edgeFace[0].getPos_3D(), point)))))
                        {
                            printEdge(edge);

                            printDouble(intersection);
                            return true;
                        }
                    }
                }
            }
            Console.WriteLine("ConeCheckFALSE");
            return false;
        }
        
        private Boolean pointEquals(double[] pos1, double[] pos2)
        {
            for(int i= 0; i < pos1.Length; i++)
            {
                if(pos1[i] != pos2[i])
                {
                    return false;
                }
            }
            return true;
        }

     
        private Boolean checkBase(ArrayList face, Vertex[] edge, double[] point, double[] endCenter)
        {
            Console.WriteLine("BaseCheck");
            Vertex[] vList = edgesToList(face);
           
            foreach (var e in face)
            {
                Vertex[] edgeFace = (Vertex[])e;
                if (!edgeEquals(edge, edgeFace))
                {
                    double[][] planeTemp = getPlane(point, edgeFace[0].getPos_3D(), edgeFace[1].getPos_3D());
                    double[][] plane = new double[][] { point, planeTemp[0], planeTemp[1] };
                    double[][] line = new double[][] { edge[0].getPos_3D(), vectorDiff(edge[0].getPos_3D(), edge[1].getPos_3D()) };
                    double[] intersection = planeIntersection(plane, line);

                    if (intersection != null && !pointEquals(intersection, edge[0].getPos_3D()) && !pointEquals(intersection, edge[1].getPos_3D()))
                    {
                        double len = Math.Round(getLength(getVectorProjection(vectorDiff(intersection, point), vectorDiff(endCenter, point))), 5);
                        double len1 = Math.Round(getLength(getVectorProjection(vectorDiff(edgeFace[0].getPos_3D(), point), vectorDiff(endCenter, point))), 5);
                        double len2 = Math.Round(getLength(getVectorProjection(vectorDiff(edgeFace[1].getPos_3D(), point), vectorDiff(endCenter, point))), 5);

                        if ((dotProduct(vectorDiff(edgeFace[0].getPos_3D(), point), vectorDiff(intersection, point)) / getLength(vectorDiff(intersection, point)) > dotProduct(vectorDiff(edgeFace[0].getPos_3D(), point), vectorDiff(edgeFace[1].getPos_3D(), point)) / getLength(vectorDiff(edgeFace[1].getPos_3D(), point)) || dotProduct(vectorDiff(edgeFace[1].getPos_3D(), point), vectorDiff(intersection, point)) / getLength(vectorDiff(intersection, point)) > dotProduct(vectorDiff(edgeFace[1].getPos_3D(), point), vectorDiff(edgeFace[0].getPos_3D(), point)) / getLength(vectorDiff(edgeFace[0].getPos_3D(), point))))
                        {
                            if (len < len1 && len < len2)
                            {
                                printEdge(edge);
                                printEdge(edgeFace);
                                printDouble(intersection);
                                printDouble(endCenter);
                                printD(len);
                                printD(len1);
                                printD(len2);
                                return true;
                            }
                            else if (len < len1 && len >= len2)
                            {
                                double[] diffVec = vectorDiff(edge[0].getPos_3D(), edge[1].getPos_3D());
                                if (dotProduct(diffVec, multScalar(-1, vectorDiff(edge[0].getPos_3D(), point))) / getLength(diffVec) < dotProduct(intersection, multScalar(-1, vectorDiff(edge[0].getPos_3D(), point))) / getLength(intersection))
                                {
                                    printEdge(edge);
                                    printDouble(intersection);
                                    printD(1);
                                    return true;
                                }
                            }
                            else if (len >= len1 && len < len2)
                            {
                                double[] diffVec = vectorDiff(edge[1].getPos_3D(), edge[0].getPos_3D());
                                if (dotProduct(diffVec, multScalar(-1, vectorDiff(edge[1].getPos_3D(), point))) / getLength(diffVec) < dotProduct(intersection, multScalar(-1, vectorDiff(edge[1].getPos_3D(), point))) / getLength(intersection))
                                {
                                    printEdge(edge);
                                    printDouble(intersection);
                                    printD(2);
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
            return false;
           
        }

        private void printEdge(Vertex[] edge)
        {
            double[] pos1 = edge[0].getPos_3D();
            double[] pos2 = edge[1].getPos_3D();
            Console.WriteLine("VERTEX 1 \t {0} \t {1} \t {2} \t VERTEX 2 \t {3} \t {4} \t {5}", pos1[0], pos1[1], pos1[2], pos2[0], pos2[1], pos2[2]);
        }

        private double[] planeToEquation(double[][] plane)
        {
            double[] crossP = getCrossP(plane[1], plane[2] );
            double constant = dotProduct(crossP, plane[0]);
            return new double[] { constant, crossP[0], crossP[1], crossP[2] };
        }

        private double[] planeIntersection(double[][] plane, double[][] line)
        {
            double[] equationP = planeToEquation(plane);
            double t_value = equationP[1] * line[1][0] + equationP[2] * line[1][1] + equationP[3] * line[1][2];
            if (t_value != 0)
            {
                double constant = equationP[0] - (equationP[1] * line[0][0] + equationP[2] * line[0][1] + equationP[3] * line[0][2]);
                double t = constant / t_value;
                return roundVector(addVector(line[0], multScalar(t, line[1])));
            }
            else
            {
                Console.WriteLine("no intersection");
                return null;
            }
            
        }
        private double[] roundVector(double[] v, int num)
        {
            for (int i = 0; i < v.Length; i++)
            {
                v[i] = Math.Round(v[i], num);
            }
            return v;
        }

        private double[] roundVector(double[] v)
        {
            for(int i = 0; i < v.Length; i++)
            {
                v[i] = Math.Round(v[i],10);
            }
            return v;
        }

        private double[][] diffList(double[][] points, double[] center)
        {
            for(int i = 0; i < points.Length; i++)
            {
                points[i] = vectorDiff(points[i], center);
            }
            return points;
        }
        private Boolean checkBounds(double[] pos, double[][] points, ArrayList bounds)
        {
            
            double[] baseCenter = (double[])bounds[0];
            double[] point = new double[3];
            //findEdgeClosest(points, bounds);
            if (dotProduct(vectorDiff(point, pos), vectorDiff(baseCenter, pos)) >= 0)
            {
                double[] endCenter = multScalar(dotProduct(vectorDiff(pos, baseCenter), vectorDiff(pos, (double[])bounds[1]))/(Math.Pow(getLength(vectorDiff(pos, baseCenter)),2)), vectorDiff(pos, baseCenter));
                ArrayList adjBound = new ArrayList();
  
                if (getLength(getVectorProjection(vectorDiff(point, pos), vectorDiff(baseCenter, pos))) <= getLength(vectorDiff(baseCenter, pos)))
                {
         
                    ArrayList typeBound = (ArrayList)bounds[2];
                    ArrayList upperBound = (ArrayList)bounds[3];
                    for(int i = 0; i< typeBound.Count; i++)
                    {
              
                        double[] bound = (double[])typeBound[i];
                        double[] upperB = (double[])upperBound[i];
                        double value = dotProduct(vectorDiff(point, pos), vectorDiff(baseCenter, pos)) / getLength(getVectorProjection(vectorDiff(upperB, pos), vectorDiff(endCenter, pos)));

                        double[] planeLoc = multScalar(value, bound);
                        adjBound.Add(addVector(pos, multScalar(value, normalize(vectorDiff(bound, pos)))));
                    }
                  
                }
                else if(getLength(getVectorProjection(vectorDiff(point, pos), vectorDiff(endCenter, pos))) <= getLength(vectorDiff(endCenter, pos)))
                {
   
                    ArrayList typeBound = (ArrayList)bounds[3];
                    
                    
                    double[] endPoint = (double[])bounds[1];
                    double[] startPoint = (double[])bounds[4];
                    double lineVal = getProjection(vectorDiff(point, endCenter),endCenter);
         
                    double projP = getProjection(point, endCenter);
                    ArrayList boundEdges = (ArrayList)bounds[5];
                    foreach (var e in boundEdges)
                    {
                        Vertex[] edge = (Vertex[])e;
                        ArrayList vList = new ArrayList();
                        ArrayList otherV = new ArrayList();
          
                        foreach(var v in edge)
                        {
                            Vertex vert = (Vertex)v;
                            double[] vPos = vert.getPos_3D();
                            double proj = getProjection(vPos, endCenter) ;
                            if(proj >= projP)
                            {
                                vList.Add(vert);
                            }
                            else
                            {
                                otherV.Add(vert);
                            }
                        }
                        if(vList.Count > 0)
                        {
                            foreach (var v in vList)
                            {
                                Vertex vert = (Vertex)v;
                                double[] vPos = vert.getPos_3D();
                                double value = getProjection(point, endCenter) / getProjection(vPos, endCenter);
                                double[] components = addVector(multScalar(getProjection(vPos, endCenter), normalize(endCenter)), multScalar(value, vectorDiff(vPos, multScalar(value, normalize(endCenter)))));
                                if (!isInList(components, adjBound))
                                {
                                    adjBound.Add(addVector(pos, components));
                                }
                            }
                            if(otherV.Count > 0)
                            {
                                Vertex vert1 = (Vertex)vList[0];
                                Vertex vert2 = (Vertex)otherV[0];
                                double[] vPos1 = vert1.getPos_3D();
                                double[] vPos2 = vert2.getPos_3D();
                                double[] vDiff = vectorDiff(vPos1, vPos2);

                                double proj1 = getProjection(vPos1, endCenter);
                                double proj2 = getProjection(vPos2, endCenter);
                                double value = (projP - proj2) / (proj1 - proj2);
                                double[] newPos = addVector(vPos2, multScalar(value, vDiff));
                                if (!isInList(newPos, adjBound))
                                {
                                    adjBound.Add(newPos);
                                }
                            }
                        }
            
                    }
                }
     
                double[] planeCenter = new double[3];
       

                foreach(var b in adjBound)
                {
                 
                    planeCenter = addVector(planeCenter, (double[])b);

                }

                planeCenter = multScalar(1 /((double) adjBound.Count), planeCenter);

                ArrayList shiftBound = new ArrayList();
                foreach(var b in adjBound)
                {
                    double[] bound = (double[])b;
                    bound = vectorDiff(bound, planeCenter);
                    shiftBound.Add(bound);
                }
                double[] checkP = vectorDiff(point, planeCenter);
  
                //FIX FINDCLOSEST
                double[][] closeP = findClosest(checkP, shiftBound);

                double[] edgeDir = getNormDirection(closeP, checkP);


                Console.WriteLine("LENGTH 1 \t {0}", getLength(edgeDir));
                Console.WriteLine("LENGTH 2 \t {0}", getProjection(checkP, edgeDir));
                if(getLength(edgeDir) >= -(getProjection(checkP, edgeDir)))
                {

                    Console.WriteLine("POINT \t {0} \t {1} \t {2}", checkP[0], checkP[1], checkP[2]);
                    Console.WriteLine("CLOSE POINT 1 \t {0} \t {1} \t {2}", closeP[0][0], closeP[0][1], closeP[0][2]);
                    Console.WriteLine("CLOSE POINT 2 \t {0} \t {1} \t {2}", closeP[1][0], closeP[1][1], closeP[1][2]);
                    Console.WriteLine("FAILED WITH DISTANCES \t {0} \t {1}", getLength(edgeDir), getProjection(checkP, edgeDir));
                    return true;
                }
            }
            else
            {
                Console.WriteLine("no this hsouildnt happen");
                printDouble(vectorDiff(point, pos));
                printDouble(vectorDiff(baseCenter, pos));
            }
            return false;
        }

        public Boolean isInList(double[] elem, ArrayList aList)
        {
            foreach(var e in aList)
            {
                double[] pos = (double[])e;
                if(pos[0] == elem[0] && pos[1] == elem[1] && pos[2] == elem[2])
                {
                    return true;
                }
            }
            return false;
        }

        public ArrayList showFace(ArrayList face, double[] point, double[] viewCenter)
        {
            ArrayList returnEdges = new ArrayList();
            foreach(var e in face)
            {
                Vertex[] edge = (Vertex[])e;
                double[] pos1 = (double[])edge[0].getPos_3D();
                double[] pos2 = (double[])edge[1].getPos_3D();
                returnEdges.Add(new double[][] { pos1, pos2 });
            }
            ArrayList faceV = edgesToVerts(face);
            foreach(var v in faceV)
            {
                Vertex vert = (Vertex)v;
                double[] vPos = (double[])vert.getPos_3D();
                returnEdges.Add(new double[][] { viewCenter, vPos });
            }
            double[] vCenter = getCenter(faceV);
            double minDist = getLength(vectorDiff(vCenter, viewCenter));

            double maxDist = 0;
            double[] minPoint = new double[3];
            double[] maxPoint = new double[3];
            for (int i = 0; i < face.Count; i++)
            {
                Vertex[] edge = (Vertex[])face[i];
    
                double[] checkP = getNearestPoint(edge, viewCenter);
        
                double dist = getDistance(viewCenter, checkP);
                
                if (dist < minDist)
                {
                    minDist = dist;
                    minPoint = checkP;
                }

                if (dist > maxDist)
                {
                    maxDist = dist;
                    maxPoint = checkP;
                }
            }

            returnEdges.Add(new double[][] { viewCenter, addVector(viewCenter, getVectorProjection(vectorDiff(maxPoint, viewCenter), vectorDiff(viewCenter, vCenter))) });
            returnEdges.Add(new double[][] { addVector(viewCenter, getVectorProjection(vectorDiff(maxPoint, viewCenter), vectorDiff(viewCenter, vCenter))), maxPoint });
            returnEdges.Add(new double[][] { viewCenter, addVector(viewCenter, getVectorProjection(vectorDiff(minPoint, viewCenter), vectorDiff(viewCenter, vCenter))) });
            returnEdges.Add(new double[][] { addVector(viewCenter, getVectorProjection(vectorDiff(minPoint, viewCenter), vectorDiff(viewCenter, vCenter))), minPoint });
            returnEdges.Add(new double[][] { viewCenter, point});
            returnEdges.Add(new double[][] { viewCenter, vCenter });
            returnEdges.Add(new double[][] { viewCenter, addVector(viewCenter, getVectorProjection(vectorDiff(point, viewCenter), vectorDiff(viewCenter, vCenter))) });
            returnEdges.Add(new double[][] { addVector(viewCenter, getVectorProjection(vectorDiff(point, viewCenter), vectorDiff(viewCenter, vCenter))) , point });
            return returnEdges;
        }

        private double[] getVectorProjection(double[] v1, double[] v2)
        {
            return multScalar(getProjection(v1, v2), normalize(v2));
        }

        private double[] getCenter(ArrayList vList)
        {
            double[] vCenter = new double[3];
            foreach(var v in vList)
            {
                vCenter = addVector(vCenter, ((Vertex)v).getPos_3D());
            }
            vCenter = multScalar(1 /((double) vList.Count), vCenter);
            return vCenter;
        }

        private ArrayList edgesToVerts(ArrayList edgeList)
        {
            ArrayList vList = new ArrayList();
            foreach(var e in edgeList)
            {
                Vertex[] edge = (Vertex[])e;
                if(!vList.Contains(edge[0]))
                {
                    vList.Add(edge[0]);
                }
                if(!vList.Contains(edge[1]))
                {
                    vList.Add(edge[1]);
                }
            }
            return vList;
        }

        private double getProjection(double[] p1, double[] p2)
        {
            return (dotProduct(p1, p2) / getLength(p2));
        }

        private double[][] closestPoints(double[] pos, double[][] points)
        {
            return null;
            double maxDotP = maxPossibleDotP;
            double[][] closePoints = new double[2][];
            double[] bestPoint = new double[3];

            foreach(var p in points)
            {
                double[] point = p;
                double dotP = dotProduct(point, pos) / getLength(pos);
                if(dotP > maxDotP)
                {
                    maxDotP = dotP;
                    bestPoint = point;
                }
            }
            closePoints[0] = bestPoint;
            if (maxDotP == getLength(pos))
            {
                closePoints[1] = bestPoint;
            }
            else
            {
                bestPoint = new double[3];
                maxDotP = maxPossibleDotP;
                foreach (var p in points)
                {
                    double[] point = p;
                    double dotP = dotProduct(point, pos) / getLength(pos);
                    if (dotP > maxDotP && dotP > dotProduct(point, closePoints[0]) / getLength(closePoints[0]))
                    {
                        maxDotP = dotP;
                        bestPoint = point;
                    }
                }

                closePoints[1] = bestPoint;
            }
            return closePoints;
        }

        private double[][] findClosest (double[] checkPoint, ArrayList points)
        {
            double maxDotP = maxPossibleDotP;
            double[][] closePoints = new double[2][];
            double[] bestPoint = new double[3];
   
            foreach (var p in points)
            {
                
                double[] point = (double[])p;
             
                
                double dotP = dotProduct(point, checkPoint)/getLength(checkPoint);
          
                if (dotP > maxDotP)
                {
                    maxDotP = dotP;
                    bestPoint = point;
   
                }
            }
            
            closePoints[0]= bestPoint;
            if (maxDotP == getLength(checkPoint))
            {
                closePoints[1] = bestPoint;
            }
            else
            {

                maxDotP = maxPossibleDotP;
                foreach (var p in points)
                {
                    double[] point = (double[])p;
                    double dotP = dotProduct(point, checkPoint) / getLength(checkPoint);
      
                    if (dotP > maxDotP && dotP > dotProduct(point, closePoints[0]) / getLength(closePoints[0]))
                    {
                        maxDotP = dotP;
                        bestPoint = point;
              
                    }
                }
            }
            closePoints[1] = bestPoint;
            return closePoints;
        }

        private double[][] getPlane(double[] pos1, double[] pos2, double[] pos3)
        {
            double[] dir1 = normalize(vectorDiff(pos1, pos2));
            double[] component = multScalar(dotProduct(dir1, pos3), dir1);
            double[] dir2 = normalize(vectorDiff(pos3, component));
            return new double[][] { dir1, dir2 };
        }

        private double getDistance(double[] pos1, double[] pos2)
        {
            return Math.Sqrt(Math.Pow(pos1[0] - pos2[0], 2) + Math.Pow(pos1[1] - pos2[1], 2) + Math.Pow(pos1[2] - pos2[2], 2));
        }

        public void shiftObj(double[] shift)
        {
            center = addVector(center, shift);
            foreach(var v in verts)
            {
                Vertex vert = (Vertex)v;
                vert.shiftPos3D(shift, cameraPos, cameraRot);
            }
        }
        public void setPos(double[] pos)
        {
            
            foreach (var v in verts)
            {
                Vertex vert = (Vertex)v;
                vert.setPos3D(addVector(pos, vectorDiff(vert.getPos_3D(), center)), cameraPos, cameraRot);
            }
            center = pos;
        }

        private Boolean inFace(ArrayList face, Vertex[] edge)
        {
            foreach(var e in face)
            {
                Vertex[] edgeFace = (Vertex[])e;
                if(edgeEquals(edge, edgeFace))
                {
                    return true;
                }
            }
            return false;
        }
    }
}
