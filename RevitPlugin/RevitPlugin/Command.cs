using System;
using Autodesk.Revit.DB;
using Autodesk.Revit.DB.Architecture;
using Autodesk.Revit.UI;
using Autodesk.Revit.UI.Selection;
using Autodesk.Revit.ApplicationServices;
using Autodesk.Revit.Attributes;
using System.Collections.Generic;


namespace RevitPlugin
{
    /// <summary>
    /// Implements the Revit add-in interface IExternalCommand
    /// </summary>
    [Autodesk.Revit.Attributes.Transaction(Autodesk.Revit.Attributes.TransactionMode.Automatic)]
    [Autodesk.Revit.Attributes.Regeneration(Autodesk.Revit.Attributes.RegenerationOption.Manual)]
    [Autodesk.Revit.Attributes.Journaling(Autodesk.Revit.Attributes.JournalingMode.NoCommandData)]
    public class Command : Autodesk.Revit.UI.IExternalCommand
    {
        
        /// <summary>
        /// Implement this method as an external command for Revit.
        /// </summary>
        /// <param name="cmdData">An object that is passed to the external application 
        /// which contains data related to the command, 
        /// such as the application object and active view.</param>
        /// <param name="msg">A message that can be set by the external application 
        /// which will be displayed if a failure or cancellation is returned by 
        /// the external command.</param>
        /// <param name="eleSet">A set of elements to which the external application 
        /// can add elements that are to be highlighted in case of failure or cancellation.</param>
        /// <returns>Return the status of the external command. 
        /// A result of Succeeded means that the API external method functioned as expected. 
        /// Cancelled can be used to signify that the user cancelled the external operation 
        /// at some point. Failure should be returned if the application is unable to proceed with 
        /// the operation.</returns>
        public Autodesk.Revit.UI.Result Execute(ExternalCommandData cmdData, ref string msg, ElementSet eleSet)
        {
            Autodesk.Revit.UI.Result res = Autodesk.Revit.UI.Result.Succeeded;
            try
            {
                Properties props = new Properties("../revit_properties.ini");
                string pathToSteerSuite = props.get("SteerSuitePath");
                string path = Environment.GetEnvironmentVariable("PATH");
                path += System.IO.Path.PathSeparator.ToString() + pathToSteerSuite;
                Environment.SetEnvironmentVariable("PATH", path);

                Autodesk.Revit.UI.Selection.Selection sel;
                sel = cmdData.Application.ActiveUIDocument.Selection;
                UIApplication uiApp = cmdData.Application;
                Document doc = uiApp.ActiveUIDocument.Document;
                

                // Find all Wall instances in the document by using category filter
                ElementCategoryFilter filter = new ElementCategoryFilter(BuiltInCategory.OST_Walls);

                // Apply the filter to the elements in the active document
                // Use shortcut WhereElementIsNotElementType() to find wall instances only
                FilteredElementCollector collector = new FilteredElementCollector(doc);
                // IList<ElementId> verts_refs = new List<ElementId>();
                IList<XYZ> verts = new List<XYZ>();
                IList<List<int>> edges = new List<List<int> >();

                IList<Element> walls =
                collector.WherePasses(filter).WhereElementIsNotElementType().ToElements();
                String prompt = "";
                const double _feet_to_meters = 0.3048;
                foreach (Element e in walls)
                {
                    BoundingBoxXYZ bbox = e.get_BoundingBox(null);
                    Autodesk.Revit.DB.LocationCurve lineLoc;
                    lineLoc = e.Location as LocationCurve;


                    List<Solid> solids = new List<Solid>();
                    Options options = new Options();
                    options.DetailLevel = ViewDetailLevel.Fine;
                    GeometryElement geomElem = e.get_Geometry(options);
                    String dat = "";
                    /*
                    foreach (GeometryObject geomObj in geomElem)
                    {
                        if (geomObj is Solid)
                        {
                            Solid solid = (Solid)geomObj;

                            if (solid.Faces.Size > 0 && solid.Volume > 0.0)
                            {
                                dat += "Faces: " + solid.Faces.ToString();
                                // solid.Faces.get_Item(0).

                                solids.Add(solid);
                            }
                            // Single-level recursive check of instances. If viable solids are more than
                            // one level deep, this example ignores them.
                        }
                        /*
                        else if (geomObj is GeometryInstance)
                        {
                            GeometryInstance geomInst = (GeometryInstance)geomObj;
                            GeometryElement instGeomElem = geomInst.GetInstanceGeometry();
                            foreach (GeometryObject instGeomObj in instGeomElem)
                            {
                                if (instGeomObj is Solid)
                                {
                                    Solid solid = (Solid)instGeomObj;
                                    if (solid.Faces.Size > 0 && solid.Volume > 0.0)
                                    {
                                        solids.Add(solid);
                                    }
                                }
                            }
                        }
                    }*/

                    List<int> edge_ = new List<int>();
                    Reference ref_ = lineLoc.Curve.GetEndPointReference(0);
                    // TaskDialog.Show("ElementID", "" + ref_);
                    Autodesk.Revit.DB.XYZ origin = lineLoc.Curve.GetEndPoint(0);
                    bool contains = false;
                    int or=0, ed=0;
                    for (int i=0; i < verts.Count; i++)
                    {
                        XYZ p = verts[i];
                        double l = Math.Sqrt(Math.Pow((p.X - origin.X),2) +
                            Math.Pow((p.Y - origin.Y),2) + Math.Pow((p.Z - origin.Z),2));
                        if ( l < 0.001 )
                        {
                            contains = true;
                            or = i;
                            break;
                        }
                    }
                    Autodesk.Revit.DB.XYZ end = lineLoc.Curve.GetEndPoint(1);
                    if (!contains)
                    {
                        or = verts.Count;
                        verts.Add(origin);
                       //  prompt += "<" + origin.X + ", " + origin.Z + ", " + origin.Y + ">\n";
                    }

                    contains = false;
                    for (int i = 0; i < verts.Count; i++)
                    {
                        XYZ p = verts[i];
                        double l = Math.Sqrt(Math.Pow((p.X - end.X), 2) +
                            Math.Pow((p.Y - end.Y), 2) + Math.Pow((p.Z - end.Z), 2));
                        if (l < 0.001)
                        {
                            contains = true;
                            ed = i;
                            break;
                        }
                    }
                    if (!contains)
                    {
                        ed = verts.Count;
                        verts.Add(end);
                        // prompt += "<" + end.X + ", " + end.Z + ", " + end.Y + ">\n";
                    }
                    // Autodesk.Revit.DB.XYZ origin = lineLoc.Curve.GetEndPoint(0);
                    // Autodesk.Revit.DB.XYZ end = lineLoc.Curve.GetEndPoint(1);
                    // Reference ref_ = lineLoc.Curve.GetEndPointReference(0);
                    bbox.Min = bbox.Min * _feet_to_meters;
                    bbox.Max = bbox.Max * _feet_to_meters;

                    edge_.Add(or);
                    edge_.Add(ed);

                    edges.Add(edge_);
                    // prompt += "<" + origin.X + ", " + origin.Z + ", " + origin.Y + "> - " +
                    //          "<" + end.X + ", " + end.Z + ", " + end.Y + ">\n";

                    // prompt += bbox.Min.X + ", " + bbox.Max.X + "," + bbox.Min.Z + ", " + bbox.Max.Z + "," +
                    //   bbox.Min.Y + ", " + bbox.Max.Y + "\n";

                }
                
                foreach (XYZ p in verts)
                {   // converted axis here
                    XYZ po = p * _feet_to_meters;
                    prompt += "v " + po.X + " " + po.Z + " " + po.Y + "\n";
                }
                prompt += "\n";
                foreach (List<int> edge_ in edges)
                {
                    prompt += "e " + edge_[0] + " " + edge_[1] + "\n";
                }

                System.IO.StreamWriter file = new System.IO.StreamWriter(pathToSteerSuite +"test.txt");
                file.WriteLine(prompt);

                file.Close();

                /*
                if (element != null)
                {
                    Autodesk.Revit.DB.LocationCurve lineLoc;
                    lineLoc = element.Location as LocationCurve;

                    if (null == lineLoc)
                    {
                        TaskDialog.Show("MoveLinear", "Please select an element which based on a Line");
                        return res;
                    }

                    Autodesk.Revit.DB.Line line;
                    //get start point via "get_EndPoint(0)"
                    Autodesk.Revit.DB.XYZ newStart = new XYZ(
                        lineLoc.Curve.GetEndPoint(0).X + 15,
                        lineLoc.Curve.GetEndPoint(0).Y,
                        lineLoc.Curve.GetEndPoint(0).Z);
                    //get end point via "get_EndPoint(1)"
                    Autodesk.Revit.DB.XYZ newEnd = new XYZ(
                        lineLoc.Curve.GetEndPoint(1).X + 15,
                        lineLoc.Curve.GetEndPoint(1).Y,
                        lineLoc.Curve.GetEndPoint(1).Z);


                    //get a new line and use it to move current element 
                    //with property "Autodesk.Revit.DB.LocationCurve.Curve"
                    line = Line.CreateBound(newStart, newEnd);
                    lineLoc.Curve = line;
                }
            */
                
            }
            catch (Exception ex)
            {
                TaskDialog.Show("MoveLinear", ex.Message);
                res = Autodesk.Revit.UI.Result.Failed;
            }
            finally
            {
            }
            return res;
        }
      
    }
}
