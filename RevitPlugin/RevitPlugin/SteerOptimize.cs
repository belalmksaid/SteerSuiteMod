using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Autodesk.Revit.UI;
using Autodesk.Revit.DB;
// using steerSuiteAdapterCSharp.SimWorld;



namespace RevitPlugin
{
    /// <summary>
    /// Implements the Revit add-in interface IExternalCommand
    /// </summary>
    [Autodesk.Revit.Attributes.Transaction(Autodesk.Revit.Attributes.TransactionMode.Automatic)]
    [Autodesk.Revit.Attributes.Regeneration(Autodesk.Revit.Attributes.RegenerationOption.Manual)]
    [Autodesk.Revit.Attributes.Journaling(Autodesk.Revit.Attributes.JournalingMode.NoCommandData)]
    class SteerOptimize : Autodesk.Revit.UI.IExternalCommand
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

                SimWorld world = new SimWorld();

                // Find all Wall instances in the document by using category filter
                ElementCategoryFilter filter = new ElementCategoryFilter(BuiltInCategory.OST_Walls);

                // Apply the filter to the elements in the active document
                // Use shortcut WhereElementIsNotElementType() to find wall instances only
                FilteredElementCollector collector = new FilteredElementCollector(doc);
                IList<Element> walls =
                collector.WherePasses(filter).WhereElementIsNotElementType().ToElements();
                String prompt = "The walls in the current document are:\n";
                const double _feet_to_meters = 0.3048;
                foreach (Element e in walls)
                {
                    BoundingBoxXYZ bbox = e.get_BoundingBox(null);

                    List<Solid> solids = new List<Solid>();
                    Options options = new Options();
                    options.DetailLevel = ViewDetailLevel.Fine;
                    GeometryElement geomElem = e.get_Geometry(options);
                    String dat = "";
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
                        }*/
                    }
                    bbox.Min = bbox.Min * _feet_to_meters;
                    bbox.Max = bbox.Max * _feet_to_meters;

                    prompt += bbox.Min.X + ", " + bbox.Max.X + "," + bbox.Min.Z + ", " + bbox.Max.Z + "," +
                         bbox.Min.Y + ", " + bbox.Max.Y + "\n";
                    world.addObstacle(bbox.Min.X, bbox.Max.X, bbox.Min.Z, bbox.Max.Z, bbox.Min.Y, bbox.Max.Y);
                }

                SteerSuite steersuite = new SteerSuite();
                steersuite.init(world);

                // steersuite.simulate();

                CMAOptimize steerFunc = new CMAOptimize(world, steersuite);

                steerFunc.optimize();

                steersuite.finish();
                System.IO.StreamWriter file = new System.IO.StreamWriter("D:\\test.txt");
                file.WriteLine(prompt);

                file.Close();
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
