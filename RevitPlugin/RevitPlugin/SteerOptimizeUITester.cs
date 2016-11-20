using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Autodesk.Revit.UI;
using Autodesk.Revit.DB;
using Autodesk.Revit.DB.Events;
using Autodesk.Revit.DB.Analysis;
using Autodesk.Revit.UI.Selection;
using Autodesk.Revit.UI.Events;
// using steerSuiteAdapterCSharp.SimWorld;



namespace RevitPlugin
{   
        /// <summary>
        /// Converts Revit Application Pointer to IWin32Window that
        /// can be passed to the Windows Form to set Revit Application as
        /// Owner of the Windows Form.
        /// </summary>
        public class WindowHandler : System.Windows.Forms.IWin32Window
    {        
        public WindowHandler(IntPtr handler)
        {
            _handler = handler;
        }

        public IntPtr Handle
        {
            get
            {
                return _handler;
            }
        }
        IntPtr _handler;
    }
    
    /// <summary>
    /// Implements the Revit add-in interface IExternalCommand
    /// </summary>
    [Autodesk.Revit.Attributes.Transaction(Autodesk.Revit.Attributes.TransactionMode.Automatic)]
    [Autodesk.Revit.Attributes.Regeneration(Autodesk.Revit.Attributes.RegenerationOption.Manual)]
    [Autodesk.Revit.Attributes.Journaling(Autodesk.Revit.Attributes.JournalingMode.NoCommandData)]
    class SteerOptimizeUITester : Autodesk.Revit.UI.IExternalCommand
    {
        private System.Windows.Forms.Form r_UI_Form, initialForm;
        public static SteerOptimizeUITester thisApp = null;
        private bool runMode = true; //false for DEV and true for User-Study
        private bool runModeA = false;
        private UIApplication uiApp;
        private Document doc;

        public Result OnShutdown(UIControlledApplication application)
        {
            if (r_UI_Form != null && r_UI_Form.Visible)
            {
                r_UI_Form.Close();
            }

            runMode = false;
            return Result.Succeeded;
        }

        public Result OnStartup(UIControlledApplication application)
        {
            r_UI_Form = null;   // no dialog needed yet; the command will bring it
            thisApp = this;  // static access to this application instance

            return Result.Succeeded;
        }

        //   The external command invokes this on the end-user's request
        public void ShowForm(Document doc, UIApplication uiapp)
        {
            // If we do not have a dialog yet, create and show it
            if (r_UI_Form == null || r_UI_Form.IsDisposed)
            {
                // A new handler to handle request posting by the dialog
                RevitUI handler = new RevitUI();

                //Register the Idling event
                uiapp.Idling  += new EventHandler<IdlingEventArgs>(handler.OnIdling);

                // External Event for the dialog to use (to post requests)
                ExternalEvent exEvent = ExternalEvent.Create(handler);

                //Gets the Revit Window Pointer to set it as Parent of the Windows Form
                System.Diagnostics.Process p = System.Diagnostics.Process.GetCurrentProcess();
                IntPtr revit_window_pointer = p.MainWindowHandle;

                // We give the objects to the new dialog;
                // The dialog becomes the owner responsible for disposing them, eventually.
                if(runModeA)
                    r_UI_Form = handler.ShowUserStudyAForm(doc, exEvent);
                else
                    r_UI_Form = handler.InitializeForm(doc, exEvent, runMode);

                if(r_UI_Form != null)
                    r_UI_Form.Show(new WindowHandler(revit_window_pointer));
            }
        }

        public void dev_Click(object sender, EventArgs e)
        {
            runMode = false; 
            if (initialForm != null && initialForm.Visible)
            {
                initialForm.Close();
            }
        }

        public void userStudyB_Click(object sender, EventArgs e)
        {
            runMode = true;
            if (initialForm != null && initialForm.Visible)
            {
                initialForm.Close();
            }            
        }

        public void userStudyA_Click(object sender, EventArgs e)
        {
            this.runModeA = true;
            if (initialForm != null && initialForm.Visible)
            {
                initialForm.Close();
            }
        }

        /// <summary>
        /// Decides whetehr to show the dev UI or user-study UI
        /// </summary>
        /// <param name="doc"></param>
        /// <param name="uiapp"></param>
        public System.Windows.Forms.Form ShowInitialForm(Document doc, UIApplication uiapp)
        {
            this.uiApp = uiapp;
            this.doc = doc;
            
            // We give the objects to the new dialog;
            // The dialog becomes the owner responsible for disposing them, eventually.                

            initialForm = new System.Windows.Forms.Form();

            System.Windows.Forms.Button userStudyA = new System.Windows.Forms.Button();
            System.Windows.Forms.Button userStudyB = new System.Windows.Forms.Button();
            System.Windows.Forms.Button dev = new System.Windows.Forms.Button();


            userStudyB.Location = new System.Drawing.Point(310, 80);
            userStudyB.Name = "userStudyB";
            userStudyB.Size = new System.Drawing.Size(260, 40);
            userStudyB.TabIndex = 2;
            userStudyB.Text = "Study-B";
            userStudyB.UseVisualStyleBackColor = true;
            userStudyB.Font = new System.Drawing.Font("Microsoft Sans Serif", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            userStudyB.Click += new System.EventHandler(userStudyB_Click);

            userStudyA.Location = new System.Drawing.Point(30, 80);
            userStudyA.Name = "userStudA";
            userStudyA.Size = new System.Drawing.Size(260, 40);
            userStudyA.TabIndex = 1;
            userStudyA.Text = "Study-A";
            userStudyA.UseVisualStyleBackColor = true;
            userStudyA.Font = new System.Drawing.Font("Microsoft Sans Serif", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            userStudyA.Click += new System.EventHandler(userStudyA_Click);

            dev.Location = new System.Drawing.Point(590, 80);
            dev.Name = "dev";
            dev.Size = new System.Drawing.Size(260, 40);
            dev.TabIndex = 3;
            dev.Text = "Development";
            dev.UseVisualStyleBackColor = true;
            dev.Font = new System.Drawing.Font("Microsoft Sans Serif", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            dev.Click += new System.EventHandler(dev_Click);

            initialForm.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            initialForm.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            initialForm.ClientSize = new System.Drawing.Size(880, 200);
            initialForm.ControlBox = true;
            initialForm.Controls.Add(dev);
            initialForm.Controls.Add(userStudyB);
            initialForm.Controls.Add(userStudyA);
            initialForm.MaximizeBox = false;
            initialForm.MinimizeBox = true;
            initialForm.Name = "initialForm";
            initialForm.ShowIcon = false;
            initialForm.StartPosition = System.Windows.Forms.FormStartPosition.Manual;
            initialForm.Text = "CODE::Revit Plugin";            
            initialForm.ResumeLayout(false);
            initialForm.PerformLayout();

            System.Drawing.Size s = System.Windows.Forms.Screen.PrimaryScreen.WorkingArea.Size;
            initialForm.Location = new System.Drawing.Point((int)(s.Width / 2) - (initialForm.Width/2), (int)(s.Height / 2) - (initialForm.Height/2));
            return initialForm;
        }

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
                //string pathToSteerSuite = "C:\\Users\\usman\\Downloads\\Crowds\\SteerSuite\\build\\bin\\";
                Properties props = new Properties("../revit_properties.ini");
                string pathToSteerSuite = props.get("SteerSuitePath");
                string path = Environment.GetEnvironmentVariable("PATH");
                path += System.IO.Path.PathSeparator.ToString() + pathToSteerSuite;
                Environment.SetEnvironmentVariable("PATH", path);

                Autodesk.Revit.UI.Selection.Selection sel;
                sel = cmdData.Application.ActiveUIDocument.Selection;
                UIApplication uiApp = cmdData.Application;                
                Document doc = uiApp.ActiveUIDocument.Document;

                //Gets the Revit Window Pointer to set it as Parent of the Windows Form
                System.Diagnostics.Process p = System.Diagnostics.Process.GetCurrentProcess();
                IntPtr revit_window_pointer = p.MainWindowHandle;
                
                //Initiates Plugin UI and Register an External Event                                
                System.Windows.Forms.Form initialUI = ShowInitialForm(doc, uiApp);
                initialUI.ShowDialog(new WindowHandler(revit_window_pointer));
                ShowForm(doc, uiApp);

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

                //SteerSuite steersuite = new SteerSuite();
                //steersuite.init(world);

                //// steersuite.simulate();

                //CMAOptimize steerFunc = new CMAOptimize(world, steersuite);

                //steerFunc.optimize();

                //steersuite.finish();
                //System.IO.StreamWriter file = new System.IO.StreamWriter("D:\\test.txt");
                //file.WriteLine(prompt);
                
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
