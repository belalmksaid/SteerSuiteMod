using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Drawing;
using System.Windows.Forms;
using Autodesk.Revit.UI;
using Autodesk.Revit.DB;
using System.IO;
using Autodesk.Revit.DB.Analysis;
using Autodesk.Revit.UI.Selection;
using Autodesk.Revit.UI.Events;
using System.Diagnostics;
using System.Xml;
using System.Threading;

namespace RevitPlugin
{
    /// <summary>
    /// Class implementing ISelectionFilter interface to create a custom Filter
    /// class to ensure that only allowed Element and Reference types can be 
    /// selected from the REVIT Document. 
    /// </summary>
    public class SelectionFilter1 : Autodesk.Revit.UI.Selection.ISelectionFilter
    {
        public bool AllowElement(Element elem)
        {
            // Just Walls for now
            if (elem is Wall) return true;
            if (elem.Category.Id.IntegerValue == (int)BuiltInCategory.OST_WallsDefault) return true;
            if (elem.Category.Id.IntegerValue == (int)BuiltInCategory.OST_WallsInsulation) return true;
            if (elem.Category.Id.IntegerValue == (int)BuiltInCategory.OST_WallsStructure) return true;

            return false;
        }

        public bool AllowReference(Reference refer, XYZ pos)
        {
            if (refer == null) return false;
            //return refer is Wall;
            return true;
        }
    }

    struct WallsGeometry
    {
        public WallsGeometry(ElementId id, XYZ p0, XYZ p1)
        {
            this.id = id;
            this.p0 = p0;
            this.p1 = p1;
        }

        public ElementId id;
        public XYZ p0;
        public XYZ p1;
    }

    /// <summary>
    /// Class Implementing the complete SteerSuite-Revit Plugin UI.
    /// It contains the declaration and definitaion of all the components 
    /// in the plugin UI along with their respective call_back functions.
    /// 
    /// REVIT API doesn't allow an external application to operate 
    /// REVIT from outside. Changing the UI model of REVIT is one of such example.
    /// To handle this, REVIT API provides an Interface called "IExternalEventHandler".
    /// Using this, an external application can invoke an External Event inside REVIT to get
    /// into the REVIT context and do the desire tasks.
    /// 
    /// So this class also implements an iterface which allow the user to invoke 
    /// an Event from from external application which is all the current Class to access all the REVIT
    /// content fron the Execute() method. 
    /// </summary>

    class RevitUI : IExternalEventHandler
    {
        // User-Study: START

        private int partcipantId = -1;
        private string pathToSteerSuite = "";
        private string pathToResults = "";
        private string pathToParticipant = "";
        private string DEFAULT_LAYOUT_FILE_PREFIX = "Graph-Default-";
        private string TIMER_FILENAME = "TimeInfo";
        private string PARAM_FILENAME = "-params-";
        private string GRAPH_FILETYPE = ".graph";
        private int participantTurns = 1;
        bool userstudyMode = false;
        private const string RESULT_DIR = "Results";
        System.Diagnostics.Stopwatch watch, watchIterationOpt, watchShared;
        bool firstIteration = true;
        bool iterationTimeEnabled = false;
        bool userstudy_A = false;
        System.Windows.Forms.Form usAForm;
        OptimizationParameters studyParams;

        // User-Study: END

        //Opt params: START

        private const string OPT_OUTPUT_FILETYPE = ".csv";
        private const string DIVERSITY_OPT_FILE_PREFIX = "diversityOpt-";
        private string REVIT_CONFIG_FILE = "config_revit_run.xml";
        private string REVIT_CONFIG_FILE_2 = "../config_revit.xml";
        OptimizationConfiguration optConfig;
        bool finishPressed = false;
        //Opt params: END

        private ExternalEvent m_ExEvent;        
        public Autodesk.Revit.DB.Document doc;       
        private bool openNewDocument = false;
        private string loadGraphFile = "";

        private IList<Element> user_selected_elements;
        private IList<Element> user_selected_elements_all;

        IList<XYZ> verts;
        IList<List<int>> edges;
        List<ElementId> eIds;
        List<uint> bestNodesIndexes;
        List<Graph> bestNodes;
        OptimizationParameters params_;
        bool constraintsPresent;
        ICollection<ElementId> elementsCopy;
        List<WallsGeometry> wallsGeometry;

        string graphFileName, steerSuiteBinDirectory;

        const double _feet_to_meters = 0.3048;
        const double _meters_to_feet = 3.28084;
        const double TOLERANCE = 0.01;
        
        Graph graph, graph2;

        XYZ worldBoundMin, worldBoundMax, regionBoundMin, regionBoundMax;
        bool spaceRegionPresent;

        int _schemaId = -1;
        bool isAnalysisDisplayStyle;
        bool isHeatmap;
        bool isSpaceGraph;
        bool deleteSpaceGraph;
        AnalysisDisplayStyle analysisDisplayStyle;
        SpatialFieldManager sfm;
        List<int> idx; //to store SpatialFieldPrimitive index
        List<PlanarFace> topFaces;
        Floor f;
        bool deleteFloor, finishFloor;
        CurveArray floorBoundaries;
        XYZ regionBoundMinFloor, regionBoundMaxFloor;

        XYZ dirVector_0, dirVector_1;
        bool UB;
        List<Floor> constraintRegions;
        List<List<XYZ>> upperBounds, lowerBounds;
        List<Floor> referenceRegions, queryRegions;
        List<List<Point>> referenceRegionsPoints, queryRegionsPoints;
        bool clearFloorRegions, clearConstraintRegions;

        public int callCounter = 0;
        public System.Drawing.Point lastCursor = new System.Drawing.Point(0, 0);
        SteerSuite steersuite, steersuiteLayout;
        List<SteerSuite> steersuiteLayouts;
        List<double> steersuiteLayoutsMetrics;
        List<int> steersuiteLayoutsIndexes;

        double upperBoundValue, lowerBoundValue;
        bool isRotational;
        XYZ rotationOrigin;
        double rotationRadius;
        double lastAngle;

        double minDeg = 0;
        double maxDeg = 1;

        /*** FORM ITEMS START ***/

        public System.Windows.Forms.Form UIForm;         
        
        public System.Windows.Forms.Label labIterationCount;
        public System.Windows.Forms.Label labOptimizer;
        public System.Windows.Forms.Label labBestResults;        
        public System.Windows.Forms.Label labSelectMetic;
        public System.Windows.Forms.Label labBound;

        public System.Windows.Forms.RadioButton radioRegionQuery;
        public System.Windows.Forms.RadioButton radioRegionReference;       

        public System.Windows.Forms.TrackBar trackbarBound;

        public System.Windows.Forms.CheckBox chkShowHeatmap;        

        public System.Windows.Forms.TextBox tbIterationCount;

        public System.Windows.Forms.Button ok;
        public System.Windows.Forms.Button btnRotation;
        public System.Windows.Forms.Button btnSelectSpaceRegion;
        public System.Windows.Forms.Button btnResetSpaceRegion;
        public System.Windows.Forms.Button btnFinishOperations;
        public System.Windows.Forms.Button btnCloseTool;
        public System.Windows.Forms.Button btnConstraintsDone;
        public System.Windows.Forms.Button btnTranslation;
        public System.Windows.Forms.Button btnCancelConstraint;

        public System.Windows.Forms.GroupBox groupSpaceAnalysis;
        public System.Windows.Forms.GroupBox groupOptimization;
        
        public System.Windows.Forms.ComboBox comboOptimizer;
        public System.Windows.Forms.ComboBox comboBestResults;        
        public System.Windows.Forms.ComboBox comboMetricSelection;

        public System.Windows.Forms.MainMenu menuMain;
        public System.Windows.Forms.MenuItem menuParams;
        public System.Windows.Forms.MenuItem menuDoc;
        public System.Windows.Forms.MenuItem menuParamsLoad;
        public System.Windows.Forms.MenuItem menuParamsWrite;
        public System.Windows.Forms.MenuItem docLoad;
        public System.Windows.Forms.MenuItem docSave;

        public DoubleVector axis;

        public List<ModelCurve> spaceLines;
        public List<List<XYZ>> spaceLinesPoints;
        public ICollection<ElementId> spaceGraphLines;
        public List<List<UV>> spaceNodes;
        public List<List<ValueAtPoint>> spaceNodesDegrees;
        
        private bool isHeatmapClicked;

        /*** FORM ITEMS END ***/

        private System.Windows.Forms.Form participantForm;
        private System.Windows.Forms.TextBox participantTb;
        public void done_Click(object sender, EventArgs e)
        {
            if (Int32.TryParse(participantTb.Text, out this.partcipantId))
            {
                if (participantForm != null && participantForm.Visible)
                {
                    participantTb.Text = "";
                    participantForm.Close();
                }
            }
            else
            {
                TaskDialog.Show("uDOME", " Please Enter an Integer Participant ID!");
                participantTb.Text = "";
            }
        }

        public System.Windows.Forms.Form CollectParticipantId()
        {
            participantForm = new System.Windows.Forms.Form();
            participantForm.Text = "Please Enter Participant ID";
            participantForm.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            participantForm.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            participantForm.HelpButton = participantForm.MinimizeBox = participantForm.MaximizeBox = false;
            participantForm.ShowIcon = participantForm.ShowInTaskbar = false;
            //participantForm.TopMost = true;
            participantForm.ClientSize = new System.Drawing.Size(500, 250);
            Size s = Screen.PrimaryScreen.WorkingArea.Size;
            participantForm.Location = new System.Drawing.Point((int)(s.Width / 2) - (participantForm.Size.Width / 2), (int)(s.Height / 2) - (participantForm.Size.Height / 2));

            int margin = 5;
            Size size = participantForm.ClientSize;

            participantTb = new System.Windows.Forms.TextBox();
            participantTb.TextAlign = HorizontalAlignment.Right;
            participantTb.TabIndex = 1;
            participantTb.Height = 20;
            participantTb.Width = size.Width - 2 * margin;
            participantTb.Location = new System.Drawing.Point(margin, margin);
            participantTb.Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right;
            participantForm.Controls.Add(participantTb);

            Button done = new Button();
            done.Text = "OK";
            done.Click += new System.EventHandler(done_Click);
            done.Size = new System.Drawing.Size(250, 40);
            done.TabIndex = 2;
            done.UseVisualStyleBackColor = true;
            done.Font = new System.Drawing.Font("Microsoft Sans Serif", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            done.Location = new System.Drawing.Point((participantForm.Width / 2) - (done.Size.Width / 2), participantForm.Height / 3);           
            participantForm.Controls.Add(done);
            
            return participantForm;
        }

        /// <summary>
        /// Initializes the data structures
        /// </summary>
        private void InitializeCode()
        {
            this.deleteFloor = false;
            this.finishFloor = false;
            this.floorBoundaries = new CurveArray();

            this.constraintRegions = new List<Floor>();
            this.referenceRegions = new List<Floor>();
            this.queryRegions = new List<Floor>();
            this.referenceRegionsPoints = new List<List<Point>>();
            this.queryRegionsPoints = new List<List<Point>>();
            this.clearFloorRegions = false;
            this.clearConstraintRegions = false;

            this.verts = new List<XYZ>();
            this.edges = new List<List<int>>();
            this.eIds = new List<ElementId>();
            this.bestNodesIndexes = new List<uint>();
            this.bestNodes = new List<Graph>();
            this.params_ = new OptimizationParameters();
            this.constraintsPresent = false;
            this.wallsGeometry = new List<WallsGeometry>();

            this.steersuiteLayouts = new List<SteerSuite>();
            this.steersuiteLayoutsMetrics = new List<double>();
            this.steersuiteLayoutsIndexes = new List<int>();
            this.steersuiteLayout = null;
            Properties props = new Properties("../revit_properties.ini");
            this.pathToSteerSuite = props.get("SteerSuitePath");

            string pathToBuild = Path.GetFullPath(Path.Combine(this.pathToSteerSuite, @"..\"));
            REVIT_CONFIG_FILE = pathToBuild + REVIT_CONFIG_FILE;

            /// This is just to get the configuration setting from the config file.
            SteerSuite steerSuite_ = new SteerSuite();
            steerSuite_.setConfigFileName(REVIT_CONFIG_FILE);
            SimWorld w = new SimWorld();
            steerSuite_.init(w);
            OptionDictionaryMap options = steerSuite_.getModuleOptions("steeropt");
            // parse command line options
            this.optConfig = new OptimizationConfiguration(options);
            // steerSuite_.finish();

            this.graphFileName = this.pathToSteerSuite + "test.txt";
            
            this.steerSuiteBinDirectory = this.pathToSteerSuite;

            this.graph = new Graph();
            this.graph2 = new Graph();

            this.axis = new DoubleVector();
            this.axis.Add(1);
            this.axis.Add(0);
            this.axis.Add(0);

            this.upperBoundValue = 1;
            this.lowerBoundValue = -1;

            this.dirVector_0 = new XYZ(0, 0, 0);
            this.dirVector_1 = new XYZ(0, 0, 0);
            this.UB = true;

            this.isRotational = false;
            this.rotationOrigin = new XYZ(0, 0, 0);
            this.rotationRadius = 0.0;
            this.lastAngle = 0.0;

            this.upperBounds = new List<List<XYZ>>();
            this.lowerBounds = new List<List<XYZ>>();

            this.user_selected_elements = new List<Element>();
            this.user_selected_elements_all = new List<Element>();
            this.elementsCopy = new List<ElementId>();
            this.isAnalysisDisplayStyle = false;
            this.isHeatmap = false;
            this.idx = new List<int>();
            this.isSpaceGraph = false;
            this.deleteSpaceGraph = false;
            this.topFaces = new List<PlanarFace>();
        }

        /// <summary>
        /// Initializes the participant's directory structure to store user-study data
        /// </summary>
        /// <param name="doc"></param>
        /// <param name="filename"></param>
        private void InitializeParticipant(Document doc)
        {            
            //Gets the Revit Window Pointer to set it as Parent of the Windows Form
            System.Diagnostics.Process p = System.Diagnostics.Process.GetCurrentProcess();
            IntPtr revit_window_pointer = p.MainWindowHandle;

            System.Windows.Forms.Form participant = CollectParticipantId();
            participant.ShowDialog(new WindowHandler(revit_window_pointer));

            string pathToBuild = Path.GetFullPath(Path.Combine(this.pathToSteerSuite, @"..\"));
            this.pathToResults = pathToBuild + RESULT_DIR;

            if (!System.IO.Directory.Exists(this.pathToResults))
            {
                System.IO.DirectoryInfo dirInfo = System.IO.Directory.CreateDirectory(this.pathToResults);
            }

            this.pathToParticipant = this.pathToResults + System.IO.Path.DirectorySeparatorChar + "P" + this.partcipantId + "-" + DateTime.Now.ToString("MM.dd.yyyy-hh.mm.ss");

            if (!System.IO.Directory.Exists(this.pathToParticipant))
            {
                System.IO.DirectoryInfo dirInfo = System.IO.Directory.CreateDirectory(this.pathToParticipant);
            }
            string filename = this.pathToParticipant + Path.DirectorySeparatorChar +
                    this.DEFAULT_LAYOUT_FILE_PREFIX + Path.GetFileNameWithoutExtension(doc.PathName) + this.GRAPH_FILETYPE;
            this.StoreGraph(doc, filename);
        }

        /// <summary>
        /// Initializes the REVIT UI and SteerSuite controls
        /// </summary>
        /// <param name="doc"></param>
        /// <param name="m_ExEvent"></param>
        /// <param name="usMode"></param>
        /// <returns></returns>
        public System.Windows.Forms.Form InitializeForm(Document doc, ExternalEvent m_ExEvent, bool usMode)
        {
            this.doc = doc;
            this.m_ExEvent = m_ExEvent;
            //this.m_Handler = m_Handler;

            InitializeCode();

            // User-Study mode
            if (usMode)
            {
                this.userstudyMode = usMode;                

                InitializeParticipant(this.doc);
            }

            this.UIForm = new System.Windows.Forms.Form();            
            this.ok = new System.Windows.Forms.Button();
            this.groupSpaceAnalysis = new System.Windows.Forms.GroupBox();
            this.btnTranslation = new System.Windows.Forms.Button();
            this.btnRotation = new System.Windows.Forms.Button();
            this.btnSelectSpaceRegion = new System.Windows.Forms.Button();
            this.btnResetSpaceRegion = new System.Windows.Forms.Button();
            this.chkShowHeatmap = new System.Windows.Forms.CheckBox();
            this.btnCancelConstraint = new System.Windows.Forms.Button();
            this.comboMetricSelection = new System.Windows.Forms.ComboBox();
            this.labSelectMetic = new System.Windows.Forms.Label();
            this.btnFinishOperations = new System.Windows.Forms.Button();
            this.btnCloseTool = new System.Windows.Forms.Button();
            this.btnConstraintsDone = new System.Windows.Forms.Button();
            this.groupOptimization = new System.Windows.Forms.GroupBox();
            this.comboOptimizer = new System.Windows.Forms.ComboBox();
            this.comboBestResults = new System.Windows.Forms.ComboBox();            
            this.tbIterationCount = new System.Windows.Forms.TextBox();
            this.labIterationCount = new System.Windows.Forms.Label();
            this.labOptimizer = new System.Windows.Forms.Label();
            this.labBestResults = new System.Windows.Forms.Label();            
            this.radioRegionQuery = new RadioButton();
            this.radioRegionReference = new RadioButton();            
            this.labBound = new Label();
            this.trackbarBound = new System.Windows.Forms.TrackBar();
            this.spaceRegionPresent = false;
            this.spaceLines = new List<ModelCurve>();
            this.spaceLinesPoints = new List<List<XYZ>>();
            this.spaceGraphLines = new List<ElementId>();
            this.spaceNodes = new List<List<UV>>();
            this.spaceNodesDegrees = new List<List<ValueAtPoint>>();
            this.isHeatmapClicked = false;
            this.groupSpaceAnalysis.SuspendLayout();
            this.groupOptimization.SuspendLayout();
            this.menuMain = new MainMenu();
            this.menuParams = new MenuItem("&Parameters");
            this.menuParamsLoad = new MenuItem("&Load Params");
            this.menuParamsWrite = new MenuItem("&Save params");
            this.menuDoc = new MenuItem("&Document");
            this.docLoad = new MenuItem("&Load Graph");
            this.docSave = new MenuItem("&Save Graph");
            this.UIForm.SuspendLayout();
            // 
            // btnTranslation
            // 
            this.btnTranslation.AutoSize = true;
            this.btnTranslation.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnTranslation.Location = new System.Drawing.Point(12, 17);
            this.btnTranslation.Name = "btnTranslation";
            this.btnTranslation.Size = new System.Drawing.Size(122, 25);
            this.btnTranslation.TabIndex = 9;
            this.btnTranslation.Text = "Translation";
            this.btnTranslation.UseVisualStyleBackColor = true;
            this.btnTranslation.Click += new System.EventHandler(this.btnTranslation_Click);            
            // 
            // btnRotation
            // 
            this.btnRotation.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnRotation.Location = new System.Drawing.Point(145, 17);
            this.btnRotation.Name = "btnRotation";
            this.btnRotation.Size = new System.Drawing.Size(122, 25);
            this.btnRotation.TabIndex = 11;
            this.btnRotation.Text = "Rotation";
            this.btnRotation.UseVisualStyleBackColor = true;
            this.btnRotation.Click += new System.EventHandler(this.btnRotation_Click);

            
            ////
            //// radioSelectionFree
            ////            
            //this.radioSelectionFree.Location = new System.Drawing.Point(18, 57);
            //this.radioSelectionFree.Size = new System.Drawing.Size(120, 19);
            //this.radioSelectionFree.Name = "radioSelectionFree";
            //this.radioSelectionFree.Text = "Draw Rectangle";
            //this.radioSelectionFree.TabIndex = 5;
            //this.radioSelectionFree.CheckedChanged += new System.EventHandler(this.radioSelection_Click);            
            //
            // Set up the trackbarBound.
            //
            this.trackbarBound.Location = new System.Drawing.Point(6, 49);
            this.trackbarBound.Size = new System.Drawing.Size(120, 22);
            this.trackbarBound.Name = "trackbarBound";
            this.trackbarBound.Minimum = 1;
            this.trackbarBound.Maximum = 100;
            this.trackbarBound.TickFrequency = 1;
            this.trackbarBound.LargeChange = 10;
            this.trackbarBound.SmallChange = 10;
            this.trackbarBound.TickStyle = TickStyle.None;
            this.trackbarBound.TabIndex = 0;
            this.trackbarBound.Value = 1;
            this.trackbarBound.Visible = false;
            this.trackbarBound.Scroll += new System.EventHandler(this.trackbarBound_Scroll);
            // 
            // labBound
            // 
            this.labBound.Location = new System.Drawing.Point(6, 52);
            this.labBound.Size = new System.Drawing.Size(125, 19);
            this.labBound.Name = "labBound";
            this.labBound.Text = "Upper Bound Constraint";
            this.labBound.TabIndex = 1;
            this.labBound.Hide();
            // 
            // tbRotation
            // 
            //this.tbRotation.Location = new System.Drawing.Point(140, 28);
            //this.tbRotation.Name = "tbRotation";
            //this.tbRotation.Size = new System.Drawing.Size(130, 22);
            //this.tbRotation.TabIndex = 3;   
            // 
            // btnCancelConstraint
            // 
            this.btnCancelConstraint.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnCancelConstraint.Location = new System.Drawing.Point(175, 15);
            this.btnCancelConstraint.Name = "btnCancelConstraint";
            this.btnCancelConstraint.Size = new System.Drawing.Size(75, 25);
            this.btnCancelConstraint.TabIndex = 2;
            this.btnCancelConstraint.Text = "Cancel";
            this.btnCancelConstraint.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.btnCancelConstraint.UseVisualStyleBackColor = true;
            this.btnCancelConstraint.Click += new System.EventHandler(this.btnCancelConstraint_Click);
            this.btnCancelConstraint.Hide();
            // 
            // btnConstraintsDone
            // 
            this.btnConstraintsDone.Location = new System.Drawing.Point(175, 42);
            this.btnConstraintsDone.Name = "btnConstraintsDone";
            this.btnConstraintsDone.Size = new System.Drawing.Size(75, 25);
            this.btnConstraintsDone.TabIndex = 18;
            this.btnConstraintsDone.Text = "Set";
            this.btnConstraintsDone.UseVisualStyleBackColor = true;
            this.btnConstraintsDone.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnConstraintsDone.Click += new System.EventHandler(this.btnConstraintsDone_Click);
            this.btnConstraintsDone.Hide();
            // 
            // groupSpaceAnalysis
            // 
            this.groupSpaceAnalysis.Controls.Add(this.btnTranslation);
            this.groupSpaceAnalysis.Controls.Add(this.btnRotation);
            this.groupSpaceAnalysis.Anchor = System.Windows.Forms.AnchorStyles.Top;            
            this.groupSpaceAnalysis.Controls.Add(this.trackbarBound);
            //this.groupSpaceAnalysis.Controls.Add(this.tbTranslationUB);
            this.groupSpaceAnalysis.Controls.Add(this.btnConstraintsDone);            
            this.groupSpaceAnalysis.Controls.Add(this.labBound);
            //this.groupSpaceAnalysis.Controls.Add(this.tbTranslationLB);            
            //this.groupSpaceAnalysis.Controls.Add(this.tbRotation);            
            //this.groupSpaceAnalysis.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.groupSpaceAnalysis.Location = new System.Drawing.Point(12, 50);
            this.groupSpaceAnalysis.Name = "groupSpaceAnalysis";
            this.groupSpaceAnalysis.Size = new System.Drawing.Size(276, 200);
            this.groupSpaceAnalysis.TabIndex = 7;
            this.groupSpaceAnalysis.TabStop = false;
            this.groupSpaceAnalysis.Text = "Analysis";
            this.groupSpaceAnalysis.Controls.Add(this.labSelectMetic);
            this.groupSpaceAnalysis.Controls.Add(this.btnSelectSpaceRegion);
            this.groupSpaceAnalysis.Controls.Add(this.btnResetSpaceRegion);
            this.groupSpaceAnalysis.Controls.Add(this.chkShowHeatmap);
            this.groupSpaceAnalysis.Controls.Add(this.btnCancelConstraint);
            this.groupSpaceAnalysis.Controls.Add(this.comboMetricSelection);
            this.groupSpaceAnalysis.Controls.Add(this.radioRegionQuery);
            this.groupSpaceAnalysis.Controls.Add(this.radioRegionReference);
            // 
            // labSelectMetic
            // 
            this.labSelectMetic.AutoSize = true;
            this.labSelectMetic.Font = new System.Drawing.Font("Microsoft Sans Serif", 11.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labSelectMetic.Location = new System.Drawing.Point(6, 93);
            this.labSelectMetic.Name = "labSelectMetic";
            this.labSelectMetic.Size = new System.Drawing.Size(107, 18);
            this.labSelectMetic.TabIndex = 23;
            this.labSelectMetic.Text = "Select Metric";
            // 
            // btnSelectSpaceRegion
            // 
            //this.btnSelectSpaceRegion.Font = new System.Drawing.Font("Microsoft Sans Serif", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnSelectSpaceRegion.Location = new System.Drawing.Point(139, 125);
            this.btnSelectSpaceRegion.Name = "btnSelectSpaceRegion";
            this.btnSelectSpaceRegion.Size = new System.Drawing.Size(122, 27);
            this.btnSelectSpaceRegion.TabIndex = 14;
            this.btnSelectSpaceRegion.Text = "Select Region";
            this.btnSelectSpaceRegion.UseVisualStyleBackColor = true;
            this.btnSelectSpaceRegion.Click += new System.EventHandler(this.btnSelectSpaceRegion_Click);

            if (this.userstudyMode)
                this.btnSelectSpaceRegion.Enabled = false;
            // 
            // comboMetricSelection
            // 
            //this.btnShowSpaceGraph.Font = new System.Drawing.Font("Microsoft Sans Serif", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.comboMetricSelection.Location = new System.Drawing.Point(140, 92);
            this.comboMetricSelection.Name = "comboMetricSelection";
            this.comboMetricSelection.Size = new System.Drawing.Size(120, 23);
            this.comboMetricSelection.TabIndex = 22;
            this.comboMetricSelection.Text = "Metric";
            this.comboMetricSelection.DropDownStyle = ComboBoxStyle.DropDownList;

            //foreach (MetricName mn in Enum.GetValues(typeof(MetricName)))
            //{
            //    if (mn.ToString().Equals("MeanDegree"))
            //    {
                    this.comboMetricSelection.Items.Add("Visibility");
                //}
                //else if (mn.ToString().Equals("MeanTreeEntropy"))
                //{
                    this.comboMetricSelection.Items.Add("Organization");
                //}
                //else if (mn.ToString().Equals("MeanTreeDepth"))
                //{
                    this.comboMetricSelection.Items.Add("Accessibility");
            //    }
            //}

            this.comboMetricSelection.Items.Add("Weighted Combination");

            this.comboMetricSelection.SelectedIndex = 0;
            this.comboMetricSelection.SelectedIndexChanged += new System.EventHandler(this.comboMetricSelection_Click);

            if (this.userstudy_A)
                this.comboMetricSelection.Enabled = false;

            // 
            // btnResetSpaceRegion
            //             
            this.btnResetSpaceRegion.Location = new System.Drawing.Point(139, 162);
            this.btnResetSpaceRegion.Name = "btnResetSpaceRegion";
            this.btnResetSpaceRegion.Size = new System.Drawing.Size(122, 25);
            this.btnResetSpaceRegion.TabIndex = 5;
            this.btnResetSpaceRegion.Text = "Reset Regions";
            this.btnResetSpaceRegion.UseVisualStyleBackColor = true;
            this.btnResetSpaceRegion.Click += new System.EventHandler(this.btnResetSpaceRegion_Click);

            if (this.userstudyMode)
                this.btnResetSpaceRegion.Enabled = false;

            //
            // radioRegionQuery
            //            
            this.radioRegionQuery.Location = new System.Drawing.Point(6, 115);
            this.radioRegionQuery.Size = new System.Drawing.Size(120, 19);
            this.radioRegionQuery.Name = "radioRegionQuery";
            this.radioRegionQuery.Text = "Query";
            this.radioRegionQuery.TabIndex = 20;
            this.radioRegionQuery.Checked = true;
            this.radioRegionQuery.CheckedChanged += new System.EventHandler(this.radioSelection_Click);

            if (this.userstudyMode)
                this.radioRegionQuery.Enabled = false;
            //
            // radioRegionReference
            //            
            this.radioRegionReference.Location = new System.Drawing.Point(6, 135);
            this.radioRegionReference.Size = new System.Drawing.Size(127, 19);
            this.radioRegionReference.Name = "radioRegionReference";
            this.radioRegionReference.Text = "Reference";
            this.radioRegionReference.TabIndex = 4;
            this.radioRegionReference.CheckedChanged += new System.EventHandler(this.radioSelection_Click);

            if (this.userstudyMode)
                this.radioRegionReference.Enabled = false;
            // 
            // chkShowHeatmap
            // 
            //this.btnShowSpaceGraph.Font = new System.Drawing.Font("Microsoft Sans Serif", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.chkShowHeatmap.Location = new System.Drawing.Point(6, 162);
            this.chkShowHeatmap.Name = "chkShowHeatmap";
            this.chkShowHeatmap.Size = new System.Drawing.Size(122, 25);
            this.chkShowHeatmap.TabIndex = 15;
            this.chkShowHeatmap.Text = "Heatmap";
            this.chkShowHeatmap.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.chkShowHeatmap.UseVisualStyleBackColor = true;
            this.chkShowHeatmap.Appearance = Appearance.Button;
            this.chkShowHeatmap.Click += new System.EventHandler(this.chkShowHeatmap_Click);
            this.chkShowHeatmap.Checked = false;            
            // 
            // groupOptimization
            // 
            this.groupOptimization.Controls.Add(this.comboOptimizer);
            this.groupOptimization.Controls.Add(this.tbIterationCount);
            this.groupOptimization.Controls.Add(this.labIterationCount);
            this.groupOptimization.Controls.Add(this.labOptimizer);                       
            this.groupOptimization.Controls.Add(this.ok);
            this.groupOptimization.Controls.Add(this.btnFinishOperations);            
            this.groupOptimization.Controls.Add(this.comboBestResults);
            this.groupOptimization.Controls.Add(this.labBestResults);
            //this.groupOptimization.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.groupOptimization.Location = new System.Drawing.Point(12, 265);
            this.groupOptimization.Name = "groupOptimization";
            this.groupOptimization.Size = new System.Drawing.Size(276, 220);
            this.groupOptimization.TabIndex = 12;
            this.groupOptimization.TabStop = false;
            this.groupOptimization.Text = "Optimization";            
            // 
            // comboOptimizer
            // 
            this.comboOptimizer.FormattingEnabled = true;
            this.comboOptimizer.DropDownStyle = ComboBoxStyle.DropDownList;
            this.comboOptimizer.Items.AddRange(new object[] {
            "Round Robin", //index 0
            "Hierarchical"}); //index 1
            //"Graph"}); //index 2
            this.comboOptimizer.SelectedIndex = 1;
            this.comboOptimizer.Location = new System.Drawing.Point(140, 31);
            this.comboOptimizer.Name = "comboOptimizer";
            this.comboOptimizer.Size = new System.Drawing.Size(130, 24);
            this.comboOptimizer.TabIndex = 10;

            if (this.userstudyMode)
                this.comboOptimizer.Enabled = false;
            // 
            // tbIterationCount
            // 
            this.tbIterationCount.Location = new System.Drawing.Point(140, 58);
            this.tbIterationCount.Name = "tbIterationCount";
            this.tbIterationCount.Size = new System.Drawing.Size(130, 22);
            this.tbIterationCount.TabIndex = 9;
            this.tbIterationCount.Hide();
            // 
            // labIterationCount
            // 
            this.labIterationCount.AutoSize = true;
            this.labIterationCount.Font = new System.Drawing.Font("Microsoft Sans Serif", 11.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labIterationCount.Location = new System.Drawing.Point(9, 59);
            this.labIterationCount.Name = "labIterationCount";
            this.labIterationCount.Size = new System.Drawing.Size(104, 18);
            this.labIterationCount.TabIndex = 8;
            this.labIterationCount.Text = "Iteration Count";
            this.labIterationCount.Hide();
            //this.labIterationCount.Click += new System.EventHandler(this.label2_Click);
            // 
            // labOptimizer
            // 
            this.labOptimizer.AutoSize = true;
            this.labOptimizer.Font = new System.Drawing.Font("Microsoft Sans Serif", 11.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labOptimizer.Location = new System.Drawing.Point(9, 31);
            this.labOptimizer.Name = "labOptimizer";
            this.labOptimizer.Size = new System.Drawing.Size(117, 18);
            this.labOptimizer.TabIndex = 7;
            this.labOptimizer.Text = "Select Optimizer";
            // 
            // labBestResults
            // 
            this.labBestResults.AutoSize = true;
            this.labBestResults.Font = new System.Drawing.Font("Microsoft Sans Serif", 11.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.labBestResults.Location = new System.Drawing.Point(9, 172);
            this.labBestResults.Name = "labBestResults";
            this.labBestResults.Size = new System.Drawing.Size(104, 18);
            this.labBestResults.TabIndex = 16;
            this.labBestResults.Text = "Best Layouts";
            this.labBestResults.Hide();
            // 
            // comboBestResults
            // 
            this.comboBestResults.FormattingEnabled = true;
            //this.comboBestResults.Items.AddRange(new object[] {"Best"});
            //this.comboBestResults.SelectedIndex = 0;
            this.comboBestResults.Location = new System.Drawing.Point(140, 172);
            this.comboBestResults.Name = "comboBestResults";
            this.comboBestResults.Size = new System.Drawing.Size(130, 24);
            this.comboBestResults.TabIndex = 16;
            this.comboBestResults.DropDownStyle = ComboBoxStyle.DropDownList;
            this.comboBestResults.SelectedIndexChanged += new System.EventHandler(this.comboBestResults_Click);
            this.comboBestResults.Hide();
            // 
            // btnFinishOperations
            // 
            this.btnFinishOperations.Location = new System.Drawing.Point(40, 85);
            this.btnFinishOperations.Name = "btnFinishOperations";
            this.btnFinishOperations.Size = new System.Drawing.Size(200, 45);
            this.btnFinishOperations.TabIndex = 17;
            this.btnFinishOperations.Text = "Choose";
            this.btnFinishOperations.UseVisualStyleBackColor = true;
            this.btnFinishOperations.Font = new System.Drawing.Font("Microsoft Sans Serif", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnFinishOperations.Click += new System.EventHandler(this.btnFinishOperations_Click);
            this.btnFinishOperations.Hide();
            // 
            // btnCloseTool
            // 
            this.btnCloseTool.Location = new System.Drawing.Point(30, 10);
            this.btnCloseTool.Name = "btnCloseTool";
            this.btnCloseTool.Size = new System.Drawing.Size(240, 25);
            this.btnCloseTool.TabIndex = 21;
            this.btnCloseTool.Text = "Finish";
            this.btnCloseTool.UseVisualStyleBackColor = true;
            this.btnCloseTool.Font = new System.Drawing.Font("Microsoft Sans Serif", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnCloseTool.Click += new System.EventHandler(this.btnCloseTool_Click);            
            // 
            // ok :: Get New Layouts
            // 
            this.ok.Location = new System.Drawing.Point(40, 85);
            this.ok.Name = "ok";
            this.ok.Size = new System.Drawing.Size(200, 45);
            this.ok.TabIndex = 6;
            this.ok.Text = "Get New Layouts";
            this.ok.UseVisualStyleBackColor = true;
            this.ok.Click += new System.EventHandler(this.ok_Click);
            this.ok.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            //
            // Menu
            //
            this.menuMain.MenuItems.Add(this.menuParams);
            this.menuMain.MenuItems.Add(this.menuDoc);
            this.menuParams.MenuItems.Add(this.menuParamsLoad);
            this.menuParams.MenuItems.Add(this.menuParamsWrite);
            this.menuDoc.MenuItems.Add(this.docLoad);
            this.menuDoc.MenuItems.Add(this.docSave);

            this.menuParamsLoad.Click += new System.EventHandler(LoadParamsFromFile);
            this.menuParamsWrite.Click += new System.EventHandler(SaveParamsToFile);
            this.docLoad.Click += new System.EventHandler(LoadGraphFromFile);
            this.docSave.Click += new System.EventHandler(SaveGraphFromFile);

            // 
            // opt_form
            // 
            this.UIForm.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.UIForm.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.UIForm.ClientSize = new System.Drawing.Size(300, 495);
            this.UIForm.ControlBox = true;
            this.UIForm.Controls.Add(this.groupOptimization);
            //this.UIForm.Controls.Add(this.btnRotation);
            //this.UIForm.Controls.Add(this.btnTranslation);            
            this.UIForm.Controls.Add(this.btnCloseTool);
            this.UIForm.Controls.Add(this.groupSpaceAnalysis);            
            //this.UIForm.Cursor = System.Windows.Forms.Cursors.Hand;
            //this.UIForm.Location = new System.Drawing.Point(100, 0);
            this.UIForm.MaximizeBox = false;
            this.UIForm.MinimizeBox = true;
            this.UIForm.Name = "opt_form";
            this.UIForm.ShowIcon = false;
            this.UIForm.StartPosition = System.Windows.Forms.FormStartPosition.Manual;
            this.UIForm.Text = "uDOME::Revit Plugin";            
            this.groupSpaceAnalysis.ResumeLayout(false);
            this.groupSpaceAnalysis.PerformLayout();
            this.groupOptimization.ResumeLayout(false);
            this.groupOptimization.PerformLayout();
            this.UIForm.ResumeLayout(false);
            this.UIForm.PerformLayout();
            this.UIForm.Menu = this.menuMain;
            this.UIForm.FormClosing += new FormClosingEventHandler(UIForm_Closing);
            this.UIForm.Load += new EventHandler(UIForm_Load);

            Size s = Screen.PrimaryScreen.WorkingArea.Size;
            this.UIForm.Location = new System.Drawing.Point((int)s.Width - this.UIForm.Width - 50, (int)s.Height - this.UIForm.Height - 100);
            //this.UIForm.Location = new System.Drawing.Point((int)System.Windows.SystemParameters.PrimaryScreenWidth - this.UIForm.Width - 50, (int)System.Windows.SystemParameters.PrimaryScreenHeight - this.UIForm.Height - 100);
            //this.UIForm.Show(); 

            if (this.userstudyMode)
            {
                HelperLoadParamsFromFile( (this.pathToSteerSuite + Path.GetFileNameWithoutExtension(doc.PathName) + ".xml") );                
            }

            return this.UIForm;
        }

        /// <summary>
        /// Represents User-Study 'A' where user has to interact with the default REVIT system
        /// We will be storing the usage time for user's interaction with the system
        /// </summary>
        /// <param name="doc"></param>
        /// <param name="uiapp"></param>
        public System.Windows.Forms.Form ShowUserStudyAForm(Document doc, ExternalEvent m_ExEvent)
        {
            this.doc = doc;
            this.m_ExEvent = m_ExEvent;
            InitializeCode();

            this.userstudy_A = true;           

            InitializeParticipant(this.doc);

            this.radioRegionQuery = new RadioButton();
            this.radioRegionReference = new RadioButton();

            usAForm = new System.Windows.Forms.Form();

            System.Windows.Forms.Button finish = new System.Windows.Forms.Button();

            finish.Location = new System.Drawing.Point(50, 80);
            finish.Name = "finish";
            finish.Size = new System.Drawing.Size(300, 40);
            finish.TabIndex = 1;
            finish.Text = "Finish";
            finish.UseVisualStyleBackColor = true;
            finish.Font = new System.Drawing.Font("Microsoft Sans Serif", 11.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            finish.Click += new System.EventHandler(ShowUserStudyAForm_Click);

            usAForm.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            usAForm.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            usAForm.ClientSize = new System.Drawing.Size(400, 200);
            usAForm.ControlBox = true;            
            usAForm.Controls.Add(finish);
            usAForm.MaximizeBox = false;
            usAForm.MinimizeBox = true;
            usAForm.Name = "usAForm";
            usAForm.ShowIcon = false;
            usAForm.StartPosition = System.Windows.Forms.FormStartPosition.Manual;
            usAForm.Text = "uDOME::Revit Plugin";
            usAForm.Load += new EventHandler(UIForm_Load);
            usAForm.ResumeLayout(false);
            usAForm.PerformLayout();

            System.Drawing.Size s = System.Windows.Forms.Screen.PrimaryScreen.WorkingArea.Size;
            usAForm.Location = new System.Drawing.Point((int)(s.Width - usAForm.Width - 50), (int)(s.Height - usAForm.Height - 50));
            this.userstudy_A = true;

            HelperLoadParamsFromFile((this.pathToSteerSuite + Path.GetFileNameWithoutExtension(doc.PathName) + ".xml"));
            return usAForm;
        }

        /// <summary>
        /// User-Study-A: Finish Click button
        /// Stores the user interaction time to File
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void ShowUserStudyAForm_Click(object sender, EventArgs e)
        {
            this.watchShared.Stop();
            string timerFile = this.pathToParticipant + Path.DirectorySeparatorChar + TIMER_FILENAME + "-" +
                    Path.GetFileNameWithoutExtension(doc.PathName) + OPT_OUTPUT_FILETYPE;
            long elapsedMs = this.watchShared.ElapsedMilliseconds;

            string resultPath = this.pathToParticipant + Path.DirectorySeparatorChar +
                "Graph-P" + this.partcipantId + "-" + Path.GetFileNameWithoutExtension(doc.PathName) + this.GRAPH_FILETYPE;

            StoreGraph(this.doc, resultPath);

            string optimizationResultFile = this.pathToParticipant + Path.DirectorySeparatorChar + "StudyResults-" +
                    Path.GetFileNameWithoutExtension(doc.PathName) + OPT_OUTPUT_FILETYPE;

            Graph g = new Graph();
            g.importFromFile(resultPath);

            DoubleVector dv = GetDefaultMetricsValues(g);

            StringBuilder results = new StringBuilder(); // user-study data dump
            string newResultLine = "";
            if (!System.IO.File.Exists(optimizationResultFile))
            {
                newResultLine = string.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10}", "PARTICIPANT_NUM", "TOTAL_TIME", 
                    "MEAN_DEGREE", "MEAN_ENTROPY", "MEAN_TREEDEPTH", 
                    "AGGREGATED_METRIC_VALUE",
                    "WEIGHTED_DEGREE", "WEIGHTED_ENTROPY", "WEIGHTED_TREEDEPTH", "CLEARENCE", "ALIGNMENT");
                results.AppendLine(newResultLine);

                newResultLine = string.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10}", this.partcipantId, (elapsedMs / 1000),
                    Math.Round(dv[0], 4).ToString(), Math.Round(dv[1], 4).ToString(), Math.Round(dv[2], 4).ToString(),
                    Math.Round(dv[3], 4).ToString(),
                Math.Round(dv[4], 4).ToString(), Math.Round(dv[5], 4).ToString(), Math.Round(dv[6], 4).ToString(),
                Math.Round(dv[7], 4).ToString(), Math.Round(dv[8], 4).ToString());

                results.AppendLine(newResultLine);
            }
            else
            {
                newResultLine = string.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10}", this.partcipantId, (elapsedMs / 1000),
                    Math.Round(dv[0], 4).ToString(), Math.Round(dv[1], 4).ToString(), Math.Round(dv[2], 4).ToString(),
                    Math.Round(dv[3], 4).ToString(),
                Math.Round(dv[4], 4).ToString(), Math.Round(dv[5], 4).ToString(), Math.Round(dv[6], 4).ToString(),
                Math.Round(dv[7], 4).ToString(), Math.Round(dv[8], 4).ToString());

                results.AppendLine(newResultLine);
            }
            //SteerSuite s = ProcessSpaceGraph2(g);

            //double meanDeg = s.get_meanDegree();
            //double meanEnt = s.get_meanTreeProp()[1];
            //double meanTDep = s.get_meanTreeProp()[0];

            //StringBuilder results = new StringBuilder(); // user-study data dump
            //string newResultLine = "";
            //if (!System.IO.File.Exists(optimizationResultFile))
            //{
            //    newResultLine = string.Format("{0},{1},{2},{3},{4}", "PARTICIPANT_NUM", "TOTAL_TIME",
            //        "MEAN_DEGREE", "MEAN_ENTROPY", "MEAN_TREEDEPTH");
            //    results.AppendLine(newResultLine);

            //    newResultLine = string.Format("{0},{1},{2},{3},{4}", this.partcipantId, (elapsedMs / 1000), Math.Round(meanDeg, 4).ToString(),
            //    Math.Round(meanEnt, 4).ToString(), Math.Round(meanTDep, 4).ToString());

            //    results.AppendLine(newResultLine);
            //}
            //else
            //{
            //    newResultLine = string.Format("{0},{1},{2},{3},{4}", this.partcipantId, (elapsedMs / 1000), Math.Round(meanDeg, 4).ToString(),
            //    Math.Round(meanEnt, 4).ToString(), Math.Round(meanTDep, 4).ToString());

            //    results.AppendLine(newResultLine);
            //}


            try
            {
                File.AppendAllText(optimizationResultFile, results.ToString());
                String svgFile = this.pathToParticipant + Path.DirectorySeparatorChar + "StudyResults-" +
                    Path.GetFileNameWithoutExtension(doc.PathName);

                OptimizationParameters _ops = new OptimizationParameters();                
                _ops.loadFromFile(this.pathToSteerSuite + Path.GetFileNameWithoutExtension(doc.PathName) + ".xml");

                g.saveSVGToFile(svgFile + ".svg", _ops);
            }
            catch (Exception ex)
            {
                TaskDialog.Show("uDOME", "Exception in Writing User-Study-A Results File. " + ex.ToString());
            }
            if (usAForm != null && usAForm.Visible)
            {
                this.clearFloorRegions = true;
                this.clearConstraintRegions = true;
                this.m_ExEvent.Raise();
                usAForm.Close();
            }
        }

        /// <summary>
        /// Called while form is loading.
        /// Starts the timer to record overall usage time.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void UIForm_Load(object sender, EventArgs e)
        {
            if(this.userstudyMode || this.userstudy_A)
                this.watchShared = System.Diagnostics.Stopwatch.StartNew();
        }
        /// <summary>
        /// Called before closing the UIForm.
        /// Used to record the overall usage time.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void UIForm_Closing(object sender, FormClosingEventArgs e)
        {
            if (this.userstudyMode || this.userstudy_A)
            {
                //private string TIMER_FILENAME = "TimeInfo";

                this.watchShared.Stop();
                string timerFile = this.pathToParticipant + Path.DirectorySeparatorChar + TIMER_FILENAME + "-" +
                        Path.GetFileNameWithoutExtension(doc.PathName) + OPT_OUTPUT_FILETYPE;
                long elapsedMs = this.watchShared.ElapsedMilliseconds;

                StringBuilder results = new StringBuilder(); // user-study data dump
                string newResultLine = "";
                if (!System.IO.File.Exists(timerFile))
                {
                    newResultLine = string.Format("{0},{1}", "STUDY(A/B)", "TOTAL_TIME");
                    results.AppendLine(newResultLine);

                    if (this.userstudy_A)
                        newResultLine = string.Format("{0},{1}", "A", (elapsedMs / 1000));
                    else
                        newResultLine = string.Format("{0},{1}", "B", (elapsedMs / 1000));
                    results.AppendLine(newResultLine);
                }
                else
                {
                    if (this.userstudy_A)
                        newResultLine = string.Format("{0},{1}", "A", (elapsedMs / 1000));
                    else
                        newResultLine = string.Format("{0},{1}", "B", (elapsedMs / 1000));
                    results.AppendLine(newResultLine);
                }

                try
                {
                    File.AppendAllText(timerFile, results.ToString());
                }
                catch (Exception ex)
                {
                    TaskDialog.Show("uDOME", "Exception in Writing User-Study Timer File. " + ex.ToString());
                }
            }

            this.userstudy_A = false;
            this.userstudyMode = false;
            this.finishPressed = true;
            ClearControls();
        }

        /// <summary>
        /// Button: Close tool
        /// Form closing event will be called afterwards
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void btnCloseTool_Click(object sender, System.EventArgs e)
        {
            this.finishPressed = true;
            this.UIForm.Close();
        }

        /// <summary>
        /// Load the optimization params from the user selected file.
        /// For now, just works for Query and Reference regions.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void LoadParamsFromFile(object sender, System.EventArgs e)
        {
            OpenFileDialog theDialog = new OpenFileDialog();
            string filename = null;
            theDialog.Title = "uDOME::Select a Parameter File";
            theDialog.Filter = "XML files|*.xml";
            theDialog.InitialDirectory = this.steerSuiteBinDirectory;
            if (theDialog.ShowDialog() == DialogResult.OK)
            {
                if ((filename = theDialog.FileName) != null)
                {
                    HelperLoadParamsFromFile(filename);   
                }
                else
                    TaskDialog.Show("uDOME", "Either File doesn't Exist or doesn't have Read Permissions.");                

                TaskDialog.Show("uDOME", "Parameter File has been Loaded Successfully.");
            }
        }

        /// <summary>
        /// Helps LoadParamsFromFile() in loading params | regions
        /// </summary>
        /// <param name="filename"></param>
        public void HelperLoadParamsFromFile(string filename)
        {
            try
            {
                if (filename != null)
                {
                    this.studyParams = new OptimizationParameters();
                    this.studyParams.loadFromFile(filename);

                    foreach (PointVector q in this.studyParams._queryRegions)
                    {
                        this.radioRegionReference.Checked = false;
                        this.radioRegionQuery.Checked = true;

                        XYZ rMin = new XYZ(q.ElementAt<Point>(0).x, q.ElementAt<Point>(0).z, 0);
                        XYZ rMax = new XYZ(q.ElementAt<Point>(1).x, q.ElementAt<Point>(1).z, 0);

                        this.HelperSelectSpaceRegion(rMin * _meters_to_feet, rMax * _meters_to_feet);
                    }

                    foreach (PointVector r in this.studyParams._refRegions)
                    {

                        this.radioRegionQuery.Checked = false;
                        this.radioRegionReference.Checked = true;

                        XYZ rMin = new XYZ(r.ElementAt<Point>(0).x, r.ElementAt<Point>(0).z, 0);
                        XYZ rMax = new XYZ(r.ElementAt<Point>(1).x, r.ElementAt<Point>(1).z, 0);

                        this.HelperSelectSpaceRegion(rMin * _meters_to_feet, rMax * _meters_to_feet);
                    }
                }
                else
                    TaskDialog.Show("uDOME", "Either File doesn't Exist or doesn't have Read Permissions.");
            }
            catch (Exception ex)
            {
                TaskDialog.Show("uDOME", "Exception in Reading Optimization Params from File: \n" + filename + ". \n\n" + ex.ToString());
            }
        }

        /// <summary>
        /// Save the optimization params to the user selected location.
        /// Uses System.XML class.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void SaveParamsToFile(object sender, System.EventArgs e)
        {
            SaveFileDialog theDialog = new SaveFileDialog();
            string filename = null;            
            theDialog.Filter = "XML files|*.xml";
            theDialog.InitialDirectory = this.steerSuiteBinDirectory;
            
            if (theDialog.ShowDialog() == DialogResult.OK)
            {
                if ((filename = theDialog.FileName) != null)
                {
                    HelperSaveParamsToFile(filename);
                }
                else
                    TaskDialog.Show("uDOME", "Filename can't be Empty.");
                
                TaskDialog.Show("uDOME", "Parameter File has been Saved Successfully.");
            }
        }

        public void HelperSaveParamsToFile(string filename)
        {
            System.Xml.XmlWriter writer = null;
            XmlWriterSettings settings = new XmlWriterSettings();
            settings.Indent = true;

            try
            {
                if (filename != null)
                {
                    //TaskDialog.Show("uDOME", "Selected Filename: " + filename);
                    // Using System.XMl
                    using (writer = XmlWriter.Create(filename, settings))
                    {
                        if (writer != null)
                        {
                            writer.WriteStartDocument();
                            writer.WriteComment("Filename: " + filename);
                            writer.WriteComment("Creation Date: " + DateTime.Now.ToString("MM.dd.yyyy-hh.mm.ss"));
                            writer.WriteComment("Stores Optimization Parameters");
                            
                            writer.WriteStartElement("parameterOptimitationExperiment");
                            writer.WriteStartElement("env_config");

                            if (this.worldBoundMax != null && this.worldBoundMin != null)
                            {
                                writer.WriteStartElement("visibilityGraph");
                                writer.WriteStartElement("region");

                                // World Min
                                writer.WriteStartElement("point");
                                writer.WriteElementString("x", this.worldBoundMin.X.ToString()); //axis swapping and unit scaling has been done already
                                writer.WriteElementString("y", this.worldBoundMin.Y.ToString()); //axis swapping and unit scaling has been done already
                                writer.WriteElementString("z", this.worldBoundMin.Z.ToString()); //axis swapping and unit scaling has been done already
                                writer.WriteEndElement(); //End point

                                // World Max
                                writer.WriteStartElement("point");
                                writer.WriteElementString("x", this.worldBoundMax.X.ToString()); //axis swapping and unit scaling has been done already
                                writer.WriteElementString("y", this.worldBoundMax.Y.ToString()); //axis swapping and unit scaling has been done already
                                writer.WriteElementString("z", this.worldBoundMax.Z.ToString()); //axis swapping and unit scaling has been done already
                                writer.WriteEndElement(); //End point

                                writer.WriteEndElement(); //End region
                                writer.WriteEndElement(); //End visibilityGraph
                            }

                            if (this.queryRegionsPoints.Count() > 0)
                            {
                                writer.WriteStartElement("queryRegions");
                                foreach (List<Point> qr in this.queryRegionsPoints)
                                {
                                    writer.WriteStartElement("region");

                                    // Query Region Min
                                    writer.WriteStartElement("point");
                                    writer.WriteElementString("x", qr.ElementAt<Point>(0).x.ToString()); //axis swapping and unit scaling has been done already
                                    writer.WriteElementString("y", qr.ElementAt<Point>(0).y.ToString()); //axis swapping and unit scaling has been done already
                                    writer.WriteElementString("z", qr.ElementAt<Point>(0).z.ToString()); //axis swapping and unit scaling has been done already
                                    writer.WriteEndElement(); //End point

                                    // Query Region Max
                                    writer.WriteStartElement("point");
                                    writer.WriteElementString("x", qr.ElementAt<Point>(1).x.ToString()); //axis swapping and unit scaling has been done already
                                    writer.WriteElementString("y", qr.ElementAt<Point>(1).y.ToString()); //axis swapping and unit scaling has been done already
                                    writer.WriteElementString("z", qr.ElementAt<Point>(1).z.ToString()); //axis swapping and unit scaling has been done already
                                    writer.WriteEndElement(); //End point

                                    writer.WriteEndElement(); //End region                                        
                                }
                                writer.WriteEndElement(); //End queryRegions
                            }

                            if (this.referenceRegionsPoints.Count() > 0)
                            {
                                writer.WriteStartElement("referenceRegions");
                                foreach (List<Point> rr in this.referenceRegionsPoints)
                                {
                                    writer.WriteStartElement("region");

                                    // Reference Region Min
                                    writer.WriteStartElement("point");
                                    writer.WriteElementString("x", rr.ElementAt<Point>(0).x.ToString()); //axis swapping and unit scaling has been done already
                                    writer.WriteElementString("y", rr.ElementAt<Point>(0).y.ToString()); //axis swapping and unit scaling has been done already
                                    writer.WriteElementString("z", rr.ElementAt<Point>(0).z.ToString()); //axis swapping and unit scaling has been done already
                                    writer.WriteEndElement(); //End point

                                    // Reference Region Max
                                    writer.WriteStartElement("point");
                                    writer.WriteElementString("x", rr.ElementAt<Point>(1).x.ToString()); //axis swapping and unit scaling has been done already
                                    writer.WriteElementString("y", rr.ElementAt<Point>(1).y.ToString()); //axis swapping and unit scaling has been done already
                                    writer.WriteElementString("z", rr.ElementAt<Point>(1).z.ToString()); //axis swapping and unit scaling has been done already
                                    writer.WriteEndElement(); //End point

                                    writer.WriteEndElement(); //End region                                        
                                }
                                writer.WriteEndElement(); //End referenceRegions
                            }

                            for (int i = 0; i < this.params_.size(); i++)
                            {
                                OptimizationParameter op = this.params_.getParameter((uint)i);

                                writer.WriteStartElement("parameter");
                                writer.WriteAttributeString("type", "float");
                                writer.WriteAttributeString("name", "p-" + (i + 1));

                                // _lb
                                writer.WriteStartElement("min");
                                writer.WriteAttributeString("type", "float");
                                writer.WriteString(op._lb.ToString()); // Unit scaling has been done already
                                writer.WriteEndElement(); //End min 

                                // _ub
                                writer.WriteStartElement("max");
                                writer.WriteAttributeString("type", "float");
                                writer.WriteString(op._ub.ToString()); // Unit scaling has been done already
                                writer.WriteEndElement(); //End max

                                // original: default in SteerSuite is ZERO (I guess?)
                                writer.WriteStartElement("original");
                                writer.WriteAttributeString("type", "float");
                                writer.WriteString("0"); // Unit scaling has been done already
                                writer.WriteEndElement(); //End original 

                                // discretization: default in SteerSuite is 20 (I guess?)
                                writer.WriteStartElement("discretization");
                                writer.WriteAttributeString("type", "float");
                                writer.WriteString("20"); // Unit scaling has been done already
                                writer.WriteEndElement(); //End discretization

                                // translationDirection or rotationOrigin
                                DoubleVector v = null;
                                if (op._p_type == OptimizationParameter.ParameterType.Translation)
                                {
                                    v = op.getTranslationDirectionV();
                                    if (v != null)
                                        writer.WriteStartElement("translationDirection");
                                }
                                else if (op._p_type == OptimizationParameter.ParameterType.Rotation)
                                {
                                    v = op.getRotationOriginV();
                                    if (v != null)
                                        writer.WriteStartElement("rotationOrigin");
                                }

                                if (v != null)
                                {
                                    writer.WriteElementString("x", v[0].ToString()); //axis swapping and unit scaling has been done already
                                    writer.WriteElementString("y", v[1].ToString()); //axis swapping and unit scaling has been done already
                                    writer.WriteElementString("z", v[2].ToString()); //axis swapping and unit scaling has been done already
                                    writer.WriteEndElement(); //End rotationOrigin or translationDirection
                                }

                                // node_ids                                    
                                Vector_SizeT nodes = op.getNodeIDs();
                                if (nodes.Count > 0)
                                {
                                    writer.WriteStartElement("node_ids");
                                    foreach (uint n in nodes)
                                    {
                                        writer.WriteElementString("node", n.ToString());
                                    }
                                    writer.WriteEndElement(); //End node_ids
                                }

                                writer.WriteEndElement(); //End parameter 
                            }

                            writer.WriteEndElement(); //End env_config
                            writer.WriteEndElement(); //End parameterOptimitationExperiment
                        }
                        else
                            TaskDialog.Show("uDOME", "Couldn't Create a New File. Check Permissions.");
                    }
                }
                else
                    TaskDialog.Show("uDOME", "Filename can't be Empty.");
            }
            catch (Exception ex)
            {
                TaskDialog.Show("uDOME", "Exception in Saving Optimization Params to File. \n\n" + ex.ToString());
            }
        }

        /// <summary>
        /// Load the graph (walls structure) from the user selected file.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void LoadGraphFromFile(object sender, System.EventArgs e)
        {
            //calls HelperLoadGraphFromFile
            OpenFileDialog theDialog = new OpenFileDialog();
            string filename = null;
            theDialog.Title = "uDOME::Select a Graph File";
            theDialog.Filter = "Graph files|*.graph";
            theDialog.InitialDirectory = this.steerSuiteBinDirectory;
            if (theDialog.ShowDialog() == DialogResult.OK)
            {
                if ((filename = theDialog.FileName) != null)
                {
                    HelperLoadGraphFromFile(filename);
                }
                else
                    TaskDialog.Show("uDOME", "Either File doesn't Exist or doesn't have Read Permissions.");

                TaskDialog.Show("uDOME", "Graph File has been Loaded Successfully.");
            }            
        }

        /// <summary>
        /// Helper: Load the graph (walls structure) from the user selected file.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void HelperLoadGraphFromFile(string graphFile)
        {
            this.openNewDocument = true;
            this.loadGraphFile = graphFile;
            this.m_ExEvent.Raise();            
        }

        /// <summary>
        /// Saves the graph (walls structure) to the user selected file.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void SaveGraphFromFile(object sender, System.EventArgs e)
        {
            SaveFileDialog theDialog = new SaveFileDialog();
            string filename = null;
            theDialog.Filter = "Graph files|*.graph";
            theDialog.InitialDirectory = this.steerSuiteBinDirectory;

            if (theDialog.ShowDialog() == DialogResult.OK)
            {
                if ((filename = theDialog.FileName) != null)
                {
                    StoreGraph(this.doc, filename);
                }
                else
                    TaskDialog.Show("uDOME", "Filename can't be Empty.");

                TaskDialog.Show("uDOME", "Graph File has been Saved Successfully.");
            }
        }

        /// <summary>
        /// Button: Translation
        /// Adds the selected element(s) into elements list and also sets the inital constraints bound.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void btnTranslation_Click(object sender, EventArgs e)
        {
            this.user_selected_elements.Clear();
            this.isRotational = false;
            this.upperBounds.Clear();
            this.lowerBounds.Clear();
            this.user_selected_elements = GetOptimizableElement(doc);

            if (this.user_selected_elements.Count < 1)
            {
                TaskDialog.Show("uDOME", "Hey! No element is selected.");
                return;
            }

            if (this.userstudyMode && !this.iterationTimeEnabled)
            {
                this.iterationTimeEnabled = true;                
                this.watch = System.Diagnostics.Stopwatch.StartNew();
            }

            this.labBound.Text = "Upper Bound Constraint";
            this.labBound.Location = new System.Drawing.Point(6, 40);
            this.labBound.Visible = true;

            this.btnCancelConstraint.Visible = true;
            this.btnConstraintsDone.Visible = true;
            this.btnTranslation.Hide();
            this.btnRotation.Hide();            
        }

        /// <summary>
        /// Button: "Rotation"
        /// Adds the selected element(s) into elements list and also asks user to select rotation origin.        
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>

        public void btnRotation_Click(object sender, EventArgs e)
        {
            this.user_selected_elements.Clear();
            this.isRotational = true;
            this.upperBounds.Clear();
            this.lowerBounds.Clear();
            this.user_selected_elements = GetOptimizableElement(doc);

            if (this.user_selected_elements.Count < 1)
            {
                TaskDialog.Show("uDOME", "Hey! No element is selected.");
                return;
            }

            if (this.userstudyMode && !this.iterationTimeEnabled)
            {
                this.iterationTimeEnabled = true;
                this.watch = System.Diagnostics.Stopwatch.StartNew();
            }

            TaskDialog.Show("uDOME", "Hey! Pick the Centre of Rotation");

            UIDocument uidoc = new UIDocument(this.doc);
            this.rotationOrigin = uidoc.Selection.PickPoint("Hey! Pick the Centre of Rotation");

            uidoc.Selection.SetElementIds((from Element _e in this.user_selected_elements select _e.Id).ToList<ElementId>());
            //TaskDialog.Show("uDOME", "Walls selected: " + uidoc.Selection.GetElementIds().Count());

            this.labBound.Text = "Upper Bound Constraint";
            this.labBound.Location = new System.Drawing.Point(6, 40);
            this.labBound.Visible = true;
            this.trackbarBound.Visible = true;
            this.UB = true;
            this.trackbarBound.Minimum = 0;
            this.trackbarBound.Maximum = 360;
            this.trackbarBound.Value = 0;
            this.lastAngle = 0;

            this.btnCancelConstraint.Visible = true;
            this.btnConstraintsDone.Visible = true;
            this.btnTranslation.Hide();
            this.btnRotation.Hide();
        }

        /// <summary>
        /// Button: Cancel
        /// Cancels the ongoing set constraint event
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void btnCancelConstraint_Click(object sender, EventArgs e)
        {            
            this.upperBounds.Clear();
            this.lowerBounds.Clear();

            this.btnCancelConstraint.Hide();
            this.btnConstraintsDone.Hide();
            this.trackbarBound.Hide();
            this.trackbarBound.Value = 1;
            this.labBound.Hide();

            this.btnTranslation.Visible = true;
            this.btnRotation.Visible = true;

            foreach (Element ele in this.user_selected_elements)
            {
                if (this.doc.ActiveView.GetElementOverrides(ele.Id).ProjectionFillColor.IsValid)
                    ChangeElementColor(this.doc, ele.Id, new Autodesk.Revit.DB.Color(255, 255, 255));
            }
            this.user_selected_elements.Clear();
            this.m_ExEvent.Raise();
        }

        /// <summary>
        /// Allow the user to choose/pick a Wall from the environment to run optimization for it.
        /// </summary>
        /// <param name="document">Gets the current document.</param>
        /// <returns>Returns the element id of the picked wall.</returns>

        public IList<Element> GetOptimizableElement(Document document)
        {
            this.wallsGeometry.Clear();
            this.edges.Clear();
            this.verts.Clear();
            StoreGraph(this.doc);
            this.graph.importFromFile(this.graphFileName);
            this.graph2.importFromFile(this.graphFileName);

            UIDocument uidoc = new UIDocument(document);

            // this is new way getting selected elements (selElementSet class has been obsoleted 2015 onwards)
            ICollection<ElementId> elements = uidoc.Selection.GetElementIds(); 

            //Reference pickedElement = uidoc.Selection.PickObject(Autodesk.Revit.UI.Selection.ObjectType.Element, "Hey! Pick a Wall to Optimize");
            //IList<Element> picked = uidoc.Selection.PickElementsByRectangle(new SelectionFilter1(), "Hey! Pick the Elements by Drawing a Rectangle");
            //IList<Reference> picked = uidoc.Selection.PickObjects(ObjectType.Element, new SelectionFilter1(), "Hey! Pick the Elements.");
            
            List<Element> ele_list = new List<Element>();

            if (elements != null)
            {
                foreach (ElementId id in elements)
                {
                    if (!(document.GetElement(id) is Wall))
                        continue;
                    
                    if (document.ActiveView.GetElementOverrides(id).ProjectionFillColor.IsValid)
                    {
                        byte r = 255, g = 255, b = 255;
                        if(document.ActiveView.GetElementOverrides(id).ProjectionFillColor.Red != r &&
                            document.ActiveView.GetElementOverrides(id).ProjectionFillColor.Green != g &&
                            document.ActiveView.GetElementOverrides(id).ProjectionFillColor.Blue != b)
                        {
                            ChangeElementColor(document, id, new Autodesk.Revit.DB.Color(255, 255, 255));
                            continue;
                        }                        
                    }

                    ChangeElementColor(document, id, new Autodesk.Revit.DB.Color(50,255,50));

                    ele_list.Add(document.GetElement(id));

                    int elementIndex = this.eIds.FindIndex(ElementId => ElementId == id);

                    int origin = this.edges[elementIndex].ElementAt<int>(0);
                    int end = this.edges[elementIndex].ElementAt<int>(1);

                    XYZ p0 = this.verts[origin];
                    XYZ p1 = this.verts[end];

                    this.wallsGeometry.Add(new WallsGeometry(
                        id, 
                        new XYZ(p0.X, p0.Y, p0.Z),
                        new XYZ(p1.X, p1.Y, p1.Z)
                        ));

                    if (!this.isRotational)
                    {
                        List<XYZ> points = new List<XYZ>();
                        points.Add(p1);
                        points.Add(p0);
                        this.upperBounds.Add(points);
                    }
                }
                //ele_list.ElementAt<Element>(0)
                //this.elementsCopy.Add(ElementTransformUtils.CopyElement(document, e.Id, new XYZ(0, 0, 0)).ElementAt<ElementId>(0));
                //e.Pinned = true;
                //TaskDialog.Show("uDOME", this.elementsCopy.Count.ToString());                
                //uidoc.Selection.SetElementIds(this.elementsCopy);

                if (!this.isRotational)
                {
                    this.upperBoundValue = 0;
                    this.UB = false;
                }
            }

            //TaskDialog.Show("uDOME", string.Format("{0} Wall(s) Selected.", ele_list.Count));
            this.m_ExEvent.Raise();
            return ele_list;
        }        

        /// <summary>
        /// Changes color of the given element to provided 
        /// color (c) in the given document (doc). 
        /// </summary>
        /// <param name="doc"></param>
        /// <param name="eId"></param>
        /// <param name="c"></param>

        public static void ChangeElementColor(Document doc, ElementId eId, Autodesk.Revit.DB.Color c)
        {
            OverrideGraphicSettings newGraphicsSettings = doc.ActiveView.GetElementOverrides(eId);
            //newGraphicsSettings.SetSurfaceTransparency(50); //To tansparent the surface of the Walls
            Element solidFill = new FilteredElementCollector(doc).OfClass(typeof(FillPatternElement)).Where(q => q.Name.Contains("Solid")).First();

            newGraphicsSettings.SetProjectionFillPatternId(solidFill.Id);
            newGraphicsSettings.SetProjectionFillColor(c);
            newGraphicsSettings.SetProjectionFillPatternVisible(true);
            doc.ActiveView.SetElementOverrides(eId, newGraphicsSettings);            
        }

        /// <summary>
        /// Changes the fill pattern of the given element to provided 
        /// pattern (pattern) in the given document (doc). 
        /// </summary>
        /// <param name="doc"></param>
        /// <param name="eId"></param>
        /// <param name="pattern"></param>

        public static void ChangeElementFillPattern(Document doc, ElementId eId, string pattern= "Crosshatch")
        {            
            OverrideGraphicSettings newGraphicsSettings = doc.ActiveView.GetElementOverrides(eId);            
            Element solidFill = new FilteredElementCollector(doc).OfClass(typeof(FillPatternElement)).Where(q => q.Name.Contains(pattern)).First();

            newGraphicsSettings.SetProjectionFillPatternId(solidFill.Id);            
            newGraphicsSettings.SetProjectionFillPatternVisible(true);
            doc.ActiveView.SetElementOverrides(eId, newGraphicsSettings);
                        
            UIDocument uidoc = new UIDocument(doc);
            uidoc.RefreshActiveView();            
        }

        /// <summary>
        /// Make the given element transparent by 50%         
        /// </summary>
        /// <param name="doc"></param>
        /// <param name="eId"></param>        

        public static void ChangeElementTransparency(Document doc, ElementId eId)
        {
            OverrideGraphicSettings newGraphicsSettings = doc.ActiveView.GetElementOverrides(eId);
            newGraphicsSettings.SetSurfaceTransparency(50);
            doc.ActiveView.SetElementOverrides(eId, newGraphicsSettings);

            UIDocument uidoc = new UIDocument(doc);
            uidoc.RefreshActiveView();
        }

        /// <summary>
        /// Button: "Pick"
        /// Allows the user to select multiple elements by draw a rectangle 
        /// inside REVIT document. 
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void btnPick_Click(object sender, EventArgs e)
        {
            this.user_selected_elements.Clear();
            this.user_selected_elements = GetOptimizableElement(doc);
            //foreach (Element ele in this.user_selected_elements)
            //{
            //    TaskDialog.Show("Selected Element", ele.Id.ToString());
            //}
        }

        public void BuildFloorBoundaries()
        {
            if (this.floorBoundaries == null)
                this.floorBoundaries = new CurveArray();
            else
                this.floorBoundaries.Clear();

            this.floorBoundaries.Append(Line.CreateBound(new XYZ(this.regionBoundMinFloor.X, this.regionBoundMinFloor.Y, 0), 
                new XYZ(this.regionBoundMaxFloor.X, this.regionBoundMinFloor.Y, 0)));
            this.floorBoundaries.Append(Line.CreateBound(new XYZ(this.regionBoundMaxFloor.X, this.regionBoundMinFloor.Y, 0), 
                new XYZ(this.regionBoundMaxFloor.X, this.regionBoundMaxFloor.Y, 0)));
            this.floorBoundaries.Append(Line.CreateBound(new XYZ(this.regionBoundMaxFloor.X, this.regionBoundMaxFloor.Y, 0), 
                new XYZ(this.regionBoundMinFloor.X, this.regionBoundMaxFloor.Y, 0)));
            this.floorBoundaries.Append(Line.CreateBound(new XYZ(this.regionBoundMinFloor.X, this.regionBoundMaxFloor.Y, 0), 
                new XYZ(this.regionBoundMinFloor.X, this.regionBoundMinFloor.Y, 0)));
        }

        /// <summary>
        /// Creates and returns a Floor object with the given region
        /// </summary>
        public Floor BuildFloor()
        {
            CurveArray floorBoundaries = new CurveArray();

            floorBoundaries.Append(Line.CreateBound(new XYZ(this.regionBoundMinFloor.X, this.regionBoundMinFloor.Y, 0),
                new XYZ(this.regionBoundMaxFloor.X, this.regionBoundMinFloor.Y, 0)));
            floorBoundaries.Append(Line.CreateBound(new XYZ(this.regionBoundMaxFloor.X, this.regionBoundMinFloor.Y, 0),
                new XYZ(this.regionBoundMaxFloor.X, this.regionBoundMaxFloor.Y, 0)));
            floorBoundaries.Append(Line.CreateBound(new XYZ(this.regionBoundMaxFloor.X, this.regionBoundMaxFloor.Y, 0),
                new XYZ(this.regionBoundMinFloor.X, this.regionBoundMaxFloor.Y, 0)));
            floorBoundaries.Append(Line.CreateBound(new XYZ(this.regionBoundMinFloor.X, this.regionBoundMaxFloor.Y, 0),
                new XYZ(this.regionBoundMinFloor.X, this.regionBoundMinFloor.Y, 0)));

            Floor _f = this.doc.Create.NewFloor(floorBoundaries, true);
            _f.get_Parameter(BuiltInParameter.FLOOR_HEIGHTABOVELEVEL_PARAM).Set(1);

            return _f;
        }

        /// <summary>
        /// Creates and returns a Floor object with the given region
        /// </summary>
        public Floor BuildFloor(double minX, double maxX, double minY, double maxY)
        {
            CurveArray floorBoundaries = new CurveArray();

            floorBoundaries.Append(Line.CreateBound(new XYZ(minX, minY, 0),
                new XYZ(maxX, minY, 0)));
            floorBoundaries.Append(Line.CreateBound(new XYZ(maxX, minY, 0),
                new XYZ(maxX, maxY, 0)));
            floorBoundaries.Append(Line.CreateBound(new XYZ(maxX, maxY, 0),
                new XYZ(minX, maxY, 0)));
            floorBoundaries.Append(Line.CreateBound(new XYZ(minX, maxY, 0),
                new XYZ(minX, minY, 0)));

            Floor _f = this.doc.Create.NewFloor(floorBoundaries, true);
            _f.get_Parameter(BuiltInParameter.FLOOR_HEIGHTABOVELEVEL_PARAM).Set(1);

            return _f;
        }

        /// <summary>
        /// Creates and returns a Floor object with the given region
        /// </summary>
        public Floor BuildFloor(XYZ dir, XYZ p1, XYZ p2, XYZ p3, XYZ p4)
        {
            //TaskDialog.Show("uDOME", "p1: " + p1.ToString() + "\np2: " + p2.ToString() + "\np3: " + p3.ToString() + "\np4: " + p4.ToString());
            
            CurveArray floorBoundaries = new CurveArray();

            if( ((Math.Abs(p1.Y - p3.Y) <= TOLERANCE) && (Math.Abs(p1.Y - p2.Y) <= TOLERANCE)) || 
                (Math.Abs(p2.Y - p4.Y) <= TOLERANCE) && (Math.Abs(p3.Y - p4.Y) <= TOLERANCE))
            {
                floorBoundaries.Append(Line.CreateBound(p1.Add(new XYZ(0,0.5,0)), p4.Add(new XYZ(0,0.5,0))));
                floorBoundaries.Append(Line.CreateBound(p4.Add(new XYZ(0, 0.5, 0)), p4.Add(new XYZ(0, -0.5, 0))));
                floorBoundaries.Append(Line.CreateBound(p4.Add(new XYZ(0, -0.5, 0)), p1.Add(new XYZ(0, -0.5, 0))));
                floorBoundaries.Append(Line.CreateBound(p1.Add(new XYZ(0, -0.5, 0)), p1.Add(new XYZ(0, 0.5, 0))));

                Floor _fSpecial = this.doc.Create.NewFloor(floorBoundaries, false);
                _fSpecial.get_Parameter(BuiltInParameter.FLOOR_HEIGHTABOVELEVEL_PARAM).Set(1);

                return _fSpecial;
            }
            else if (((Math.Abs(p1.X - p3.X) <= TOLERANCE) && (Math.Abs(p1.X - p2.X) <= TOLERANCE)) ||
                (Math.Abs(p2.X - p4.X) <= TOLERANCE) && (Math.Abs(p3.X - p4.X) <= TOLERANCE))
            {
                floorBoundaries.Append(Line.CreateBound(p1.Add(new XYZ(0.5, 0, 0)), p4.Add(new XYZ(0.5, 0, 0))));
                floorBoundaries.Append(Line.CreateBound(p4.Add(new XYZ(0.5, 0, 0)), p4.Add(new XYZ(-0.5, 0, 0))));
                floorBoundaries.Append(Line.CreateBound(p4.Add(new XYZ(-0.5, 0, 0)), p1.Add(new XYZ(-0.5, 0, 0))));
                floorBoundaries.Append(Line.CreateBound(p1.Add(new XYZ(-0.5, 0, 0)), p1.Add(new XYZ(0.5, 0, 0))));

                Floor _fSpecial = this.doc.Create.NewFloor(floorBoundaries, false);
                _fSpecial.get_Parameter(BuiltInParameter.FLOOR_HEIGHTABOVELEVEL_PARAM).Set(1);

                return _fSpecial;
            }

            if (Math.Abs(p1.DistanceTo(p2)) > TOLERANCE) floorBoundaries.Append(Line.CreateBound(p1, p2));
            else floorBoundaries.Append(Line.CreateBound(p1.Add(dir.Multiply(-1)), p2.Add(dir.Multiply(1))));

            if (Math.Abs(p2.DistanceTo(p4)) > TOLERANCE) floorBoundaries.Append(Line.CreateBound(p2, p4));
            else floorBoundaries.Append(Line.CreateBound(p2.Add(dir.Multiply(-1)), p4.Add(dir.Multiply(1))));

            if (Math.Abs(p4.DistanceTo(p3)) > TOLERANCE) floorBoundaries.Append(Line.CreateBound(p4, p3));
            else floorBoundaries.Append(Line.CreateBound(p4.Add(dir.Multiply(-1)), p3.Add(dir.Multiply(1))));

            if (Math.Abs(p3.DistanceTo(p1)) > TOLERANCE) floorBoundaries.Append(Line.CreateBound(p3, p1));
            else floorBoundaries.Append(Line.CreateBound(p3.Add(dir.Multiply(-1)), p1.Add(dir.Multiply(1))));
            
            Floor _f = this.doc.Create.NewFloor(floorBoundaries, false);
            _f.get_Parameter(BuiltInParameter.FLOOR_HEIGHTABOVELEVEL_PARAM).Set(1);

            return _f;
        }

        /// <summary>
        /// Button: "Select Region"
        /// Asks user to select World Bound and Region Bound by drawing a Rectangle
        /// and then it computes space visibility graph for the Region Bound. For now,
        /// it will just show the degree metric value.        /// 
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>

        public void btnSelectSpaceRegion_Click(object sender, EventArgs e)
        {
            UIDocument uidoc = new UIDocument(this.doc);            
            Autodesk.Revit.UI.Selection.PickedBox pickedBox2 = uidoc.Selection.PickBox(Autodesk.Revit.UI.Selection.PickBoxStyle.Enclosing, "Hey! Select the World Bounds by Drawing a Rectangle.");
            HelperSelectSpaceRegion(pickedBox2.Min, pickedBox2.Max);
        }

        /// <summary>
        /// Helps the btnSelectSpaceRegion_Click()
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void HelperSelectSpaceRegion(XYZ rMin, XYZ rMax)
        {
            UIDocument uidoc = new UIDocument(this.doc);
            //TaskDialog.Show("uDOME", "Hey! Select a Region by Drawing Rectangle.");

            ElementCategoryFilter filter = new ElementCategoryFilter(BuiltInCategory.OST_Walls);
            FilteredElementCollector collector = new FilteredElementCollector(this.doc);
            IList<Element> walls =
            collector.WherePasses(filter).WhereElementIsNotElementType().ToElements();

            double xMin, xMax, yMin, yMax;
            xMin = xMax = yMin = yMax = 0;

            bool check = true;
            foreach (Element wall in walls)
            {
                BoundingBoxXYZ bbox = wall.get_BoundingBox(null);
                if (check)
                {
                    xMin = bbox.Min.X;
                    xMax = bbox.Max.X;
                    yMin = bbox.Min.Y;
                    yMax = bbox.Max.Y;

                    check = false;
                    continue;
                }

                if (bbox.Min.X <= xMin)
                    xMin = bbox.Min.X;
                else if (bbox.Max.X >= xMax)
                    xMax = bbox.Max.X;
                else if (bbox.Min.Y <= yMin)
                    yMin = bbox.Min.Y;
                else if (bbox.Max.Y >= yMax)
                    yMax = bbox.Max.Y;
            }

            //adding some buffer
            xMin -= 5;
            xMax += 5;
            yMin += 5; // Because rmin.y is actually a positive value we want to make larger.
            yMax -= 5;

            this.worldBoundMin = new XYZ(xMin, 0, yMin) * _feet_to_meters;
            this.worldBoundMax = new XYZ(xMax, 0, yMax) * _feet_to_meters;

            if (this.steersuite == null)
                ProcessSpaceGraph();

            //For World Bound
            //Autodesk.Revit.UI.Selection.PickedBox pickedBox1 = uidoc.Selection.PickBox(Autodesk.Revit.UI.Selection.PickBoxStyle.Enclosing, "Hey! Select the World Bounds by Drawing a Rectangle.");

            //Swapping Y and Z
            //UP-Direction in Revit is Z
            //UP-Direction in SteerSuite is Y
            // this.worldBoundMin = new XYZ(pickedBox1.Min.X, pickedBox1.Min.Z, pickedBox1.Min.Y) * _feet_to_meters;
            // this.worldBoundMax = new XYZ(pickedBox1.Max.X, pickedBox1.Max.Z, pickedBox1.Max.Y) * _feet_to_meters;
            //TaskDialog.Show("uDOME", "GridBound Min: " + this.worldBoundMin.ToString() + "\nGridBound Max: " + this.worldBoundMax.ToString());
                        
            this.regionBoundMin = rMin;
            this.regionBoundMax = rMax;

            this.regionBoundMinFloor = new XYZ(rMin.X, rMin.Y, rMin.Z);
            this.regionBoundMaxFloor = new XYZ(rMax.X, rMax.Y, rMax.Z);

            //Swapping Y and Z
            //UP-Direction in Revit is Z
            //UP-Direction in SteerSuite is Y
            this.regionBoundMin = new XYZ(rMin.X, 0, rMin.Y) * _feet_to_meters;
            this.regionBoundMax = new XYZ(rMax.X, 0, rMax.Y) * _feet_to_meters;
            //TaskDialog.Show("uDOME", "Region Bound Min: " + this.regionBoundMin.ToString() + "\nRegion Bound Max: " + this.regionBoundMax.ToString());

            if (this.radioRegionQuery.Checked)
            {
                this.steersuite.add_queryRegion(
                    new Point((float)this.regionBoundMin.X, (float)this.regionBoundMin.Y, (float)this.regionBoundMax.Z),
                    new Point((float)this.regionBoundMax.X, (float)this.regionBoundMax.Y, (float)this.regionBoundMin.Z)
                    );

                Floor _qf = BuildFloor();
                _qf.Pinned = true;
                ChangeElementColor(this.doc, _qf.Id, new Autodesk.Revit.DB.Color(255, 0, 127));
                this.queryRegions.Add(_qf);

                List<Point> _queryPoints = new List<Point>();
                _queryPoints.Add(new Point((float)this.regionBoundMin.X, (float)this.regionBoundMin.Y, (float)this.regionBoundMax.Z));
                _queryPoints.Add(new Point((float)this.regionBoundMax.X, (float)this.regionBoundMax.Y, (float)this.regionBoundMin.Z));
                this.queryRegionsPoints.Add(_queryPoints);
            }
            else if (this.radioRegionReference.Checked)
            {
                this.steersuite.add_refRegion(
                   new Point((float)this.regionBoundMin.X, (float)this.regionBoundMin.Y, (float)this.regionBoundMax.Z),
                    new Point((float)this.regionBoundMax.X, (float)this.regionBoundMax.Y, (float)this.regionBoundMin.Z)
                    );

                Floor _rf = BuildFloor();
                _rf.Pinned = true;
                ChangeElementColor(this.doc, _rf.Id, new Autodesk.Revit.DB.Color(255, 255, 0));
                this.referenceRegions.Add(_rf);

                List<Point> _referencePoints = new List<Point>();
                _referencePoints.Add(new Point((float)this.regionBoundMin.X, (float)this.regionBoundMin.Y, (float)this.regionBoundMax.Z));
                _referencePoints.Add(new Point((float)this.regionBoundMax.X, (float)this.regionBoundMax.Y, (float)this.regionBoundMin.Z));
                this.referenceRegionsPoints.Add(_referencePoints);
            }
            else
                TaskDialog.Show("uDOME", "Hey! Set Region Type (Query or Reference) and select again.");

            //if (this.f == null)
            //{
            //    BuildFloorBoundaries();
            //    this.f = doc.Create.NewFloor(this.floorBoundaries, true);
            //    this.f.get_Parameter(BuiltInParameter.FLOOR_HEIGHTABOVELEVEL_PARAM).Set(1);
            //}
            //else
            //{
            //    this.deleteFloor = true;
            //}

            this.m_ExEvent.Raise();

            if (!this.userstudy_A)
            {

                if (this.worldBoundMax != null && this.worldBoundMin != null &&
                    this.regionBoundMax != null && this.regionBoundMin != null)
                {
                    this.spaceLinesPoints.Clear();
                    this.spaceNodesDegrees.Clear();
                    this.spaceNodes.Clear();
                    this.spaceRegionPresent = true;
                    this.ClearSpacesyntexVisualization();
                    if (this.isHeatmapClicked)
                    {
                        this.isHeatmapClicked = false;
                        this.chkShowHeatmap_Click(new object(), new EventArgs());
                    }
                }
            }
        }

        /// <summary>
        /// Remove the Query and Reference regions from the visibility graphs
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        void btnResetSpaceRegion_Click(object sender, EventArgs e)
        {
            if (this.isHeatmapClicked)
            {
                this.ClearSpacesyntexVisualization();
            }

            //if (this.steersuite == null)
            //    ProcessSpaceGraph();

            if(this.steersuite != null)
                this.steersuite.clear_visibilityGraph();

            this.clearFloorRegions = true;
            this.finishPressed = true;
            this.m_ExEvent.Raise();
        }

        /// <summary>
        /// Button: "Show Space Heatmap"
        /// It computes the degree metric for the the user defined
        /// grid and region bounds using spacesyntex and shows it to user.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>

        public void chkShowHeatmap_Click(object sender, EventArgs e)
        {
            if (this.isHeatmapClicked)
            {
                this.ClearSpacesyntexVisualization();
                return;
            }
            //Check if the Grid and Region Bounds are already set or not.
            if (this.worldBoundMax == null || this.worldBoundMin == null ||
                this.regionBoundMax == null || this.regionBoundMin == null ||
                this.spaceRegionPresent == false || this.queryRegions.Count < 1)
            {
                TaskDialog.Show("uDOME", "Hey! Select the Grid and Region Bounds First to Render Heatmap.");
                this.chkShowHeatmap.Checked = false;
                this.ClearSpacesyntexVisualization();
                return;
            }

            this.isHeatmapClicked = true;
            this.isHeatmap = true;

            ProcessSpaceGraph();

            foreach (List<Point> query in this.queryRegionsPoints)
                this.steersuite.add_queryRegion(query.ElementAt<Point>(0), query.ElementAt<Point>(1));

            if (this.referenceRegionsPoints.Count > 0)
            {
                foreach (List<Point> reference in this.referenceRegionsPoints)
                    this.steersuite.add_refRegion(reference.ElementAt<Point>(0), reference.ElementAt<Point>(1));
            }
            else //otherwise consider query region also as reference region
            {
                foreach (List<Point> reference in this.queryRegionsPoints)
                    this.steersuite.add_refRegion(reference.ElementAt<Point>(0), reference.ElementAt<Point>(1));
            }

            this.comboMetricSelection_Click(new object(), new EventArgs());
        }               

        /// <summary>
        /// Combo: "Metrics List"
        /// List of availabe metrics for spacesyntex analysis
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>

        public void comboMetricSelection_Click(object sender, EventArgs e)
        {
            if (!this.isHeatmapClicked)
                return;
            
            this.spaceNodesDegrees.Clear();
            this.spaceNodes.Clear();
            this.topFaces.Clear();
            this.minDeg = 0;
            this.maxDeg = 1;
            bool _first_point = true;
            //Preparing data to visualize space lines
            SteerSuite steersuite;
            if (this.steersuiteLayout == null)
            {
                steersuite = this.steersuite;
                steersuite.setup_visibilityGraph();
            }
            else
                steersuite = this.steersuiteLayout;
                                    
            PointVector pv = steersuite.get_queryNodes();
            List<List<double>> _metric_values = new List<List<double>>();
            List<double> _value_deg = new List<double>(); //degree
            List<double> _value_ent = new List<double>(); //entropy
            List<double> _value_dep = new List<double>(); //tree depth
            List<double> _value_com = new List<double>(); //weighted combination
            
            foreach (Point _p in pv)
            {
                double value = 0.0;

                double _p_deg = (double)steersuite.get_nodeDegree(_p);
                double _p_ent = (double)steersuite.get_nodeTreeEntropy(_p);
                double _p_dep = (double)steersuite.get_nodeTreeDepth(_p);
                double _p_weighted_comb = ((_p_deg * this.optConfig._degree_weight) + (_p_ent * this.optConfig._entropy_weight) + (_p_dep * this.optConfig._depth_weight));

                _value_deg.Add(_p_deg);
                _value_ent.Add(_p_ent);
                _value_dep.Add(_p_dep);
                _value_com.Add(_p_weighted_comb);

                if (this.comboMetricSelection.SelectedIndex == 0) // Visibility (Degree)
                {
                    value = _p_deg;
                    if (_first_point)
                    {
                        this.minDeg = value;
                        this.maxDeg = value;
                        _first_point = false;
                    }
                }
                else if (this.comboMetricSelection.SelectedIndex == 1) // Organization (Entropy)
                {
                    value = _p_ent;
                    if (_first_point)
                    {
                        this.minDeg = value;
                        this.maxDeg = value;
                        _first_point = false;
                    }
                }
                else if (this.comboMetricSelection.SelectedIndex == 2) // Accessibility (Tree Depth)
                {
                    value = _p_dep;
                    if (_first_point)
                    {
                        this.minDeg = value;
                        this.maxDeg = value;
                        _first_point = false;
                    }
                }
                else if (this.comboMetricSelection.SelectedIndex == 3) // Weighted Combination
                {
                    value = _p_weighted_comb;
                    if (_first_point)
                    {
                        this.minDeg = value;
                        this.maxDeg = value;
                        _first_point = false;
                    }
                }

                if (value < this.minDeg)
                    this.minDeg = value;

                if (value >= this.maxDeg)
                    this.maxDeg = value;
            }

            if (this.comboMetricSelection.SelectedIndex == 2) // Accessibility (Tree Depth)
            {
                for (int v=0; v <_value_dep.Count; v++)
                {
                    if (_value_dep[v] == 0)
                        _value_dep[v] = this.maxDeg;
                }
            }

            //TaskDialog.Show("uDOME", "Degree Count: " + _value_deg.Count.ToString());
            //TaskDialog.Show("uDOME", "Entropy Count: " + _value_ent.Count.ToString());
            //TaskDialog.Show("uDOME", "Depth Count: " + _value_dep.Count.ToString());
            //TaskDialog.Show("uDOME", "Combination Count: " + _value_com.Count.ToString());

            _metric_values.Add(_value_deg); //degree
            _metric_values.Add(_value_ent); //entropy
            _metric_values.Add(_value_dep); //tree depth
            _metric_values.Add(_value_com); //weighted combination
            
            if ((this.maxDeg - this.minDeg) == 0)
            {
                this.minDeg = 0;
                this.maxDeg = 1;
            }

            if (this.comboMetricSelection.SelectedIndex == 0) // Visibility (Degree)
            {
                for (int i = 0; i < this.queryRegions.Count; i++)
                {
                    this.spaceNodes.Add(new List<UV>());
                    this.spaceNodesDegrees.Add(new List<ValueAtPoint>());
                    PlanarFace topFace = null;
                    GeometryElement geomElem = this.queryRegions.ElementAt<Floor>(i).get_Geometry(new Options());
                    
                    foreach (GeometryObject geomObj in geomElem)
                    {
                        if (geomObj is Solid)
                        {
                            Solid solid = (Solid)geomObj;
                            FaceArray faces = solid.Faces;
                            foreach (Face f in faces)
                            {
                                PlanarFace pf = f as PlanarFace;
                                if (null != pf && ((pf.FaceNormal.X < 0.0001) && (pf.FaceNormal.Y < 0.0001)))
                                {
                                    if ((null == topFace)
                                      || (topFace.Origin.Z < pf.Origin.Z))
                                    {
                                        topFace = pf;
                                    }
                                }
                            }
                            this.topFaces.Add(topFace);                            
                            break;
                        }
                    }
                }

                int ind = 0;                       
                foreach (Point p in pv)
                {
                    XYZ _p = new XYZ(p.x, p.y, p.z);
                    //Texture mapping
                    //XYZ => UV                   
                    XYZ q = new XYZ(p.x, p.z, 0) * _meters_to_feet;
                    XYZ d;
                    double u, v;

                    int i = 0,k = 0;
                    for (i = 0; i < this.queryRegionsPoints.Count; i++)
                    {
                        List<Point> region = this.queryRegionsPoints.ElementAt<List<Point>>(i);
                        Point pMin = region.ElementAt<Point>(0);                      
                        Point pMax = region.ElementAt<Point>(1);
                        double pMinX = Math.Min(pMin.x, pMax.x);
                        double pMaxX = Math.Max(pMin.x, pMax.x);
                        double pMinZ = Math.Min(pMin.z, pMax.z);
                        double pMaxZ = Math.Max(pMin.z, pMax.z);

                        if ( (pMinX - this.optConfig._grid_cells_pre_meter) <= _p.X && (pMaxX + this.optConfig._grid_cells_pre_meter) >= _p.X)
                        {
                            if ((pMinZ - this.optConfig._grid_cells_pre_meter) <= _p.Z && (pMaxZ + this.optConfig._grid_cells_pre_meter) >= _p.Z)
                            {
                                d = q - this.topFaces[i].Origin;
                                u = d.DotProduct(this.topFaces[i].XVector);
                                v = d.DotProduct(this.topFaces[i].YVector);
                                this.spaceNodes.ElementAt<List<UV>>(i).Add(new UV(u, v));
                                k = i;

                                List<double> valueList = new List<double>();
                                double pDeg = (_metric_values[0][ind] - this.minDeg) / (this.maxDeg - this.minDeg);
                                valueList.Add(pDeg);                    
                                this.spaceNodesDegrees.ElementAt<List<ValueAtPoint>>(k).Add(new ValueAtPoint(valueList));
                                break;
                            }
                        }
                    }
                    ind++;
                }
            }
            else if (this.comboMetricSelection.SelectedIndex == 1) // Organization (Entropy)
            {
                for (int i = 0; i < this.queryRegions.Count; i++)
                {
                    this.spaceNodes.Add(new List<UV>());
                    this.spaceNodesDegrees.Add(new List<ValueAtPoint>());
                    PlanarFace topFace = null;
                    GeometryElement geomElem = this.queryRegions.ElementAt<Floor>(i).get_Geometry(new Options());

                    foreach (GeometryObject geomObj in geomElem)
                    {
                        if (geomObj is Solid)
                        {
                            Solid solid = (Solid)geomObj;
                            FaceArray faces = solid.Faces;
                            foreach (Face f in faces)
                            {
                                PlanarFace pf = f as PlanarFace;
                                if (null != pf && ((pf.FaceNormal.X < 0.0001) && (pf.FaceNormal.Y < 0.0001)))
                                {
                                    if ((null == topFace)
                                      || (topFace.Origin.Z < pf.Origin.Z))
                                    {
                                        topFace = pf;
                                    }
                                }
                            }
                            this.topFaces.Add(topFace);
                            break;
                        }
                    }
                }
                int ind = 0;
                foreach (Point p in pv)
                {
                    XYZ _p = new XYZ(p.x, p.y, p.z);
                    //Texture mapping
                    //XYZ => UV                   
                    XYZ q = new XYZ(p.x, p.z, 0) * _meters_to_feet;
                    XYZ d;
                    double u, v;

                    int i = 0, k = 0;
                    for (i = 0; i < this.queryRegionsPoints.Count; i++)
                    {
                        List<Point> region = this.queryRegionsPoints.ElementAt<List<Point>>(i);
                        Point pMin = region.ElementAt<Point>(0);
                        Point pMax = region.ElementAt<Point>(1);
                        double pMinX = Math.Min(pMin.x, pMax.x);
                        double pMaxX = Math.Max(pMin.x, pMax.x);
                        double pMinZ = Math.Min(pMin.z, pMax.z);
                        double pMaxZ = Math.Max(pMin.z, pMax.z);

                        if ((pMinX - this.optConfig._grid_cells_pre_meter) <= _p.X && (pMaxX + this.optConfig._grid_cells_pre_meter) >= _p.X)
                        {
                            if ((pMinZ - this.optConfig._grid_cells_pre_meter) <= _p.Z && (pMaxZ + this.optConfig._grid_cells_pre_meter) >= _p.Z)
                            {
                                d = q - this.topFaces[i].Origin;
                                u = d.DotProduct(this.topFaces[i].XVector);
                                v = d.DotProduct(this.topFaces[i].YVector);
                                this.spaceNodes.ElementAt<List<UV>>(i).Add(new UV(u, v));
                                k = i;

                                List<double> valueList = new List<double>();
                                double pDeg = (_metric_values[1][ind] - this.minDeg) / (this.maxDeg - this.minDeg);
                                valueList.Add(pDeg);
                                this.spaceNodesDegrees.ElementAt<List<ValueAtPoint>>(k).Add(new ValueAtPoint(valueList));
                                break;
                            }
                        }
                    }
                    ind++;
                }
            }
            else if (this.comboMetricSelection.SelectedIndex == 2) // Accessibility (Tree Depth)
            {
                for (int i = 0; i < this.queryRegions.Count; i++)
                {
                    this.spaceNodes.Add(new List<UV>());
                    this.spaceNodesDegrees.Add(new List<ValueAtPoint>());
                    PlanarFace topFace = null;
                    GeometryElement geomElem = this.queryRegions.ElementAt<Floor>(i).get_Geometry(new Options());

                    foreach (GeometryObject geomObj in geomElem)
                    {
                        if (geomObj is Solid)
                        {
                            Solid solid = (Solid)geomObj;
                            FaceArray faces = solid.Faces;
                            foreach (Face f in faces)
                            {
                                PlanarFace pf = f as PlanarFace;
                                if (null != pf && ((pf.FaceNormal.X < 0.0001) && (pf.FaceNormal.Y < 0.0001)))
                                {
                                    if ((null == topFace)
                                      || (topFace.Origin.Z < pf.Origin.Z))
                                    {
                                        topFace = pf;
                                    }
                                }
                            }
                            this.topFaces.Add(topFace);
                            break;
                        }
                    }
                }
                int ind = 0;
                foreach (Point p in pv)
                {
                    XYZ _p = new XYZ(p.x, p.y, p.z);
                    //Texture mapping
                    //XYZ => UV                   
                    XYZ q = new XYZ(p.x, p.z, 0) * _meters_to_feet;
                    XYZ d;
                    double u, v;

                    int i = 0, k = 0;
                    for (i = 0; i < this.queryRegionsPoints.Count; i++)
                    {
                        List<Point> region = this.queryRegionsPoints.ElementAt<List<Point>>(i);
                        Point pMin = region.ElementAt<Point>(0);
                        Point pMax = region.ElementAt<Point>(1);
                        double pMinX = Math.Min(pMin.x, pMax.x);
                        double pMaxX = Math.Max(pMin.x, pMax.x);
                        double pMinZ = Math.Min(pMin.z, pMax.z);
                        double pMaxZ = Math.Max(pMin.z, pMax.z);

                        if ((pMinX - this.optConfig._grid_cells_pre_meter) <= _p.X && (pMaxX + this.optConfig._grid_cells_pre_meter) >= _p.X)
                        {
                            if ((pMinZ - this.optConfig._grid_cells_pre_meter) <= _p.Z && (pMaxZ + this.optConfig._grid_cells_pre_meter) >= _p.Z)
                            {
                                d = q - this.topFaces[i].Origin;
                                u = d.DotProduct(this.topFaces[i].XVector);
                                v = d.DotProduct(this.topFaces[i].YVector);
                                this.spaceNodes.ElementAt<List<UV>>(i).Add(new UV(u, v));
                                k = i;

                                List<double> valueList = new List<double>();
                                double pDeg = (_metric_values[2][ind] - this.minDeg) / (this.maxDeg - this.minDeg);
                                valueList.Add(pDeg);
                                this.spaceNodesDegrees.ElementAt<List<ValueAtPoint>>(k).Add(new ValueAtPoint(valueList));
                                break;
                            }
                        }
                    }
                    ind++;
                }
            }
            else if (this.comboMetricSelection.SelectedIndex == 3) // Weighted Combination
            {
                for (int i = 0; i < this.queryRegions.Count; i++)
                {
                    this.spaceNodes.Add(new List<UV>());
                    this.spaceNodesDegrees.Add(new List<ValueAtPoint>());
                    PlanarFace topFace = null;
                    GeometryElement geomElem = this.queryRegions.ElementAt<Floor>(i).get_Geometry(new Options());

                    foreach (GeometryObject geomObj in geomElem)
                    {
                        if (geomObj is Solid)
                        {
                            Solid solid = (Solid)geomObj;
                            FaceArray faces = solid.Faces;
                            foreach (Face f in faces)
                            {
                                PlanarFace pf = f as PlanarFace;
                                if (null != pf && ((pf.FaceNormal.X < 0.0001) && (pf.FaceNormal.Y < 0.0001)))
                                {
                                    if ((null == topFace)
                                      || (topFace.Origin.Z < pf.Origin.Z))
                                    {
                                        topFace = pf;
                                    }
                                }
                            }
                            this.topFaces.Add(topFace);
                            break;
                        }
                    }
                }
                int ind = 0;
                foreach (Point p in pv)
                {
                    XYZ _p = new XYZ(p.x, p.y, p.z);
                    //Texture mapping
                    //XYZ => UV                   
                    XYZ q = new XYZ(p.x, p.z, 0) * _meters_to_feet;
                    XYZ d;
                    double u, v;

                    int i = 0, k = 0;
                    for (i = 0; i < this.queryRegionsPoints.Count; i++)
                    {
                        List<Point> region = this.queryRegionsPoints.ElementAt<List<Point>>(i);
                        Point pMin = region.ElementAt<Point>(0);
                        Point pMax = region.ElementAt<Point>(1);
                        double pMinX = Math.Min(pMin.x, pMax.x);
                        double pMaxX = Math.Max(pMin.x, pMax.x);
                        double pMinZ = Math.Min(pMin.z, pMax.z);
                        double pMaxZ = Math.Max(pMin.z, pMax.z);

                        if ((pMinX - this.optConfig._grid_cells_pre_meter) <= _p.X && (pMaxX + this.optConfig._grid_cells_pre_meter) >= _p.X)
                        {
                            if ((pMinZ - this.optConfig._grid_cells_pre_meter) <= _p.Z && (pMaxZ + this.optConfig._grid_cells_pre_meter) >= _p.Z)
                            {
                                d = q - this.topFaces[i].Origin;
                                u = d.DotProduct(this.topFaces[i].XVector);
                                v = d.DotProduct(this.topFaces[i].YVector);
                                this.spaceNodes.ElementAt<List<UV>>(i).Add(new UV(u, v));
                                k = i;

                                List<double> valueList = new List<double>();
                                double pDeg = (_metric_values[3][ind] - this.minDeg) / (this.maxDeg - this.minDeg);
                                valueList.Add(pDeg);
                                this.spaceNodesDegrees.ElementAt<List<ValueAtPoint>>(k).Add(new ValueAtPoint(valueList));
                                break;
                            }
                        }
                    }
                    ind++;
                }
            }
            else
            {
                return;
            }

            if (this.isHeatmapClicked)
            {
                //TaskDialog.Show("uDOME", "Mean Metric Value: " + mean_metric_value.ToString());
                this.isHeatmap = true;
                this.m_ExEvent.Raise();
            }            
        }

        /// <summary>
        /// Computes the spacegraph and saves the 
        /// SteerSuite instance associated with it.
        /// </summary>
        public void ProcessSpaceGraph()
        {
            if (!userstudy_A)
            {
                this.spaceNodes.Clear();
                this.spaceNodesDegrees.Clear();
            }

            SimWorld world = new SimWorld();
                                  
            uint gridNumX = (uint)((this.worldBoundMax.X - this.worldBoundMin.X) * this.optConfig._grid_cells_pre_meter);
            uint gridNumZ = (uint)((this.worldBoundMax.Z - this.worldBoundMin.Z) * this.optConfig._grid_cells_pre_meter);
            float height = 0;

            // Find all Wall instances in the document by using category filter
            ElementCategoryFilter filter = new ElementCategoryFilter(BuiltInCategory.OST_Walls);

            // Apply the filter to the elements in the active document
            // Use shortcut WhereElementIsNotElementType() to find wall instances only
            FilteredElementCollector collector = new FilteredElementCollector(this.doc);
            IList<Element> walls =
            collector.WherePasses(filter).WhereElementIsNotElementType().ToElements();

            foreach (Element wall in walls)
            {
                BoundingBoxXYZ bbox = wall.get_BoundingBox(null);
                bbox.Min = bbox.Min * _feet_to_meters;
                bbox.Max = bbox.Max * _feet_to_meters;
                world.addObstacle(bbox.Min.X, bbox.Max.X, bbox.Min.Z, bbox.Max.Z, bbox.Min.Y, bbox.Max.Y);
            }

            this.steersuite = new SteerSuite();
            this.steersuite.setConfigFileName(REVIT_CONFIG_FILE);
            this.steersuite.init(world);
                        
            this.steersuite.init_visibilityGraph(
                new Point((float)this.worldBoundMin.X, (float)this.worldBoundMin.Y, (float)this.worldBoundMin.Z),
                new Point((float)this.worldBoundMax.X, (float)this.worldBoundMax.Y, (float)this.worldBoundMax.Z),
                gridNumX,
                gridNumZ,
                height
                );            

            //return this.steersuite;
        }

        /// <summary>
        /// Setup and returns the SteerSuite instance with the
        /// layouts from the given graph.
        /// </summary>
        public SteerSuite ProcessSpaceGraph2(Graph graph)
        {
            SimWorld world = new SimWorld();

            for (UInt64 edge = 0; edge < graph.num_edges(); edge++)
            {
                // Eigen::Vector3d cp_ = (graph.nodes.at(graph.edges[edge]._origin) + graph.nodes.at(graph.edges[edge]._end)) / 2.0;
                DoubleVector n0 = graph.getNode(graph.get_edge((uint)edge)._origin);
                DoubleVector n1 = graph.getNode(graph.get_edge((uint)edge)._end);
                DoubleVector n2 = new DoubleVector();
                n2.Add((n0[0] + n1[0]) / 2.0);
                n2.Add((n0[1] + n1[1]) / 2.0);
                n2.Add((n0[2] + n1[2]) / 2.0);


                // Util::Point cp(cp_(0), cp_(1), cp_(2));
                // Eigen::Vector3d edge_ = (graph.nodes.at(graph.edges[edge]._end) - graph.nodes.at(graph.edges[edge]._origin)).normalized();
                DoubleVector edge_ = new DoubleVector();
                edge_.Add(n1[0] - n0[0]);
                edge_.Add(n1[1] - n0[1]);
                edge_.Add(n1[2] - n0[2]);

                float length = (float)Math.Sqrt((edge_[0] * edge_[0]) + (edge_[1] * edge_[1]) + (edge_[2] * edge_[2]));
                edge_[0] = edge_[0] / length;
                edge_[1] = edge_[1] / length;
                edge_[2] = edge_[2] / length;
                float theta = (float)(Math.Atan2(edge_[2], edge_[0]) * 180.0 / Math.PI);
                world.addOrientedObstacle(n2, length, 0.1f, 0.0f, 1.0f, -theta);
            }

            SteerSuite ss = new SteerSuite();
            ss.setConfigFileName(REVIT_CONFIG_FILE);
            ss.init(world);            

            uint gridNumX = (uint)((this.worldBoundMax.X - this.worldBoundMin.X) * this.optConfig._grid_cells_pre_meter);
            uint gridNumZ = (uint)((this.worldBoundMax.Z - this.worldBoundMin.Z) * this.optConfig._grid_cells_pre_meter);
            float height = 0;

            ss.init_visibilityGraph(
                new Point((float)this.worldBoundMin.X, (float)this.worldBoundMin.Y, (float)this.worldBoundMin.Z),
                new Point((float)this.worldBoundMax.X, (float)this.worldBoundMax.Y, (float)this.worldBoundMax.Z),
                gridNumX,
                gridNumZ,
                height
                );

            foreach (List<Point> query in this.queryRegionsPoints)
                ss.add_queryRegion(query.ElementAt<Point>(0), query.ElementAt<Point>(1));

            if(this.referenceRegionsPoints.Count() > 0)
                foreach (List<Point> reference in this.referenceRegionsPoints)
                    ss.add_refRegion(reference.ElementAt<Point>(0), reference.ElementAt<Point>(1));
            else
                foreach (List<Point> reference in this.queryRegionsPoints)
                    ss.add_refRegion(reference.ElementAt<Point>(0), reference.ElementAt<Point>(1));

            ss.setup_visibilityGraph();

            return ss;
        }

        /// <summary>
        /// Computes and returns the default metrices values for the given graph
        /// </summary>
        /// <param name="g"></param>
        /// <returns></returns>
        public DoubleVector GetDefaultMetricsValues(Graph g)
        {
            SteerSuite s = new SteerSuite();
            SimWorld world = new SimWorld();
            world.clearWorld();

            var steerFunc = new HierarchicalOptimization(world, s);

            g.clearMinkowskiSums();
            g.computeMinkowskiSums(this.optConfig._clearence_distance);
            steerFunc.setGraph(g);
            steerFunc.setOptimiationConfiguration(this.optConfig);
            steerFunc.setMaxEvals(this.optConfig._max_fevals);

            OptimizationParameters ops = new OptimizationParameters(); //empty         

            steerFunc.setOptimizationParameters(ops);

            //Y and Z axis are already swapped
            steerFunc._gridBound1 = new Point((float)this.worldBoundMin.X, 0, (float)this.worldBoundMin.Z);
            steerFunc._gridBound2 = new Point((float)this.worldBoundMax.X, 0, (float)this.worldBoundMax.Z);
            steerFunc._gridNumX = (uint)((steerFunc._gridBound2.x - steerFunc._gridBound1.x) * this.optConfig._grid_cells_pre_meter);
            steerFunc._gridNumZ = (uint)((steerFunc._gridBound2.z - steerFunc._gridBound1.z) * this.optConfig._grid_cells_pre_meter);
            steerFunc._height = 0;
            steerFunc._meanDegree = 0;

            steerFunc._steerSuite.init_visibilityGraph(steerFunc._gridBound1, steerFunc._gridBound2, steerFunc._gridNumX, 
                steerFunc._gridNumZ, steerFunc._height);
                        
            for (int r = 0; r < this.studyParams._queryRegions.Count; r++)
		    {
                steerFunc._steerSuite.add_queryRegion(this.studyParams._queryRegions[r][0], this.studyParams._queryRegions[r][1]);
            }
            for (int r = 0; r < this.studyParams._refRegions.Count; r++)
		    {
                steerFunc._steerSuite.add_refRegion(this.studyParams._refRegions[r][0], this.studyParams._refRegions[r][1]);
            }
            
            s.init(world);

            DoubleVector best_x2 = new DoubleVector();

            steerFunc.preprocessEnvironment(best_x2);
            double metric = steerFunc.calcMultiObjective(best_x2);
            DoubleVector metrics = steerFunc._calcMultiObjective(best_x2);
            double degree = steerFunc._steerSuite.get_meanDegree();
            FloatVector entropy_depth = steerFunc._steerSuite.get_meanTreeProp();

            steerFunc.postprocessEnvironment(best_x2);
            //s.finish();

            DoubleVector dv = new DoubleVector();
            dv.Add(degree); // mean degree
            dv.Add(entropy_depth[1]); //mean entropy
            dv.Add(entropy_depth[0]); //mean tree depth
            dv.Add(metric); // aggregated metric
            dv.Add(metrics[0]); //weighted degree
            dv.Add(metrics[2]); //weighted entropy
            dv.Add(metrics[1]); //weighted tree depth
            dv.Add(metrics[3]); //clearence
            dv.Add(metrics[4]); //alignment

            return dv;
        }

        /// <summary>
        /// Button: "Finish"
        /// Finishes all the current going operations and selections.
        /// It sets the currently selected layout from the Best Layouts list as
        /// default layout and resets all other controls indicating that optimization
        /// is done and system is back to normal.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>

        public void btnFinishOperations_Click(object sender, EventArgs e)
        {
            this.ok.Visible = true;

            int s_index = this.comboBestResults.SelectedIndex;
            int selectedLayoutIndex;

            if (this.comboBestResults.SelectedIndex == this.comboBestResults.Items.Count - 1)
                selectedLayoutIndex = this.comboBestResults.SelectedIndex;
            else
                selectedLayoutIndex = this.steersuiteLayoutsIndexes.ElementAt<int>(s_index);
            //int selectedLayoutIndex = this.comboBestResults.SelectedIndex;

            if (this.comboBestResults.SelectedIndex == this.comboBestResults.Items.Count - 1)
            {
                //Rest the layout to default locations for the selected elements
                foreach (Element ele in this.user_selected_elements_all)
                {
                    if (this.doc.ActiveView.GetElementOverrides(ele.Id).ProjectionFillColor.IsValid)
                        ChangeElementColor(this.doc, ele.Id, new Autodesk.Revit.DB.Color(255, 255, 255));
                }
                ResetLayout();
            }
            else if (selectedLayoutIndex >= 0)
            {
                foreach (Element ele in this.user_selected_elements_all)
                {
                    int elementIndex = this.eIds.FindIndex(ElementId => ElementId == ele.Id);

                    int origin = this.edges[elementIndex].ElementAt<int>(0);
                    int end = this.edges[elementIndex].ElementAt<int>(1);

                    DoubleVector v0 = this.bestNodes[selectedLayoutIndex].getNode((uint)origin);
                    XYZ p0 = ((new XYZ(v0[0], v0[2], v0[1])) * _meters_to_feet);  //Y and Z axis are swapped in Revit

                    DoubleVector v1 = this.bestNodes[selectedLayoutIndex].getNode((uint)end);
                    XYZ p1 = ((new XYZ(v1[0], v1[2], v1[1])) * _meters_to_feet);  //Y and Z axis are swapped in Revit

                    // get the current wall location
                    LocationCurve wallLocation = ele.Location as LocationCurve;

                    // create a new LineBound                
                    Line newWallLine = Line.CreateBound(p0, p1);

                    // update the wall curve
                    wallLocation.Curve = newWallLine;

                    if (this.doc.ActiveView.GetElementOverrides(ele.Id).ProjectionFillColor.IsValid)
                        ChangeElementColor(this.doc, ele.Id, new Autodesk.Revit.DB.Color(255, 255, 255));                        
                }
                    
                this.m_ExEvent.Raise();
            }

            if (this.userstudyMode)
            {
                string newResultLine = "";
                string optimizationResultFile = this.pathToParticipant + Path.DirectorySeparatorChar + "StudyResults-" +
                    Path.GetFileNameWithoutExtension(doc.PathName) + OPT_OUTPUT_FILETYPE;

                double mean_metric = this.steersuiteLayoutsMetrics.ElementAt<double>(selectedLayoutIndex);
                this.watch.Stop();
                long elapsedMs = this.watch.ElapsedMilliseconds;
                StringBuilder results = new StringBuilder(); // user-study data dump

                if (!System.IO.File.Exists(optimizationResultFile) && this.firstIteration)
                {
                    newResultLine = string.Format("{0},{1},{2},{3},{4},{5}", "PARTICIPANT_NUM", "OPT_NUM", "ITERATION_TIME", "OPTIMIZATION_TIME", "SOLUTION_NUM", "MEAN_METRIC_VAL");                    
                    results.AppendLine(newResultLine);

                    if (selectedLayoutIndex < this.comboBestResults.Items.Count - 1)
                    {
                        newResultLine = string.Format("{0},{1},{2},{3},{4},{5}", this.partcipantId, (this.participantTurns - 1), (elapsedMs / 1000), (this.watchIterationOpt.ElapsedMilliseconds / 1000),
                        (selectedLayoutIndex + 1), mean_metric);
                    }
                    else
                    {
                        newResultLine = string.Format("{0},{1},{2},{3},{4},{5}", this.partcipantId, (this.participantTurns - 1), (elapsedMs / 1000), (this.watchIterationOpt.ElapsedMilliseconds / 1000),
                        "Default", mean_metric);
                    }
                    results.AppendLine(newResultLine);                    
                }
                else
                {
                    if (selectedLayoutIndex < this.comboBestResults.Items.Count - 1)
                    {
                        newResultLine = string.Format("{0},{1},{2},{3},{4},{5}", this.partcipantId, (this.participantTurns - 1), (elapsedMs / 1000), (this.watchIterationOpt.ElapsedMilliseconds / 1000),
                        (selectedLayoutIndex + 1), mean_metric);
                    }
                    else
                    {
                        newResultLine = string.Format("{0},{1},{2},{3},{4},{5}", this.partcipantId, (this.participantTurns - 1), (elapsedMs / 1000), (this.watchIterationOpt.ElapsedMilliseconds / 1000),
                        "Default", mean_metric);
                    }
                    results.AppendLine(newResultLine);
                }

                try
                {
                    File.AppendAllText(optimizationResultFile, results.ToString());
                }
                catch (Exception ex)
                {
                    TaskDialog.Show("uDOME", "Exception in Writing User-Study Results File. " + ex.ToString());
                }

                this.firstIteration = false;
                this.iterationTimeEnabled = false;
            }

            //Clear everything for the new Optimization Cycle
            this.btnTranslation.Enabled = true;
            this.btnRotation.Enabled = true;
            ClearControls();            
        }

        /// <summary>
        /// Button: "Get New Layouts"
        /// Runs the optimization and perform corressponding tasks i.e.
        /// updating Revit Model, etc.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>

        public void ok_Click(object sender, EventArgs e)
        {
            if (this.user_selected_elements_all.Count <= 0)
            {
                TaskDialog.Show("uDOME", "Hey! Pick the Elements First to Get New Layouts for them.");
                return;
            }

            //It means no constrians are set and "Set" Button is pressed.
            //So create default params: LB=-40, UB=35, Direction=(1,0,0)?
            if (!this.constraintsPresent || ((int)this.params_.size() < 1))
            {
                TaskDialog.Show("uDOME", "Hey! First set some constraints on the selected elements.");
                return;
            }

            //Checks if the space regions are defined or not
            if (!this.spaceRegionPresent)
            {
                TaskDialog.Show("uDOME", "Hey! Select the Grid and Region Bounds First to Get the new Layouts");
                return;
            }

            //check if query region is present
            if (this.queryRegions.Count <= 0)
            {
                TaskDialog.Show("uDOME", "Hey! Select the Query Region First to Get the new Layouts");
                return;
            }


            
            //Store the graph for the Architectural Model currently open in ActiveUIDocument inside Revit 
            //this.edges.Clear();
            //this.verts.Clear();
            //this.StoreGraph(this.doc);
            this.comboBestResults.Items.Clear();
            this.bestNodes.Clear(); //Graphs of best Layouts
            this.steersuiteLayoutsMetrics.Clear();
            this.steersuiteLayoutsIndexes.Clear();

            //imports stored graph in graph object            
            //this.graph.importFromFile(graphFileName);

            //this.graph has been initialized already.
            this.graph.computeMinkowskiSums(0.5);
            Console.WriteLine("Finished importing graph");                              
            
            SimWorld world = new SimWorld();
            SteerSuite steerSuite = new SteerSuite();
            // steerSuite.setConfigFileName(REVIT_CONFIG_FILE_2);

            String optimizationFilename = "";
            optimizationFilename = this.pathToParticipant + Path.DirectorySeparatorChar + "Opt[" + this.participantTurns + "]-" + "OptData-" +
                    Path.GetFileNameWithoutExtension(doc.PathName) + OPT_OUTPUT_FILETYPE;

            this.optConfig._logFileName = optimizationFilename;

            OptimizationParameters tmp_params_ = new OptimizationParameters();
            var steerFunc = new RoundRobinOptimization(world, steerSuite);

            if (this.comboOptimizer.SelectedIndex == 0)
                steerFunc = new RoundRobinOptimization(world, steerSuite);
            else if (this.comboOptimizer.SelectedIndex == 1)
                steerFunc = new HierarchicalOptimization(world, steerSuite);
            
            steerFunc.setOptimizationParameters(params_);
            steerFunc.setOptimiationConfiguration(this.optConfig);
            steerFunc.setGraph(this.graph);
            steerFunc.setMaxEvals(this.optConfig._max_fevals);  
                      
            //Y and Z axis are already swapped
            steerFunc._gridBound1 = new Point((float)this.worldBoundMin.X, 0, (float)this.worldBoundMin.Z);
            steerFunc._gridBound2 = new Point((float)this.worldBoundMax.X, 0, (float)this.worldBoundMax.Z);            
            steerFunc._gridNumX = (uint)((steerFunc._gridBound2.x - steerFunc._gridBound1.x) * this.optConfig._grid_cells_pre_meter);
            steerFunc._gridNumZ = (uint)((steerFunc._gridBound2.z - steerFunc._gridBound1.z) * this.optConfig._grid_cells_pre_meter);
            steerFunc._height = 0;
            steerFunc._meanDegree = 0;

            tmp_params_._visibilitiRegions.Clear();
            PointVector points__ = new PointVector();
            points__.Add(steerFunc._gridBound1);
            points__.Add(steerFunc._gridBound2);
            tmp_params_._visibilitiRegions.Add(points__);

            steerFunc._steerSuite.init_visibilityGraph(steerFunc._gridBound1, steerFunc._gridBound2, 
                steerFunc._gridNumX, steerFunc._gridNumZ, steerFunc._height);

            foreach (List<Point> query in this.queryRegionsPoints)
            {
                steerFunc._steerSuite.add_queryRegion(query.ElementAt<Point>(0), query.ElementAt<Point>(1));
                PointVector points_ = new PointVector();
                foreach (Point p in query)
                {
                    points_.Add(p);
                }
                tmp_params_._queryRegions.Add(points_);
            }

            if (this.referenceRegionsPoints.Count > 0)
            {
                foreach (List<Point> reference in this.referenceRegionsPoints)
                {
                    steerFunc._steerSuite.add_refRegion(reference.ElementAt<Point>(0), reference.ElementAt<Point>(1));
                    PointVector points_ = new PointVector();
                    foreach (Point p in reference)
                    {
                        points_.Add(p);
                    }
                    tmp_params_._refRegions.Add(points_);
                }
            }
            else //otherwise consider query region also as reference region
            {
                foreach (List<Point> reference in this.queryRegionsPoints)
                    steerFunc._steerSuite.add_refRegion(reference.ElementAt<Point>(0), reference.ElementAt<Point>(1));
            }

            Console.WriteLine("Done setting up optimization, starting optimization");
            string pathToDevSub = "";

            if (!this.userstudyMode)
            {
                string pathToBuild = Path.GetFullPath(Path.Combine(this.pathToSteerSuite, @"..\"));
                string pathToDevResults = pathToBuild + "DEV-" + RESULT_DIR;

                if (!System.IO.Directory.Exists(pathToDevResults))
                {
                    System.IO.DirectoryInfo dirInfo = System.IO.Directory.CreateDirectory(pathToDevResults);
                }

                pathToDevSub = pathToDevResults + System.IO.Path.DirectorySeparatorChar + "Dev-Run-" + DateTime.Now.ToString("MM.dd.yyyy-hh.mm.ss");

                if (!System.IO.Directory.Exists(pathToDevSub))
                {
                    System.IO.DirectoryInfo dirInfo = System.IO.Directory.CreateDirectory(pathToDevSub);
                }
                string filename = pathToDevSub + Path.DirectorySeparatorChar +
                        this.DEFAULT_LAYOUT_FILE_PREFIX + Path.GetFileNameWithoutExtension(doc.PathName) + this.GRAPH_FILETYPE;
                this.StoreGraph(this.graph2, filename);
                this.graph2.saveSVGToFile(filename + ".svg", tmp_params_);

                string convergenceFile = pathToDevSub + Path.DirectorySeparatorChar + "OptData-" + DateTime.Now.ToString("MM.dd.yyyy-hh.mm.ss") + 
                    Path.GetFileNameWithoutExtension(doc.PathName) + OPT_OUTPUT_FILETYPE;

                steerFunc.logOptimization(convergenceFile);

                string optParams = pathToDevSub + Path.DirectorySeparatorChar + "Opt" + PARAM_FILENAME + DateTime.Now.ToString("MM.dd.yyyy-hh.mm.ss") + 
                    Path.GetFileNameWithoutExtension(doc.PathName) + ".xml";

                HelperSaveParamsToFile(optParams);
            }
            if (this.userstudyMode)
            {
                string optParams = this.pathToParticipant + Path.DirectorySeparatorChar + "Opt[" + this.participantTurns + "]" + PARAM_FILENAME +
                    Path.GetFileNameWithoutExtension(doc.PathName) + ".xml";

                HelperSaveParamsToFile(optParams);
                
                this.watchIterationOpt = System.Diagnostics.Stopwatch.StartNew();
                steerFunc.logOptimization(optimizationFilename);
            }

            // TaskDialog.Show("uDOME", "Optimization started.");
            steerFunc.optimize();
            // TaskDialog.Show("uDOME", "Optimization done.");
            Console.WriteLine("Optimization complete");

            if (this.userstudyMode)
            {
                this.watchIterationOpt.Stop();
            }

            StringBuilder results = new StringBuilder(); // user-study data dump
            string optResultFile = ""; //user-study: opt results file name
            bool fileExist = false;
            
            for (uint m = 0; m < steerFunc.getNumMembers(); m++)
            {
                // TaskDialog.Show("uDOME", "Getting member metrics.");
                world.clearWorld();
                steerSuite.init(world);
                DoubleVector best_x = steerFunc.getBestMember(m);
                steerFunc.preprocessEnvironment(best_x);
                double metric = steerFunc.calcMultiObjective(best_x);
                DoubleVector metrics = steerFunc._calcMultiObjective(best_x);
                double _deg = steerFunc._steerSuite.get_meanDegree();
                FloatVector _entr_depth = steerFunc._steerSuite.get_meanTreeProp();
                // TaskDialog.Show("uDOME", "Done getting member metrics.");
                if (this.userstudyMode)
                {
                    string gFile = this.pathToParticipant + Path.DirectorySeparatorChar + "Graph-Opt[" + this.participantTurns + "]-" +
                        "Sol[" + (((int)m) + 1) + "]-" + Path.GetFileNameWithoutExtension(doc.PathName) + this.GRAPH_FILETYPE;
                    //storinh each candidate layout
                    StoreGraph(steerFunc._tmp_graph, gFile);
                    steerFunc._tmp_graph.saveSVGToFile(gFile + ".svg", tmp_params_);
                }
                else
                {
                    string gFile = pathToDevSub + Path.DirectorySeparatorChar + "Graph-Opt-" +
                        "Sol[" + (((int)m) + 1) + "]-" + Path.GetFileNameWithoutExtension(doc.PathName) + this.GRAPH_FILETYPE;
                    //storinh each candidate layout
                    StoreGraph(steerFunc._tmp_graph, gFile);
                    steerFunc._tmp_graph.saveSVGToFile(gFile + ".svg", tmp_params_);
                }
                // TaskDialog.Show("uDOME", "graph files saved");
                Graph _tmp_graph = new Graph();
                _tmp_graph.initFrom(steerFunc._tmp_graph);
                this.bestNodes.Add(_tmp_graph);
                SteerSuite ss = ProcessSpaceGraph2(_tmp_graph);
                float mDeg = (float) metric;

                //if (mDeg > this.maxDeg)
                //    this.maxDeg = mDeg;
                //if (mDeg < this.minDeg)
                //    this.minDeg = mDeg;

                //this.comboBestResults.Items.Insert((int)m, "M" + (m + 1).ToString() + ": Value " + Math.Round(mDeg, 2).ToString());
                this.steersuiteLayoutsMetrics.Add(Math.Round(mDeg, 2));
                this.steersuiteLayouts.Add(ss);


                if (this.userstudyMode)
                {
                    optResultFile = this.pathToParticipant + Path.DirectorySeparatorChar + "OptResults-" + 
                        Path.GetFileNameWithoutExtension(doc.PathName) + OPT_OUTPUT_FILETYPE;
                }
                else
                {
                    optResultFile = pathToDevSub + Path.DirectorySeparatorChar + "OptResults-" + 
                        Path.GetFileNameWithoutExtension(doc.PathName) + OPT_OUTPUT_FILETYPE;
                }
                                
                if (!System.IO.File.Exists(optResultFile) && !fileExist)
                {
                    string newResultLine = string.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10}", "OPT_NUM", "SOL_NUM",
                        "MEAN_DEGREE", "MEAN_ENTROPY", "MEAN_TREEDEPTH",
                        "AGGREGATED_METRIC_VALUE",
                        "WEIGHTED_DEGREE", "WEIGHTED_ENTROPY", "WEIGHTED_TREEDEPTH", "CLEARENCE", "ALIGNMENT");
                    results.AppendLine(newResultLine);

                    fileExist = true;
                }

                string newDataLine = string.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10}", this.participantTurns, m + 1,
                    Math.Round(_deg, 4).ToString(), Math.Round(_entr_depth[1], 4).ToString(), Math.Round(_entr_depth[0], 4).ToString(),
                    Math.Round(mDeg, 4).ToString(),
                    Math.Round(metrics[0], 4).ToString(), Math.Round(metrics[2], 4).ToString(), Math.Round(metrics[1], 4).ToString(),
                    Math.Round(metrics[3], 4).ToString(), Math.Round(metrics[4], 4).ToString());
                results.AppendLine(newDataLine);
                
                steerFunc.postprocessEnvironment(best_x);
                //steerSuite.finish();
            }

            List<double> tmpMetricList = new List<double>();
            foreach (double d in this.steersuiteLayoutsMetrics)
                tmpMetricList.Add(d);

            tmpMetricList.Sort();
            tmpMetricList.Reverse();

            for(int n= 0; n < tmpMetricList.Count; n++)
            {
                this.comboBestResults.Items.Insert(n, "Member-" + (n + 1).ToString());
            }

            for (int z = 0; z < tmpMetricList.Count; z++)
            {
                for (int m = 0; m < this.steersuiteLayoutsMetrics.Count; m++)
                {
                    if ( (tmpMetricList.ElementAt<double>(z) == this.steersuiteLayoutsMetrics.ElementAt<double>(m)) && (!this.steersuiteLayoutsIndexes.Contains(m)))
                    {
                        this.steersuiteLayoutsIndexes.Add(m);
                        break;
                    }
                }
            }

            world.clearWorld();
            steerSuite.init(world);
            DoubleVector best_x2 = new DoubleVector();
            for (uint i = 0; i < params_.size(); i++ )
            {
                best_x2.Add(0.0);
            }
            steerFunc.preprocessEnvironment(best_x2);
            double metric2 = steerFunc.calcMultiObjective(best_x2);
            DoubleVector metrics2 = steerFunc._calcMultiObjective(best_x2);
            double _deg2 = steerFunc._steerSuite.get_meanDegree();
            FloatVector _entr_depth2 = steerFunc._steerSuite.get_meanTreeProp();
            steerFunc.postprocessEnvironment(best_x2);
            //steerSuite.finish();

            Graph tmp_graph = new Graph();
            tmp_graph.initFrom(this.graph);
            this.bestNodes.Add(tmp_graph);
            SteerSuite ss_default = ProcessSpaceGraph2(tmp_graph);
            double defDeg = metric2; // ss_default.get_meanDegree();
            this.steersuiteLayoutsMetrics.Add(Math.Round(defDeg, 2));

            // if (this.userstudyMode)
            {
                string newResultLine = string.Format("{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10}", this.participantTurns, "Default",
                    Math.Round(_deg2, 4).ToString(), Math.Round(_entr_depth2[1], 4).ToString(), Math.Round(_entr_depth2[0], 4).ToString(),
                    Math.Round(defDeg, 4).ToString(),
                    Math.Round(metrics2[0], 4).ToString(), Math.Round(metrics2[2], 4).ToString(), Math.Round(metrics2[1], 4).ToString(),
                    Math.Round(metrics2[3], 4).ToString(), Math.Round(metrics2[4], 4).ToString());
               
                results.AppendLine(newResultLine);
            }

            //if (defDeg > this.maxDeg)
            //    this.maxDeg = defDeg;
            //if (defDeg < this.minDeg)
            //    this.minDeg = defDeg;

            this.comboBestResults.Items.Insert(this.comboBestResults.Items.Count, "Member-default ");            
            this.steersuiteLayouts.Add(ss_default);
            //steerSuite.finish();
            //Visible the following controls at this time
            this.ok.Hide();
            this.comboBestResults.Visible = true;
            this.btnFinishOperations.Visible = true;
            this.labBestResults.Visible = true;

            // if (this.userstudyMode)
            {
                try
                {
                    File.AppendAllText(optResultFile, results.ToString());                    
                }
                catch (Exception ex)
                {
                    TaskDialog.Show("uDOME", "Exception in Writing Optimization Results File. " + ex.ToString());
                }
                
                this.participantTurns++;
            }

            this.btnTranslation.Enabled = false;
            this.btnRotation.Enabled = false;
        }
        
        /// <summary>
        /// Combo Box: "Layouts"
        /// Wheneve a new layout is selected from the list, it
        /// allows the user to visualize that selected layout inside Revit while
        /// also showing the orignal layout.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>

        public void comboBestResults_Click(object sender, System.EventArgs e)
        {
            //System.Windows.Forms.ComboBox comboBox = (System.Windows.Forms.ComboBox)sender;
            //int selectedLayoutIndex = comboBox.SelectedIndex;

            if (this.comboBestResults.SelectedIndex == this.comboBestResults.Items.Count - 1)
            {
                //Rest the layout to default locations for the selected elements
                ResetLayout();
                //this.steersuiteLayout = this.steersuite;
                this.steersuiteLayout = this.steersuiteLayouts.ElementAt<SteerSuite>(this.comboBestResults.SelectedIndex);
                if (this.isHeatmapClicked)
                {
                    this.ClearSpacesyntexVisualization();
                    this.isHeatmapClicked = true;
                    this.isHeatmap = true;
                    this.comboMetricSelection_Click(new object(), new EventArgs());
                }
            }
            else
            {
                int s_index = this.comboBestResults.SelectedIndex;
                int selectedLayoutIndex = this.steersuiteLayoutsIndexes.ElementAt<int>(s_index);
                foreach (Element ele in this.user_selected_elements_all)
                {
                    int elementIndex = this.eIds.FindIndex(ElementId => ElementId == ele.Id);

                    int origin = this.edges[elementIndex].ElementAt<int>(0);
                    int end = this.edges[elementIndex].ElementAt<int>(1);

                    DoubleVector v0 = this.bestNodes[selectedLayoutIndex].getNode((uint)origin);
                    XYZ p0 = ((new XYZ(v0[0], v0[2], v0[1])) * _meters_to_feet);  //Y and Z axis are swapped in Revit

                    DoubleVector v1 = this.bestNodes[selectedLayoutIndex].getNode((uint)end);
                    XYZ p1 = ((new XYZ(v1[0], v1[2], v1[1])) * _meters_to_feet);  //Y and Z axis are swapped in Revit

                    // get the current wall location
                    LocationCurve wallLocation = ele.Location as LocationCurve;

                    // create a new LineBound                
                    Line newWallLine = Line.CreateBound(p0, p1);

                    // update the wall curve
                    wallLocation.Curve = newWallLine;
                }
                this.steersuiteLayout = this.steersuiteLayouts.ElementAt<SteerSuite>(selectedLayoutIndex);
                if (this.isHeatmapClicked)
                {
                    this.ClearSpacesyntexVisualization();
                    this.isHeatmapClicked = true;
                    this.isHeatmap = true;
                    this.comboMetricSelection_Click(new object(), new EventArgs());
                }
                this.m_ExEvent.Raise();
            }
        }


        /// <summary>
        /// Creates an OptimizationParameter object with the given nodes id and
        /// user defined lower and upper bounds for the translation.
        /// </summary>
        /// <param name="node_ids"></param>
        /// <returns>An OptimizationParameter object with the given nodes id.</returns>

        private OptimizationParameter getOptimizationParams(Vector_SizeT node_ids, double lb, double ub, XYZ direction)
        {
            if (this.isRotational)
            {
                DoubleVector axis = new DoubleVector();
                axis.Add(direction.X * _feet_to_meters); // X-Axis
                axis.Add(0); // Y-Axis and Z-Axis are swapped in SteerSite
                axis.Add(direction.Y * _feet_to_meters); // Y-Axis and Z-Axis are swapped in SteerSite

                OptimizationParameter p = new OptimizationParameter(
                    0,
                    lb * (Math.PI / 180),
                    ub * (Math.PI / 180),
                    node_ids,
                    OptimizationParameter.ParameterType.Rotation);

                p.setRotationVector(axis);
                return p;
            }
            else
            {
                DoubleVector axis = new DoubleVector();
                axis.Add(direction.X); // X-Axis
                axis.Add(0); // Y-Axis and Z-Axis are swapped in SteerSite
                axis.Add(direction.Y); // Y-Axis and Z-Axis are swapped in SteerSite

                OptimizationParameter p = new OptimizationParameter(
                    0,
                    lb * _feet_to_meters,
                    ub * _feet_to_meters,
                    node_ids,
                    OptimizationParameter.ParameterType.Translation);

                p.setTranslation(axis);

                return p;
            }
        }

        /// <summary>
        /// Stores the given graph at the given location
        /// </summary>
        /// <param name="g"></param>
        /// <param name="fileName"></param>
        private void StoreGraph(Graph g, string fileName)
        {
            try
            {
                string prompt = "";
                int num_of_edges = (int)g.num_edges();
                int num_of_nodes = (int)g.num_nodes();

                for(int i=0; i < num_of_nodes; i++)
                {
                    DoubleVector dv = g.getNode((uint)i);
                    prompt += "v " + dv[0] + " " + dv[1] + " " + dv[2] + "\n";
                }

                prompt += "\n";

                for (int j = 0; j < num_of_edges; j++)
                {
                    Edge e = g.get_edge((uint)j);
                    prompt += "e " + e._origin + " " + e._end + "\n";
                }

                System.IO.StreamWriter file = new System.IO.StreamWriter(fileName);
                file.WriteLine(prompt);
                file.Close();
            }
            catch (Exception ex)
            {
                TaskDialog.Show("uDOME", "Exception in Storing Graph. " + ex.ToString());
            }
        }
        /// <summary>
        /// Stores the layout of the given document as graph representation at the given location
        /// </summary>
        /// <param name="doc"></param>
        /// <param name="graphFile"></param>
        private void StoreGraph(Document doc, string graphFile = "")
        {
            if (graphFile.Length < 1)
                graphFile = this.graphFileName;
            try
            {
                this.edges.Clear();
                this.verts.Clear();
                this.eIds.Clear();
                // Find all Wall instances in the document by using category filter
                ElementCategoryFilter filter = new ElementCategoryFilter(BuiltInCategory.OST_Walls);

                // Apply the filter to the elements in the active document
                // Use shortcut WhereElementIsNotElementType() to find wall instances only
                FilteredElementCollector collector = new FilteredElementCollector(doc);
                
                IList<Element> walls =
                collector.WherePasses(filter).WhereElementIsNotElementType().ToElements();
                String prompt = ""; // "The walls in the current document are:\n";

                foreach (Element e in walls)
                {
                    Autodesk.Revit.DB.LocationCurve lineLoc;
                    lineLoc = e.Location as LocationCurve;
                    
                    List<int> edge_ = new List<int>();

                    Autodesk.Revit.DB.XYZ origin = lineLoc.Curve.GetEndPoint(0);
                    Autodesk.Revit.DB.XYZ end = lineLoc.Curve.GetEndPoint(1);

                    bool contains = false;
                    int or = 0, ed = 0;
                    for (int i = 0; i < this.verts.Count; i++)
                    {
                        XYZ p = this.verts[i];
                        double l = Math.Sqrt(Math.Pow((p.X - origin.X), 2) +
                            Math.Pow((p.Y - origin.Y), 2) + Math.Pow((p.Z - origin.Z), 2));
                        if (l < 0.001)
                        {
                            contains = true;
                            or = i;
                            break;
                        }
                    }
                    
                    if (!contains)
                    {
                        this.verts.Add(origin);
                        or = this.verts.Count - 1;                        
                        //  prompt += "<" + origin.X + ", " + origin.Z + ", " + origin.Y + ">\n";
                    }

                    contains = false;
                    for (int i = 0; i < this.verts.Count; i++)
                    {
                        XYZ p = this.verts[i];
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
                        this.verts.Add(end);
                        ed = this.verts.Count -1;                        
                        // prompt += "<" + end.X + ", " + end.Z + ", " + end.Y + ">\n";
                    }
                    
                    edge_.Add(or);
                    edge_.Add(ed);
                    this.edges.Add(edge_);
                    this.eIds.Add(e.Id);                    
                }

                foreach (XYZ p in this.verts)
                {   // converted axis here
                    XYZ po = p * _feet_to_meters;
                    prompt += "v " + po.X + " " + po.Z + " " + po.Y + "\n";
                }
                prompt += "\n";
                foreach (List<int> edge_ in this.edges)
                {
                    prompt += "e " + edge_[0] + " " + edge_[1] + "\n";
                }

                System.IO.StreamWriter file = new System.IO.StreamWriter(graphFile);
                file.WriteLine(prompt);

                file.Close();

            }
            catch (Exception ex)
            {
                TaskDialog.Show("uDOME", "Exception in Storing Graph. " + ex.ToString());
            }
        }

        /// <summary>
        /// Creats New Wall inside Revit.
        /// </summary>
        /// <param name="document">Document inside which the Wall is going to be created</param>
        /// <param name="start">Starting point of the Line/Curve</param>
        /// <param name="end">Ending point of the Line/Curve</param>
        /// <param name="level">Level within document where the Wall will be created</param>
        /// <param name="offset"></param>
        /// <param name="height">Height of Wall</param>
        /// <param name="wallType">Type of Wall i.e. Basic Wall</param>
        /// <returns></returns>

        public ICollection<ElementId> CopyLayout(Autodesk.Revit.DB.Document document, XYZ translation)
        {
            // Find all Wall instances in the document by using category filter
            ElementCategoryFilter filter = new ElementCategoryFilter(BuiltInCategory.OST_Walls);

            // Apply the filter to the elements in the active document
            // Use shortcut WhereElementIsNotElementType() to find wall instances only
            FilteredElementCollector collector = new FilteredElementCollector(this.doc);
            IList<Element> walls =  collector.WherePasses(filter).WhereElementIsNotElementType().ToElements();

            ICollection<ElementId> elements = new List<ElementId>();

            foreach (Element wall in walls)
                elements.Add(wall.Id);
            
            // Return the element ids of new Copied elements given in ele
            return ElementTransformUtils.CopyElements(doc, elements, translation);            
        }

        /// <summary>
        /// Button: "Set"
        /// It is called when the user presses the "Set" button to
        /// finalize the constraint changes. It creates a new OptimizationParameter
        /// object for each time when it is called and adds it to OptimizationParameters List
        /// which then used for optimization. It also sets the selected elements to 
        /// their default location. Indicating that user is done with the constraints setting.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void btnConstraintsDone_Click(object sender, EventArgs e)
        {
            if (this.user_selected_elements.Count <= 0)
            {
                TaskDialog.Show("uDOME", "Hey! Pick the Elements First to Set their Constraints.");
                return;
            }
            SetConstrainsHelper(); //Compute constrains add them into OptimizationParametesrs list (params_)
            //if (this.isRotational && !this.UB)
            //{
            //    this.isRotational = false;
            ResetLayout();            
            //}
            //else
            //    this.m_ExEvent.Raise();
        }

        /// <summary>
        /// Checks what selection type is selected by the user
        /// and sets a boolean variable for it accordingly.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void radioSelection_Click(object sender, EventArgs e)
        {
            //if (this.radioSelectionSingle.Checked)
            //    this.radioSelection = 0;
            //else if (this.radioSelectionMultiple.Checked)
            //    this.radioSelection = 1;
            //else if (this.radioSelectionFree.Checked)
            //    this.radioSelection = 2;
        }

        /// <summary>
        /// Helper function use to compute constrains and add them into Optimization
        /// Parameters list (params_). It can also be used to create a default set
        /// of constrains if no constraints ever specified by user before running optimization.
        /// 
        /// Also keep track of the distances between old and new element positions.
        /// </summary>

        private void SetConstrainsHelper()
        {
            //List<Vector_SizeT> constraintsList = new List<Vector_SizeT>();            
            Vector_SizeT node_ids = new Vector_SizeT();
            XYZ mid_p0 = new XYZ(0, 0, 0);
            XYZ mid_p1 = new XYZ(0, 0, 0);

            int index = -1;
            foreach (Element ele in this.user_selected_elements)
            {
                index++;
                this.user_selected_elements_all.Add(ele);

                int elementIndex = this.eIds.FindIndex(ElementId => ElementId == ele.Id);

                WallsGeometry wallG = this.wallsGeometry.First<WallsGeometry>(wall => wall.id==ele.Id);
                
                int origin = this.edges[elementIndex].ElementAt<int>(0);
                int end = this.edges[elementIndex].ElementAt<int>(1);

                // get the current wall location
                LocationCurve wallLocation = ele.Location as LocationCurve;
                               
                // get new wallcurve endpoints
                XYZ p0_new = wallLocation.Curve.GetEndPoint(0);
                XYZ p1_new = wallLocation.Curve.GetEndPoint(1);

                //TaskDialog.Show("uDOME", "p0_old: " + wallG.p0.ToString() + "\np1_old: " + wallG.p1.ToString());
                //TaskDialog.Show("uDOME", "p0_new: " + p0_new.ToString() + "\np1_new: " + p1_new.ToString());
                //TaskDialog.Show("uDOME", "dist_p0: " + p0_new.DistanceTo(wallG.p0) + "\ndist_p1: " + p1_new.DistanceTo(wallG.p1));

                XYZ res_p0 = p0_new.Subtract(wallG.p0);
                XYZ res_p1 = p1_new.Subtract(wallG.p1);
                
                double dist_res_p0 = Math.Sqrt(res_p0.X * res_p0.X + res_p0.Y * res_p0.Y);
                double dist_res_p1 = Math.Sqrt(res_p1.X * res_p1.X + res_p1.Y * res_p1.Y);
                //TaskDialog.Show("uDOME", "dist_p0_calc: " + dist_res_p0 + "\ndist_p1_calc: " + dist_res_p1);

                if (dist_res_p0 > TOLERANCE)
                {
                    if (!node_ids.Contains((uint)origin))
                        node_ids.Add((uint)origin);                    
                }

                if (dist_res_p1 > TOLERANCE)
                {
                    if (!node_ids.Contains((uint)end))
                        node_ids.Add((uint)end);
                }

                if (!this.UB && !this.isRotational)
                {
                    // create a new LineBound  
                    double midX_p0 = (wallG.p0.X + p0_new.X) / 2;
                    double midY_p0 = (wallG.p0.Y + p0_new.Y) / 2;
                    double midX_p1 = (wallG.p1.X + p1_new.X) / 2;
                    double midY_p1 = (wallG.p1.Y + p1_new.Y) / 2;

                    mid_p0 = new XYZ(midX_p0, midY_p0, 0);
                    mid_p1 = new XYZ(midX_p1, midY_p1, 0);

                    double p0_x_m = midX_p0 * _feet_to_meters;
                    double p0_z_m = midY_p0 * _feet_to_meters;
                    double p1_x_m = midX_p1 * _feet_to_meters;
                    double p1_z_m = midY_p1 * _feet_to_meters;

                    //Line newWallLine = Line.CreateBound(mid_p0, mid_p1);
                    DoubleVector newOrigin = new DoubleVector();
                    newOrigin.Add(p0_x_m);
                    newOrigin.Add(0);
                    newOrigin.Add(p0_z_m);

                    DoubleVector newEnd = new DoubleVector();
                    newEnd.Add(p1_x_m);
                    newEnd.Add(0);
                    newEnd.Add(p1_z_m);

                    // update the wall curve
                    //wallLocation.Curve = newWallLine;
                    this.graph.setNode(newOrigin, (uint)origin);
                    this.graph.setNode(newEnd, (uint)end);
                    this.verts[origin] = mid_p0;
                    this.verts[end] = mid_p1;
                }

                List<XYZ> points = new List<XYZ>();
                points.Add(p1_new);
                points.Add(p0_new);

                if (this.UB) //Setting Upper Bound Constraints
                {
                    //this.dirVector_0 = res_p0.Normalize();
                    //this.dirVector_1 = res_p1.Normalize();
                    //TaskDialog.Show("uDOME", "V_p0_calc: " + res_p0.Normalize().ToString() + "\nV_p1_calc: " + res_p1.Normalize().ToString());

                    this.upperBounds.Add(points);

                    if (this.isRotational)
                    {
                        this.upperBoundValue = this.trackbarBound.Value;
                        //TaskDialog.Show("uDOME", "Upper Bound Rotation: " + this.upperBoundValue);
                    }
                    else
                        this.upperBoundValue = Math.Max(dist_res_p0, dist_res_p1);                    
                }
                else
                {
                    this.dirVector_0 = res_p0.Normalize();
                    this.dirVector_1 = res_p1.Normalize();
                    this.lowerBounds.Add(points);

                    if (this.isRotational)
                    {
                        this.lowerBoundValue = this.trackbarBound.Value * -1;
                        //TaskDialog.Show("uDOME", "Lower Bound ROtation: " + this.lowerBoundValue);

                        // Create a new arc defined by its center, radios, angles and 2 axes
                        XYZ _top, _down;
                        Arc arc;

                        if (this.rotationOrigin.DistanceTo(wallG.p1) > this.rotationOrigin.DistanceTo(wallG.p0))
                        {
                            //TaskDialog.Show("uDOME", "Dist_p0: " + dist_res_p0 + "\nDist_p1: " + dist_res_p1);
                            if (dist_res_p0  < 5)
                            {
                                _top = this.upperBounds.ElementAt<List<XYZ>>(index).ElementAt<XYZ>(0);
                                _down = this.lowerBounds.ElementAt<List<XYZ>>(index).ElementAt<XYZ>(0);
                                arc = Arc.Create(_down, _top, wallG.p1);

                                CurveArray floorBoundaries = new CurveArray();
                                floorBoundaries.Append(arc);
                                floorBoundaries.Append(Line.CreateBound(_top, this.rotationOrigin));
                                floorBoundaries.Append(Line.CreateBound(this.rotationOrigin, _down));

                                Floor _fSpecial = this.doc.Create.NewFloor(floorBoundaries, false);
                                _fSpecial.get_Parameter(BuiltInParameter.FLOOR_HEIGHTABOVELEVEL_PARAM).Set(1);

                                if (_fSpecial != null)
                                {
                                    ChangeElementFillPattern(this.doc, _fSpecial.Id);
                                    this.constraintRegions.Add(_fSpecial);
                                }
                            }
                            else
                            {
                                _top = this.upperBounds.ElementAt<List<XYZ>>(index).ElementAt<XYZ>(0);
                                _down = this.lowerBounds.ElementAt<List<XYZ>>(index).ElementAt<XYZ>(0);
                                arc = Arc.Create(_down, _top, wallG.p1);

                                CurveArray floorBoundaries = new CurveArray();
                                floorBoundaries.Append(arc);
                                floorBoundaries.Append(Line.CreateBound(_top, this.rotationOrigin));
                                floorBoundaries.Append(Line.CreateBound(this.rotationOrigin, _down));

                                Floor _fSpecial = this.doc.Create.NewFloor(floorBoundaries, false);
                                _fSpecial.get_Parameter(BuiltInParameter.FLOOR_HEIGHTABOVELEVEL_PARAM).Set(1);

                                if (_fSpecial != null)
                                {
                                    ChangeElementFillPattern(this.doc, _fSpecial.Id);
                                    this.constraintRegions.Add(_fSpecial);
                                }

                                _top = this.upperBounds.ElementAt<List<XYZ>>(index).ElementAt<XYZ>(1);
                                _down = this.lowerBounds.ElementAt<List<XYZ>>(index).ElementAt<XYZ>(1);
                                arc = Arc.Create(_down, _top, wallG.p0);

                                floorBoundaries = new CurveArray();
                                floorBoundaries.Append(arc);
                                floorBoundaries.Append(Line.CreateBound(_top, this.rotationOrigin));
                                floorBoundaries.Append(Line.CreateBound(this.rotationOrigin, _down));

                                Floor _fSpecial2 = this.doc.Create.NewFloor(floorBoundaries, false);
                                _fSpecial2.get_Parameter(BuiltInParameter.FLOOR_HEIGHTABOVELEVEL_PARAM).Set(1);

                                if (_fSpecial2 != null)
                                {
                                    ChangeElementFillPattern(this.doc, _fSpecial2.Id);
                                    this.constraintRegions.Add(_fSpecial2);
                                }
                            }
                        }
                        else
                        {
                            //TaskDialog.Show("uDOME", "Dist_p0: " + dist_res_p0 + "\nDist_p1: " + dist_res_p1);
                            if (dist_res_p1 < 5)
                            {
                                _top = this.upperBounds.ElementAt<List<XYZ>>(index).ElementAt<XYZ>(1);
                                _down = this.lowerBounds.ElementAt<List<XYZ>>(index).ElementAt<XYZ>(1);
                                arc = Arc.Create(_down, _top, wallG.p0);

                                CurveArray floorBoundaries = new CurveArray();
                                floorBoundaries.Append(arc);
                                floorBoundaries.Append(Line.CreateBound(_top, this.rotationOrigin));
                                floorBoundaries.Append(Line.CreateBound(this.rotationOrigin, _down));

                                Floor _fSpecial = this.doc.Create.NewFloor(floorBoundaries, false);
                                _fSpecial.get_Parameter(BuiltInParameter.FLOOR_HEIGHTABOVELEVEL_PARAM).Set(1);

                                if (_fSpecial != null)
                                {
                                    ChangeElementFillPattern(this.doc, _fSpecial.Id);
                                    this.constraintRegions.Add(_fSpecial);
                                }
                            }
                            else
                            {
                                _top = this.upperBounds.ElementAt<List<XYZ>>(index).ElementAt<XYZ>(1);
                                _down = this.lowerBounds.ElementAt<List<XYZ>>(index).ElementAt<XYZ>(1);
                                arc = Arc.Create(_down, _top, wallG.p0);

                                CurveArray floorBoundaries = new CurveArray();
                                floorBoundaries.Append(arc);
                                floorBoundaries.Append(Line.CreateBound(_top, this.rotationOrigin));
                                floorBoundaries.Append(Line.CreateBound(this.rotationOrigin, _down));

                                Floor _fSpecial = this.doc.Create.NewFloor(floorBoundaries, false);
                                _fSpecial.get_Parameter(BuiltInParameter.FLOOR_HEIGHTABOVELEVEL_PARAM).Set(1);

                                if (_fSpecial != null)
                                {
                                    ChangeElementFillPattern(this.doc, _fSpecial.Id);
                                    this.constraintRegions.Add(_fSpecial);
                                }

                                _top = this.upperBounds.ElementAt<List<XYZ>>(index).ElementAt<XYZ>(0);
                                _down = this.lowerBounds.ElementAt<List<XYZ>>(index).ElementAt<XYZ>(0);
                                arc = Arc.Create(_down, _top, wallG.p1);

                                floorBoundaries = new CurveArray();
                                floorBoundaries.Append(arc);
                                floorBoundaries.Append(Line.CreateBound(_top, this.rotationOrigin));
                                floorBoundaries.Append(Line.CreateBound(this.rotationOrigin, _down));

                                Floor _fSpecial2 = this.doc.Create.NewFloor(floorBoundaries, false);
                                _fSpecial2.get_Parameter(BuiltInParameter.FLOOR_HEIGHTABOVELEVEL_PARAM).Set(1);

                                if (_fSpecial2 != null)
                                {
                                    ChangeElementFillPattern(this.doc, _fSpecial2.Id);
                                    this.constraintRegions.Add(_fSpecial2);
                                }
                            }


                        }

                        //if (p0_new.X >= wallG.p0.X || p0_new.Y >= wallG.p0.Y)
                        //{
                        //    Arc arc = Arc.Create(_down, _top, wallG.p1);
                        //}

                        
                    }
                    else
                    {
                        this.upperBoundValue = (Math.Abs(Math.Max(wallG.p0.DistanceTo(mid_p0), wallG.p1.DistanceTo(mid_p1))));
                        this.lowerBoundValue = (Math.Abs(Math.Max(mid_p0.DistanceTo(p0_new), mid_p1.DistanceTo(p1_new))) * -1);
                        Floor _cf = null;
                        XYZ p00 = this.upperBounds.ElementAt<List<XYZ>>(index).ElementAt<XYZ>(0);
                        XYZ p11 = this.upperBounds.ElementAt<List<XYZ>>(index).ElementAt<XYZ>(1);

                        _cf = BuildFloor(this.dirVector_0, p00, p11, p1_new, p0_new);

                        if (_cf != null)
                        {
                            ChangeElementFillPattern(this.doc, _cf.Id);
                            this.constraintRegions.Add(_cf);
                        }

                        //StoreGraph(this.doc); 
                    }
                }                
            }

            if (this.UB)
            {
                this.labBound.Text = "Lower Bound Constraint";
                this.UB = false;

                if (this.isRotational)
                {
                    this.trackbarBound.Minimum = 0;
                    this.trackbarBound.Maximum = 360 - (int)this.upperBoundValue;
                    this.trackbarBound.Value = 0;
                    this.lastAngle = 0;
                }
                else
                {                    
                    //this.labBound.Location = new System.Drawing.Point(6, 40);
                    //this.trackbarBound.Visible = true;                   
                }
            }
            else
            {
                if (this.isRotational)
                {
                    if ((this.lowerBoundValue < this.upperBoundValue) && (this.upperBoundValue >= 0 && this.upperBoundValue < 360))
                    {
                        this.params_.addParameter(getOptimizationParams(node_ids, this.lowerBoundValue, this.upperBoundValue, this.rotationOrigin));
                        //TaskDialog.Show("uDOME", "Num of Nodes: " + node_ids.Count() + "\nLower Bound Angle: " + this.lowerBoundValue
                        //    + "\nUpper Bound Angle: " + this.upperBoundValue + "\nRotation Origin: " + this.rotationOrigin.ToString());
                        this.constraintsPresent = true;
                        this.lowerBoundValue = -1;
                        this.upperBoundValue = 1;
                    }

                    this.isRotational = false;
                    //this.chkRotation.Checked = false;
                    this.rotationOrigin = new XYZ(0, 0, 0);
                    this.rotationRadius = 0;
                }
                else
                {
                    if ((this.lowerBoundValue < this.upperBoundValue) && (this.upperBoundValue >= 0))
                    {
                        this.params_.addParameter(getOptimizationParams(node_ids, this.lowerBoundValue, this.upperBoundValue, this.dirVector_0 * -1));
                        //TaskDialog.Show("uDOME", "Num of Nodes: " + node_ids.Count() + "\nLower Bound: " + this.lowerBoundValue
                        //    + "\nUpper Bound: " + this.upperBoundValue + "\nTranslation Vector: " + this.dirVector_0.ToString());
                        this.constraintsPresent = true;
                        this.lowerBoundValue = -1;
                        this.upperBoundValue = 1;
                    }                    
                }

                this.labBound.Text = "Upper Bound Constraint";
                this.labBound.Hide();
                //this.labBound.Location = new System.Drawing.Point(6, 102);
                this.trackbarBound.Visible = false;
                this.trackbarBound.Minimum = 1;
                this.trackbarBound.Maximum = 100;
                this.trackbarBound.Value = 1;
                this.UB = true;
                this.upperBounds.Clear();
                this.lowerBounds.Clear();
                this.m_ExEvent.Raise();

                this.btnCancelConstraint.Hide();
                this.btnConstraintsDone.Hide();
                this.labBound.Hide();
                this.btnTranslation.Visible = true;
                this.btnRotation.Visible = true;
            }
            
            //TaskDialog.Show("uDOME", "Upper Bound: " + this.upperBoundValue.ToString() + "\nLower Bound: " + this.lowerBoundValue.ToString());

                    
        }
        /// <summary>
        /// Slider/TrackBar: "trackbarBound"
        /// It allows the user to visualize the realtime translation of Walls
        /// by changing sliders for setting bound constraints for optimization
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>

        private void trackbarBound_Scroll(object sender, EventArgs e)
        {
            if (this.user_selected_elements.Count <= 0)
            {
                TaskDialog.Show("uDOME", "Hey! Pick the Elements First to Set their Constraints.");
                return;
            }
                       
            TrackBar tb = sender as TrackBar;
            
            foreach (Element ele in this.user_selected_elements)
            {                
                //int elementIndex = this.eIds.FindIndex(ElementId => ElementId == ele.Id);

                //int origin = this.edges[elementIndex].ElementAt<int>(0);
                //int end = this.edges[elementIndex].ElementAt<int>(1);

                //XYZ p0 = this.verts[origin];
                //XYZ p1 = this.verts[end];

                if (this.isRotational)
                {
                    Location position = ele.Location;
                    
                    XYZ _a1 = new XYZ(this.rotationOrigin.X, this.rotationOrigin.Y, 0);
                    XYZ _a2 = new XYZ(this.rotationOrigin.X, this.rotationOrigin.Y, 1);

                    Line axis = Line.CreateBound(_a1, _a2);

                    if(this.UB)
                        position.Rotate(axis, ((Math.PI / 180) * ((this.trackbarBound.Value - this.lastAngle) * -1.0))); // in radian
                    else
                        position.Rotate(axis, ((Math.PI / 180) * (this.trackbarBound.Value - this.lastAngle))); // in radian
                }
                else
                {
                    XYZ p00, p11;

                    WallsGeometry wallG = this.wallsGeometry.First<WallsGeometry>(wall => wall.id == ele.Id);

                    p00 = wallG.p0.Add(this.dirVector_0.Multiply(this.trackbarBound.Value * -1));
                    p11 = wallG.p1.Add(this.dirVector_1.Multiply(this.trackbarBound.Value * -1));

                    // get the current wall location
                    LocationCurve wallLocation = ele.Location as LocationCurve;

                    // create a new LineBound                                
                    Line newWallLine = Line.CreateBound(p00, p11);

                    // update the wall curve
                    wallLocation.Curve = newWallLine;
                }
            }

            this.lastAngle = this.trackbarBound.Value;
            this.m_ExEvent.Raise();            
        }

        /// <summary>
        /// Resets the layout of selected elements to their default location 
        /// </summary>

        private void ResetLayout()
        {
            if (this.user_selected_elements_all.Count > 0)
            {
                foreach (Element ele in this.user_selected_elements_all)
                {
                    int elementIndex = this.eIds.FindIndex(ElementId => ElementId == ele.Id);

                    int origin = this.edges[elementIndex].ElementAt<int>(0);
                    int end = this.edges[elementIndex].ElementAt<int>(1);

                    XYZ p0 = this.verts[origin];
                    XYZ p1 = this.verts[end];

                    // get the current wall location
                    LocationCurve wallLocation = ele.Location as LocationCurve;

                    // create a new LineBound                
                    Line newWallLine = Line.CreateBound(p0, p1);

                    // update the wall curve
                    wallLocation.Curve = newWallLine;
                }
                //Commit Changes in Document
                this.m_ExEvent.Raise();
            }
            else
            {
                TaskDialog.Show("uDOME", "Hey! Pick the Elements First to Set their Constraints.");
            }
        }

        /// <summary>
        /// Creates the model line for the given two points
        /// </summary>
        /// <param name="line"></param>
        /// <returns>Returns Newly Created Model Line Object</returns>
        
        public ModelCurve CreateModelLine(Document doc, XYZ p0, XYZ p1)
        {            
            XYZ norm;
            if (p0.X == p1.X)
            {
                norm = XYZ.BasisX;
            }
            else if (p0.Y == p1.Y)
            {
                norm = XYZ.BasisY;
            }
            else
            {
                norm = XYZ.BasisZ;
            }

            Autodesk.Revit.DB.Plane plane = new Autodesk.Revit.DB.Plane(norm, p0);            
            ModelCurve mc = doc.Create.NewModelCurve(Line.CreateBound(p0,p1), SketchPlane.Create(doc, plane));
            
            mc.Pinned = true;
            //m_ExEvent.Raise();
            return mc;
        }

        /// <summary>
        /// This method is executed when an Event request is Raised()
        /// from an external application.
        /// </summary>
        /// <param name="app">REVIT Application ID</param>
        public void Execute(UIApplication app)
        {
            try
            {
                Document doc = app.ActiveUIDocument.Document;
                if (null == doc)
                {
                    TaskDialog.Show("External Event", "No Document Found!"); // no document, nothing to do
                }

                //Draw the lines to visualize the space graph
                if (this.isHeatmap)
                {
                    using (Transaction t = new Transaction(doc, "Rendering Points"))
                    {
                        t.Start();
                        ShowValues(doc, this.spaceNodes, this.spaceNodesDegrees);
                        //ChangeElementTransparency(doc, this.sfm.Id);
                        //UIDocument d = new UIDocument(doc);
                        //d.RefreshActiveView();
                        //doc.Regenerate();
                        t.Commit();
                    }
                    this.isHeatmap = false;
                }
                else if (this.deleteSpaceGraph)
                {
                    using (Transaction t = new Transaction(doc, "Rendering Points"))
                    {
                        t.Start();
                        doc.Delete(this.spaceGraphLines);
                        UIDocument d = new UIDocument(doc);
                        d.RefreshActiveView();
                        doc.Regenerate();
                        t.Commit();
                    }
                    this.deleteSpaceGraph = false;
                }
                else if (this.deleteFloor)
                {
                    using (Transaction t = new Transaction(doc, "Deleting Existing Floor"))
                    {
                        t.Start();
                        if (this.f != null)
                        {
                            this.f.Pinned = false;
                            doc.Delete(this.f.Id);
                            BuildFloorBoundaries();
                            this.f = doc.Create.NewFloor(this.floorBoundaries, true);
                            this.f.get_Parameter(BuiltInParameter.FLOOR_HEIGHTABOVELEVEL_PARAM).Set(1);
                        }
                        t.Commit();
                    }
                    this.deleteFloor = false;
                }
                else if (this.finishFloor)
                {
                    using (Transaction t = new Transaction(doc, "Deleting Floor. Tool is Exiting."))
                    {
                        t.Start();
                        if (this.f != null)
                        {
                            this.f.Pinned = false;
                            doc.Delete(this.f.Id);
                        }
                        t.Commit();
                    }
                    this.finishFloor = false;
                }
                else if (this.clearFloorRegions)
                {
                    using (Transaction t = new Transaction(doc, "Deleting all region floors."))
                    {
                        t.Start();
                        if (!this.userstudyMode && this.finishPressed)
                        {
                            foreach (Floor _qf in this.queryRegions)
                                if (_qf != null)
                                {
                                    _qf.Pinned = false;
                                    doc.Delete(_qf.Id);
                                }

                            foreach (Floor _rf in this.referenceRegions)
                                if (_rf != null)
                                {
                                    _rf.Pinned = false;
                                    doc.Delete(_rf.Id);
                                }
                        }
                        if (this.clearConstraintRegions)
                        {
                            foreach (Floor _cf in this.constraintRegions)
                                if (_cf != null)
                                {
                                    _cf.Pinned = false;
                                    doc.Delete(_cf.Id);
                                }
                        }
                        t.Commit();
                    }

                    if (!this.userstudyMode && !this.userstudy_A && this.finishPressed)
                    {
                        this.queryRegions.Clear();
                        this.referenceRegions.Clear();
                        this.queryRegionsPoints.Clear();
                        this.referenceRegionsPoints.Clear();
                        this.topFaces.Clear();
                    }
                    this.clearFloorRegions = false;
                    this.clearConstraintRegions = false;
                    this.finishPressed = false;
                    this.constraintRegions.Clear();                    
                }
                else if (this.openNewDocument)
                {
                    using (Transaction t = new Transaction(doc, "Loading Revit Document from Template File"))
                    {
                        t.Start();

                        Graph g = new Graph();
                        g.importFromFile(loadGraphFile);

                        for (int i = 0; i < g.num_edges(); i++)
                        {
                            Edge e = g.get_edge((uint)i);
                            DoubleVector or = g.getNode((uint)e._origin);
                            DoubleVector en = g.getNode((uint)e._end);
                            Level l = Level.Create(doc, 0);
                            Line profile = (Line.CreateBound(new XYZ(or[0],or[2], en[1]) * _meters_to_feet, new XYZ(en[0],en[2], en[1]) * _meters_to_feet));

                            Wall.Create(doc, profile, l.Id, true);
                        }
                                                                     
                        t.Commit();
                    }
                    this.openNewDocument = false;
                    StoreGraph(doc);
                }
                //else if (this.clearConstraintRegions)
                //{
                //    using (Transaction t = new Transaction(doc, "Deleting all constraint floors."))
                //    {
                //        t.Start();
                //        foreach (Floor _cf in this.constraintRegions)
                //            if (_cf != null && _cf.)
                //                doc.Delete(_cf.Id);                        
                //        t.Commit();
                //    }
                //    this.constraintRegions.Clear();
                //    this.clearConstraintRegions = false;
                //}
                else
                {
                    using (Transaction trans = new Transaction(doc, "Committing UI Changes"))
                    {
                        trans.Start();
                        //TaskDialog.Show("External Event", "Click Close to close.");
                        UIDocument d = new UIDocument(doc);
                        d.RefreshActiveView();
                        doc.Regenerate();
                        trans.Commit();
                    }
                }
            }
            catch (Exception ex)
            {
                TaskDialog.Show("uDOME", "External Event Exception. " + ex.ToString());
            }
        }

        public string GetName()
        {
            return "UI External Event";
        }

        /// <summary>
        /// It clears all the set controls for a fresh start
        /// </summary>

        private void ClearControls()
        {
            this.btnTranslation.Enabled = true;
            this.btnRotation.Enabled = true;
            this.comboBestResults.Hide();
            this.btnFinishOperations.Hide();
            this.bestNodes.Clear();
            this.bestNodesIndexes.Clear();
            this.params_ = new OptimizationParameters();
            this.constraintsPresent = false;
            this.eIds.Clear();
            this.edges.Clear();
            this.user_selected_elements.Clear();
            this.user_selected_elements_all.Clear();
            this.verts.Clear();
            this.comboBestResults.Items.Clear();
            this.labBestResults.Hide();            
            this.axis = new DoubleVector();    //Default rotation X-Axis
            this.axis.Add(1.0);
            this.axis.Add(0);
            this.axis.Add(0);
            this.isHeatmapClicked = false;
            this.spaceLines.Clear();
            this.spaceLinesPoints.Clear();
            this.spaceRegionPresent = false;
            this.isAnalysisDisplayStyle = false;            

            this.radioRegionQuery.Checked = true;
            this.radioRegionReference.Checked = false;
            this.labBound.Text = "Upper Bound Constraint";
            this.labBound.Location = new System.Drawing.Point(6, 102);
            this.UB = true;

            this.upperBoundValue = 1;
            this.lowerBoundValue = -1;
            this.upperBounds.Clear();
            this.lowerBounds.Clear();
            this.dirVector_0 = new XYZ(0, 0, 0);
            this.dirVector_1 = new XYZ(0, 0, 0);

            this.trackbarBound.Value = 1;
            this.trackbarBound.Visible = false;

            this.ClearSpacesyntexVisualization();
            if(this.analysisDisplayStyle != null) this.analysisDisplayStyle.Dispose();
            //if(this.steersuite != null) this.steersuite.finish();
            this.steersuite = null;
            this.spaceGraphLines.Clear();
            this.isSpaceGraph = false;

            if (this.constraintRegions.Count > 0)
                this.clearConstraintRegions = true;

            this.clearFloorRegions = true;
            this.m_ExEvent.Raise();
            
            if (this.f != null)
            {
                this.finishFloor = true;
                this.m_ExEvent.Raise();
            }                      

            foreach (SteerSuite s in this.steersuiteLayouts)
                if(s != null)
                    s.finish();

            this.steersuiteLayouts.Clear();
            //if (this.steersuiteLayout != null)
            //    this.steersuiteLayout.finish();
            this.steersuiteLayout = null;
            
            this.comboOptimizer.SelectedIndex = 1;
            this.minDeg = 0;
            this.maxDeg = 1;
                        
            if (this.userstudyMode)
            {
                //loading regions from params file.
                //file should have same name as of current REVIT document.
                //HelperLoadParamsFromFile((this.pathToSteerSuite + Path.GetFileNameWithoutExtension(doc.PathName) + ".xml"));
                this.spaceRegionPresent = true;
                this.watch = System.Diagnostics.Stopwatch.StartNew();
            }
            
            if (!this.finishPressed)
                this.spaceRegionPresent = true;

            this.steersuiteLayoutsMetrics.Clear();
            this.steersuiteLayoutsIndexes.Clear();
        }
        
        public void UIDesign_Form_Load(object sender, EventArgs e) { }

        /// <summary>
        /// Get the Spatial Field Manager from the current view, or create if it needs it.
        /// </summary>
        /// <param name="doc"></param>
        /// <returns></returns>
        /// 
        public void InitializeSpatialFieldManager(Document doc)
        {
            this.sfm = SpatialFieldManager.GetSpatialFieldManager(doc.ActiveView);
            // create it on the active view, with one-dimension data
            if (this.sfm == null) this.sfm = SpatialFieldManager.CreateSpatialFieldManager(doc.ActiveView, 1);            
        }

        /// <summary>
        /// Display the spacegraph nodes with their respective
        /// degree values in the heatmap.
        /// </summary>
        /// <param name="doc"></param>
        /// <param name="points"></param>
        /// <param name="values"></param>

        public void ShowValues(Document doc, List<List<UV>> points, List<List<ValueAtPoint>> values)
        {
            //Creates the Analyss Dsplay Style
            if(!this.isAnalysisDisplayStyle)
                CreateAnalysisDisplayStyle(doc);

            // take care of Spatial Field Manager / AnalysisResultSchema, etc.
            InitializeSpatialFieldManager(doc);
            this.sfm.Clear();
            this.idx.Clear();

            AnalysisResultSchema schema = new AnalysisResultSchema("ShowNodes", "Data Points");
            IList<string> names = new List<string>();
            IList<double> doubles = new List<double>();
            names.Add(this.comboMetricSelection.SelectedItem.ToString());            
            doubles.Add(1.0);

            schema.SetUnits(names, doubles);            
            this._schemaId = this.sfm.RegisterResult(schema);

            // now we need to populate the FieldDomainPointsByXYZ and values
            
            for (int k = 0; k < this.queryRegions.Count; k++)
            {
                if (points.ElementAt<List<UV>>(k).Count == 0 && values.ElementAt<List<ValueAtPoint>>(k).Count == 0)
                    continue;

                    int numOfPoints = points.ElementAt<List<UV>>(k).Count / 1000;

                if (numOfPoints == 0)
                {
                    int idX = (this.sfm.AddSpatialFieldPrimitive(this.topFaces.ElementAt<PlanarFace>(k), Transform.CreateTranslation(new XYZ(0, 0, 0.5))));

                    this.sfm.UpdateSpatialFieldPrimitive(idX,
                        new FieldDomainPointsByUV(points.ElementAt<List<UV>>(k)),
                        new FieldValues(values.ElementAt<List<ValueAtPoint>>(k)), this._schemaId);

                    if (this.regionBoundMin.X > this.regionBoundMax.X)
                        this.sfm.LegendPosition = new XYZ(this.regionBoundMin.X * _meters_to_feet + 15,
                           this.regionBoundMin.Z * _meters_to_feet - 5, 0);
                    else
                        this.sfm.LegendPosition = new XYZ(this.regionBoundMax.X * _meters_to_feet + 15,
                           this.regionBoundMax.Z * _meters_to_feet - 5, 0);

                    this.idx.Add(idX);
                }
                else
                {
                    for (int i = 0; i < numOfPoints; i++)
                    {
                        int idX = this.sfm.AddSpatialFieldPrimitive(this.topFaces.ElementAt<PlanarFace>(k), Transform.CreateTranslation(new XYZ(0, 0, 0.5)));

                        List<UV> fewPoints = points.ElementAt<List<UV>>(k).GetRange(i * 1000, 1000);
                        List<ValueAtPoint> fewValues = values.ElementAt<List<ValueAtPoint>>(k).GetRange(i * 1000, 1000);

                        this.sfm.UpdateSpatialFieldPrimitive(idX,
                            new FieldDomainPointsByUV(fewPoints),
                            new FieldValues(fewValues), this._schemaId);

                        if (this.regionBoundMin.X > this.regionBoundMax.X)
                            this.sfm.LegendPosition = new XYZ(this.regionBoundMin.X * _meters_to_feet + 15,
                               this.regionBoundMin.Z * _meters_to_feet - 5, this.regionBoundMin.Y * _meters_to_feet);
                        else
                            this.sfm.LegendPosition = new XYZ(this.regionBoundMax.X * _meters_to_feet + 15,
                               this.regionBoundMax.Z * _meters_to_feet - 5, this.regionBoundMax.Y * _meters_to_feet);

                        this.idx.Add(idX);
                    }
                }                
            }
        }

        /// <summary>
        /// Creates style to vsualize spacegraph in the given document.
        /// </summary>
        /// <param name="doc"></param>

        public void CreateAnalysisDisplayStyle(Document doc)
        {
            // Look for an existing analysis display style with a specific name
            FilteredElementCollector collector1 = new FilteredElementCollector(doc);
            ICollection<Element> collection = collector1.OfClass(typeof(AnalysisDisplayStyle)).ToElements();
            var displayStyle = from element in collection
                               where element.Name == "Space-Graph Heatmap"
                               select element;
            if (displayStyle.Count() != 0)
            {
                doc.ActiveView.AnalysisDisplayStyleId = displayStyle.Cast<AnalysisDisplayStyle>().ElementAt<AnalysisDisplayStyle>(0).Id;
                return;
            }

            //AnalysisDisplayMarkersAndTextSettings markerSettings
            //    = new AnalysisDisplayMarkersAndTextSettings();

            //markerSettings.MarkerSize = 0.08;
            //markerSettings.MarkerType = AnalysisDisplayStyleMarkerType.Square;            
            
            AnalysisDisplayColoredSurfaceSettings coloredSurfaceSettings = new AnalysisDisplayColoredSurfaceSettings();            
            coloredSurfaceSettings.ShowGridLines = false;

            AnalysisDisplayColorSettings colorSettings = new AnalysisDisplayColorSettings();
            Autodesk.Revit.DB.Color blue = new Autodesk.Revit.DB.Color(0, 0, 255);
            Autodesk.Revit.DB.Color red = new Autodesk.Revit.DB.Color(255, 0, 0);
            colorSettings.MaxColor = red;
            List<AnalysisDisplayColorEntry> colors = new List<AnalysisDisplayColorEntry>();            
            colors.Add(new AnalysisDisplayColorEntry(new Autodesk.Revit.DB.Color(0, 255, 0)));
            colors.Add(new AnalysisDisplayColorEntry(new Autodesk.Revit.DB.Color(255, 255, 0)));
            colorSettings.SetIntermediateColors(colors);
            colorSettings.MinColor = blue;
                        
            AnalysisDisplayLegendSettings legendSettings = new AnalysisDisplayLegendSettings();
            //legendSettings.NumberOfSteps = 1;
            //legendSettings.ShowDataName = false;
            //legendSettings.ShowUnits = true;
            
            //legendSettings.Rounding = 0.05;
            //legendSettings.ShowDataDescription = false;
            //legendSettings.ShowLegend = true;

            //this.analysisDisplayStyle = AnalysisDisplayStyle.CreateAnalysisDisplayStyle(doc,
            //    "Space-Graph Heatmap", markerSettings, colorSettings, legendSettings);

            this.analysisDisplayStyle = AnalysisDisplayStyle.CreateAnalysisDisplayStyle(doc,
                "Space-Graph Heatmap", coloredSurfaceSettings, colorSettings, legendSettings);

            doc.ActiveView.AnalysisDisplayStyleId = this.analysisDisplayStyle.Id;
            this.isAnalysisDisplayStyle = true;            
        }

        /// <summary>
        /// Clears the visualization of spacesyntex
        /// i.e. heatmap (if exists)
        /// </summary>
        public void ClearSpacesyntexVisualization()
        {
            if (this.idx.Count > 1)
            {
                foreach(int x in this.idx)
                    this.sfm.RemoveSpatialFieldPrimitive(x);
                this.idx.Clear();
            }
            this.isHeatmapClicked = false;
            this.isHeatmap = false;
            this.chkShowHeatmap.Checked = false;            
            if(this.sfm != null) this.sfm.Clear();
            this.m_ExEvent.Raise();
        }
                
        public void OnIdling(object sender, IdlingEventArgs e)
        {
            if (!this.isSpaceGraph)
                return;

            callCounter++;
            ProcessLines();
            
            //When Revit users don't move the mouse
            if (Math.Abs(lastCursor.X - Cursor.Position.X) <= 1)
            {
                //move the cursor left and right with small distance:
                //1 pixel. so it looks like it is stable
                //this way can trigger the Idling event repeatedly
                if (callCounter % 2 == 0)
                {
                    Cursor.Position = new System.Drawing.Point(
                      Cursor.Position.X + 1, Cursor.Position.Y + 1);
                }
                else if (callCounter % 2 == 1)
                {
                    Cursor.Position = new System.Drawing.Point(
                      Cursor.Position.X - 1, Cursor.Position.Y - 1);
                }
            }
            lastCursor = Cursor.Position;
            //this.m_ExEvent.Raise();
        }

        /// <summary>
        /// Checks for the pending spacegraph lines
        /// and draw them in REVIT
        /// </summary>
        public void ProcessLines()
        {
            using (Transaction t = new Transaction(doc, "Drawing Line"))
            {
                t.Start();

                //CreateModelLine(doc, new XYZ(-10, 0, 0), new XYZ(10, 0, 0));
                foreach (List<XYZ> line in this.spaceLinesPoints)
                {
                    this.spaceGraphLines.Add(CreateModelLine(this.doc, line.ElementAt<XYZ>(0), line.ElementAt<XYZ>(1)).Id);
                    this.spaceLinesPoints.Remove(line);
                    break;
                }                
                t.Commit();
            }
        }

    }
}