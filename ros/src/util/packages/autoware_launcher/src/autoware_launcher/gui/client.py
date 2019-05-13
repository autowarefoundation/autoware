from logging import getLogger
logger = getLogger(__name__)

from python_qt_binding import QtCore
from python_qt_binding import QtWidgets

from ..core  import myutils
from .guimgr import AwQtGuiManager
from .mirror import AwLaunchTreeMirror

# For Gui Manager
from .window     import AwMainWindow
from .treeview   import AwTreeViewPanel
from .treeview   import AwControlPanel
from .procmgr    import AwProcessPanel
from .summary    import AwSummaryPanel
from .network    import AwTcpServerPanel
from .quickstart import AwQuickStartPanel
from .simulation import AwRosbagSimulatorWidget
from .simulation import AwLgsvlSimulatorWidget
from .simulation import AwGazeboSimulatorWidget



class AwQtGuiClient(object):

    def __init__(self, sysarg, server):
        self.__sysarg = sysarg
        self.__panels = []
        self.__guimgr = AwQtGuiManager(self)
        self.__mirror = AwLaunchTreeMirror(self)

        self.__server = server
        self.__server.register_client(self)

    def guimgr(self):
        return self.__guimgr

    def select_config(self, lpath): # ToDo: consider moving to guimgr
        self.__treeview.select_config(lpath)

    def start(self, profile=None, skin=None):

        application = QtWidgets.QApplication(self.__sysarg)
        resolution = application.desktop().screenGeometry()
        resolution = min(resolution.width(), resolution.height())

        skin = skin or "autoware"
        with open(myutils.package("resources/skins/{}.css").format(skin)) as fp:
            stylesheet = fp.read()
        stylesheet += "* { font-size: " + str(resolution/100) + "px; }"
        application.setStyleSheet(stylesheet)

        self.__treeview   = AwTreeViewPanel(self) # ToDo: consider moving to guimgr
        self.__control    = AwControlPanel(self)  # ToDo: consider moving to guimgr
        self.__summary    = AwSummaryPanel(self)  # ToDo: consider moving to guimgr
        self.__process    = AwProcessPanel(self)  # ToDo: consider moving to guimgr
        self.__network    = AwTcpServerPanel()
        self.__quickstart = AwQuickStartPanel(self.__guimgr)
        self.__sim_rosbag = AwRosbagSimulatorWidget(self.__guimgr)
        self.__sim_lgsvl  = AwLgsvlSimulatorWidget (self.__guimgr)
        self.__sim_gazebo = AwGazeboSimulatorWidget(self.__guimgr)

        tabwidget = QtWidgets.QTabWidget()
        tabwidget.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding)
        tabwidget.addTab(self.__summary, "Summary")
        tabwidget.addTab(self.__process, "Process")

        #vsplitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        vsplitter = QtWidgets.QWidget()
        vsplitter.setLayout(QtWidgets.QVBoxLayout())
        vsplitter.layout().setContentsMargins(0, 0, 0, 0)
        vsplitter.layout().setSpacing(0)
        vsplitter.layout().addWidget(self.__treeview)
        vsplitter.layout().addWidget(self.__control)

        self.__develop = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.__develop.addWidget(vsplitter)
        self.__develop.addWidget(tabwidget)

        #mainwidget = QtWidgets.QTabWidget()
        #mainwidget.addTab(hsplitter,         "Profile Edit")
        #mainwidget.addTab(self.__quickstart, "Quick Start")
        #mainwidget.addTab(self.__network,    "Server Debug")

        mainwidget = QtWidgets.QStackedWidget()
        mainwidget.addWidget(self.__quickstart)
        mainwidget.addWidget(self.__develop)
        mainwidget.addWidget(self.__network)

        simulations = QtWidgets.QTabWidget()
        simulations.addTab(self.__sim_rosbag, "Rosbag")
        simulations.addTab(self.__sim_lgsvl,  "LGSVL")
        simulations.addTab(self.__sim_gazebo, "Gazebo")

        mainsplitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        mainsplitter.addWidget(mainwidget)
        mainsplitter.addWidget(simulations)

        #dock = QtWidgets.QDockWidget()
        #dock.setWidget( )
        #window.addDockWidget(QtCore.Qt.RightDockWidgetArea, dock)

        window = AwMainWindow(self)
        window.setCentralWidget(mainsplitter)
        window.show()

        simulations.hide()
        window.addViewMenu("Simulation", simulations.setVisible)

        def switch_develop_mode(checked):
            if checked:
                mainwidget.setCurrentWidget(self.__develop)
            else:
                mainwidget.setCurrentWidget(self.__quickstart)
        window.addViewMenu("Develop Mode", switch_develop_mode)

        simulations.hide()
        self.__sim_rosbag.rosbag_file.path.setText(myutils.userhome(".autoware/sample_moriyama_150324.bag"))

        # connect objects
        self.__server.register_runner(self.__process)
        self.__process.register_server(self.__server)

        self.__server.register_client(self.__network)
        self.__network.register_server(self.__server)

        self.__panels.append(self.__treeview)
        self.__panels.append(self.__summary)
        self.__panels.append(self.__process)
        self.__panels.append(self.__quickstart)

        self.__treeview.register_select_listener(self.__summary)
        self.__treeview.register_select_listener(self.__process)
        self.__treeview.register_select_listener(self.__control)

        if profile:
            self.__server.load_profile(profile)
        else:
            self.__server.make_profile("root/default")

        return application.exec_()

    def profile_updated(self):
        self.__mirror.clear()
        for panel in self.__panels: panel.profile_ui_cleared()

        for lpath in self.__server.list_node():
            lnode = self.__mirror.create(lpath)
            for panel in self.__panels: panel.node_ui_created(lnode)

        self.__treeview.expandAll()
        #self.__treeview.expandToDepth(0)

    def node_created(self, lpath):
        logger.debug("node_created: " + lpath)
        lnode = self.__mirror.create(lpath)
        for panel in self.__panels: panel.node_ui_created(lnode)

        lpath = myutils.parentpath(lpath)
        while lpath:
            logger.debug("node_updated: " + lpath)
            self.__mirror.clear(lpath)
            lnode = self.__mirror.create(lpath)
            for panel in self.__panels: panel.node_ui_updated(lnode)
            lpath = myutils.parentpath(lpath)

    def node_updated(self, lpath):
        while lpath:
            logger.debug("node_updated: " + lpath)
            self.__mirror.clear(lpath)
            lnode = self.__mirror.create(lpath)
            for panel in self.__panels: panel.node_ui_updated(lnode)
            lpath = myutils.parentpath(lpath)

    def status_updated(self, lpath, state):
        logger.debug("status_updated:" + lpath + " " + str(state))
        self.__treeview.status_ui_updated(lpath, state)
        self.__quickstart.status_ui_updated(lpath, state)

    def load_profile(self, fpath):
        self.__server.load_profile(fpath)

    def save_profile(self, fpath):
        self.__server.save_profile(fpath)

    def export_profile(self, fpath):
        self.__server.export_profile(fpath)

    def find_node(self, lpath):
        return self.__server.find_node(lpath)

    def launch_node(self, lpath, xmode):
        return self.__server.launch_node(lpath, xmode)

    def create_node(self, lpath, ppath):
        return self.__server.create_node(lpath, ppath)

    def update_node(self, lpath, ldata):
        return self.__server.update_node(lpath, ldata)
