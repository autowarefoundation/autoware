from python_qt_binding import QtWidgets
from python_qt_binding import QtNetwork

from ..core import AwLaunchClientIF

class AwTcpServerPanel(QtWidgets.QTextEdit, AwLaunchClientIF):

    def __init__(self):

        super(AwTcpServerPanel, self).__init__()
        self.server = None
        self.tcpsvr = QtNetwork.QTcpServer(self)
        self.connections = []
        self.setReadOnly(True)
        
        if self.tcpsvr.listen(QtNetwork.QHostAddress.Any, 33136):
            self.append("Server started")
            self.tcpsvr.newConnection.connect(self.on_new_connection)
        else:
            self.append("Server start error")

    def register_server(self, server):
        self.server = server

    def on_new_connection(self):

        socket = self.tcpsvr.nextPendingConnection()
        socket.buff = ""
        self.connections.append(socket)
        self.append("New connection {} {}".format(socket.localAddress().toString(), socket.localPort()))

        socket.error.connect(self.on_client_error)
        #socket.disconnected.connect
        socket.readyRead.connect(self.on_client_read_ready)

    def on_client_read_ready(self):

        socket = self.sender()
        requests = str(socket.readAll()).split("\0")
        requests[0] = socket.buff + requests[0]
        socket.buff = requests.pop()
        for request in requests:
            self.request_json(request)

    def on_client_error(self):

        socket = self.sender()
        self.append("Error " + socket.errorString())
        if socket in self.connections:
            socket.close()
            self.connections.remove(socket)

    def request_json(self, request):
        self.append("Request " + request)
        response = self.server.request_json(request)
        self.append("Response " + response)
