#include "mainwindow.h"
#include "ui_mainwindow.h"

static int BEAMNUM;
static double STEP;
static double MINFLOOR;
static double MAXFLOOR;
static double MAXCEILING;
static double MINCEILING;
static double ROADSLOPMINHEIGHT;
static double ROADSLOPMAXHEIGHT;
static double ROTATION;
static double OBSTACLEMINHEIGHT;
static double MAXBACKDISTANCE;
static double PASSHEIGHT;

static double MAXRANGE;
static double MINRANGE;
static double GRIDSIZE;
static double IMAGESIZE;

static bool SYNC;

MainWindow::MainWindow(QWidget* parent)
  : QMainWindow(parent)
#ifdef DEBUG_GUI
  , ui(new Ui::MainWindow)
#endif
{
#ifdef DEBUG_GUI
  ui->setupUi(this);
#endif
  ros::NodeHandle private_nh("~");
  private_nh.param("BEAMNUM", BEAMNUM, 1440);
  private_nh.param("STEP", STEP, 0.1);
  private_nh.param("MINFLOOR", MINFLOOR, -3.0);
  private_nh.param("MAXFLOOR", MAXFLOOR, -1.3);
  private_nh.param("MAXCEILING", MAXCEILING, 6.0);
  private_nh.param("MINCEILING", MINCEILING, -0.5);
  private_nh.param("ROADSLOPMINHEIGHT", ROADSLOPMINHEIGHT, 80.0);
  private_nh.param("ROADSLOPMAXHEIGHT", ROADSLOPMAXHEIGHT, 30.0);
  private_nh.param("ROTATION", ROTATION, (double)3);
  private_nh.param("OBSTACLEMINHEIGHT", OBSTACLEMINHEIGHT, (double)1);
  private_nh.param("MAXBACKDISTANCE", MAXBACKDISTANCE, (double)1);
  private_nh.param("PASSHEIGHT", PASSHEIGHT, (double)2);

  private_nh.param("MAXRANGE", MAXRANGE, 80.0);
  private_nh.param("MINRANGE", MINRANGE, (double)3);
  private_nh.param("GRIDSIZE", GRIDSIZE, 10.0);
  private_nh.param("IMAGESIZE", IMAGESIZE, 1000.0);

  private_nh.param("SYNC", SYNC, false);

  if (SYNC == false)
  {
    velodyne = new ROSSub<sensor_msgs::PointCloud2ConstPtr>("points_raw", 1000, 10);
  }
  else
  {
    velodyne = new ROSSub<sensor_msgs::PointCloud2ConstPtr>("/sync_drivers/points_raw", 1000, 10);
  }
  connect(velodyne, SIGNAL(receiveMessageSignal()), this, SLOT(generateVirtualScanSlot()));
  vsros = new ROSPub<sensor_msgs::PointCloud2>("/vscan_points", 1000);
  scanros = new ROSPub<sensor_msgs::LaserScan>("/scan", 1000);

#ifdef DEBUG_GUI
  double PI = 3.141592654;
  double density = 2 * PI / BEAMNUM;
  for (int i = 0; i < BEAMNUM; i++)
  {
    ui->comboBox->insertItem(i, QString("%1").arg((i * density - PI) * 180.0 / PI));
  }
  ui->comboBox->insertItem(BEAMNUM, "NULL");

  connect(ui->comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(showMatrixSlot(int)));
  connect(ui->step, SIGNAL(valueChanged(double)), this, SLOT(recalculateSlot()));
  connect(ui->rotation, SIGNAL(valueChanged(double)), this, SLOT(recalculateSlot()));
  connect(ui->minrange, SIGNAL(valueChanged(double)), this, SLOT(recalculateSlot()));

  ui->label->resize(IMAGESIZE, IMAGESIZE);
  image = QImage(IMAGESIZE, IMAGESIZE, QImage::Format_RGB888);
  drawGrid();
#endif

  velodyne->startReceiveSlot();
}

MainWindow::~MainWindow()
{
  velodyne->stopReceiveSlot();
  delete velodyne;
  delete vsros;
  delete scanros;
#ifdef DEBUG_GUI
  delete ui;
#endif
}

void MainWindow::generateVirtualScanSlot()
{
  virtualscan.velodyne = velodyne->getMessage();
  recalculateSlot();
}

void MainWindow::showMatrixSlot(int beamid)
{
  if (beamid >= BEAMNUM)
  {
    return;
  }
  ui->tableWidget->clear();
  int size = int((MAXCEILING - MINFLOOR) / STEP + 0.5);
  ui->tableWidget->setRowCount(size);
  ui->tableWidget->setColumnCount(5);
  for (int i = 0; i < size; i++)
  {
    ui->tableWidget->setVerticalHeaderItem(i, new QTableWidgetItem(QString("%1").arg(MINFLOOR + i * STEP)));
  }
  ui->tableWidget->setHorizontalHeaderLabels(QStringList() << "RotID"
                                                           << "RotLength"
                                                           << "RotHeight"
                                                           << "Length"
                                                           << "Height");

  for (int i = 0; i < size; i++)
  {
    ui->tableWidget->setItem(i, 0, new QTableWidgetItem(QString("%1").arg(virtualscan.svs[beamid][i].rotid)));
    ui->tableWidget->setItem(i, 1, new QTableWidgetItem(QString("%1").arg(virtualscan.svs[beamid][i].rotlength)));
    ui->tableWidget->setItem(i, 2, new QTableWidgetItem(QString("%1").arg(virtualscan.svs[beamid][i].rotheight)));
    ui->tableWidget->setItem(i, 3, new QTableWidgetItem(QString("%1").arg(virtualscan.svs[beamid][i].length)));
    ui->tableWidget->setItem(i, 4, new QTableWidgetItem(QString("%1").arg(virtualscan.svs[beamid][i].height)));
  }
  drawBeam(beamid);
}

void MainWindow::recalculateSlot()
{
  double PI = 3.141592654;
  double density = 2 * PI / BEAMNUM;
#ifdef DEBUG_GUI
  QTime start = QTime::currentTime();
#endif
  virtualscan.calculateVirtualScans(BEAMNUM, STEP, MINFLOOR, MAXCEILING, OBSTACLEMINHEIGHT, MAXBACKDISTANCE,
                                    ROTATION * PI / 180.0, MINRANGE);
#ifdef DEBUG_GUI
  QTime end = QTime::currentTime();
#endif
  virtualscan.getVirtualScan(ROADSLOPMINHEIGHT * PI / 180.0, ROADSLOPMAXHEIGHT * PI / 180.0, MAXFLOOR, MINCEILING,
                             PASSHEIGHT, beams);
#ifdef DEBUG_GUI
  ui->tc->setText(QString("%1").arg(start.msecsTo(end)));
#endif

  {
    sensor_msgs::PointCloud2 msg;
    msg.header = virtualscan.velodyne->header;
    msg.fields = virtualscan.velodyne->fields;
    msg.height = 2;
    msg.width = BEAMNUM;
    msg.point_step = 8 * sizeof(float);
    msg.row_step = msg.width * msg.point_step;
    msg.data.resize(msg.height * msg.width * msg.point_step);
    unsigned char* base = msg.data.data();

    for (int i = 0; i < BEAMNUM; i++)
    {
      double theta = i * density - PI;
      float* data;
      int* ring;

      data = (float*)(base + (2 * i) * msg.point_step);
      data[0] = beams[i] * cos(theta);
      data[1] = beams[i] * sin(theta);
      data[2] = virtualscan.minheights[i];
      data[4] = 255;
      ring = (int*)(data + 5 * sizeof(float));
      *ring = 0;

      data = (float*)(base + (2 * i + 1) * msg.point_step);
      data[0] = beams[i] * cos(theta);
      data[1] = beams[i] * sin(theta);
      data[2] = virtualscan.maxheights[i];
      data[4] = 255;
      ring = (int*)(data + 5 * sizeof(float));
      *ring = 1;
    }
    vsros->sendMessage(msg);
  }

  {
    sensor_msgs::LaserScan msg;
    msg.header.frame_id = virtualscan.velodyne->header.frame_id;
    msg.header.seq = virtualscan.velodyne->header.seq;
    msg.header.stamp = virtualscan.velodyne->header.stamp;
    msg.angle_min = -PI;
    msg.angle_max = PI;
    msg.angle_increment = density;
    msg.time_increment = 0;
    msg.scan_time = 0.1;
    msg.range_min = 0.1;
    msg.range_max = 100;
    msg.ranges.resize(BEAMNUM);
    msg.intensities.resize(BEAMNUM);
    for (int i = 0; i < BEAMNUM; i++)
    {
      msg.ranges[i] = beams[i];
      msg.intensities[i] = 255;
    }
    scanros->sendMessage(msg);
  }

#ifdef DEBUG_GUI
  drawGrid();
  drawPoints();
  showMatrixSlot(ui->comboBox->currentIndex());
#endif
}

QPointF MainWindow::convert2RealPoint(QPoint point)
{
  QPointF result;
  double ratio = 2 * MAXRANGE / IMAGESIZE;

  result.setX((IMAGESIZE / 2.0 - point.y()) * ratio);
  result.setY((IMAGESIZE / 2.0 - point.x()) * ratio);

  return result;
}

QPoint MainWindow::convert2ImagePoint(QPointF point)
{
  QPoint result;
  double ratio = IMAGESIZE / (2 * MAXRANGE);

  result.setX(int(IMAGESIZE / 2.0 - point.y() * ratio + 0.5));
  result.setY(int(IMAGESIZE / 2.0 - point.x() * ratio + 0.5));

  return result;
}

void MainWindow::drawGrid()
{
  image.fill(QColor(255, 255, 255));
  QPainter painter;
  painter.begin(&image);
  painter.setPen(QColor(128, 128, 128));

  QPoint center(IMAGESIZE / 2, IMAGESIZE / 2);
  double gridstep = (GRIDSIZE / MAXRANGE) * (IMAGESIZE / 2);
  for (int i = gridstep; i <= IMAGESIZE / 2; i += gridstep)
  {
    painter.drawEllipse(center, i, i);
  }
  painter.end();
  ui->label->setPixmap(QPixmap::fromImage(image));
}

void MainWindow::drawPoints()
{
  QPainter painter;
  painter.begin(&image);
  painter.setPen(QColor(255, 0, 0));
  double PI = 3.141592654;
  double density = 2 * PI / BEAMNUM;

  for (int i = 0; i < BEAMNUM; i++)
  {
    double theta = i * density - PI;
    QPoint point = convert2ImagePoint(QPointF(beams[i] * cos(theta), beams[i] * sin(theta)));
    painter.drawEllipse(point, 1, 1);
  }
  painter.end();
  ui->label->setPixmap(QPixmap::fromImage(image));
}

void MainWindow::drawBeam(int beamid)
{
  drawGrid();
  drawPoints();
  if (beamid >= BEAMNUM)
  {
    return;
  }
  QPainter painter;
  painter.begin(&image);
  painter.setPen(QColor(0, 0, 255));
  QPoint center(IMAGESIZE / 2, IMAGESIZE / 2);
  double PI = 3.141592654;
  double density = 2 * PI / BEAMNUM;
  double theta = beamid * density - PI;
  QPoint point = convert2ImagePoint(QPointF(beams[beamid] * cos(theta), beams[beamid] * sin(theta)));
  painter.drawLine(center, point);
  painter.end();
  ui->label->setPixmap(QPixmap::fromImage(image));
}
