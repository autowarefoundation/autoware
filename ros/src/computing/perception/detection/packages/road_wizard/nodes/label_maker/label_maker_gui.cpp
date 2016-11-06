#include "label_maker_gui.h"
#include "file_system_operator.h"
#include "ui_label_maker_gui.h"

#include <iostream>
#include <map>

#include <QFileDialog>
#include <QMessageBox>

LabelMakerGui::LabelMakerGui(QWidget *parent) :
  QMainWindow(parent),
  ui_(new Ui::LabelMakerGui) {
  // Initialize GUI
  ui_->setupUi(this);

  // Get dataset directory path
  dataset_path_ = GetTargetDirectoryPath();

  // Get image list
  image_list_ = file_system_operator_.GetImageList(dataset_path_.toStdString() + "/Images/");

  // Open target file for saving data
  file_system_operator_.CheckPreSavedData(dataset_path_.toStdString() + "/Annotations/");

  // Setup GUI
  ui_->image_id_horizontal_slider_->setMaximum(image_list_.size() - 1);
  ui_->image_id_spin_box_->setMaximum(image_list_.size() - 1);
  ResetRadioButtonsBackGround();

  // Display first image
  ShowImage();

  // Define Interfaces behavior
  connect(ui_->image_id_horizontal_slider_,
          SIGNAL(valueChanged(int)),
          ui_->image_id_spin_box_,
          SLOT(setValue(int)));

  connect(ui_->image_id_spin_box_,
          SIGNAL(valueChanged(int)),
          ui_->image_id_horizontal_slider_,
          SLOT(setValue(int)));

  connect(ui_->image_id_spin_box_,
          SIGNAL(valueChanged(int)),
          this,
          SLOT(ShowImage()));

  connect(ui_->radio_button_group_,
          SIGNAL(buttonClicked(QAbstractButton*)),
          this,
          SLOT(SetRadioButtonsColor(QAbstractButton*)));

  connect(ui_->next_push_button_,
          SIGNAL(pressed()),
          this,
          SLOT(SaveAndGoNext()));

  connect(ui_->previous_push_button_,
          SIGNAL(pressed()),
          this,
          SLOT(SaveAndGoPrevious()));

  connect(ui_->reset_push_button_,
          SIGNAL(pressed()),
          this,
          SLOT(ResetSelection()));
} // LabelMakerGui::LabelMakerGui()


LabelMakerGui::~LabelMakerGui()
{
  delete ui_;
} // LabelMakerGui::~LabelMakerGui()


QString LabelMakerGui::GetTargetDirectoryPath() {
  // Loop getting path until correct one is acquired
  QString path;

  while (true) {
    path = QFileDialog::getExistingDirectory(this,
                                             "Select Existing Dataset Directory");
    if (path == "") {
      // "Cancel" button is pushed. Treminate program
      exit(EXIT_SUCCESS);
    }

    // Check specified direcotry is corrent one
    QDir images_direcotry(path + "/Images");
    if (images_direcotry.exists()) {
      break;
    } else {
      // Create error message
      QString error_message;
      error_message = "\"Images\" direcotry cannnot be found in \"" +
                      path + "\". \n\n" +
                      "Make sure DataSet directory surely exists. \n" +
                      "Or Execute \"rosrun road_wizard roi_extractor\" first in order to create dataset source ";

      QMessageBox::warning(this,
                           "ERROR",
                           error_message);
    }
  }

  return path;
} // QString LabelMakerGui::GetTargetDirectoryPath() {


void LabelMakerGui::ResetRadioButtonsBackGround() {
  // Reset radio buttons' background color
  ui_->green_radio_button_->setStyleSheet("background-color:grey");
  ui_->yellow_radio_button_->setStyleSheet("background-color:grey");
  ui_->red_radio_button_->setStyleSheet("background-color:grey");
  ui_->unknown_radio_button_->setStyleSheet("background-color:grey");
}


void LabelMakerGui::ShowImage() {
  // Load current ID's image
  int id = ui_->image_id_spin_box_->text().toInt();
  std::string file_name = image_list_[id];
  QString image_path = dataset_path_ + "/Images/" + QString(file_name.c_str());
  QImage image;
  if (image.load(image_path)) {
    // Display loaded image on UI
    ui_->graphics_view_->SetPixmap(image);
  } else {
    // Not found corresponding image to this ID
    ui_->graphics_view_->SetText("No Image corresponding to this ID is found.");
  }
}


void LabelMakerGui::SetRadioButtonsColor(QAbstractButton *selected_button) {
  // Reset background
  ResetRadioButtonsBackGround();

  // Change background color of radio button to corresponding color
  QString button_name = selected_button->text();
  if (button_name == "GREEN") {
    ui_->green_radio_button_->setStyleSheet("background-color:green");
  } else if (button_name == "YELLOW") {
    ui_->yellow_radio_button_->setStyleSheet("background-color:yellow");
  } else if (button_name == "RED") {
    ui_->red_radio_button_->setStyleSheet("background-color:red");
  } else if (button_name == "UNKNOWN") {
    ui_->unknown_radio_button_->setStyleSheet("background-color:black");
  }
}


bool LabelMakerGui::SaveCurrentState() {
  // Get selected state
  QAbstractButton* selected = ui_->radio_button_group_->checkedButton();
  if (selected == 0) { // Show warning window as no state is selected
    QMessageBox::warning(this,
                         "WARNING",
                         "No State is selected.");
    return false;
  }

  QString button_name = selected->text();
  FileSystemOperator::LightState state = FileSystemOperator::LightState::UNKNOWN;

  if (button_name == "GREEN") {
    state = FileSystemOperator::LightState::GREEN;
  } else if (button_name == "YELLOW") {
    state = FileSystemOperator::LightState::YELLOW;
  } else if (button_name == "RED") {
    state = FileSystemOperator::LightState::RED;
  } else if (button_name == "UNKNOWN") {
    state = FileSystemOperator::LightState::UNKNOWN;
  }

  int current_image_id = ui_->image_id_spin_box_->text().toInt();

  // Get selected area
  QPoint start;
  QPoint end;
  if (!ui_->graphics_view_->GetSelectedArea(&start, &end)) {
    QMessageBox::warning(this,
                         "WARNING",
                         "No Area is specified.");
    return false;
  }

  // Save specified state into file
  file_system_operator_.WriteStateToFile("Images", // Image file should be under "Image" directory
                                         image_list_[current_image_id],
                                         state,
                                         start.x(),
                                         start.y(),
                                         end.x(),
                                         end.y());
  return true;
}


void LabelMakerGui::SaveAndGoNext() {
  // Save Process
  if (!SaveCurrentState()) {
    return;
  }

  // Go to next image
  int image_id = ui_->image_id_spin_box_->text().toInt();
  image_id++;
  if (ui_->image_id_spin_box_->maximum() < image_id) {
    QMessageBox::information(this,
                             "Information",
                             "This is the end of image list.\n Thank you for your great work!");
    return;
  }
  ui_->image_id_spin_box_->setValue(image_id);
}


void LabelMakerGui::SaveAndGoPrevious() {
  // Save Process
  if (!SaveCurrentState()) {
    return;
  }

  // Go to previous image
  int image_id = ui_->image_id_spin_box_->text().toInt();
  image_id--;
  if (image_id < ui_->image_id_spin_box_->minimum()) {
    QMessageBox::information(this,
                             "Information",
                             "This is the first image of the list.");
    return;
  }
  ui_->image_id_spin_box_->setValue(image_id);
}


void LabelMakerGui::ResetSelection() {
  // Reset radiobutton selection
  ui_->radio_button_group_->setExclusive(false);
  ui_->green_radio_button_->setChecked(false);
  ui_->yellow_radio_button_->setChecked(false);
  ui_->red_radio_button_->setChecked(false);
  ui_->unknown_radio_button_->setChecked(false);
  ui_->radio_button_group_->setExclusive(true);

  // Reset radiobuttons background
  ResetRadioButtonsBackGround();

  // Reset selected area
  ui_->graphics_view_->ResetSelectedArea();
}
