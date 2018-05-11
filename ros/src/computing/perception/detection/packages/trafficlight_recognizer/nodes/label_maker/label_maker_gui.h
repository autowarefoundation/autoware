#ifndef LABEL_MAKER_GUI_H
#define LABEL_MAKER_GUI_H

#include "file_system_operator.h"

#include <map>
#include <string>

#include <QMainWindow>
#include <QString>
#include <QLabel>
#include <QAbstractButton>

namespace Ui {
  class LabelMakerGui;
}

class LabelMakerGui : public QMainWindow
{
  Q_OBJECT

public:
  explicit LabelMakerGui(QWidget *parent = 0);
  ~LabelMakerGui();

private slots:
  // Show current ID's image
  void ShowImage();

  // Change background color to corresponding one
  void SetRadioButtonsColor(QAbstractButton* selected_button);

  // The behavior of "Next" and "Previous" button
  void SaveAndGoNext();
  void SaveAndGoPrevious();

  // The behavior of "Reset Selection" button
  void ResetSelection();
private:
  // The utility function to get directory path
  QString GetTargetDirectoryPath();

  // Reset radio buttons status
  void ResetRadioButtonsBackGround();

  // Save the current status
  bool SaveCurrentState();

  // The GUI handelr
  Ui::LabelMakerGui *ui_;

  // Path to the target dataset directory
  QString dataset_path_;

  // The list of path and ID contained in "Images" directory
  std::map<int, std::string> image_list_;

  // The class instance to operate file system
  FileSystemOperator file_system_operator_;
};

#endif // LABEL_MAKER_GUI_H
