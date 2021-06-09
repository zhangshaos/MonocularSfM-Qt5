#include "RuntimeSetting.h"

#include "SfM/system_info.h"
#include "ui_RuntimeSetting.h"

RuntimeSetting::RuntimeSetting(QWidget* parent)
    : QDialog(parent), ui(new Ui::RuntimeSetting) {
  setupUi();
}

RuntimeSetting::~RuntimeSetting() { delete ui; }

void RuntimeSetting::setupUi() {
  ui->setupUi(this);
  // set default runtime settings
  // setting of processing images
  ui->max_features->setValue(sys.max_features());
  ui->sift_contrast_threshold->setValue(sys.sift_contrast_threshold());
  ui->sitf_sigma->setValue(sys.sift_sigma());
  ui->dist_ratio_in_filter_matches->setValue(
      sys.dist_ratio_in_filter_matches());
  ui->min_matches_edge_in_create_image_graph->setValue(
      sys.min_matches_edge_in_create_image_graph());
  // setting of map initialization
  ui->min_inliers_in_2d2d_matching->setValue(
      sys.min_inliers_in_2d2d_matching());
  ui->ransac_confidence_in_compute_H_E->setValue(
      sys.ransac_confidence_in_compute_H_E());
  ui->max_error_in_compute_F_E->setValue(sys.max_error_in_compute_F_E());
  ui->max_error_in_compute_H->setValue(sys.max_error_in_compute_H());
  ui->max_error_in_init_triangulte->setValue(
      sys.max_error_in_init_triangulte());
  ui->min_angle_in_init_triangulte->setValue(
      sys.min_angle_in_init_triangulte());
  // settting of pnp
  ui->min_inlers_in_pnp->setValue(sys.min_inlers_in_pnp());
  ui->ransac_times_in_pnp->setValue(sys.ransac_times_in_pnp());
  ui->ransac_conifidence_in_pnp->setValue(sys.ransac_confidence_in_pnp());
  ui->max_error_in_pnp->setValue(sys.max_error_in_pnp());
  // setting of tri
  ui->max_error_in_triangulate->setValue(sys.max_error_in_triangulate());
  ui->min_anglue_in_triangulate->setValue(sys.min_anglue_in_triangulate());
  // setting of BA
  ui->max_error_in_BA_filter->setValue(sys.max_error_in_BA_filter());
  ui->min_angle_in_BA_filter->setValue(sys.min_angle_in_BA_filter());
  ui->max_image_count_in_localBA->setValue(sys.max_image_count_in_localBA());
  ui->max_found_image_count_in_BA_to_run_globalBA->setValue(
      sys.max_found_image_count_in_BA_to_run_globalBA());
  ui->max_image_optimized_times->setValue(sys.max_image_optimized_times());
}

std::any RuntimeSetting::getConf() const {
  SystemInfo::SysConfig conf;
  // setting of processing images
  conf._max_features = ui->max_features->value();
  conf._sift_contrast_threshold = ui->sift_contrast_threshold->value();
  conf._sitf_sigma = ui->sitf_sigma->value();
  conf._dist_ratio_in_filter_matches =
      ui->dist_ratio_in_filter_matches->value();
  conf._min_matches_edge_in_create_image_graph =
      ui->min_matches_edge_in_create_image_graph->value();
  // setting of map initialization
  conf._min_inliers_in_2d2d_matching =
      ui->min_inliers_in_2d2d_matching->value();
  conf._ransac_confidence_in_compute_H_E =
      ui->ransac_confidence_in_compute_H_E->value();
  conf._max_error_in_compute_F_E = ui->max_error_in_compute_F_E->value();
  conf._max_error_in_compute_H = ui->max_error_in_compute_H->value();
  conf._max_error_in_init_triangulte =
      ui->max_error_in_init_triangulte->value();
  conf._min_angle_in_init_triangulte =
      ui->min_angle_in_init_triangulte->value();
  // setting of Pnp
  conf._min_inlers_in_pnp = ui->min_inlers_in_pnp->value();
  conf._ransac_times_in_pnp = ui->ransac_times_in_pnp->value();
  conf._ransac_conifidence_in_pnp = ui->ransac_conifidence_in_pnp->value();
  conf._max_error_in_pnp = ui->max_error_in_pnp->value();
  // setting of tri
  conf._max_error_in_triangulate = ui->max_error_in_triangulate->value();
  conf._min_anglue_in_triangulate = ui->min_anglue_in_triangulate->value();
  // setting of BA
  conf._max_error_in_BA_filter = ui->max_error_in_BA_filter->value();
  conf._min_angle_in_BA_filter = ui->min_angle_in_BA_filter->value();
  conf._max_image_count_in_localBA = ui->max_image_count_in_localBA->value();
  conf._max_found_image_count_in_BA_to_run_globalBA =
      ui->max_found_image_count_in_BA_to_run_globalBA->value();
  conf._max_image_optimized_times = ui->max_image_optimized_times->value();
  return conf;
}
