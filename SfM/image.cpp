#include "image.h"

#include <opencv2/opencv.hpp>

#include "system_info.h"

Image::Image() : self(new Image_impl) {}

void Image::GetNewImageID(int& id) {
  static mtx m;
  static int _id_ = 0;
  ulock<mtx> lock(m);
  id = _id_++;
}

void Image::id(int id) { self->id = id; }

int Image::id() const { return self->id; }

const cv::Mat& Image::gray_mat() const { return self->gray_mat; }

void Image::gray_mat(const cv::Mat& m) { self->gray_mat = m; }

const cv::Mat& Image::rgb_mat() const { return self->rgb_mat; }

void Image::rgb_mat(const cv::Mat& m) { self->rgb_mat = m; }

const std::vector<KeyPoint>& Image::kpts() const { return self->kpts; }

std::vector<KeyPoint>& Image::kpts() { return self->kpts; }

void Image::kpts(std::vector<KeyPoint>&& kps) { self->kpts = std::move(kps); }

const cv::Mat& Image::descp() const { return self->descp; }

cv::Mat& Image::descp() { return self->descp; }

void Image::descp(const cv::Mat& m) { self->descp = m; }

const cv::Mat& Image::Rcw() const { return self->Rcw; }

const cv::Mat& Image::tcw() const { return self->tcw; }

void Image::Rcw(const cv::Mat& Rcw) { self->Rcw = Rcw; }

void Image::tcw(const cv::Mat& tcw) { self->tcw = tcw; }

const std::string& Image::path() const { return self->path; }

void Image::path(const std::string& path) { self->path = path; }

uint64_t Image::R_ba_times() const { return self->R_ba_times; }

uint64_t Image::t_ba_times() const { return self->t_ba_times; }

void Image::inc_R_ba_times() { ++self->R_ba_times; }

void Image::inc_t_ba_times() { ++self->t_ba_times; }

int64_t Image::getMapptOfKpt(int kp_idx) const {
  return kpts().at(kp_idx).id();
}

void Image::setMapptOfKpt(int kp_idx, int64_t mp_idx) {
  self->kpts.at(kp_idx).id(mp_idx);
}
