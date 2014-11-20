// Copyright sxniu 2014-10
#include "include/Gmm.h"

#include <math.h>

#include "include/ImageData.h"
#include "include/utils.h"

double Gmm::SceneProbability(int color) const {
  double res = 0;
  for(int ci = 0; ci < m_components_count; ++ci) {
    res += m_weights[ci] * ClusterProbability(ci, color);
  }
  return res;
}

double Gmm::ClusterProbability(int ci, int color) const {
  double res = 0;
  if(m_weights[ci] > 0) {
    int diff[3] = GET_THREE_COORDINATE(color);
    diff[0] -= m_mean[ci][0];
    diff[1] -= m_mean[ci][1];
    diff[2] -= m_mean[ci][2];
    double mult = diff[0] * (diff[0] * m_inverse_covs[ci][0][0] +
                             diff[1] * m_inverse_covs[ci][1][0] +
                             diff[2] * m_inverse_covs[ci][2][0]) +
                  diff[1] * (diff[0] * m_inverse_covs[ci][0][1] +
                             diff[1] * m_inverse_covs[ci][1][1] +
                             diff[2] * m_inverse_covs[ci][2][1]) +
                  diff[2] * (diff[0] * m_inverse_covs[ci][0][2] +
                             diff[1] * m_inverse_covs[ci][1][2] +
                             diff[2] * m_inverse_covs[ci][2][2]);
    res = 1.0f / sqrt(m_cov_determs[ci]) * exp(-0.5f * mult);
  }
  return res;
}

int Gmm::WhichComponent(int color) const {
  int k = 0;
  double max = 0;

  for(int ci = 0; ci < m_components_count; ci++) {
    double p = ClusterProbability(ci, color);
    if(p > max) {
      k = ci;
      max = p;
    }
  }
  return k;
}

void Gmm::InitLearning() {
  for(int ci = 0; ci < m_components_count; ++ci) {
    m_sums[ci][0] = m_sums[ci][1] = m_sums[ci][2] = 0;
    m_prods[ci][0][0] = m_prods[ci][0][1] = m_prods[ci][0][2] = 0;
    m_prods[ci][1][0] = m_prods[ci][1][1] = m_prods[ci][1][2] = 0;
    m_prods[ci][2][0] = m_prods[ci][2][1] = m_prods[ci][2][2] = 0;
    m_sample_counts[ci] = 0;
  }
  m_total_sample_count = 0;
}

void Gmm::AddSample(int ci, int color) {
  int col[3] = GET_THREE_COORDINATE(color);
  m_sums[ci][0] += col[0];
  m_sums[ci][1] += col[1];
  m_sums[ci][2] += col[2];
  m_prods[ci][0][0] += col[0] * col[0];
  m_prods[ci][0][1] += col[0] * col[1];
  m_prods[ci][0][2] += col[0] * col[2];

  m_prods[ci][1][0] += col[1] * col[0];
  m_prods[ci][1][1] += col[1] * col[1];
  m_prods[ci][1][2] += col[1] * col[2];

  m_prods[ci][2][0] += col[2] * col[0];
  m_prods[ci][2][1] += col[2] * col[1];
  m_prods[ci][2][2] += col[2] * col[2];
  m_sample_counts[ci]++;
  m_total_sample_count++;
}

void Gmm::EndLearning() {
  const double variance = 0.01;
  for(int ci = 0; ci < m_components_count; ++ci) {
    int n = m_sample_counts[ci];
    if(n == 0) {
      m_weights[ci] = 0;
    } else {
      m_weights[ci] = (double)n / m_total_sample_count;

      m_mean[ci][0] = m_sums[ci][0] / n;
      m_mean[ci][1] = m_sums[ci][1] / n;
      m_mean[ci][2] = m_sums[ci][2] / n;

      m_cov[ci][0] = m_prods[ci][0][0] / n - m_mean[ci][0] * m_mean[ci][0];
      m_cov[ci][1] = m_prods[ci][0][1] / n - m_mean[ci][0] * m_mean[ci][1];
      m_cov[ci][2] = m_prods[ci][0][2] / n - m_mean[ci][0] * m_mean[ci][2];

      m_cov[ci][3] = m_prods[ci][1][0] / n - m_mean[ci][1] * m_mean[ci][0];
      m_cov[ci][4] = m_prods[ci][1][1] / n - m_mean[ci][1] * m_mean[ci][1];
      m_cov[ci][5] = m_prods[ci][1][2] / n - m_mean[ci][1] * m_mean[ci][2];

      m_cov[ci][6] = m_prods[ci][2][0] / n - m_mean[ci][2] * m_mean[ci][0];
      m_cov[ci][7] = m_prods[ci][2][1] / n - m_mean[ci][2] * m_mean[ci][1];
      m_cov[ci][8] = m_prods[ci][2][2] / n - m_mean[ci][2] * m_mean[ci][2];

      double dtrm = m_cov[ci][0] * (m_cov[ci][4] * m_cov[ci][8] - m_cov[ci][5] * m_cov[ci][7]) -
                    m_cov[ci][1] * (m_cov[ci][3] * m_cov[ci][8] - m_cov[ci][5] * m_cov[ci][6]) +
                    m_cov[ci][2] * (m_cov[ci][3] * m_cov[ci][7] - m_cov[ci][4] * m_cov[ci][6]);
      if(dtrm <= EPSILON) {
        // Adds the white noise to avoid singular covariance matrix.
        m_cov[ci][0] += variance;
        m_cov[ci][4] += variance;
        m_cov[ci][8] += variance;
      }
      CalcInverseCovAndDeterm(ci);
    }
  }
}

void Gmm::CalcInverseCovAndDeterm(int ci) {
  if(m_weights[ci] > 0) {
    double dtrm = m_cov_determs[ci] =
      m_cov[ci][0] * (m_cov[ci][4] * m_cov[ci][8] - m_cov[ci][5] * m_cov[ci][7]) -
      m_cov[ci][1] * (m_cov[ci][3] * m_cov[ci][8] - m_cov[ci][5] * m_cov[ci][6]) +
      m_cov[ci][2] * (m_cov[ci][3] * m_cov[ci][7] - m_cov[ci][4] * m_cov[ci][6]);

    m_inverse_covs[ci][0][0] =  (m_cov[ci][4] * m_cov[ci][8] - m_cov[ci][5] * m_cov[ci][7]) / dtrm;
    m_inverse_covs[ci][1][0] = -(m_cov[ci][3] * m_cov[ci][8] - m_cov[ci][5] * m_cov[ci][6]) / dtrm;
    m_inverse_covs[ci][2][0] =  (m_cov[ci][3] * m_cov[ci][7] - m_cov[ci][4] * m_cov[ci][6]) / dtrm;
    m_inverse_covs[ci][0][1] = -(m_cov[ci][1] * m_cov[ci][8] - m_cov[ci][2] * m_cov[ci][7]) / dtrm;
    m_inverse_covs[ci][1][1] =  (m_cov[ci][0] * m_cov[ci][8] - m_cov[ci][2] * m_cov[ci][6]) / dtrm;
    m_inverse_covs[ci][2][1] = -(m_cov[ci][0] * m_cov[ci][7] - m_cov[ci][1] * m_cov[ci][6]) / dtrm;
    m_inverse_covs[ci][0][2] =  (m_cov[ci][1] * m_cov[ci][5] - m_cov[ci][2] * m_cov[ci][4]) / dtrm;
    m_inverse_covs[ci][1][2] = -(m_cov[ci][0] * m_cov[ci][5] - m_cov[ci][2] * m_cov[ci][3]) / dtrm;
    m_inverse_covs[ci][2][2] =  (m_cov[ci][0] * m_cov[ci][4] - m_cov[ci][1] * m_cov[ci][3]) / dtrm;
  }
}
