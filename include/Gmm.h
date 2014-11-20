// Copyright sxniu 2014-10
#ifndef INCLUDE_GMM_H_
#define INCLUDE_GMM_H_

template <class T>
class ImageData;

class Gmm {
 public:
  static const int m_components_count = 5;

  double SceneProbability(int color) const;
  double ClusterProbability(int ci, int color) const;
  int WhichComponent(int color) const;

  void InitLearning();
  void AddSample(int ci, int color);
  void EndLearning();

 private:
  void CalcInverseCovAndDeterm(int ci);
  double m_weights[m_components_count];
  double m_mean[m_components_count][3];
  double m_cov[m_components_count][9];

  double m_inverse_covs[m_components_count][3][3];
  double m_cov_determs[m_components_count];

  double m_sums[m_components_count][3];
  double m_prods[m_components_count][3][3];
  int m_sample_counts[m_components_count];
  int m_total_sample_count;
};

#endif  // INCLUDE_GMM_H_
