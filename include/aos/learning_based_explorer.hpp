#ifndef ORBIT_PLANNER_LEARNING_BASED_EXPLORER_HPP
#define ORBIT_PLANNER_LEARNING_BASED_EXPLORER_HPP

#include <memory>
#include <vector>
#include <unordered_map>
#include <queue>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <Eigen/Dense>

namespace orbit_planner
{

/**
 * @brief Learning-based exploration strategy
 * 
 * This class implements machine learning approaches for exploration,
 * including reinforcement learning and experience-based planning.
 */
class LearningBasedExplorer
{
public:
  /**
   * @brief Exploration state representation
   */
  struct ExplorationState
  {
    Eigen::VectorXd features;
    double reward;
    bool terminal;
    std::string state_id;
    
    ExplorationState() : reward(0.0), terminal(false) {}
  };

  /**
   * @brief Action representation
   */
  struct ExplorationAction
  {
    geometry_msgs::msg::Point target;
    double confidence;
    std::string action_type;
    
    ExplorationAction() : confidence(0.0) {}
  };

  /**
   * @brief Experience replay buffer
   */
  struct Experience
  {
    ExplorationState state;
    ExplorationAction action;
    double reward;
    ExplorationState next_state;
    bool done;
    
    Experience() : reward(0.0), done(false) {}
  };

  /**
   * @brief Learning configuration
   */
  struct LearningConfig
  {
    bool enable_learning;
    bool enable_experience_replay;
    double learning_rate;
    double discount_factor;
    double epsilon_start;
    double epsilon_end;
    double epsilon_decay;
    int replay_buffer_size;
    int batch_size;
    int update_frequency;
    bool use_prioritized_replay;
    double alpha; // Prioritization exponent
    double beta;  // Importance sampling correction
    
    LearningConfig()
      : enable_learning(true)
      , enable_experience_replay(true)
      , learning_rate(0.001)
      , discount_factor(0.99)
      , epsilon_start(1.0)
      , epsilon_end(0.01)
      , epsilon_decay(0.995)
      , replay_buffer_size(10000)
      , batch_size(32)
      , update_frequency(100)
      , use_prioritized_replay(true)
      , alpha(0.6)
      , beta(0.4)
    {}
  };

  /**
   * @brief Constructor
   * @param config Learning configuration
   */
  explicit LearningBasedExplorer(const LearningConfig & config = LearningConfig());

  /**
   * @brief Destructor
   */
  ~LearningBasedExplorer();

  /**
   * @brief Initialize the learning system
   * @param state_dimension State feature dimension
   * @param action_dimension Action space dimension
   */
  void initialize(int state_dimension, int action_dimension);

  /**
   * @brief Load pre-trained model
   * @param model_path Path to saved model
   * @return true if loading successful
   */
  bool loadModel(const std::string & model_path);

  /**
   * @brief Save current model
   * @param model_path Path to save model
   * @return true if saving successful
   */
  bool saveModel(const std::string & model_path);

  /**
   * @brief Select action using current policy
   * @param state Current exploration state
   * @param available_actions Available actions
   * @return Selected action
   */
  ExplorationAction selectAction(
    const ExplorationState & state,
    const std::vector<ExplorationAction> & available_actions);

  /**
   * @brief Update policy with new experience
   * @param experience New experience
   */
  void updatePolicy(const Experience & experience);

  /**
   * @brief Train the model on collected experiences
   * @return Training loss
   */
  double trainModel();

  /**
   * @brief Extract features from current state
   * @param robot_pose Current robot pose
   * @param point_cloud Current point cloud
   * @param exploration_area Exploration area
   * @return Feature vector
   */
  Eigen::VectorXd extractFeatures(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const sensor_msgs::msg::PointCloud2 & point_cloud,
    const geometry_msgs::msg::PolygonStamped & exploration_area);

  /**
   * @brief Calculate reward for action
   * @param state Current state
   * @param action Taken action
   * @param next_state Resulting state
   * @return Reward value
   */
  double calculateReward(
    const ExplorationState & state,
    const ExplorationAction & action,
    const ExplorationState & next_state);

  /**
   * @brief Generate action candidates
   * @param state Current state
   * @param max_candidates Maximum number of candidates
   * @return Generated action candidates
   */
  std::vector<ExplorationAction> generateActionCandidates(
    const ExplorationState & state,
    int max_candidates = 10);

  /**
   * @brief Update exploration statistics
   * @param state Current state
   * @param action Taken action
   * @param reward Received reward
   */
  void updateStatistics(
    const ExplorationState & state,
    const ExplorationAction & action,
    double reward);

  /**
   * @brief Get learning statistics
   * @return Learning statistics
   */
  struct LearningStatistics
  {
    int total_episodes;
    int total_steps;
    double average_reward;
    double total_reward;
    double epsilon;
    double learning_loss;
    int replay_buffer_size;
    double exploration_rate;
  };
  
  LearningStatistics getLearningStatistics() const;

  /**
   * @brief Reset learning state
   */
  void reset();

  /**
   * @brief Set learning configuration
   * @param config New configuration
   */
  void setConfig(const LearningConfig & config);

  /**
   * @brief Get current configuration
   * @return Current configuration
   */
  const LearningConfig & getConfig() const;

private:
  /**
   * @brief Neural network forward pass
   */
  Eigen::VectorXd forwardPass(const Eigen::VectorXd & input);

  /**
   * @brief Neural network backward pass
   */
  void backwardPass(const Eigen::VectorXd & input, const Eigen::VectorXd & target);

  /**
   * @brief Sample batch from replay buffer
   */
  std::vector<Experience> sampleBatch(int batch_size);

  /**
   * @brief Calculate TD error for experience
   */
  double calculateTDError(const Experience & experience);

  /**
   * @brief Update target network
   */
  void updateTargetNetwork();

  /**
   * @brief Calculate priority for experience
   */
  double calculatePriority(const Experience & experience);

  /**
   * @brief Apply importance sampling correction
   */
  double applyImportanceSampling(double priority, int buffer_size);

  /**
   * @brief Epsilon-greedy action selection
   */
  ExplorationAction epsilonGreedyAction(
    const ExplorationState & state,
    const std::vector<ExplorationAction> & available_actions);

  /**
   * @brief Greedy action selection
   */
  ExplorationAction greedyAction(
    const ExplorationState & state,
    const std::vector<ExplorationAction> & available_actions);

  // Configuration
  LearningConfig config_;
  
  // Neural network parameters
  std::vector<int> network_layers_;
  std::vector<Eigen::MatrixXd> weights_;
  std::vector<Eigen::VectorXd> biases_;
  std::vector<Eigen::MatrixXd> target_weights_;
  std::vector<Eigen::VectorXd> target_biases_;
  
  // Experience replay buffer
  std::vector<Experience> replay_buffer_;
  std::vector<double> priorities_;
  int buffer_index_;
  bool buffer_full_;
  
  // Learning state
  int state_dimension_;
  int action_dimension_;
  int episode_count_;
  int step_count_;
  double current_epsilon_;
  double total_reward_;
  double average_reward_;
  double learning_loss_;
  
  // Statistics
  LearningStatistics statistics_;
  
  // Random number generation
  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_real_distribution<double> uniform_dist_;
  std::normal_distribution<double> normal_dist_;
};

} // namespace orbit_planner

#endif // ORBIT_PLANNER_LEARNING_BASED_EXPLORER_HPP
