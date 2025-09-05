#pragma once

#include <algorithm>
#include <builtin_interfaces/msg/time.hpp>
#include <game_state_interfaces/msg/match.hpp>
#include <game_state_interfaces/msg/team.hpp>
#include <game_state_interfaces/srv/end_match.hpp>
#include <game_state_interfaces/srv/update_score.hpp>
#include <httplib.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <unordered_map>

namespace tourobo2024 {
namespace Score {
constexpr int UNLOCK = 10;
constexpr int TYPE_1 = 10;
constexpr int TYPE_2 = 20;
constexpr int TYPE_3 = 50;
}; // namespace Score
class StateManager : public rclcpp::Node {
private:
  std::shared_ptr<httplib::Client> tourobo_cli_;
  std::shared_ptr<httplib::Client> update_match_cli_;
  game_state_interfaces::msg::Match current_match_;
  std::string competition_id_;
  std::string api_key_;

  rclcpp::Publisher<game_state_interfaces::msg::Match>::SharedPtr match_status_pub_;
  rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr ore_ore_clock_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr load_next_map_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_match_srv_;
  rclcpp::Service<game_state_interfaces::srv::EndMatch>::SharedPtr end_match_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_match_srv_;
  rclcpp::Service<game_state_interfaces::srv::UpdateScore>::SharedPtr red_update_score_srv_;
  rclcpp::Service<game_state_interfaces::srv::UpdateScore>::SharedPtr blue_update_score_srv_;

  // timer
  rclcpp::TimerBase::SharedPtr _3_min_timer_;
  rclcpp::TimerBase::SharedPtr match_status_pub_timer_;
  rclcpp::TimerBase::SharedPtr dbg_state_output_timer_;

  bool is_match_start_ = false;
  bool is_match_end_ = true;
  bool is_match_confirmed_ = true;

  // Ownership grid: 0 = none, 1 = red, 2 = blue
  int grid_owner_[3][3] = {{0}}; // [row][col], type_1_a = [0][0], type_1_b = [0][1], ..., type_3_c = [2][2]

  // Track first unlock time for each team (seconds since match start, -1 if not yet)
  double red_unlock_time_ = -1;
  double blue_unlock_time_ = -1;

  // Helper: get cell count for a team
  int get_cell_count(const game_state_interfaces::msg::Team &team, int row, int col) {
    if (row == 0 && col == 0)
      return team.type_1_a;
    if (row == 0 && col == 1)
      return team.type_1_b;
    if (row == 0 && col == 2)
      return team.type_1_c;
    if (row == 1 && col == 0)
      return team.type_2_a;
    if (row == 1 && col == 1)
      return team.type_2_b;
    if (row == 1 && col == 2)
      return team.type_2_c;
    if (row == 2 && col == 0)
      return team.type_3_a;
    if (row == 2 && col == 1)
      return team.type_3_b;
    if (row == 2 && col == 2)
      return team.type_3_c;
    return 0;
  }

  // Update grid ownership after score change
  void update_grid_ownership() {
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        int red_count = get_cell_count(current_match_.red_team, row, col);
        int blue_count = get_cell_count(current_match_.blue_team, row, col);
        if (red_count > blue_count) {
          grid_owner_[row][col] = 1;
        } else if (blue_count > red_count) {
          grid_owner_[row][col] = 2;
        } else {
          grid_owner_[row][col] = 0;
        }
      }
    }
    // Update current_match_ cell ownership info
    current_match_.type_1_a = grid_owner_[0][0] == 1   ? game_state_interfaces::msg::Match::RED
                              : grid_owner_[0][0] == 2 ? game_state_interfaces::msg::Match::BLUE
                                                       : game_state_interfaces::msg::Match::UNKNOWN;
    current_match_.type_1_b = grid_owner_[0][1] == 1   ? game_state_interfaces::msg::Match::RED
                              : grid_owner_[0][1] == 2 ? game_state_interfaces::msg::Match::BLUE
                                                       : game_state_interfaces::msg::Match::UNKNOWN;
    current_match_.type_1_c = grid_owner_[0][2] == 1   ? game_state_interfaces::msg::Match::RED
                              : grid_owner_[0][2] == 2 ? game_state_interfaces::msg::Match::BLUE
                                                       : game_state_interfaces::msg::Match::UNKNOWN;
    current_match_.type_2_a = grid_owner_[1][0] == 1   ? game_state_interfaces::msg::Match::RED
                              : grid_owner_[1][0] == 2 ? game_state_interfaces::msg::Match::BLUE
                                                       : game_state_interfaces::msg::Match::UNKNOWN;
    current_match_.type_2_b = grid_owner_[1][1] == 1   ? game_state_interfaces::msg::Match::RED
                              : grid_owner_[1][1] == 2 ? game_state_interfaces::msg::Match::BLUE
                                                       : game_state_interfaces::msg::Match::UNKNOWN;
    current_match_.type_2_c = grid_owner_[1][2] == 1   ? game_state_interfaces::msg::Match::RED
                              : grid_owner_[1][2] == 2 ? game_state_interfaces::msg::Match::BLUE
                                                       : game_state_interfaces::msg::Match::UNKNOWN;
    current_match_.type_3_a = grid_owner_[2][0] == 1   ? game_state_interfaces::msg::Match::RED
                              : grid_owner_[2][0] == 2 ? game_state_interfaces::msg::Match::BLUE
                                                       : game_state_interfaces::msg::Match::UNKNOWN;
    current_match_.type_3_b = grid_owner_[2][1] == 1   ? game_state_interfaces::msg::Match::RED
                              : grid_owner_[2][1] == 2 ? game_state_interfaces::msg::Match::BLUE
                                                       : game_state_interfaces::msg::Match::UNKNOWN;
    current_match_.type_3_c = grid_owner_[2][2] == 1   ? game_state_interfaces::msg::Match::RED
                              : grid_owner_[2][2] == 2 ? game_state_interfaces::msg::Match::BLUE
                                                       : game_state_interfaces::msg::Match::UNKNOWN;
  }

  // Count bingos for a team (team_id: 1=red, 2=blue)
  int count_bingo(int team_id) {
    int bingo_count = 0;
    // Rows
    for (int row = 0; row < 3; ++row) {
      if (grid_owner_[row][0] == team_id && grid_owner_[row][1] == team_id && grid_owner_[row][2] == team_id)
        ++bingo_count;
    }
    // Columns
    for (int col = 0; col < 3; ++col) {
      if (grid_owner_[0][col] == team_id && grid_owner_[1][col] == team_id && grid_owner_[2][col] == team_id)
        ++bingo_count;
    }
    // Diagonals
    if (grid_owner_[0][0] == team_id && grid_owner_[1][1] == team_id && grid_owner_[2][2] == team_id)
      ++bingo_count;
    if (grid_owner_[0][2] == team_id && grid_owner_[1][1] == team_id && grid_owner_[2][0] == team_id)
      ++bingo_count;
    return bingo_count;
  }

  // Count spot ownership for a team (total, type3, type2, type1)
  void count_spots(int team_id, int &total, int &type3, int &type2, int &type1) {
    total = type3 = type2 = type1 = 0;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        if (grid_owner_[row][col] == team_id) {
          ++total;
          if (row == 2)
            ++type3;
          else if (row == 1)
            ++type2;
          else if (row == 0)
            ++type1;
        }
      }
    }
  }

  bool _match_already_set(game_state_interfaces::msg::Match &m) { return current_match_.id == m.id; }

  bool _is_match_ready() { return !is_match_start_ && !is_match_confirmed_; }

  bool _is_match_running() { return is_match_start_ && !is_match_end_; }

  bool _is_match_end() { return is_match_start_ && is_match_end_; }

  bool _is_match_confirmed() { return is_match_confirmed_; }

  void update_score(game_state_interfaces::msg::Team &team, game_state_interfaces::srv::UpdateScore::Request::SharedPtr req) {
    using ScoreCmd = game_state_interfaces::srv::UpdateScore::Request;
    switch (req->command) {
    case ScoreCmd::UNLOCK:
      // Record first unlock time for each team
      if (&team == &current_match_.red_team && team.unlock == 0 && req->data > 0 && red_unlock_time_ < 0) {
        red_unlock_time_ = (this->get_clock()->now() - rclcpp::Time(current_match_.start_time)).seconds();
      }
      if (&team == &current_match_.blue_team && team.unlock == 0 && req->data > 0 && blue_unlock_time_ < 0) {
        blue_unlock_time_ = (this->get_clock()->now() - rclcpp::Time(current_match_.start_time)).seconds();
      }
      team.unlock += req->data;
      break;
    case ScoreCmd::TYPE_1_A:
      team.type_1_a += req->data;
      break;
    case ScoreCmd::TYPE_1_B:
      team.type_1_b += req->data;
      break;
    case ScoreCmd::TYPE_1_C:
      team.type_1_c += req->data;
      break;
    case ScoreCmd::TYPE_2_A:
      team.type_2_a += req->data;
      break;
    case ScoreCmd::TYPE_2_B:
      team.type_2_b += req->data;
      break;
    case ScoreCmd::TYPE_2_C:
      team.type_2_c += req->data;
      break;
    case ScoreCmd::TYPE_3_A:
      team.type_3_a += req->data;
      break;
    case ScoreCmd::TYPE_3_B:
      team.type_3_b += req->data;
      break;
    case ScoreCmd::TYPE_3_C:
      team.type_3_c += req->data;
      break;
    case ScoreCmd::V_GOAL:
      team.v_goal = req->data;
      if (team.v_goal) {
        current_match_.end_time = this->get_clock()->now();
      }
      break;
    default:
      break;
    }

    // score calculation
    int unlock_score = 0;
    int type1_score = 0;
    int type2_score = 0;
    int type3_score = 0;
    unlock_score = team.unlock * Score::UNLOCK;
    type1_score = team.type_1_a * Score::TYPE_1 + team.type_1_b * Score::TYPE_1 + team.type_1_c * Score::TYPE_1;
    type2_score = team.type_2_a * Score::TYPE_2 + team.type_2_b * Score::TYPE_2 + team.type_2_c * Score::TYPE_2;
    type3_score = team.type_3_a * Score::TYPE_3 + team.type_3_b * Score::TYPE_3 + team.type_3_c * Score::TYPE_3;
    team.score = unlock_score + type1_score + type2_score + type3_score;

    // Update grid ownership and count bingos
    update_grid_ownership();
    int red_bingo_count = count_bingo(1);
    int blue_bingo_count = count_bingo(2);

    // You can add logic here if you want to do something when bingo is achieved
    if (red_bingo_count > 0) {
      RCLCPP_INFO(this->get_logger(), "Red team achieved %d BINGO(s)!", red_bingo_count);
      // Optionally: team.score += bonus * red_bingo_count;
    }
    if (blue_bingo_count > 0) {
      RCLCPP_INFO(this->get_logger(), "Blue team achieved %d BINGO(s)!", blue_bingo_count);
      // Optionally: team.score += bonus * blue_bingo_count;
    }
  }

public:
  StateManager(const rclcpp::NodeOptions &options) : StateManager("", options) {}

  StateManager(const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("state_manager_node", name_space, options) {
    using namespace std::chrono_literals;
    // httplib setup
    std::string tourobo_base_url = this->declare_parameter<std::string>("next_match_api", "https://rims.tourobo.net");
    tourobo_cli_ = std::make_shared<httplib::Client>(tourobo_base_url);

    // param
    competition_id_ = this->declare_parameter<std::string>("competition_id");
    api_key_ = this->declare_parameter<std::string>("api_key");

    match_status_pub_ = this->create_publisher<game_state_interfaces::msg::Match>("/match/status", rclcpp::QoS(10).keep_last(10));
    ore_ore_clock_pub_ = this->create_publisher<builtin_interfaces::msg::Time>("/match/clock", rclcpp::QoS(10));
    load_next_map_srv_ =
        this->create_service<std_srvs::srv::Empty>("/match/load_next", std::bind(&StateManager::load_next_match, this, std::placeholders::_1, std::placeholders::_2));
    start_match_srv_ = this->create_service<std_srvs::srv::Empty>("/match/start", std::bind(&StateManager::start_match, this, std::placeholders::_1, std::placeholders::_2));
    end_match_srv_ =
        this->create_service<game_state_interfaces::srv::EndMatch>("/match/end", std::bind(&StateManager::end_match, this, std::placeholders::_1, std::placeholders::_2));
    reset_match_srv_ = this->create_service<std_srvs::srv::Empty>("/match/reset", std::bind(&StateManager::reset_match, this, std::placeholders::_1, std::placeholders::_2));
    red_update_score_srv_ = this->create_service<game_state_interfaces::srv::UpdateScore>(
        "/red/update_score", std::bind(&StateManager::red_update_score, this, std::placeholders::_1, std::placeholders::_2));
    blue_update_score_srv_ = this->create_service<game_state_interfaces::srv::UpdateScore>(
        "/blue/update_score", std::bind(&StateManager::blue_update_score, this, std::placeholders::_1, std::placeholders::_2));

    _3_min_timer_ = this->create_wall_timer(180s, std::bind(&StateManager::match_time_callback, this));
    match_status_pub_timer_ = this->create_wall_timer(100ms, [this]() { this->match_status_pub_->publish(this->current_match_); });
    dbg_state_output_timer_ = this->create_wall_timer(1s, std::bind(&StateManager::dump_current_state, this));
    _3_min_timer_->cancel();
  }

  void load_next_match(const std_srvs::srv::Empty_Request::SharedPtr, const std_srvs::srv::Empty_Response::SharedPtr) {
    httplib::Headers headers = {{"Authorization", "Bearer " + api_key_}};
    std::string url = "/api/v2/alignment/get_match/?id=" + competition_id_;
    auto res = tourobo_cli_->Get(url.c_str(), headers);
    if (res) {
      nlohmann::json res_body = nlohmann::json::parse(res->body);

      game_state_interfaces::msg::Match match;
      game_state_interfaces::msg::Team red_team;
      game_state_interfaces::msg::Team blue_team;

      // 1試合のみ取得
      const auto &match_json = res_body["match"][0];

      red_team.name = match_json["zone1"]["name"];
      red_team.university = match_json["zone1"]["organization"];
      red_team.id = match_json["zone1"]["id"];
      blue_team.name = match_json["zone2"]["name"];
      blue_team.university = match_json["zone2"]["organization"];
      blue_team.id = match_json["zone2"]["id"];

      match.red_team = red_team;
      match.blue_team = blue_team;
      match.id = match_json["gameID"];
      match.title = match_json["title"];
      if (!_match_already_set(match)) {
        current_match_ = match;
        init_match();
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "error : %s", httplib::to_string(res.error()).c_str());
    }
  }

  void init_match() {
    if (!_3_min_timer_->is_canceled()) {
      _3_min_timer_->cancel();
    }
    is_match_start_ = false;
    is_match_end_ = false;
    is_match_confirmed_ = false;

    current_match_.winner = game_state_interfaces::msg::Match::UNKNOWN;
    current_match_.red_team.unlock = 0;
    current_match_.red_team.type_1_a = 0;
    current_match_.red_team.type_1_b = 0;
    current_match_.red_team.type_1_c = 0;
    current_match_.red_team.type_2_a = 0;
    current_match_.red_team.type_2_b = 0;
    current_match_.red_team.type_2_c = 0;
    current_match_.red_team.type_3_a = 0;
    current_match_.red_team.type_3_b = 0;
    current_match_.red_team.type_3_c = 0;
    current_match_.red_team.v_goal = false;
    current_match_.red_team.score = 0;
    current_match_.blue_team.unlock = 0;
    current_match_.blue_team.type_1_a = 0;
    current_match_.blue_team.type_1_b = 0;
    current_match_.blue_team.type_1_c = 0;
    current_match_.blue_team.type_2_a = 0;
    current_match_.blue_team.type_2_b = 0;
    current_match_.blue_team.type_2_c = 0;
    current_match_.blue_team.type_3_a = 0;
    current_match_.blue_team.type_3_b = 0;
    current_match_.blue_team.type_3_c = 0;
    current_match_.blue_team.v_goal = false;
    current_match_.blue_team.score = 0;
    current_match_.type_1_a = game_state_interfaces::msg::Match::UNKNOWN;
    current_match_.type_1_b = game_state_interfaces::msg::Match::UNKNOWN;
    current_match_.type_1_c = game_state_interfaces::msg::Match::UNKNOWN;
    current_match_.type_2_a = game_state_interfaces::msg::Match::UNKNOWN;
    current_match_.type_2_b = game_state_interfaces::msg::Match::UNKNOWN;
    current_match_.type_2_c = game_state_interfaces::msg::Match::UNKNOWN;
    current_match_.type_3_a = game_state_interfaces::msg::Match::UNKNOWN;
    current_match_.type_3_b = game_state_interfaces::msg::Match::UNKNOWN;
    current_match_.type_3_c = game_state_interfaces::msg::Match::UNKNOWN;
    current_match_.start_time.sec = 0;
    current_match_.start_time.nanosec = 0;
    current_match_.end_time.sec = 0;
    current_match_.end_time.nanosec = 0;

    // Reset grid ownership
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 3; ++col)
        grid_owner_[row][col] = 0;
  }

  void start_match(const std_srvs::srv::Empty_Request::SharedPtr, const std_srvs::srv::Empty_Response::SharedPtr) {
    if (!_is_match_ready()) {
      return;
    }

    is_match_start_ = true;
    is_match_end_ = false;
    is_match_confirmed_ = false;

    current_match_.start_time = this->get_clock()->now();
    _3_min_timer_->reset();
  }

  void end_match(const game_state_interfaces::srv::EndMatch::Request::SharedPtr req, const game_state_interfaces::srv::EndMatch::Response::SharedPtr res) {
    using EndMatchRefereeCall = game_state_interfaces::srv::EndMatch::Request;
    auto red_team = current_match_.red_team;
    auto blue_team = current_match_.blue_team;

    if ((req->referee_call == EndMatchRefereeCall::NONE) && (red_team.score == blue_team.score)) {
      res->result = false;
      return;
    }

    is_match_start_ = true;
    is_match_end_ = true;
    if (current_match_.end_time.sec == 0 && current_match_.end_time.nanosec == 0) {
      current_match_.end_time = this->get_clock()->now();
    }
    if (!_3_min_timer_->is_canceled()) {
      _3_min_timer_->cancel();
    }

    if (red_team.v_goal || (req->referee_call == EndMatchRefereeCall::RED) || (red_team.score > blue_team.score)) {
      current_match_.winner = game_state_interfaces::msg::Match::RED;
    } else {
      current_match_.winner = game_state_interfaces::msg::Match::BLUE;
    }

    res->result = match_confirm();
  }

  void reset_match(const std_srvs::srv::Empty::Request::SharedPtr, const std_srvs::srv::Empty::Response::SharedPtr) { init_match(); }

  void red_update_score(const game_state_interfaces::srv::UpdateScore::Request::SharedPtr req, const game_state_interfaces::srv::UpdateScore::Response::SharedPtr res) {
    if (_is_match_ready()) {
      res->result = true;
      return;
    }
    if (!_is_match_running()) {
      res->result = false;
      return;
    }
    update_score(current_match_.red_team, req);
    res->result = true;
  }

  void blue_update_score(const game_state_interfaces::srv::UpdateScore::Request::SharedPtr req, const game_state_interfaces::srv::UpdateScore::Response::SharedPtr res) {
    if (_is_match_ready()) {
      res->result = true;
      return;
    }
    if (!_is_match_running()) {
      res->result = false;
      return;
    }
    update_score(current_match_.blue_team, req);
    res->result = true;
  }

  bool match_confirm() {
    if (!_is_match_end() || _is_match_confirmed()) {
      return false;
    }

    auto winner = current_match_.winner == game_state_interfaces::msg::Match::RED ? current_match_.red_team : current_match_.blue_team;
    int time_sec = static_cast<int>(std::floor((rclcpp::Time(current_match_.end_time) - rclcpp::Time(current_match_.start_time)).seconds()));

    // Calculate additional scores for both teams
    int red_bingo_count = count_bingo(1);
    int blue_bingo_count = count_bingo(2);

    int red_spot_total, red_type3, red_type2, red_type1;
    int blue_spot_total, blue_type3, blue_type2, blue_type1;
    count_spots(1, red_spot_total, red_type3, red_type2, red_type1);
    count_spots(2, blue_spot_total, blue_type3, blue_type2, blue_type1);

    int red_unlock_score = current_match_.red_team.unlock * Score::UNLOCK;
    int blue_unlock_score = current_match_.blue_team.unlock * Score::UNLOCK;

    nlohmann::json body = {
        {"competitionID", competition_id_},
        {"api_key", api_key_},
        {"gameID", current_match_.id},
        {"status", "confirmation"},
        {"winner", current_match_.winner == game_state_interfaces::msg::Match::RED ? "zone1" : "zone2"},
        {"zone1_score", current_match_.red_team.score},
        {"zone2_score", current_match_.blue_team.score},
        // Additional scores for red team
        {"zone1_score_additional_1", red_bingo_count},
        {"zone1_score_additional_2", red_spot_total},
        {"zone1_score_additional_3", red_type3},
        {"zone1_score_additional_4", red_type2},
        {"zone1_score_additional_5", red_type1},
        {"zone1_score_additional_6", red_unlock_score},
        {"zone1_score_additional_7", red_unlock_time_ >= 0 ? (-1 * static_cast<int>(red_unlock_time_)) : 0},
        {"zone1_score_additional_8", 0},
        // Additional scores for blue team
        {"zone2_score_additional_1", blue_bingo_count},
        {"zone2_score_additional_2", blue_spot_total},
        {"zone2_score_additional_3", blue_type3},
        {"zone2_score_additional_4", blue_type2},
        {"zone2_score_additional_5", blue_type1},
        {"zone2_score_additional_6", blue_unlock_score},
        {"zone2_score_additional_7", blue_unlock_time_ >= 0 ? (-1 * static_cast<int>(blue_unlock_time_)) : 0},
        {"zone2_score_additional_8", 0},
        {"vgole", winner.v_goal},
        {"vgole_time", winner.v_goal ? time_sec : 0},
    };

    // v2 APIパスに変更
    httplib::Headers headers = {{"Authorization", "Bearer " + api_key_}};
    std::string url = "/api/v2/alignment/update_match/?id=" + competition_id_;
    auto res = tourobo_cli_->Post(url.c_str(), headers, body.dump(), "application/json");
    if (res) {
      RCLCPP_INFO(this->get_logger(), "last match successfully confirmed");
      is_match_confirmed_ = true;
    } else {
      RCLCPP_WARN(this->get_logger(), "error : %s", httplib::to_string(res.error()).c_str());
    }

    return res;
  }

  void match_time_callback() {
    _3_min_timer_->cancel();
    if (!current_match_.red_team.v_goal && !current_match_.blue_team.v_goal) {
      current_match_.end_time = this->get_clock()->now();
    }
  }

  void dump_current_state() {
    builtin_interfaces::msg::Time now_msg = this->get_clock()->now();
    ore_ore_clock_pub_->publish(now_msg);

    RCLCPP_INFO(this->get_logger(), "===================");
    if (_is_match_confirmed()) {
      RCLCPP_INFO(this->get_logger(), "no match data");
      RCLCPP_INFO(this->get_logger(), "===================");
      return;
    }
    if (_is_match_ready()) {
      RCLCPP_INFO(this->get_logger(), "Match %s is scheduled", current_match_.id.c_str());
      RCLCPP_INFO(this->get_logger(), "Red zone %s from %s", current_match_.red_team.name.c_str(), current_match_.red_team.university.c_str());
      RCLCPP_INFO(this->get_logger(), "Blue zone %s from %s", current_match_.blue_team.name.c_str(), current_match_.blue_team.university.c_str());
    } else if (_is_match_running()) {
      int time_sec;
      if (_3_min_timer_->is_canceled()) {
        time_sec = static_cast<int>(std::floor((rclcpp::Time(current_match_.end_time) - rclcpp::Time(current_match_.start_time)).seconds()));
      } else {
        time_sec = static_cast<int>(std::floor((this->get_clock()->now() - rclcpp::Time(current_match_.start_time)).seconds()));
      }
      RCLCPP_INFO(this->get_logger(), "Match %s is running : %d[s]", current_match_.id.c_str(), time_sec);
      RCLCPP_INFO(this->get_logger(), "Red zone %s from %s", current_match_.red_team.name.c_str(), current_match_.red_team.university.c_str());
      RCLCPP_INFO(this->get_logger(), "Blue zone %s from %s", current_match_.blue_team.name.c_str(), current_match_.blue_team.university.c_str());
      RCLCPP_INFO(this->get_logger(), "Red point %d vs Blue point %d", current_match_.red_team.score, current_match_.blue_team.score);
    } else if (_is_match_end()) {
      auto winner = current_match_.winner == game_state_interfaces::msg::Match::RED ? current_match_.red_team : current_match_.blue_team;
      int time_sec = static_cast<int>(std::floor((rclcpp::Time(current_match_.end_time) - rclcpp::Time(current_match_.start_time)).seconds()));
      RCLCPP_INFO(this->get_logger(), "Match %s ended in %d[s] but unconfirmed", current_match_.id.c_str(), time_sec);
      if (winner.id == "") {
        RCLCPP_INFO(this->get_logger(), "DRAW MATCH");
      } else {
        RCLCPP_INFO(this->get_logger(), "winner : %s", winner.name.c_str());
        if (winner.v_goal) {
          RCLCPP_INFO(this->get_logger(), "archieved v goal");
        }
        RCLCPP_INFO(this->get_logger(), "score : %d", winner.score);
      }
    }
    int red_bingo_count = count_bingo(1);
    int blue_bingo_count = count_bingo(2);
    RCLCPP_INFO(this->get_logger(), "Red team BINGO count: %d", red_bingo_count);
    RCLCPP_INFO(this->get_logger(), "Blue team BINGO count: %d", blue_bingo_count);
    RCLCPP_INFO(this->get_logger(), "===================");
  }
};
} // namespace tourobo2024
