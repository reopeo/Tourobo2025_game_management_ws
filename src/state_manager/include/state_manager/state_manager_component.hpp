#pragma once

#define CPPHTTPLIB_OPENSSL_SUPPORT

#include <httplib.h>

#include <algorithm>
#include <game_state_interfaces/msg/match.hpp>
#include <game_state_interfaces/msg/team.hpp>
#include <game_state_interfaces/srv/end_match.hpp>
#include <game_state_interfaces/srv/update_score.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <unordered_map>

namespace torobo2024 {
namespace Score {
namespace MANUAL {
constexpr int SEEDLINGS = 5;
constexpr int IMMIGRATION = 15;
constexpr int TYPE_1 = 5;
constexpr int TYPE_2 = 10;
}; // namespace MANUAL
namespace AUTO {
constexpr int SEEDLINGS = 10;
constexpr int IMMIGRATION = 15;
constexpr int TYPE_1 = 10;
constexpr int TYPE_2 = 20;
}; // namespace AUTO
}; // namespace Score
class StateManager : public rclcpp::Node {
private:
  std::shared_ptr<httplib::Client> tourobo_cli_;
  std::shared_ptr<httplib::Client> update_match_cli_;
  game_state_interfaces::msg::Match current_match_;
  std::string competition_id_;
  std::string api_key_;

  rclcpp::Publisher<game_state_interfaces::msg::Match>::SharedPtr match_status_pub_;
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

  bool _match_already_set(game_state_interfaces::msg::Match &m) { return current_match_.id == m.id; }

  bool _is_match_ready() { return !is_match_start_ && !is_match_confirmed_; }

  bool _is_match_running() { return is_match_start_ && !is_match_end_; }

  bool _is_match_end() { return is_match_start_ && is_match_end_; }

  bool _is_match_confirmed() { return is_match_confirmed_; }

  void update_score(game_state_interfaces::msg::Team &team, game_state_interfaces::srv::UpdateScore::Request::SharedPtr req) {
    using ScoreCmd = game_state_interfaces::srv::UpdateScore::Request;
    switch (req->command) {
    case ScoreCmd::IS_AUTO:
      team.is_auto = static_cast<bool>(req->data);
      break;
    case ScoreCmd::SEEDLINGS:
      team.seedlings += req->data;
      break;
    case ScoreCmd::IMMIGRATION:
      team.immigration = req->data;
      break;
    case ScoreCmd::TYPE_1_A:
      team.type_1_a = req->data;
      break;
    case ScoreCmd::TYPE_1_B:
      team.type_1_b = req->data;
      break;
    case ScoreCmd::TYPE_2:
      team.type_2 = req->data;
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
    int seedling_score = 0;
    int immigration_score = 0;
    int type1_score = 0;
    int type2_score = 0;
    if (team.is_auto) {
      seedling_score = team.seedlings * Score::AUTO::SEEDLINGS;
      immigration_score = team.immigration * Score::AUTO::IMMIGRATION;
      type1_score = (team.type_1_a * Score::AUTO::TYPE_1) + (team.type_1_b * Score::AUTO::TYPE_1);
      type2_score = team.type_2 * Score::AUTO::TYPE_2;
    } else {
      seedling_score = team.seedlings * Score::MANUAL::SEEDLINGS;
      immigration_score = team.immigration * Score::MANUAL::IMMIGRATION;
      type1_score = team.type_1_a * Score::MANUAL::TYPE_1 + team.type_1_b * Score::MANUAL::TYPE_2;
      type2_score = team.type_2 * Score::MANUAL::TYPE_2;
    }
    team.score = seedling_score + immigration_score + type1_score + type2_score;
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
    nlohmann::json body = {
        {"competitionID", competition_id_},
        {"api_key", api_key_},
    };
    auto res = tourobo_cli_->Post("/interface/api/nextmatch.php", body.dump(), "application/json");
    if (res) {
      nlohmann::json res_body = nlohmann::json::parse(res->body);

      game_state_interfaces::msg::Match match;
      game_state_interfaces::msg::Team red_team;
      game_state_interfaces::msg::Team blue_team;

      red_team.name = res_body["match"]["zone1"]["name"];
      red_team.university = res_body["match"]["zone1"]["organization"];
      red_team.id = res_body["match"]["zone1"]["teamID"];
      blue_team.name = res_body["match"]["zone2"]["name"];
      blue_team.university = res_body["match"]["zone2"]["organization"];
      blue_team.id = res_body["match"]["zone2"]["teamID"];

      match.red_team = red_team;
      match.blue_team = blue_team;
      match.id = res_body["match"]["gameID"];
      match.title = res_body["match"]["title"];
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
    current_match_.red_team.is_auto = false;
    current_match_.red_team.seedlings = 0;
    current_match_.red_team.immigration = false;
    current_match_.red_team.type_1_a = false;
    current_match_.red_team.type_1_b = false;
    current_match_.red_team.type_2 = false;
    current_match_.red_team.v_goal = false;
    current_match_.red_team.score = 0;
    current_match_.blue_team.is_auto = false;
    current_match_.blue_team.seedlings = 0;
    current_match_.blue_team.immigration = false;
    current_match_.blue_team.type_1_a = false;
    current_match_.blue_team.type_1_b = false;
    current_match_.blue_team.type_2 = false;
    current_match_.blue_team.v_goal = false;
    current_match_.blue_team.score = 0;

    current_match_.start_time.sec = 0;
    current_match_.start_time.nanosec = 0;
    current_match_.end_time.sec = 0;
    current_match_.end_time.nanosec = 0;
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
    if (!_is_match_running()) {
      res->result = false;
      return;
    }
    update_score(current_match_.red_team, req);
    res->result = true;
  }

  void blue_update_score(const game_state_interfaces::srv::UpdateScore::Request::SharedPtr req, const game_state_interfaces::srv::UpdateScore::Response::SharedPtr res) {
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

    nlohmann::json body = {
        {"competitionID", competition_id_},
        {"api_key", api_key_},
        {"gameID", current_match_.id},
        {"status", "confirmation"},
        {"winner", current_match_.winner == game_state_interfaces::msg::Match::RED ? "zone1" : "zone2"},
        {"zone1_score", current_match_.red_team.score},
        {"zone2_score", current_match_.blue_team.score},
        {"vgole", winner.v_goal},
        {"vgole_time", winner.v_goal ? time_sec : 0},
    };

    auto res = tourobo_cli_->Post("/interface/api/update_match.php", body.dump(), "application/json");
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
    RCLCPP_INFO(this->get_logger(), "===================");
  }
};
} // namespace torobo2024
