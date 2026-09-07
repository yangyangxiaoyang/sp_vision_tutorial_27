#ifndef SP_DECISION_PLUGIN_CONDITION_JUDGE_HPP_
#define SP_DECISION_PLUGIN_CONDITION_JUDGE_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "behaviortree_cpp_v3/control_node.h"
#include "rclcpp/rclcpp.hpp"

namespace sp_decision
{

/**
 * @class ConditionJudge
 * @brief BehaviorTree ControlNode that routes execution to one of N child nodes
 *        based on a list of exprtk arithmetic/logic expressions evaluated every
 *        tick against named blackboard variables.
 *
 * Input Ports
 * ───────────
 *   bb_vars    (std::string)  Semicolon-separated blackboard key names whose
 *                             values are exposed to the expressions as doubles.
 *                             e.g. "hp;game_status;enemy_detected"
 *
 *   conditions (std::string)  Semicolon-separated exprtk expressions, one per
 *                             child node.  The first expression that evaluates
 *                             to non-zero selects the child to tick.
 *                             e.g. "game_status<1; game_status==1 && hp>60; true"
 *
 *             Raw comparison symbols <, >, &&, || may be used directly in the
 *             XML attribute; the node sanitises XML entities before parsing.
 *
 * Behaviour
 * ─────────
 *   - Expressions are pre-compiled once on the first tick.
 *   - Every tick: blackboard variables are synced → expressions evaluated in
 *     order → first truthy index wins → if the winning child differs from the
 *     currently running one, the current child is halted before the new one is
 *     ticked.
 *   - Returns the ticked child's status (RUNNING / SUCCESS / FAILURE).
 *   - If no expression is truthy, returns FAILURE.
 *
 * XML example
 * ───────────
 *   <ConditionJudge bb_vars="game_status;hp"
 *                   conditions="game_status<1; game_status==1 && hp>60; true">
 *     <SubTree ID="IdleTree"/>
 *     <SubTree ID="AttackTree"/>
 *     <SubTree ID="FallbackTree"/>
 *   </ConditionJudge>
 */
class ConditionJudge : public BT::ControlNode
{
public:
  explicit ConditionJudge(
    const std::string & name,
    const BT::NodeConfiguration & config);

  ~ConditionJudge() override;

  static BT::PortsList providedPorts();

  // ── ControlNode interface ──────────────────────────────────────────────────
  BT::NodeStatus tick() override;
  void           halt() override;

private:
  // ── Lazy initialisation ───────────────────────────────────────────────────
  bool initialized_{false};

  /**
   * @brief Called once at first tick.  Reads ports, builds symbol table,
   *        compiles all expressions.  Returns false and logs errors on failure.
   */
  bool init_expressions();

  // ── Expression engine (PIMPL hides the huge exprtk.hpp from users) ────────
  struct ExprImpl;
  std::unique_ptr<ExprImpl> impl_;

  // ── Blackboard variable mirror ─────────────────────────────────────────────
  std::vector<std::string>           var_names_;   ///< ordered variable names
  std::unordered_map<std::string, double> var_vals_;  ///< current values

  /** @brief Sync all registered variables from the blackboard. */
  void sync_vars();

  /**
   * @brief Try to read a blackboard entry as double regardless of stored type.
   *        Handles double / float / int32_t / bool / string.
   * @return 0.0 if the key is absent or type is unrecognised.
   */
  double read_bb_double(const std::string & key) const;

  // ── Routing state ─────────────────────────────────────────────────────────
  int current_child_idx_{-1};   ///< index of the child currently being ticked

  // ── Helpers ───────────────────────────────────────────────────────────────
  /**
   * @brief Replace XML character entities so the user can write raw operators
   *        ( < > & ) inside XML attribute values.
   *        &lt; → <   &gt; → >   &amp;&amp; → &&   &amp; → &
   */
  static std::string sanitize_expr(const std::string & raw);

  /**
   * @brief Split string by delimiter, stripping leading/trailing whitespace
   *        from each token.
   */
  static std::vector<std::string> split(const std::string & s, char delim);
};

}  // namespace sp_decision

#endif  // SP_DECISION_PLUGIN_CONDITION_JUDGE_HPP_
