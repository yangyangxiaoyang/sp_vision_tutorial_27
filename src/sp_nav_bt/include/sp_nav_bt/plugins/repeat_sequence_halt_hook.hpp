#ifndef SP_DECISION_PLUGIN_REPEAT_SEQUENCE_HALT_HOOK_HPP_
#define SP_DECISION_PLUGIN_REPEAT_SEQUENCE_HALT_HOOK_HPP_

namespace sp_nav_bt
{

class RepeatSequenceHaltHook
{
public:
  virtual ~RepeatSequenceHaltHook() = default;
  virtual void onRepeatSequenceHalt() = 0;
};

}  // namespace sp_nav_bt

#endif  // SP_DECISION_PLUGIN_REPEAT_SEQUENCE_HALT_HOOK_HPP_