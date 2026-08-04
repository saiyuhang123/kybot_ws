#pragma once
#include <string>
#include <functional>
#include <map>
#include <vector>
#include "json/json.h"

namespace agent {

// ===== Skill 统一返回结构 =====
struct SkillResult {
    bool success;
    std::string message;           // 给 LLM 看的文字描述
    aiui_va::Json::Value data;     // 可选结构化数据（项目 jsoncpp 在 aiui_va::Json 命名空间下）
};

// ===== Skill 执行函数签名 =====
using SkillFunc = std::function<SkillResult(const aiui_va::Json::Value &params)>;

// ===== Skill 定义（注册给 LLM 看的）=====
struct SkillDef {
    std::string name;               // 如 "speak", "navigate", "pick"
    std::string description;        // 功能描述，会拼进 system prompt
    aiui_va::Json::Value paramsSchema;  // 参数 JSON Schema（可选，给 LLM 参考）
    SkillFunc execute;              // 执行函数
};

// ===== Skill 注册表 =====
class SkillRegistry {
public:
    void registerSkill(const SkillDef &skill);
    const SkillDef* find(const std::string &name) const;
    std::vector<SkillDef> listAll() const;
    std::string describeAll() const;   // 生成给 LLM 看的文本

private:
    std::map<std::string, SkillDef> skills_;
};

// ===== 初始化所有 Skill（在 agent_bridge.cpp 中实现）=====
void initSkills(SkillRegistry &reg);

// ===== Agent 模式入口 =====
bool handleText(const std::string &userText);

} // namespace agent